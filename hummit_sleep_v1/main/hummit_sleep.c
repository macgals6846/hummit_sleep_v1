/* Control with a touch pad playing MP3 files from SD Card
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "filter_resample.h"

#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_touch.h"
#include "periph_button.h"
#include "input_key_service.h"
#include "periph_adc_button.h"
#include "board.h"

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/ledc.h"

#include "iot_button.h"

///////// BLE API ///////////////////////////////////////
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "bleprph.h"

////////// Local API //////////////////////////////////
#include "const_vars.h"


/////////// SD Card API /////////////////////////////////////
#include "sdcard_list.h"
#include "sdcard_scan.h"


static const char *TAG = "hummit_sleep";
TaskHandle_t motorTaskHandle = NULL; //// Global variable to hold the task handle for motor command

audio_pipeline_handle_t pipeline;
audio_element_handle_t i2s_stream_writer, mp3_decoder, fatfs_stream_reader, rsp_handle;
playlist_operator_handle_t sdcard_list_handle = NULL;

///////////////////functions////////////////////////////
static void music_next_ifnot_state_initial(){
    audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
    if(el_state==AEL_STATE_INIT){
        ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
        if (audio_pipeline_run(pipeline) != ESP_OK) {
            ESP_LOGI(TAG, "Failed to start pipeline");
        }
    }
    else{
        ESP_LOGI(TAG, "[ * ] Stopped, advancing to the next song");
        char *url = NULL;
        audio_pipeline_stop(pipeline);
        audio_pipeline_wait_for_stop(pipeline);
        audio_pipeline_terminate(pipeline);
        sdcard_list_next(sdcard_list_handle, 1, &url);
        ESP_LOGW(TAG, "URL: %s", url);
        audio_element_set_uri(fatfs_stream_reader, url);
        audio_pipeline_reset_ringbuffer(pipeline);
        audio_pipeline_reset_elements(pipeline);
        audio_pipeline_run(pipeline);
    }     
}
static void button_event_double_cb(void *arg, void *data)
{
    iot_button_print_event((button_handle_t)arg);
    motor_cmd(motor_program_1);
    music_next_ifnot_state_initial();
}
static void button_event_multiple_cb(void *arg, void *data)
{
    iot_button_print_event((button_handle_t)arg);
    motor_cmd(motor_program_2);
    music_next_ifnot_state_initial();
}
void hummit_button_init(uint32_t button_num)
{
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = button_num,
            .active_level = BUTTON_ACTIVE_LEVEL,
#if CONFIG_GPIO_BUTTON_SUPPORT_POWER_SAVE
            .enable_power_save = true,
#endif
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    assert(btn);
    //err = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_PRESS_UP, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT_DONE, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_event_cb, NULL);
    esp_err_t err = iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_event_double_cb, NULL);
    button_event_config_t btn_evt_cfg = {
        .event = BUTTON_MULTIPLE_CLICK,
        .event_data.multiple_clicks.clicks = 3,
    };
    err |= iot_button_register_event_cb(btn, btn_evt_cfg, button_event_multiple_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, button_event_cb, NULL);
    //err |= iot_button_register_cb(btn, BUTTON_PRESS_END, button_event_cb, NULL);

    ESP_ERROR_CHECK(err);
}


void vibro_motor_set_freq(uint32_t pwm){
    // Adjust frequency if needed later
    if(pwm<=1024 && pwm>0){
            // Configure timer for PWM
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_10_BIT,  // Resolution of PWM duty
            .freq_hz = 5000,                       // Frequency of PWM signal
            .speed_mode = LEDC_LOW_SPEED_MODE,     // Timer mode
            .timer_num = LEDC_TIMER_0,             // Timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
        ledc_timer_config(&ledc_timer);

        // Configure channel for PWM
        ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = pwm,                     // Set duty to 50%
            .gpio_num   = 8,                       // GPIO number
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        };
        ledc_channel_config(&ledc_channel);
    }
    else if(pwm==0){
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // 0 for low logic level on stop
    }
}
// Task to be executed every 100 ms
void executeMotorCommand(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Convert 100 ms to ticks
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint8_t repeatCycles = *((int *)pvParameters);

    const int motor_speed = 1024; //10 bit pwm
    const int increaseTime = 15000;  // 15 seconds in milliseconds
    const int maintainTime = 45000;  // 45 seconds in milliseconds
    const int decreaseTime = 15000;  // 15 seconds in milliseconds
    const int waitTime =180000;  // 180 seconds in milliseconds
    const int period = 100;          // Period of 100 ms
    const int stepIncrease = motor_speed / (increaseTime / period);
    const int stepDecrease = motor_speed / (decreaseTime / period);

    int cycle = 0;
    int timeElapsed = 0;
    int rpm = 0;
    int phase = 0; // 0: increase, 1: maintain, 2: decrease

    while (cycle < repeatCycles) {
        switch (phase) {
            case 0:  // Ramp up phase
                rpm += stepIncrease;
                vibro_motor_set_freq(rpm);
                if (timeElapsed >= increaseTime) {
                    rpm = motor_speed;
                    vibro_motor_set_freq(rpm);
                    phase = 1;
                    timeElapsed = 0;
                }
                break;
            case 1:  // Maintain phase
                rpm = motor_speed;
                if (timeElapsed >= maintainTime) {
                    phase = 2;
                    timeElapsed = 0;
                }
                break;
            case 2:  // Ramp down phase
                rpm -= stepDecrease;
                vibro_motor_set_freq(rpm);
                if (timeElapsed >= decreaseTime) {
                    rpm = 0;
                    vibro_motor_set_freq(rpm);
                    timeElapsed = 0;
                    phase = 3;
                    cycle++;
                }
                break;
            case 3:  // Maintain phase
                rpm = 0;
                if (timeElapsed >= waitTime) {
                    phase = 0;
                    timeElapsed = 0;
                }
                break;
        }

        ESP_LOGI(TAG, "Cycle %d: RPM at %d\n", cycle + 1, rpm);

        // Wait until the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        timeElapsed += period;
    }

    // Once done, delete the task if it does not need to run again
    motorTaskHandle = NULL;
    vTaskDelete(NULL);

}

void motor_cmd(uint8_t cmd){
    ESP_LOGI(TAG, "motor command: %d",cmd);
    if(cmd == motor_program_1){
        if (motorTaskHandle != NULL) {
            ESP_LOGI(TAG, "Deleting motor task from control task...");
            vTaskDelete(motorTaskHandle);
            motorTaskHandle = NULL;  // Reset the handle to indicate task is deleted
        }
        //motor_program = motor_program_1;
        uint8_t repeatCyclesFirst = 5; // Motor Program 1
        xTaskCreatePinnedToCore(executeMotorCommand, "Motor Program 1", 4096, &repeatCyclesFirst, 5, &motorTaskHandle, 1);
    }
    else if(cmd == motor_program_2){
        if (motorTaskHandle != NULL) {
            ESP_LOGI(TAG, "Deleting motor task from control task...");
            vTaskDelete(motorTaskHandle);
            motorTaskHandle = NULL;  // Reset the handle to indicate task is deleted
        }
        //motor_program = motor_program_2;
        uint8_t repeatCyclesSecond = 10; // Motor Program 1
        xTaskCreatePinnedToCore(executeMotorCommand, "Motor Program 2", 4096, &repeatCyclesSecond, 5, &motorTaskHandle, 1);
    }
    else if(cmd == motor_stop){
        vibro_motor_set_freq(0);
        if (motorTaskHandle != NULL) {
            ESP_LOGI(TAG, "Deleting motor task from control task...");
            vTaskDelete(motorTaskHandle);
            motorTaskHandle = NULL;  // Reset the handle to indicate task is deleted
        }
    }
    else{
        ESP_LOGI(TAG, "Motor Command not defined");
    }
}

void music_cmd(uint8_t cmd){
    ESP_LOGI(TAG, "music command: %d",cmd);
    //audio_board_handle_t board_handle = audio_board_get_handle();
    int player_volume;
    //audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    if(cmd == music_play){
        ESP_LOGI(TAG, "music play");
        audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
        switch (el_state) {
            case AEL_STATE_INIT :
                ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                if (audio_pipeline_run(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to start pipeline");
                }
                break;
            case AEL_STATE_RUNNING :
                ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                if (audio_pipeline_pause(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to pause pipeline");
                }
                break;
            case AEL_STATE_PAUSED :
                ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                if (audio_pipeline_resume(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to resume pipeline");
                }
                break;
            default :
                ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
        }
    }
    else if(cmd == music_set){
       ESP_LOGI(TAG, "music set");
       ESP_LOGI(TAG, "[ * ] [Set] input key event");
       ESP_LOGI(TAG, "[ * ] Stopped, advancing to the next song");
       char *url = NULL;
       audio_pipeline_stop(pipeline);
       audio_pipeline_wait_for_stop(pipeline);
       audio_pipeline_terminate(pipeline);
       sdcard_list_next(sdcard_list_handle, 1, &url);
       ESP_LOGW(TAG, "URL: %s", url);
       audio_element_set_uri(fatfs_stream_reader, url);
       audio_pipeline_reset_ringbuffer(pipeline);
       audio_pipeline_reset_elements(pipeline);
       audio_pipeline_run(pipeline);
    }
    else if(cmd == music_volup){
        ESP_LOGI(TAG, "music volume up");
        ESP_LOGI(TAG, "[ * ] [Vol+] input key event");
        player_volume += 10;
        if (player_volume > 100) {
            player_volume = 100;
        }
        //audio_hal_set_volume(board_handle->audio_hal, player_volume);
        ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
    }
    else if(cmd == music_voldown){
        ESP_LOGI(TAG, "music volume down");
        ESP_LOGI(TAG, "[ * ] [Vol-] input key event");
        player_volume -= 10;
        if (player_volume < 0) {
            player_volume = 0;
        }
        //audio_hal_set_volume(board_handle->audio_hal, player_volume);
        ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
    }
    else{
        ESP_LOGI(TAG, "Music Command not defined");
    }
}


void ble_store_config_init(void);

/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    ESP_LOGI(TAG, "handle=%d our_ota_addr_type=%d our_ota_addr=%02x:%02x:%02x:%02x:%02x:%02x",
             desc->conn_handle, desc->our_ota_addr.type,
             desc->our_ota_addr.val[5],
             desc->our_ota_addr.val[4],
             desc->our_ota_addr.val[3],
             desc->our_ota_addr.val[2],
             desc->our_ota_addr.val[1],
             desc->our_ota_addr.val[0]);

    ESP_LOGI(TAG, "our_id_addr_type=%d our_id_addr=%02x:%02x:%02x:%02x:%02x:%02x",
             desc->our_id_addr.type,
             desc->our_id_addr.val[5],
             desc->our_id_addr.val[4],
             desc->our_id_addr.val[3],
             desc->our_id_addr.val[2],
             desc->our_id_addr.val[1],
             desc->our_id_addr.val[0]);

    ESP_LOGI(TAG, "peer_ota_addr_type=%d peer_ota_addr=%02x:%02x:%02x:%02x:%02x:%02x",
             desc->peer_ota_addr.type,
             desc->peer_ota_addr.val[5],
             desc->peer_ota_addr.val[4],
             desc->peer_ota_addr.val[3],
             desc->peer_ota_addr.val[2],
             desc->peer_ota_addr.val[1],
             desc->peer_ota_addr.val[0]);

    ESP_LOGI(TAG, "peer_id_addr_type=%d peer_id_addr=%02x:%02x:%02x:%02x:%02x:%02x",
             desc->peer_id_addr.type,
             desc->peer_id_addr.val[5],
             desc->peer_id_addr.val[4],
             desc->peer_id_addr.val[3],
             desc->peer_id_addr.val[2],
             desc->peer_id_addr.val[1],
             desc->peer_id_addr.val[0]);

    ESP_LOGI(TAG, "conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void 
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included in transmit data; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    // complete name fits in one packet
    fields.name_is_complete = 1; 

    /*
    SVC_ALERT - commonly used in wearables and IoT devices that provide real-time notifications from a paired device.
    */
    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID) //service that deals with alert notifications
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; //undericted, advertise without targeting specific peer
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; //generally discoverable by any scanning device that is in the range
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
        }

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);

        /* Connection terminated; resume advertising. */
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        ESP_LOGI(TAG, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU: //MTU size defines the largest amount of data that can be sent in a single BLE packet
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    return 0;
}

static void
bleprph_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void
bleprph_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address:%02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5],
             addr_val[4],
             addr_val[3],
             addr_val[2],
             addr_val[1],
             addr_val[0]);
    /* Begin advertising. */
    bleprph_advertise();
}

void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    /* Handle touch pad events
           to start, pause, resume, finish current song and adjust volume
        */
    audio_board_handle_t board_handle = (audio_board_handle_t) ctx;
    int player_volume;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        ESP_LOGI(TAG, "[ * ] input key id is %d", (int)evt->data);
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_PLAY:
                ESP_LOGI(TAG, "[ * ] [Play] input key event");
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                switch (el_state) {
                    case AEL_STATE_INIT :
                        ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                        if (audio_pipeline_run(pipeline) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to start pipeline");
                        }
                        break;
                    case AEL_STATE_RUNNING :
                        ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                        if (audio_pipeline_pause(pipeline) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to pause pipeline");
                        }
                        break;
                    case AEL_STATE_PAUSED :
                        ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                        if (audio_pipeline_resume(pipeline) != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to resume pipeline");
                        }
                        break;
                    default :
                        ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
                }
                break;
            case INPUT_KEY_USER_ID_SET:
                ESP_LOGI(TAG, "[ * ] [Set] input key event");
                ESP_LOGI(TAG, "[ * ] Stopped, advancing to the next song");
                char *url = NULL;
                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_terminate(pipeline);
                sdcard_list_next(sdcard_list_handle, 1, &url);
                ESP_LOGW(TAG, "URL: %s", url);
                audio_element_set_uri(fatfs_stream_reader, url);
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                audio_pipeline_run(pipeline);
                break;
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGI(TAG, "[ * ] [Vol+] input key event");
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGI(TAG, "[ * ] [Vol-] input key event");
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
                break;
        }
    }

    return ESP_OK;
}

void sdcard_url_save_cb(void *user_data, char *url)
{
    playlist_operator_handle_t sdcard_handle = (playlist_operator_handle_t)user_data;
    esp_err_t ret = sdcard_list_save(sdcard_handle, url);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail to save sdcard url to sdcard playlist");
    }
}

void nimBLE_init(){
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Welcome");

    //ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    //wifi_init_sta();
    //do_ping_cmd();

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return;
    }

        /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("hummit");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(bleprph_host_task);
}


void ledc_motor_init(){
    int rc;
    //////////////////////////////////////////////////////////////////////////////////////////initialize motor
    // Prepare and then apply the LEDC PWM timer configuration
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,  // resolution of PWM duty
        .freq_hz = 0,                       // frequency of PWM signal, starts at Zero or Stop
        .speed_mode = LEDC_LOW_SPEED_MODE,     // timer mode
        .timer_num = LEDC_TIMER_0,             // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    rc = ledc_timer_config(&ledc_timer);
    //assert(rc == ESP_OK);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 512,                     // Set duty to 50% of 10-bit value
        .gpio_num   = 8,                       // GPIO number
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    rc = ledc_channel_config(&ledc_channel);
    //assert(rc == ESP_OK);
    vibro_motor_set_freq(0); //Set initially to Stop
}

void app_main(void)
{
    esp_log_level_set("MP3_DECODER", ESP_LOG_VERBOSE);
    esp_log_level_set("AUDIO_EVT", ESP_LOG_VERBOSE);
    esp_log_level_set("AUDIO_PIPELINE", ESP_LOG_VERBOSE);
    esp_log_level_set("FATFS_STREAM", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    nimBLE_init();
    ledc_motor_init();

    ESP_LOGI(TAG, "[1.0] Initialize peripherals management");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[1.1] Initialize and start peripherals");
    //audio_board_key_init(set);
    audio_board_sdcard_init(set, SD_MODE_SPI);

    ESP_LOGI(TAG, "[1.2] Set up a sdcard playlist and scan sdcard music save to it");
    sdcard_list_create(&sdcard_list_handle);
    sdcard_scan(sdcard_url_save_cb, "/sdcard", 0, (const char *[]) {"mp3"}, 1, sdcard_list_handle);
    sdcard_list_show(sdcard_list_handle);

    //ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    //audio_board_handle_t board_handle = audio_board_init();
    //audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

/*
    ESP_LOGI(TAG, "[ 3 ] Create and start input key service");
    //input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_info_t input_key_info_custom[] = {
        {
            .type = PERIPH_ID_BUTTON,
            .user_id = INPUT_KEY_USER_ID_PLAY,
            .act_id = PERIPH_BUTTON_PRESSED,
            //.pin = BUTTON_PLAY_ID,
        },
    };
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info_custom, 1);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);
    */

    ESP_LOGI(TAG, "[4.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[4.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    i2s_stream_set_clk(i2s_stream_writer, 44100, 16, 2);

    ESP_LOGI(TAG, "[4.2] Create mp3 decoder to decode mp3 file");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    /* ZL38063 audio chip on board of ESP32-LyraTD-MSC does not support 44.1 kHz sampling frequency,
       so resample filter has been added to convert audio data to other rates accepted by the chip.
       You can resample the data to 16 kHz or 48 kHz.
    

    ESP_LOGI(TAG, "[4.3] Create resample filter");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_handle = rsp_filter_init(&rsp_cfg);
    */
    ESP_LOGI(TAG, "[4.4] Create fatfs stream to read data from sdcard");
    char *url = NULL;
    sdcard_list_current(sdcard_list_handle, &url);
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = AUDIO_STREAM_READER;
    fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);
    audio_element_set_uri(fatfs_stream_reader, url);

    ESP_LOGI(TAG, "[4.5] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    //audio_pipeline_register(pipeline, rsp_handle, "filter");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[4.6] Link it together [sdcard]-->fatfs_stream-->mp3_decoder-->resample-->i2s_stream-->[codec_chip]");
    const char *link_tag[3] = {"file", "mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);

    ESP_LOGI(TAG, "[5.0] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[5.1] Listen for all pipeline events");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGW(TAG, "[ 6 ] Press the keys to control music player:");
    ESP_LOGW(TAG, "      [Play] to start, pause and resume, [Set] next song.");
    ESP_LOGW(TAG, "      [Vol-] or [Vol+] to adjust volume.");

    hummit_button_init(5);

    /*
    ESP_LOGI(TAG, "[ * ] [Play] input key event");
    audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
    switch (el_state) {
        case AEL_STATE_INIT :
            ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
            audio_pipeline_run(pipeline);
            break;
        case AEL_STATE_RUNNING :
            ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
            audio_pipeline_pause(pipeline);
            break;
        case AEL_STATE_PAUSED :
            ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
            audio_pipeline_resume(pipeline);
            break;
        default :
            ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
    }
    */
                /*
                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_terminate(pipeline);
                sdcard_list_next(sdcard_list_handle, 1, &url);
                ESP_LOGW(TAG, "URL: %s", url);
                audio_element_set_uri(fatfs_stream_reader, url);
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                audio_pipeline_run(pipeline);

                vTaskDelay(pdMS_TO_TICKS(1000));

                if (audio_pipeline_run(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to start pipeline");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));

                ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                if (audio_pipeline_pause(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to pause pipeline");
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
         
                ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                if (audio_pipeline_resume(pipeline) != ESP_OK) {
                    ESP_LOGI(TAG, "Failed to resume pipeline");
                }
                */

    while (1) {
        // Handle event interface messages from pipeline
        //   to set music info and to advance to the next song

        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT) {
            // Set music info for a new song to be played
            if (msg.source == (void *) mp3_decoder
                && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
                audio_element_info_t music_info = {0};
                audio_element_getinfo(mp3_decoder, &music_info);
                ESP_LOGI(TAG, "[ * ] Received music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                         music_info.sample_rates, music_info.bits, music_info.channels);
                audio_element_setinfo(i2s_stream_writer, &music_info);
                //rsp_filter_set_src_info(rsp_handle, music_info.sample_rates, music_info.channels);
                continue;
            }
            // Advance to the next song when previous finishes
            if (msg.source == (void *) i2s_stream_writer
                && msg.cmd == AEL_MSG_CMD_REPORT_STATUS) {
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                if (el_state == AEL_STATE_FINISHED) {
                    ESP_LOGI(TAG, "[ * ] Finished, advancing to the next song");
                    sdcard_list_next(sdcard_list_handle, 1, &url);
                    ESP_LOGW(TAG, "URL: %s", url);
                    // In previous versions, audio_pipeline_terminal() was called here. It will close all the element task and when we use
                    // the pipeline next time, all the tasks should be restarted again. It wastes too much time when we switch to another music.
                    // So we use another method to achieve this as below.
                    
                    audio_element_set_uri(fatfs_stream_reader, url);
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                    audio_pipeline_run(pipeline);
                }
                continue;
            }
        }
    }

    ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, mp3_decoder);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    //audio_pipeline_unregister(pipeline, rsp_handle);

    // Terminate the pipeline before removing the listener 
    audio_pipeline_remove_listener(pipeline);

    // Stop all peripherals before removing the listener 
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    // Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface 
    audio_event_iface_destroy(evt);

    // Release all resources
    sdcard_list_destroy(sdcard_list_handle);
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);
    //audio_element_deinit(rsp_handle);
    //periph_service_destroy(input_ser);
    esp_periph_set_destroy(set);

}
