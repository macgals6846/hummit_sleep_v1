#ifndef H_CONST_VARS_
#define H_CONST_VARS_

///////////// constants ////////////////////////
//WIFI
#define ESP_WIFI_SSID      "myssid"
#define ESP_WIFI_PASS      "mypassword"
#define ESP_MAXIMUM_RETRY  5
#define PING_IP            "93.184.216.34"
#define PING_COUNT         10000
#define PING_INTERVAL      1
//BLE



//SYSTEM
enum motor_cmd {
    motor_stop = 0,
    motor_program_1 = 1,
    motor_program_2 = 2,
    motor_program_3 = 4
};

enum music_cmd {
    music_play = 0,
    music_set = 1,
    music_volup = 2,
    music_voldown = 4
};
#define BUTTON_ACTIVE_LEVEL 1

//SD CARD
 #define MOUNT_POINT "/sdcard"



///////////// variables ////////////////////////
//WIFI

//BLE

//SYSTEM
static volatile uint8_t motor_program = motor_stop; // Flag to control task execution

//SDCARD



///////////// function declarations ////////////////////////
//WIFI

//BLE


//SYSTEM
void motor_cmd(uint8_t cmd);
void vibro_motor_set_freq(uint32_t freq);
void nimBLE_init();
void ledc_motor_init();
void music_cmd(uint8_t cmd);


#endif