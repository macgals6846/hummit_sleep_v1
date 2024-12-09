#ifndef H_BLEPRPH_
#define H_BLEPRPH_

#include <stdbool.h>
#include "nimble/ble.h"
#include "modlog/modlog.h"
#ifdef __cplusplus
extern "C" {
#endif

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;
static uint8_t own_addr_type;

/** GATT server. */
#define GATT_SVR_SVC_ALERT_UUID               0x1811

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
#ifdef __cplusplus
}
#endif

#endif