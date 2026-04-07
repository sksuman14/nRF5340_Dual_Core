/* ========================================================
 * nRF5340 Data Logger — NETWORK CORE (cpunet)
 * NCS v2.5.1
 *
 * Responsibilities:
 *   - Run BLE stack (extended advertising + passive scanning)
 *   - Scan for trigger / reset packets from mobile
 *   - On trigger  → send IPC_MSG_TYPE_TRIGGER to APP core
 *   - On reset    → send IPC_MSG_TYPE_RESET   to APP core
 *   - On IPC DATA → advertise packet via BLE extended advertising
 * ======================================================== */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/ipc/ipc_service.h>
#include <string.h>
#include <errno.h>

#include "ipc_msg.h"

/* ============================
 * Matching constants from original code
 * ============================ */
#define PACKET_RETRIES        3
#define ADVERT_DURATION_MS    200
#define RETRY_GAP_MS          200
#define RSSI_THRESHOLD        -50

#define SCAN_INTERVAL_SECONDS 10
#define SCAN_DURATION_MS      100
#define FAST_SCAN_INTERVAL_MS 1000
#define FAST_SCAN_DURATION_MS 2000
#define FAST_SCAN_CYCLES      3

/* Trigger / reset signatures — same as original nRF52832 code */
static const uint8_t trigger_signature[] = {0x59, 0x00, 0xBB, 0xCC};
static const uint8_t reset_signature[]   = {0x59, 0x00, 0xFF, 0xFF};

#define SENSOR_ADDRESS "DE:AD:BE:AF:BA:58"

/* ============================
 * LED — blink to confirm net core is alive
 * ============================ */
static const struct gpio_dt_spec net_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* ============================
 * BLE advertising
 * ============================ */
static struct bt_le_ext_adv *adv;

static struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_NONE | BT_LE_ADV_OPT_EXT_ADV |
    BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_NO_2M,
    0x30, 0x30, NULL);

static struct bt_le_ext_adv_start_param ext_adv_param =
    BT_LE_EXT_ADV_START_PARAM_INIT(0, 1);

static const struct bt_data name_ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static uint8_t mfg_payload[TX_VALID_SIZE] __aligned(4);

/* ============================
 * BLE scanning
 * ============================ */
static struct bt_le_scan_param scan_param = {
    .type     = BT_LE_SCAN_TYPE_PASSIVE,
    .options  = BT_LE_SCAN_OPT_NONE,
    .interval = 0x0030,
    .window   = 0x0030,
};

static volatile bool    is_scanning       = false;
static volatile bool    fast_scan_mode    = false;
static volatile uint8_t fast_scan_counter = 0;
static volatile bool    scan_work_pending = false;
static volatile bool    tx_active         = false;

/* ============================
 * IPC
 * ============================ */
static struct ipc_ept net_ept;
static K_SEM_DEFINE(ipc_bound_sem, 0, 1);

/* Queue for incoming data packets from APP core */
K_MSGQ_DEFINE(data_msgq, IPC_MAX_MSG_SIZE, 8, 4);

/* ============================
 * Work & Timers
 * ============================ */
static struct k_work_delayable scan_work;
static struct k_work_delayable stop_scan_work;
static struct k_timer          scan_scheduler_timer;
static struct k_work           tx_work;
static struct k_work           led_blink_work;

K_MUTEX_DEFINE(scan_ctrl_mutex);

/* ============================
 * Forward declarations
 * ============================ */
static void adv_init(void);
static void transmit_name(void);
static void transmit_mfg_data(void);
static int  try_set_mfg_data(uint8_t *buf, size_t len, size_t *out_len);
static void enter_fast_scan_mode(void);
static void scan_work_handler(struct k_work *w);
static void stop_scan_work_handler(struct k_work *w);
static void scan_timer_cb(struct k_timer *t);
static void tx_work_handler(struct k_work *w);
static void led_blink_handler(struct k_work *w);
static bool ad_parser_cb(struct bt_data *data, void *user_data);
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad_buf);
static void ipc_bound_cb(void *priv);
static void ipc_recv_cb(const void *data, size_t len, void *priv);
static void ipc_send_cmd(uint8_t type);

/* ============================
 * IPC callbacks
 * ============================ */
static const struct ipc_ept_cfg net_ept_cfg = {
    .name = "app_ept",   /* must match app_core name exactly */
    .prio = 0,
    .cb   = {
        .bound    = ipc_bound_cb,
        .received = ipc_recv_cb,
    },
};

static void ipc_bound_cb(void *priv)
{
    ARG_UNUSED(priv);
    printk("[NET] IPC endpoint bound\n");
    k_sem_give(&ipc_bound_sem);
}

static void ipc_recv_cb(const void *data, size_t len, void *priv)
{
    ARG_UNUSED(priv);
    if (len < 1) return;

    const uint8_t *msg = (const uint8_t *)data;

    if (msg[0] == IPC_MSG_TYPE_DATA) {
        printk("[NET] Data packet received from APP (%d bytes)\n",
               (int)len);

        uint8_t buf[IPC_MAX_MSG_SIZE];
        size_t copy_len = (len <= IPC_MAX_MSG_SIZE) ? len : IPC_MAX_MSG_SIZE;
        memcpy(buf, msg, copy_len);

        if (k_msgq_put(&data_msgq, buf, K_NO_WAIT) != 0) {
            printk("[NET] Queue full — dropping packet\n");
        } else {
            k_work_submit(&tx_work);
        }
    }
}

static void ipc_send_cmd(uint8_t type)
{
    uint8_t msg[1] = { type };
    int err = ipc_service_send(&net_ept, msg, sizeof(msg));
    if (err < 0) {
        printk("[NET] IPC send failed (err %d)\n", err);
    }
}

/* ============================
 * LED blink — confirms net core is running
 * ============================ */
static struct k_timer led_blink_timer;

static void led_blink_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&led_blink_work);
}

static void led_blink_handler(struct k_work *w)
{
    ARG_UNUSED(w);
    gpio_pin_toggle_dt(&net_led);
}

/* ============================
 * BLE advertising helpers
 * ============================ */
static void adv_init(void)
{
    int err = bt_le_ext_adv_create(&adv_param, NULL, &adv);
    if (err) {
        printk("[NET] Failed to create ext adv set (err %d)\n", err);
    } else {
        printk("[NET] Extended adv set created\n");
    }
}

static void transmit_name(void)
{
    int err = bt_le_ext_adv_set_data(adv, name_ad,
                                     ARRAY_SIZE(name_ad), NULL, 0);
    if (err) {
        printk("[NET] set_data (name) failed %d\n", err);
        return;
    }
    err = bt_le_ext_adv_start(adv, &ext_adv_param);
    if (err) {
        printk("[NET] adv_start (name) failed %d\n", err);
        return;
    }
    k_msleep(200);
    bt_le_ext_adv_stop(adv);
    printk("[NET] Name adv done\n");
}

static int try_set_mfg_data(uint8_t *buf, size_t len, size_t *out_len)
{
    const size_t step = 16;
    size_t cur = len;
    int err;

    while (cur >= 16) {
        struct bt_data tmp[] = {
            BT_DATA(BT_DATA_MANUFACTURER_DATA, buf, cur),
        };
        err = bt_le_ext_adv_set_data(adv, tmp, ARRAY_SIZE(tmp), NULL, 0);
        if (!err) {
            if (out_len) *out_len = cur;
            return 0;
        }
        if (cur <= step) break;
        cur -= step;
    }

    uint8_t small[16] = {0};
    struct bt_data fb[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, small, sizeof(small))
    };
    err = bt_le_ext_adv_set_data(adv, fb, ARRAY_SIZE(fb), NULL, 0);
    if (!err) {
        if (out_len) *out_len = sizeof(small);
        return 0;
    }
    return -EIO;
}

static void transmit_mfg_data(void)
{
    size_t used_len = 0;
    int err = try_set_mfg_data(mfg_payload, TX_VALID_SIZE, &used_len);
    if (err) {
        printk("[NET] All mfg_data attempts failed\n");
        return;
    }
    err = bt_le_ext_adv_start(adv, &ext_adv_param);
    if (err) {
        printk("[NET] adv_start (data) failed %d\n", err);
        return;
    }
    k_msleep(ADVERT_DURATION_MS);
    bt_le_ext_adv_stop(adv);
    printk("[NET] Data adv sent (len=%zu)\n", used_len);
}

/* ============================
 * TX work: dequeue packets and advertise
 * ============================ */
static void tx_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);
    uint8_t buf[IPC_MAX_MSG_SIZE];

    /* Stop scanning while transmitting */
    k_mutex_lock(&scan_ctrl_mutex, K_FOREVER);
    if (is_scanning) {
        bt_le_scan_stop();
        is_scanning = false;
        k_work_cancel_delayable(&stop_scan_work);
    }
    k_timer_stop(&scan_scheduler_timer);
    k_mutex_unlock(&scan_ctrl_mutex);

    tx_active = true;

    static int64_t last_name_tx = 0;

    while (k_msgq_get(&data_msgq, buf, K_NO_WAIT) == 0) {
        /* buf[0] = IPC_MSG_TYPE_DATA, buf[1..] = TX_VALID_SIZE bytes */
        memcpy(mfg_payload, &buf[1], TX_VALID_SIZE);

        /* Send name burst every 5 s */
        int64_t now = k_uptime_get();
        if ((now - last_name_tx) > 5000) {
            for (int i = 0; i < 3; i++) {
                transmit_name();
                k_msleep(50);
            }
            last_name_tx = now;
        }

        /* Transmit with retries */
        for (int r = 0; r < PACKET_RETRIES; r++) {
            transmit_mfg_data();
            if (r < PACKET_RETRIES - 1) {
                k_msleep(RETRY_GAP_MS);
            }
        }

        ipc_send_cmd(IPC_MSG_TYPE_ACK);
    }

    tx_active = false;

    /* Restart scanning */
    k_mutex_lock(&scan_ctrl_mutex, K_FOREVER);
    if (!fast_scan_mode) {
        k_timer_start(&scan_scheduler_timer,
                      K_SECONDS(SCAN_INTERVAL_SECONDS),
                      K_SECONDS(SCAN_INTERVAL_SECONDS));
    } else {
        k_timer_start(&scan_scheduler_timer,
                      K_MSEC(FAST_SCAN_INTERVAL_MS), K_NO_WAIT);
    }
    k_mutex_unlock(&scan_ctrl_mutex);
}

/* ============================
 * BLE Scanning
 * ============================ */
struct adv_parse_ctx {
    int8_t rssi;
    bool   trigger_found;
    bool   reset_found;
};

static bool ad_parser_cb(struct bt_data *data, void *user_data)
{
    struct adv_parse_ctx *ctx = (struct adv_parse_ctx *)user_data;

    if (data->type == BT_DATA_MANUFACTURER_DATA) {
        if (data->data_len >= sizeof(trigger_signature) &&
            memcmp(data->data, trigger_signature,
                   sizeof(trigger_signature)) == 0) {
            ctx->trigger_found = true;
            return false;
        }
        if (data->data_len >= sizeof(reset_signature) &&
            memcmp(data->data, reset_signature,
                   sizeof(reset_signature)) == 0) {
            ctx->reset_found = true;
            return false;
        }
    }
    return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad_buf)
{
    if (rssi < RSSI_THRESHOLD) return;

    static struct adv_parse_ctx ctx;
    ctx.rssi          = rssi;
    ctx.trigger_found = false;
    ctx.reset_found   = false;

    bt_data_parse(ad_buf, ad_parser_cb, &ctx);

    if (ctx.trigger_found) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
        printk("[NET] Trigger from %s RSSI=%d\n", addr_str, rssi);
        enter_fast_scan_mode();
        ipc_send_cmd(IPC_MSG_TYPE_TRIGGER);
    } else if (ctx.reset_found) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
        printk("[NET] Reset from %s RSSI=%d\n", addr_str, rssi);
        ipc_send_cmd(IPC_MSG_TYPE_RESET);
    }
}

/* ============================
 * Scan scheduling
 * ============================ */
static void scan_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    if (!scan_work_pending) {
        scan_work_pending = true;
        k_work_schedule(&scan_work, K_NO_WAIT);
    }
}

static void scan_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);
    scan_work_pending = false;

    if (tx_active) {
        printk("[NET] Skip scan — TX active\n");
        k_mutex_lock(&scan_ctrl_mutex, K_FOREVER);
        k_timer_start(&scan_scheduler_timer,
                      fast_scan_mode
                          ? K_MSEC(FAST_SCAN_INTERVAL_MS)
                          : K_SECONDS(SCAN_INTERVAL_SECONDS),
                      K_NO_WAIT);
        k_mutex_unlock(&scan_ctrl_mutex);
        return;
    }

    if (is_scanning) {
        printk("[NET] Already scanning\n");
        return;
    }

    int err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        printk("[NET] Scan start failed %d\n", err);
        is_scanning = false;
    } else {
        is_scanning = true;
        printk("[NET] Scanning started\n");
    }

    uint32_t dur = fast_scan_mode ? FAST_SCAN_DURATION_MS : SCAN_DURATION_MS;
    k_work_schedule(&stop_scan_work, K_MSEC(dur));
    printk("[NET] Will stop scan in %u ms\n", dur);
}

static void stop_scan_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);
    if (!is_scanning) return;

    bt_le_scan_stop();
    is_scanning = false;

    if (fast_scan_mode) {
        printk("[NET] Fast scan %d/%d done\n",
               FAST_SCAN_CYCLES - fast_scan_counter + 1,
               FAST_SCAN_CYCLES);
        fast_scan_counter--;
        if (fast_scan_counter == 0) {
            fast_scan_mode = false;
            printk("[NET] Returning to normal schedule\n");
            k_timer_start(&scan_scheduler_timer,
                          K_SECONDS(SCAN_INTERVAL_SECONDS),
                          K_SECONDS(SCAN_INTERVAL_SECONDS));
        } else {
            k_timer_start(&scan_scheduler_timer,
                          K_MSEC(FAST_SCAN_INTERVAL_MS), K_NO_WAIT);
        }
    } else {
        printk("[NET] Normal scan done\n");
    }
}

static void enter_fast_scan_mode(void)
{
    if (k_mutex_lock(&scan_ctrl_mutex, K_NO_WAIT) != 0) {
        printk("[NET] Cannot enter fast scan — busy\n");
        return;
    }

    if (fast_scan_mode) {
        fast_scan_counter = FAST_SCAN_CYCLES;
        printk("[NET] Fast scan counter reset\n");
    } else {
        printk("[NET] Entering fast scan mode\n");
        fast_scan_mode    = true;
        fast_scan_counter = FAST_SCAN_CYCLES;

        k_timer_stop(&scan_scheduler_timer);

        if (is_scanning) {
            bt_le_scan_stop();
            is_scanning = false;
            k_work_cancel_delayable(&stop_scan_work);
        }

        if (scan_work_pending) {
            k_work_cancel_delayable(&scan_work);
            scan_work_pending = false;
        }

        k_work_schedule(&scan_work, K_MSEC(100));
    }

    k_mutex_unlock(&scan_ctrl_mutex);
}

/* ============================
 * main()
 * ============================ */
int main(void)
{
    printk("=== nRF5340 NET CORE — BLE stack ===\n");

    /* LED to confirm net core is alive */
    if (device_is_ready(net_led.port)) {
        gpio_pin_configure_dt(&net_led, GPIO_OUTPUT_INACTIVE);
    }

    /* Set static random BT address */
    bt_addr_le_t addr;
    if (bt_addr_le_from_str(SENSOR_ADDRESS, "random", &addr) == 0) {
        bt_id_create(&addr, NULL);
    }

    /* Enable Bluetooth */
    int err = bt_enable(NULL);
    if (err) {
        printk("[NET] BT enable failed %d\n", err);
        return -1;
    }
    printk("[NET] BT enabled\n");

    adv_init();
    k_msleep(100);

    /* IPC setup */
    const struct device *ipc_dev = DEVICE_DT_GET(DT_NODELABEL(ipc0));
    if (!device_is_ready(ipc_dev)) {
        printk("[NET] FATAL: IPC device not ready\n");
        return -1;
    }

    err = ipc_service_open_instance(ipc_dev);
    if (err < 0 && err != -EALREADY) {
        printk("[NET] ipc_service_open_instance failed (%d)\n", err);
        return err;
    }

    err = ipc_service_register_endpoint(ipc_dev, &net_ept, &net_ept_cfg);
    if (err < 0) {
        printk("[NET] ipc_service_register_endpoint failed (%d)\n", err);
        return err;
    }

    printk("[NET] Waiting for APP core IPC bind...\n");
    k_sem_take(&ipc_bound_sem, K_FOREVER);
    printk("[NET] IPC ready\n");

    /* Work & timer init */
    k_work_init(&tx_work,       tx_work_handler);
    k_work_init(&led_blink_work, led_blink_handler);
    k_work_init_delayable(&scan_work,      scan_work_handler);
    k_work_init_delayable(&stop_scan_work, stop_scan_work_handler);
    k_timer_init(&scan_scheduler_timer, scan_timer_cb, NULL);

    /* LED blink timer — 500ms blink confirms net core running */
    k_timer_init(&led_blink_timer, led_blink_timer_cb, NULL);
    k_timer_start(&led_blink_timer, K_MSEC(500), K_MSEC(500));

    /* Start scanning schedule */
    k_timer_start(&scan_scheduler_timer,
                  K_SECONDS(2),
                  K_SECONDS(SCAN_INTERVAL_SECONDS));

    printk("[NET] Scan schedule started — %dms window every %ds\n",
           SCAN_DURATION_MS, SCAN_INTERVAL_SECONDS);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}