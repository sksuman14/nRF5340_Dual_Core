/* ========================================================
 * nRF5340 Data Logger — APPLICATION CORE (cpuapp)
 * NCS v2.5.1
 *
 * Responsibilities:
 *   - Release network core from reset
 *   - Store dummy data (1..10 repeating) into internal RAM ring buffer
 *   - Receive trigger/reset commands from Network core via IPC
 *   - Send stored packets to Network core via IPC for BLE TX
 * ======================================================== */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/ipc/ipc_service.h>
#include <nrf.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "ipc_msg.h"

/* ============================
 * Packet Layout (256 bytes total)
 *  [0]       header byte  0x89
 *  [1]       node id
 *  [2..241]  payload (240 bytes of dummy data)
 *  [242..243] original packet id (little-endian)
 *  [244..245] current packet id  (little-endian, filled at TX time)
 *  [246]     footer 0xFE
 * ============================ */
#define NODE_ID  5

/* ============================
 * RAM Ring Buffer
 * 10 x 256 = 2560 bytes of RAM
 * Increase MAX_PACKETS if more RAM is available
 * ============================ */
#define MAX_PACKETS    10
#define TX_PACKET_SIZE 256

static uint8_t  ram_buffer[MAX_PACKETS][TX_PACKET_SIZE];
static int      write_idx    = 0;
static int      read_idx     = 0;
static int      stored_count = 0;
static uint16_t packet_id    = 1;

/* ============================
 * Dummy data
 * ============================ */
#define DUMMY_DATA_LEN 10
static const uint8_t dummy_data[DUMMY_DATA_LEN] = {1,2,3,4,5,6,7,8,9,10};
static uint8_t dummy_pos = 0;
static uint8_t tx_index  = 0;
static uint8_t tx_buffer[TX_PACKET_SIZE];

/* ============================
 * State flags
 * ============================ */
static volatile bool sending_active = false;

/* ============================
 * IPC
 * ============================ */
static struct ipc_ept app_ept;
static K_SEM_DEFINE(ipc_bound_sem, 0, 1);

/* ============================
 * Work & Timer
 * ============================ */
static struct k_work  store_work;
static struct k_work  send_work;
static struct k_work  reset_work;
static struct k_timer store_timer;

/* ============================
 * LEDs
 * ============================ */
static const struct gpio_dt_spec led0 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/* ============================
 * Forward declarations
 * ============================ */
static void store_dummy_sample(struct k_work *w);
static void send_data_to_net(struct k_work *w);
static void do_reset(struct k_work *w);
static void store_timer_cb(struct k_timer *t);
static void ipc_recv_cb(const void *data, size_t len, void *priv);
static void ipc_bound_cb(void *priv);

/* ============================
 * IPC callbacks
 * ============================ */
static const struct ipc_ept_cfg app_ept_cfg = {
    .name = "app_ept",
    .prio = 0,
    .cb   = {
        .bound    = ipc_bound_cb,
        .received = ipc_recv_cb,
    },
};

static void ipc_bound_cb(void *priv)
{
    ARG_UNUSED(priv);
    printk("[APP] IPC endpoint bound\n");
    k_sem_give(&ipc_bound_sem);
}

static void ipc_recv_cb(const void *data, size_t len, void *priv)
{
    ARG_UNUSED(priv);
    if (len < 1) return;

    const uint8_t *msg = (const uint8_t *)data;

    switch (msg[0]) {
    case IPC_MSG_TYPE_TRIGGER:
        printk("[APP] Trigger received — starting data replay\n");
        if (!sending_active) {
            sending_active = true;
            k_work_submit(&send_work);
        }
        break;

    case IPC_MSG_TYPE_RESET:
        printk("[APP] Reset command received\n");
        k_work_submit(&reset_work);
        break;

    case IPC_MSG_TYPE_ACK:
        printk("[APP] ACK from NET — packet delivered\n");
        break;

    default:
        printk("[APP] Unknown IPC msg: 0x%02X\n", msg[0]);
        break;
    }
}

/* ============================
 * Work: store dummy data sample into RAM buffer
 * Called every 750 ms by timer
 * ============================ */
static void store_dummy_sample(struct k_work *w)
{
    ARG_UNUSED(w);

    if (sending_active) {
        return;
    }

    /* First sample in a new packet — init header */
    if (tx_index == 0) {
        memset(tx_buffer, 0x00, TX_PACKET_SIZE);
        tx_buffer[0] = 0x89;
        tx_buffer[1] = NODE_ID;
    }

    /* Append 3 dummy bytes (mimics X/Y/Z axis) */
    tx_buffer[TX_HEADER_SIZE + tx_index++] =
        dummy_data[dummy_pos % DUMMY_DATA_LEN]; dummy_pos++;
    tx_buffer[TX_HEADER_SIZE + tx_index++] =
        dummy_data[dummy_pos % DUMMY_DATA_LEN]; dummy_pos++;
    tx_buffer[TX_HEADER_SIZE + tx_index++] =
        dummy_data[dummy_pos % DUMMY_DATA_LEN]; dummy_pos++;

    printk("[APP] Buffered sample tx_index=%d dummy_pos=%d\n",
           tx_index, dummy_pos);

    /* Packet full at TX_PAYLOAD_SIZE (240) bytes */
    if (tx_index >= TX_PAYLOAD_SIZE) {
        /* Original packet id */
        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 0] =
             packet_id & 0xFF;
        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 1] =
            (packet_id >> 8) & 0xFF;
        /* Current packet id placeholder */
        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 2] = 0x00;
        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 3] = 0x00;
        /* End marker */
        tx_buffer[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 4] = 0xFE;

        if (stored_count < MAX_PACKETS) {
            memcpy(ram_buffer[write_idx], tx_buffer, TX_PACKET_SIZE);
            write_idx = (write_idx + 1) % MAX_PACKETS;
            stored_count++;
            printk("[APP] Stored packet pid=%d (%d/%d slots)\n",
                   packet_id, stored_count, MAX_PACKETS);
            gpio_pin_toggle_dt(&led0);
        } else {
            printk("[APP] RAM buffer full — dropping pid=%d\n",
                   packet_id);
        }

        packet_id++;
        tx_index = 0;
    }
}

/* ============================
 * Work: send stored packets to NET core via IPC
 * ============================ */
static void send_data_to_net(struct k_work *w)
{
    ARG_UNUSED(w);

    if (stored_count == 0) {
        printk("[APP] No packets to send\n");
        sending_active = false;
        return;
    }

    uint16_t current_id = packet_id - 1;
    printk("[APP] Replaying %d packet(s) to NET core\n", stored_count);

    while (stored_count > 0) {
        uint8_t *pkt = ram_buffer[read_idx];

        /* Stamp current packet id */
        pkt[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 2] =
             current_id & 0xFF;
        pkt[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 3] =
            (current_id >> 8) & 0xFF;

        uint16_t orig_id =
            ((uint16_t)pkt[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 1] << 8) |
                       pkt[TX_HEADER_SIZE + TX_PAYLOAD_SIZE + 0];

        printk("[APP] Sending pkt orig_id=%d total_stored=%d\n",
               orig_id, current_id);

        /* IPC message: [type | TX_VALID_SIZE bytes] */
        uint8_t ipc_msg[1 + TX_VALID_SIZE];
        ipc_msg[0] = IPC_MSG_TYPE_DATA;
        memcpy(&ipc_msg[1], pkt, TX_VALID_SIZE);

        int err = ipc_service_send(&app_ept, ipc_msg, sizeof(ipc_msg));
        if (err < 0) {
            printk("[APP] IPC send failed (err %d)\n", err);
            break;
        }

        read_idx = (read_idx + 1) % MAX_PACKETS;
        stored_count--;
        k_msleep(50);
    }

    printk("[APP] Replay done\n");
    sending_active = false;
}

/* ============================
 * Work: reset RAM buffer
 * ============================ */
static void do_reset(struct k_work *w)
{
    ARG_UNUSED(w);

    gpio_pin_set_dt(&led1, 1);

    write_idx      = 0;
    read_idx       = 0;
    stored_count   = 0;
    packet_id      = 1;
    tx_index       = 0;
    dummy_pos      = 0;
    sending_active = false;

    memset(ram_buffer, 0x00, sizeof(ram_buffer));
    memset(tx_buffer,  0x00, sizeof(tx_buffer));

    printk("[APP] Reset complete — RAM buffer cleared\n");
    gpio_pin_set_dt(&led1, 0);
}

/* ============================
 * Timer callback
 * ============================ */
static void store_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&store_work);
}

/* ============================
 * main()
 * ============================ */
int main(void)
{
    printk("=== nRF5340 APP CORE — RAM Data Logger ===\n");

    /* LEDs */
    if (device_is_ready(led0.port)) {
        gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    }
    if (device_is_ready(led1.port)) {
        gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    }

    /* ------------------------------------------------
     * STEP 2: IPC setup
     * ------------------------------------------------ */
    const struct device *ipc_dev = DEVICE_DT_GET(DT_NODELABEL(ipc0));
    if (!device_is_ready(ipc_dev)) {
        printk("[APP] FATAL: IPC device not ready\n");
        return -1;
    }

    int err = ipc_service_open_instance(ipc_dev);
    if (err < 0 && err != -EALREADY) {
        printk("[APP] ipc_service_open_instance failed (%d)\n", err);
        return err;
    }

    err = ipc_service_register_endpoint(ipc_dev, &app_ept, &app_ept_cfg);
    if (err < 0) {
        printk("[APP] ipc_service_register_endpoint failed (%d)\n", err);
        return err;
    }

    /* Wait until NET core binds the IPC endpoint */
    printk("[APP] Waiting for NET core IPC bind...\n");
    k_sem_take(&ipc_bound_sem, K_FOREVER);
    printk("[APP] IPC ready\n");

    /* ------------------------------------------------
     * STEP 3: Work items & timer
     * ------------------------------------------------ */
    k_work_init(&store_work, store_dummy_sample);
    k_work_init(&send_work,  send_data_to_net);
    k_work_init(&reset_work, do_reset);

    k_timer_init(&store_timer, store_timer_cb, NULL);
    k_timer_start(&store_timer, K_MSEC(500), K_MSEC(750));

    printk("[APP] Logging dummy data every 750ms (%d RAM slots)\n",
           MAX_PACKETS);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}