/* ========================================================
 * ipc_msg.h — shared IPC message definitions
 * Keep this file in common/ folder
 * Include in BOTH app_core and net_core
 * ======================================================== */
#ifndef IPC_MSG_H
#define IPC_MSG_H

#include <stdint.h>

/* Message type — first byte of every IPC message */
#define IPC_MSG_TYPE_DATA    0x01   /* APP→NET: packet to advertise  */
#define IPC_MSG_TYPE_TRIGGER 0x02   /* NET→APP: start sending data   */
#define IPC_MSG_TYPE_RESET   0x03   /* NET→APP: erase stored data    */
#define IPC_MSG_TYPE_ACK     0x04   /* NET→APP: packet advertised OK */

/* Packet geometry */
#define TX_HEADER_SIZE   2
#define TX_PAYLOAD_SIZE  240
#define TX_FOOTER_SIZE   5
#define TX_VALID_SIZE    (TX_HEADER_SIZE + TX_PAYLOAD_SIZE + TX_FOOTER_SIZE)

/* Max IPC message = 1 type byte + packet data */
#define IPC_MAX_MSG_SIZE (1 + TX_VALID_SIZE)

#endif /* IPC_MSG_H */