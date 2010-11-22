#ifndef PPRZ_TRANSPORT_FUN_H
#define PPRZ_TRANSPORT_FUN_H

#include "std.h"
#include "airframe.h"

#define STX_PPRZ  0x99
/** 4 = STX + len + ck_a + ck_b */
#define PPRZ_PROTOCOL_OVERHEAD 4

struct DownlinkTransport *pprz_transport_new(struct uart_periph *uart);

#define TX_BUF_LEN 256
#define DL_PAYLOAD_LEN 256

struct pprz_transport {
  /*
   * Downlink
   */
  char tx_buf[TX_BUF_LEN];
  uint16_t tx_buf_idx;
  uint8_t tx_ck_a, tx_ck_b;
  
  /*
   * Uplink
   */
  uint8_t dl_payload[DL_PAYLOAD_LEN];
  volatile uint8_t dl_payload_len;
  volatile bool_t dl_msg_received;
  uint8_t dl_ovrn, dl_nb_err;
  uint8_t dl_status;
  uint8_t rx_ck_a, rx_ck_b, payload_idx;

  struct uart_periph* uart;
};


/*
 * Parsing of uplink msg
 */
#define UNINIT 0
#define GOT_STX 1
#define GOT_LENGTH 2
#define GOT_PAYLOAD 3
#define GOT_CRC1 4

static inline void parse_dl( struct pprz_transport *tp, uint8_t c ) {

  switch (tp->dl_status) {
  case UNINIT:
    if (c == STX_PPRZ)
      tp->dl_status++;
    break;
  case GOT_STX:
    if (tp->dl_msg_received) {
      tp->dl_ovrn++;
      goto error;
    }
    tp->dl_payload_len = c-4; /* Counting STX, LENGTH and CRC1 and CRC2 */
    tp->rx_ck_a = tp->rx_ck_b = c;
    tp->dl_status++;
    tp->payload_idx = 0;
    break;
  case GOT_LENGTH:
    tp->dl_payload[tp->payload_idx] = c;
    tp->rx_ck_a += c; tp->rx_ck_b += tp->rx_ck_a;
    tp->payload_idx++;
    if (tp->payload_idx == tp->dl_payload_len)
      tp->dl_status++;
    break;
  case GOT_PAYLOAD:
    if (c != tp->rx_ck_a)
      goto error;
    tp->dl_status++;
    break;
  case GOT_CRC1:
    if (c != tp->rx_ck_b)
      goto error;
    tp->dl_msg_received = TRUE;
    goto restart;
  }
  return;
 error:
  tp->dl_nb_err++;
 restart:
  tp->dl_status = UNINIT;
  return;
}

#endif /* PPRZ_TRANSPORT_FUN_H */

