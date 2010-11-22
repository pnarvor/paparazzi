#include <stdlib.h>
#include "fun_transport_fun.h"
#include "uart.h"
#include "downlink_transport.h"


static void put_1byte(struct pprz_transport *pprz, const uint8_t x)
{
  //pprz->tx_buf[pprz->tx_buf_idx] = x;
  //pprz->tx_buf_idx++;
  uart_transmit(pprz->uart, x);
}

static void put_uint8_t(struct pprz_transport *pprz, const uint8_t byte)
{
  pprz->tx_ck_a += byte;
  pprz->tx_ck_b += pprz->tx_ck_a;
  put_1byte(pprz, byte);
}

static void put_named_uint8_t(struct pprz_transport *pprz, char *name __attribute__((unused)), const uint8_t byte)
{
  put_uint8_t(pprz, byte);
}

static void put_bytes(void *impl, enum DownlinkDataType data_type __attribute__((unused)), uint8_t len, const void *buf)
{
  struct pprz_transport *pprz = (struct pprz_transport *) impl;
	const uint8_t *bytes = (const uint8_t *) buf;
  for (int i = 0; i < len; i++) {
    put_uint8_t(pprz, bytes[i]);
  }
}

static void header(struct pprz_transport *pprz, uint8_t payload_len)
{
  uint32_t msg_timestamp = MSG_TIMESTAMP;
  put_1byte(pprz, STX_PPRZ);
  uint8_t msg_len = payload_len + PPRZ_PROTOCOL_OVERHEAD;
  put_uint8_t(pprz, msg_len);
  pprz->tx_ck_a = pprz->tx_ck_b = msg_len;
}

static void start_message(void *impl, char *name, uint8_t msg_id, uint8_t payload_len)
{
  struct pprz_transport *pprz = (struct pprz_transport *) impl;
  header(pprz, payload_len);
  //header(pprz, 2 + payload_len); ??? +2
  put_uint8_t(pprz, AC_ID);
  put_named_uint8_t(pprz, name, msg_id);
}

static void end_message(void *impl)
{
  struct pprz_transport *pprz = (struct pprz_transport *) impl;
  put_1byte(pprz, pprz->tx_ck_a);
  put_1byte(pprz, pprz->tx_ck_b);
  // send_message(void *impl);
}

static void overrun(void *impl __attribute__((unused)))
{
  
}

static void count_bytes(void *pprz __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
	
}

static int check_free_space(void *pprz __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
	return TRUE;
}

static uint8_t size_of(void *pprz __attribute__((unused)), uint8_t len)
{
	return len + 2;
}

static void periodic(void *impl)
{
  struct pprz_transport *pprz = (struct pprz_transport *) impl;
  if (pprz->udpt_tx_buf_idx > 0) {
    network_write(pprz->network, pprz->updt_tx_buf, pprz->udpt_tx_buf_idx);
    pprz->udpt_tx_buf_idx = 0;
  }
}

struct DownlinkTransport *pprz_transport_new(struct uart_periph *uart)
{
  struct DownlinkTransport *tp = calloc(1, sizeof(struct DownlinkTransport));
  struct pprz_transport *pprz = calloc(1, sizeof(struct pprz_transport));

  pprz->dl_status = UNINIT;
  pprz->uart = uart;
  tp->impl = pprz;

  tp->StartMessage = start_message;
  tp->EndMessage = end_message;
  tp->PutBytes = put_bytes;

  tp->Overrun = overrun;
  tp->CountBytes = count_bytes;
  tp->SizeOf = size_of;
  tp->CheckFreeSpace = check_free_space;
  tp->Periodic = periodic;

  return tp;
}
