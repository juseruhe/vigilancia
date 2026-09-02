/****************************************************************************
 * apps/examples/lora_rx/sx1278.h
 *
 * Driver SX1278 (LoRa) en espacio de usuario sobre el character driver
 * SPI de NuttX (/dev/spiN).  Equivalente funcional a la libreria
 * arduino-LoRa (sandeepmistry) para el caso de uso RX continuo + TX de ACK.
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_LORA_RX_SX1278_H
#define __APPS_EXAMPLES_LORA_RX_SX1278_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Abre el bus SPI y comprueba REG_VERSION (0x12 para SX1276/77/78).
 * Devuelve 0 si OK, valor negativo (-errno) si falla.
 */

int sx1278_open(const char *spidev);

void sx1278_close(void);

/* Configuracion del modem.
 *   freq_hz   : 433000000 para tu caso
 *   sf        : 6..12          (7)
 *   bw_hz     : 125000         (125E3)
 *   cr_denom  : 5..8  -> 4/5   (5)
 *   crc_on    : true           (enableCrc)
 *   syncword  : 0x12
 *   txpower   : 2..17 dBm por PA_BOOST (17)
 */

int sx1278_config(uint32_t freq_hz, uint8_t sf, uint32_t bw_hz,
                  uint8_t cr_denom, bool crc_on, uint8_t syncword,
                  uint8_t txpower);

/* Pone la radio en RX continuo */

int sx1278_receive_mode(void);

/* Equivalente a LoRa.parsePacket() + LoRa.read().
 *   > 0 : numero de bytes copiados en buf
 *     0 : no hay paquete
 *   < 0 : -EBADMSG si CRC invalido, u otro -errno
 * rssi en dBm, snr_q4 en cuartos de dB (multiplo de 0.25).
 */

int sx1278_parse_packet(uint8_t *buf, size_t buflen,
                        int *rssi, int *snr_q4);

/* Transmite un paquete y vuelve a RX continuo */

int sx1278_send(const uint8_t *buf, size_t len);

#endif /* __APPS_EXAMPLES_LORA_RX_SX1278_H */
