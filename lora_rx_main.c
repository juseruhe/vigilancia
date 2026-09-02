/****************************************************************************
 * apps/examples/lora_rx/lora_rx_main.c
 *
 * Puerto a NuttX del sketch Arduino "Pico LoRa RX" (SX1278 + SSD1306).
 * No hay setup()/loop(): esto es una tarea POSIX normal con su main().
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#include "sx1278.h"
#include "ssd1306.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_DEVPATH     "/dev/spi0"
#define I2C_DEVPATH     "/dev/i2c0"
#define OLED_ADDR       0x3c

#define FREQ_HZ         433000000ul
#define SPREAD_FACTOR   7
#define BANDWIDTH_HZ    125000ul
#define CODING_RATE     5           /* 4/5 */
#define SYNC_WORD       0x12        /* el mismo que la Pi 4 */
#define TX_POWER_DBM    17          /* PA_BOOST */

#define RX_TIMEOUT_MS   6000
#define MSG_MAX         64
#define POLL_US         2000

/****************************************************************************
 * Private Data  (el "estado global" del sketch)
 ****************************************************************************/

static unsigned int g_rxcount  = 0;
static unsigned int g_lostcount = 0;
static int          g_rssi     = 0;
static int          g_snr_q4   = 0;
static char         g_lastmsg[MSG_MAX] = "";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t now_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* SNR en cuartos de dB -> "-7.25" sin necesidad de printf con float */

static void fmt_snr(char *out, size_t n, int q4)
{
  int ip = q4 / 4;
  int fp = q4 % 4;

  if (fp < 0)
    {
      fp = -fp;
    }

  if (q4 < 0 && ip == 0)
    {
      snprintf(out, n, "-0.%02d", fp * 25);
    }
  else
    {
      snprintf(out, n, "%d.%02d", ip, fp * 25);
    }
}

static void update_display(void)
{
  char line[SSD1306_COLS + 1];
  char snrbuf[12];

  ssd1306_clear();

  snprintf(line, sizeof(line), "RX%u  LOST%u", g_rxcount, g_lostcount);
  ssd1306_text(0, 0, line);

  fmt_snr(snrbuf, sizeof(snrbuf), g_snr_q4);
  snprintf(line, sizeof(line), "RSSI%d SNR%s", g_rssi, snrbuf);
  ssd1306_text(0, 11, line);

  snprintf(line, sizeof(line), "%s", g_lastmsg);   /* trunca a 21 chars */
  ssd1306_text(0, 22, line);

  ssd1306_flush();
}

static void display_error(const char *l1, const char *l2)
{
  ssd1306_clear();
  ssd1306_text(0, 4, l1);
  ssd1306_text(0, 18, l2);
  ssd1306_flush();
}

static void display_splash(void)
{
  ssd1306_clear();
  ssd1306_text(16, 4, "Pico LoRa RX");
  ssd1306_text(16, 16, "433MHZ SF7 CRC");
  ssd1306_flush();
  usleep(1500000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  uint8_t  pkt[MSG_MAX];
  char     ack[MSG_MAX];
  uint32_t lastrx;
  int      ret;
  int      n;

  ret = ssd1306_open(I2C_DEVPATH, OLED_ADDR);
  if (ret < 0)
    {
      fprintf(stderr, "Error SSD1306: %d\n", ret);
      return EXIT_FAILURE;
    }

  display_splash();

  ret = sx1278_open(SPI_DEVPATH);
  if (ret < 0)
    {
      fprintf(stderr, "Error SX1278: %d\n", ret);
      display_error("SX1278 fallo", "revisar cableado");
      ssd1306_close();
      return EXIT_FAILURE;
    }

  sx1278_config(FREQ_HZ, SPREAD_FACTOR, BANDWIDTH_HZ, CODING_RATE,
                true, SYNC_WORD, TX_POWER_DBM);
  sx1278_receive_mode();

  printf("Pico RX listo.\n");

  strncpy(g_lastmsg, "Esperando Pi4...", sizeof(g_lastmsg) - 1);
  update_display();

  lastrx = now_ms();

  for (; ; )
    {
      n = sx1278_parse_packet(pkt, sizeof(pkt) - 1, &g_rssi, &g_snr_q4);

      if (n > 0)
        {
          char snrbuf[12];

          pkt[n] = '\0';
          strncpy(g_lastmsg, (char *)pkt, sizeof(g_lastmsg) - 1);
          g_lastmsg[sizeof(g_lastmsg) - 1] = '\0';

          g_rxcount++;
          lastrx = now_ms();      /* <-- esto faltaba en el sketch original */

          fmt_snr(snrbuf, sizeof(snrbuf), g_snr_q4);
          printf("RX: %s | RSSI: %d dBm | SNR: %s dB\n",
                 g_lastmsg, g_rssi, snrbuf);

          update_display();

          /* ACK */

          usleep(100000);
          snprintf(ack, sizeof(ack), "Pico->Pi4 ACK #%u", g_rxcount);

          if (sx1278_send((const uint8_t *)ack, strlen(ack)) < 0)
            {
              printf("Fallo al enviar ACK\n");
            }
          else
            {
              printf("ACK enviado: %s\n", ack);
            }

          update_display();
        }
      else if (n == -EBADMSG)
        {
          printf("Paquete con CRC invalido, descartado\n");
        }
      else if (g_rxcount > 0 && (now_ms() - lastrx) > RX_TIMEOUT_MS)
        {
          g_lostcount++;
          lastrx = now_ms();
          strncpy(g_lastmsg, "Sin senal...", sizeof(g_lastmsg) - 1);
          update_display();
        }

      usleep(POLL_US);
    }

  sx1278_close();
  ssd1306_close();
  return EXIT_SUCCESS;
}
