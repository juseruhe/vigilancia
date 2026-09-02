/****************************************************************************
 * apps/examples/lora_rx/sx1278.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <time.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "sx1278.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SX1278_SPI_FREQ   1000000     /* 1 MHz, seguro para el Ra-02 */
#define SX1278_SPI_DEVID  SPIDEV_USER(0)
#define SX1278_XTAL       32000000ull

/* Registros (mapa LoRa) */

#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

/* Modos */

#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

/* Flags de IRQ */

#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define PA_BOOST                 0x80
#define RF_MID_BAND_THRESHOLD    525000000ul

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int      g_spifd = -1;
static uint32_t g_freq  = 433000000ul;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sx1278_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
  struct spi_trans_s    trans;
  struct spi_sequence_s seq;

  memset(&trans, 0, sizeof(trans));
  memset(&seq,   0, sizeof(seq));

  trans.deselect = true;          /* CS alto al terminar la transaccion */
  trans.delay    = 0;
  trans.nwords   = len;           /* nbits = 8 -> nwords == nbytes */
  trans.txbuffer = tx;
  trans.rxbuffer = rx;

  seq.dev       = SX1278_SPI_DEVID;
  seq.mode      = SPIDEV_MODE0;
  seq.nbits     = 8;
  seq.ntrans    = 1;
  seq.frequency = SX1278_SPI_FREQ;
  seq.trans     = &trans;

  if (ioctl(g_spifd, SPIIOC_TRANSFER, (unsigned long)((uintptr_t)&seq)) < 0)
    {
      return -errno;
    }

  return 0;
}

static int sx1278_readreg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];
  int ret;

  tx[0] = reg & 0x7f;
  tx[1] = 0x00;

  ret = sx1278_xfer(tx, rx, 2);
  if (ret < 0)
    {
      return ret;
    }

  return rx[1];
}

static int sx1278_writereg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80;
  tx[1] = val;

  return sx1278_xfer(tx, rx, 2);
}

static int sx1278_setmode(uint8_t mode)
{
  return sx1278_writereg(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

static uint32_t sx1278_now_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/* Indice del ancho de banda tal y como lo espera MODEM_CONFIG_1 */

static uint8_t sx1278_bw_index(uint32_t bw_hz)
{
  static const uint32_t bws[] =
  {
    7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000
  };

  int i;

  for (i = 0; i < (int)(sizeof(bws) / sizeof(bws[0])); i++)
    {
      if (bw_hz <= bws[i])
        {
          return (uint8_t)i;
        }
    }

  return 9;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sx1278_open(const char *spidev)
{
  int version;

  g_spifd = open(spidev, O_RDWR);
  if (g_spifd < 0)
    {
      return -errno;
    }

  version = sx1278_readreg(REG_VERSION);
  if (version != 0x12)
    {
      fprintf(stderr, "sx1278: REG_VERSION = 0x%02x (se esperaba 0x12)\n",
              version);
      close(g_spifd);
      g_spifd = -1;
      return -ENODEV;
    }

  return 0;
}

void sx1278_close(void)
{
  if (g_spifd >= 0)
    {
      sx1278_setmode(MODE_SLEEP);
      close(g_spifd);
      g_spifd = -1;
    }
}

int sx1278_config(uint32_t freq_hz, uint8_t sf, uint32_t bw_hz,
                  uint8_t cr_denom, bool crc_on, uint8_t syncword,
                  uint8_t txpower)
{
  uint64_t frf;
  uint8_t  mc1;
  uint8_t  mc2;
  uint8_t  mc3;

  g_freq = freq_hz;

  /* Sleep -> permite entrar en modo LoRa y limpia la FIFO */

  sx1278_setmode(MODE_SLEEP);
  usleep(10000);

  /* Frecuencia portadora */

  frf = ((uint64_t)freq_hz << 19) / SX1278_XTAL;
  sx1278_writereg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  sx1278_writereg(REG_FRF_MID, (uint8_t)(frf >> 8));
  sx1278_writereg(REG_FRF_LSB, (uint8_t)(frf >> 0));

  /* Bases de la FIFO: todo el buffer de 256 B para RX y para TX */

  sx1278_writereg(REG_FIFO_TX_BASE_ADDR, 0x00);
  sx1278_writereg(REG_FIFO_RX_BASE_ADDR, 0x00);

  /* LNA boost */

  sx1278_writereg(REG_LNA, sx1278_readreg(REG_LNA) | 0x03);

  /* AGC automatico + optimizacion de baja tasa si procede */

  mc3 = 0x04;
  if (sf > 10 && bw_hz <= 125000)
    {
      mc3 |= 0x08;
    }

  sx1278_writereg(REG_MODEM_CONFIG_3, mc3);

  /* MODEM_CONFIG_1: BW | CR | cabecera explicita */

  if (cr_denom < 5)
    {
      cr_denom = 5;
    }
  else if (cr_denom > 8)
    {
      cr_denom = 8;
    }

  mc1 = (uint8_t)((sx1278_bw_index(bw_hz) << 4) | ((cr_denom - 4) << 1));
  sx1278_writereg(REG_MODEM_CONFIG_1, mc1);

  /* MODEM_CONFIG_2: SF | CRC | timeout MSB */

  if (sf < 6)
    {
      sf = 6;
    }
  else if (sf > 12)
    {
      sf = 12;
    }

  if (sf == 6)
    {
      sx1278_writereg(REG_DETECTION_OPTIMIZE, 0xc5);
      sx1278_writereg(REG_DETECTION_THRESHOLD, 0x0c);
    }
  else
    {
      sx1278_writereg(REG_DETECTION_OPTIMIZE, 0xc3);
      sx1278_writereg(REG_DETECTION_THRESHOLD, 0x0a);
    }

  mc2 = (uint8_t)(sf << 4);
  if (crc_on)
    {
      mc2 |= 0x04;
    }

  sx1278_writereg(REG_MODEM_CONFIG_2, mc2);

  /* Preambulo de 8 simbolos (igual que la libreria de Arduino) */

  sx1278_writereg(REG_PREAMBLE_MSB, 0x00);
  sx1278_writereg(REG_PREAMBLE_LSB, 0x08);

  /* Sync word: debe coincidir con el de la Pi 4 */

  sx1278_writereg(REG_SYNC_WORD, syncword);

  /* Potencia de salida por PA_BOOST (obligatorio en el Ra-02) */

  if (txpower < 2)
    {
      txpower = 2;
    }
  else if (txpower > 17)
    {
      txpower = 17;
    }

  sx1278_writereg(REG_PA_DAC, 0x84);
  sx1278_writereg(REG_PA_CONFIG, PA_BOOST | (uint8_t)(txpower - 2));
  sx1278_writereg(REG_OCP, 0x20 | 0x0b);   /* OCP ~100 mA */

  sx1278_setmode(MODE_STDBY);
  return 0;
}

int sx1278_receive_mode(void)
{
  sx1278_writereg(REG_FIFO_ADDR_PTR, 0x00);
  return sx1278_setmode(MODE_RX_CONTINUOUS);
}

int sx1278_parse_packet(uint8_t *buf, size_t buflen, int *rssi, int *snr_q4)
{
  uint8_t tx[257];
  uint8_t rx[257];
  int irq;
  int len;
  int raw;
  int ret;

  irq = sx1278_readreg(REG_IRQ_FLAGS);
  if (irq < 0)
    {
      return irq;
    }

  if ((irq & IRQ_RX_DONE_MASK) == 0)
    {
      return 0;
    }

  /* Los flags de IRQ se limpian escribiendo un 1 */

  sx1278_writereg(REG_IRQ_FLAGS, (uint8_t)irq);

  if ((irq & IRQ_PAYLOAD_CRC_ERROR_MASK) != 0)
    {
      return -EBADMSG;
    }

  len = sx1278_readreg(REG_RX_NB_BYTES);
  if (len <= 0)
    {
      return 0;
    }

  if ((size_t)len > buflen)
    {
      len = (int)buflen;
    }

  if (rssi != NULL)
    {
      raw = sx1278_readreg(REG_PKT_RSSI_VALUE);
      *rssi = raw - (g_freq < RF_MID_BAND_THRESHOLD ? 164 : 157);
    }

  if (snr_q4 != NULL)
    {
      raw = sx1278_readreg(REG_PKT_SNR_VALUE);
      *snr_q4 = (int)(int8_t)raw;    /* unidades de 0.25 dB */
    }

  /* Situar el puntero de FIFO al inicio del paquete y leer en rafaga */

  sx1278_writereg(REG_FIFO_ADDR_PTR,
                  (uint8_t)sx1278_readreg(REG_FIFO_RX_CURRENT_ADDR));

  memset(tx, 0, (size_t)len + 1);
  tx[0] = REG_FIFO & 0x7f;

  ret = sx1278_xfer(tx, rx, (size_t)len + 1);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(buf, &rx[1], (size_t)len);

  sx1278_writereg(REG_FIFO_ADDR_PTR, 0x00);
  return len;
}

int sx1278_send(const uint8_t *buf, size_t len)
{
  uint8_t  tx[257];
  uint8_t  rx[257];
  uint32_t start;
  int      irq;
  int      ret;

  if (len == 0 || len > 255)
    {
      return -EINVAL;
    }

  sx1278_setmode(MODE_STDBY);

  sx1278_writereg(REG_IRQ_FLAGS, 0xff);
  sx1278_writereg(REG_FIFO_ADDR_PTR, 0x00);
  sx1278_writereg(REG_PAYLOAD_LENGTH, 0x00);

  tx[0] = REG_FIFO | 0x80;
  memcpy(&tx[1], buf, len);

  ret = sx1278_xfer(tx, rx, len + 1);
  if (ret < 0)
    {
      return ret;
    }

  sx1278_writereg(REG_PAYLOAD_LENGTH, (uint8_t)len);
  sx1278_setmode(MODE_TX);

  /* Espera bloqueante a TxDone (sin DIO0), con timeout de 3 s */

  start = sx1278_now_ms();
  for (; ; )
    {
      irq = sx1278_readreg(REG_IRQ_FLAGS);
      if (irq >= 0 && (irq & IRQ_TX_DONE_MASK) != 0)
        {
          break;
        }

      if (sx1278_now_ms() - start > 3000)
        {
          sx1278_receive_mode();
          return -ETIMEDOUT;
        }

      usleep(1000);
    }

  sx1278_writereg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  /* Volver a escuchar */

  sx1278_receive_mode();
  return (int)len;
}
