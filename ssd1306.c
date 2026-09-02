/****************************************************************************
 * apps/examples/lora_rx/ssd1306.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <nuttx/i2c/i2c_master.h>

#include "ssd1306.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SSD1306_I2C_FREQ   400000
#define FB_SIZE            (SSD1306_W * SSD1306_H / 8)   /* 512 bytes */
#define CHUNK              32

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int     g_i2cfd = -1;
static uint8_t g_addr  = 0x3c;
static uint8_t g_fb[FB_SIZE];

/* Fuente 5x7, orientada por columnas, bit0 = fila superior.
 * El indice del glifo es la posicion del caracter en g_chars.
 * Las minusculas se convierten a mayusculas; lo no soportado -> espacio.
 */

static const char g_chars[] =
  " !#%()*+,-./0123456789:<=>?ABCDEFGHIJKLMNOPQRSTUVWXYZ_";

static const uint8_t g_font[][5] =
{
  {0x00, 0x00, 0x00, 0x00, 0x00},   /* ' ' */
  {0x00, 0x00, 0x5f, 0x00, 0x00},   /* '!' */
  {0x14, 0x7f, 0x14, 0x7f, 0x14},   /* '#' */
  {0x23, 0x13, 0x08, 0x64, 0x62},   /* '%' */
  {0x00, 0x1c, 0x22, 0x41, 0x00},   /* '(' */
  {0x00, 0x41, 0x22, 0x1c, 0x00},   /* ')' */
  {0x14, 0x08, 0x3e, 0x08, 0x14},   /* '*' */
  {0x08, 0x08, 0x3e, 0x08, 0x08},   /* '+' */
  {0x00, 0x50, 0x30, 0x00, 0x00},   /* ',' */
  {0x08, 0x08, 0x08, 0x08, 0x08},   /* '-' */
  {0x00, 0x60, 0x60, 0x00, 0x00},   /* '.' */
  {0x20, 0x10, 0x08, 0x04, 0x02},   /* '/' */
  {0x3e, 0x51, 0x49, 0x45, 0x3e},   /* '0' */
  {0x00, 0x42, 0x7f, 0x40, 0x00},   /* '1' */
  {0x42, 0x61, 0x51, 0x49, 0x46},   /* '2' */
  {0x21, 0x41, 0x45, 0x4b, 0x31},   /* '3' */
  {0x18, 0x14, 0x12, 0x7f, 0x10},   /* '4' */
  {0x27, 0x45, 0x45, 0x45, 0x39},   /* '5' */
  {0x3c, 0x4a, 0x49, 0x49, 0x30},   /* '6' */
  {0x01, 0x71, 0x09, 0x05, 0x03},   /* '7' */
  {0x36, 0x49, 0x49, 0x49, 0x36},   /* '8' */
  {0x06, 0x49, 0x49, 0x29, 0x1e},   /* '9' */
  {0x00, 0x36, 0x36, 0x00, 0x00},   /* ':' */
  {0x08, 0x14, 0x22, 0x41, 0x00},   /* '<' */
  {0x14, 0x14, 0x14, 0x14, 0x14},   /* '=' */
  {0x41, 0x22, 0x14, 0x08, 0x00},   /* '>' */
  {0x02, 0x01, 0x51, 0x09, 0x06},   /* '?' */
  {0x7e, 0x11, 0x11, 0x11, 0x7e},   /* 'A' */
  {0x7f, 0x49, 0x49, 0x49, 0x36},   /* 'B' */
  {0x3e, 0x41, 0x41, 0x41, 0x22},   /* 'C' */
  {0x7f, 0x41, 0x41, 0x22, 0x1c},   /* 'D' */
  {0x7f, 0x49, 0x49, 0x49, 0x41},   /* 'E' */
  {0x7f, 0x09, 0x09, 0x01, 0x01},   /* 'F' */
  {0x3e, 0x41, 0x41, 0x51, 0x32},   /* 'G' */
  {0x7f, 0x08, 0x08, 0x08, 0x7f},   /* 'H' */
  {0x00, 0x41, 0x7f, 0x41, 0x00},   /* 'I' */
  {0x20, 0x40, 0x41, 0x3f, 0x01},   /* 'J' */
  {0x7f, 0x08, 0x14, 0x22, 0x41},   /* 'K' */
  {0x7f, 0x40, 0x40, 0x40, 0x40},   /* 'L' */
  {0x7f, 0x02, 0x04, 0x02, 0x7f},   /* 'M' */
  {0x7f, 0x04, 0x08, 0x10, 0x7f},   /* 'N' */
  {0x3e, 0x41, 0x41, 0x41, 0x3e},   /* 'O' */
  {0x7f, 0x09, 0x09, 0x09, 0x06},   /* 'P' */
  {0x3e, 0x41, 0x51, 0x21, 0x5e},   /* 'Q' */
  {0x7f, 0x09, 0x19, 0x29, 0x46},   /* 'R' */
  {0x46, 0x49, 0x49, 0x49, 0x31},   /* 'S' */
  {0x01, 0x01, 0x7f, 0x01, 0x01},   /* 'T' */
  {0x3f, 0x40, 0x40, 0x40, 0x3f},   /* 'U' */
  {0x1f, 0x20, 0x40, 0x20, 0x1f},   /* 'V' */
  {0x7f, 0x20, 0x18, 0x20, 0x7f},   /* 'W' */
  {0x63, 0x14, 0x08, 0x14, 0x63},   /* 'X' */
  {0x03, 0x04, 0x78, 0x04, 0x03},   /* 'Y' */
  {0x61, 0x51, 0x49, 0x45, 0x43},   /* 'Z' */
  {0x40, 0x40, 0x40, 0x40, 0x40},   /* '_' */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ssd1306_write(const uint8_t *data, size_t len)
{
  struct i2c_msg_s      msg;
  struct i2c_transfer_s xfer;

  msg.frequency = SSD1306_I2C_FREQ;
  msg.addr      = g_addr;
  msg.flags     = 0;
  msg.buffer    = (uint8_t *)data;
  msg.length    = (ssize_t)len;

  xfer.msgv = &msg;
  xfer.msgc = 1;

  if (ioctl(g_i2cfd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer)) < 0)
    {
      return -errno;
    }

  return 0;
}

static int ssd1306_cmd(uint8_t cmd)
{
  uint8_t buf[2];

  buf[0] = 0x00;        /* Co = 0, D/C = 0 -> comando */
  buf[1] = cmd;

  return ssd1306_write(buf, 2);
}

static void ssd1306_pixel(int x, int y)
{
  if (x < 0 || x >= SSD1306_W || y < 0 || y >= SSD1306_H)
    {
      return;
    }

  g_fb[(y / 8) * SSD1306_W + x] |= (uint8_t)(1 << (y % 8));
}

static const uint8_t *ssd1306_glyph(char c)
{
  const char *p;

  c = (char)toupper((unsigned char)c);
  p = strchr(g_chars, c);

  if (p == NULL || c == '\0')
    {
      return g_font[0];
    }

  return g_font[p - g_chars];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ssd1306_open(const char *i2cdev, uint8_t addr)
{
  static const uint8_t initseq[] =
  {
    0xae,               /* display off */
    0xd5, 0x80,         /* clock div */
    0xa8, 0x1f,         /* multiplex = 31 (128x32) */
    0xd3, 0x00,         /* display offset */
    0x40,               /* start line 0 */
    0x8d, 0x14,         /* charge pump ON */
    0x20, 0x00,         /* addressing mode horizontal */
    0xa1,               /* segment remap */
    0xc8,               /* COM scan dec */
    0xda, 0x02,         /* COM pins config (128x32) */
    0x81, 0x8f,         /* contraste */
    0xd9, 0xf1,         /* precharge */
    0xdb, 0x40,         /* VCOM detect */
    0xa4,               /* resume RAM content */
    0xa6,               /* modo normal */
    0x2e,               /* scroll off */
    0xaf                /* display on */
  };

  int i;
  int ret;

  g_addr = addr;

  g_i2cfd = open(i2cdev, O_RDWR);
  if (g_i2cfd < 0)
    {
      return -errno;
    }

  for (i = 0; i < (int)sizeof(initseq); i++)
    {
      ret = ssd1306_cmd(initseq[i]);
      if (ret < 0)
        {
          close(g_i2cfd);
          g_i2cfd = -1;
          return ret;
        }
    }

  ssd1306_clear();
  return ssd1306_flush();
}

void ssd1306_close(void)
{
  if (g_i2cfd >= 0)
    {
      ssd1306_cmd(0xae);
      close(g_i2cfd);
      g_i2cfd = -1;
    }
}

void ssd1306_clear(void)
{
  memset(g_fb, 0, sizeof(g_fb));
}

void ssd1306_text(int x, int y, const char *str)
{
  int col;
  int row;
  const uint8_t *glyph;

  while (*str != '\0' && x < SSD1306_W)
    {
      glyph = ssd1306_glyph(*str);

      for (col = 0; col < 5; col++)
        {
          for (row = 0; row < 7; row++)
            {
              if ((glyph[col] & (1 << row)) != 0)
                {
                  ssd1306_pixel(x + col, y + row);
                }
            }
        }

      x += 6;                     /* 5 px de glifo + 1 px de separacion */
      str++;
    }
}

int ssd1306_flush(void)
{
  uint8_t buf[CHUNK + 1];
  int off;
  int n;
  int ret;

  ret = ssd1306_cmd(0x21);        /* column address */
  if (ret < 0)
    {
      return ret;
    }

  ssd1306_cmd(0x00);
  ssd1306_cmd(SSD1306_W - 1);

  ssd1306_cmd(0x22);              /* page address */
  ssd1306_cmd(0x00);
  ssd1306_cmd((SSD1306_H / 8) - 1);

  for (off = 0; off < FB_SIZE; off += CHUNK)
    {
      n = (FB_SIZE - off < CHUNK) ? (FB_SIZE - off) : CHUNK;

      buf[0] = 0x40;              /* Co = 0, D/C = 1 -> datos */
      memcpy(&buf[1], &g_fb[off], (size_t)n);

      ret = ssd1306_write(buf, (size_t)n + 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  return 0;
}
