/****************************************************************************
 * apps/examples/lora_rx/ssd1306.h
 *
 * SSD1306 128x32 monocromo por I2C, usando el character driver /dev/i2cN
 * de NuttX.  Sustituto minimo de Adafruit_SSD1306 + Adafruit_GFX:
 * framebuffer en RAM (512 B) + fuente 5x7 -> 21 caracteres por linea.
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_LORA_RX_SSD1306_H
#define __APPS_EXAMPLES_LORA_RX_SSD1306_H

#include <stdint.h>

#define SSD1306_W        128
#define SSD1306_H        32
#define SSD1306_COLS     21     /* caracteres por linea con fuente 5x7 */

int  ssd1306_open(const char *i2cdev, uint8_t addr);
void ssd1306_close(void);
void ssd1306_clear(void);
void ssd1306_text(int x, int y, const char *str);
int  ssd1306_flush(void);

#endif /* __APPS_EXAMPLES_LORA_RX_SSD1306_H */
