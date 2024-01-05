/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org) for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "boards.h"

#if defined(DISPLAY_PIN_SCK)

#include <string.h>
#include <stdlib.h>

// Overlap 4x chars by this much.
#define CHAR4_KERNING 3

// Width of a single 4x char, adjusted by kerning
#define CHAR4_KERNED_WIDTH  (6 * 4 - CHAR4_KERNING)

#define COL0(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define COL(c) COL0((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff)

enum {
  COLOR_BLACK = 0,
  COLOR_WHITE = 1,
  COLOR_RED = 2,
  COLOR_PINK = 3,
  COLOR_ORANGE = 4,
  COLOR_YELLOW = 5,
  COLOR_CYAN = 6,
  COLOR_GREEN = 7,
  COLOR_BLUE = 8,
  COLOR_AQUA = 9,
  COLOR_PURPLE = 10,
};

// 16-bit 565 color from 24-bit 888 format
const uint16_t palette[] = {
    COL(0x000000), // 0
    COL(0xffffff), // 1
    COL(0xff2121), // 2
    COL(0xff93c4), // 3
    COL(0xff8135), // 4
    COL(0xfff609), // 5
    COL(0x249ca3), // 6
    COL(0x78dc52), // 7
    COL(0x003fad), // 8
    COL(0x87f2ff), // 9
    COL(0x8e2ec4), // 10

    COL(0xa4839f), // 11
    COL(0x5c406c), // 12
    COL(0xe5cdc4), // 13
    COL(0x91463d), // 14
    COL(0x000000), // 15
};

// TODO only buffer partial screen to save SRAM
// ESP32s2 can only statically allocated DRAM up to 160KB.
// the remaining 160KB can only be allocated at runtime as heap.
static uint8_t frame_buf[DISPLAY_WIDTH * DISPLAY_HEIGHT];
//static uint8_t* frame_buf;

extern const uint8_t font8[];
extern const uint8_t fileLogo[];
extern const uint8_t pendriveLogo[];
extern const uint8_t arrowLogo[];

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// print character with font size = 1
static void printch(int x, int y, int color, const uint8_t* fnt) {
  for (int i = 0; i < 6; ++i) {
    uint8_t* p = frame_buf + (x + i) * DISPLAY_HEIGHT + y;
    uint8_t mask = 0x01;
    for (int j = 0; j < 8; ++j) {
      if (*fnt & mask) {
        *p = color;
      }
      p++;
      mask <<= 1;
    }
    fnt++;
  }
}

// print character with font size = 4
static void printch4(int x, int y, int color, const uint8_t* fnt) {
  for (int i = 0; i < 6 * 4; ++i) {
    uint8_t* p = frame_buf + (x + i) * DISPLAY_HEIGHT + y;
    uint8_t mask = 0x01;
    for (int j = 0; j < 8; ++j) {
      for (int k = 0; k < 4; ++k) {
        if (*fnt & mask) {
          *p = color;
        }
        p++;
      }
      mask <<= 1;
    }
    if ((i & 3) == 3) {
      fnt++;
    }
  }
}

// print icon
static void printicon(int x, int y, int color, const uint8_t* icon) {
  int w = *icon++;
  int h = *icon++;
  int sz = *icon++;

  uint8_t mask = 0x80;
  int runlen = 0;
  int runbit = 0;
  uint8_t lastb = 0x00;

  for (int i = 0; i < w; ++i) {
    uint8_t* p = frame_buf + (x + i) * DISPLAY_HEIGHT + y;
    for (int j = 0; j < h; ++j) {
      int c = 0;
      if (mask != 0x80) {
        if (lastb & mask) {
          c = 1;
        }
        mask <<= 1;
      } else if (runlen) {
        if (runbit) {
          c = 1;
        }
        runlen--;
      } else {
        if (sz-- <= 0) {
          //TU_LOG1("Screen Panic code = 10");
        }
        lastb = *icon++;
        if (lastb & 0x80) {
          runlen = lastb & 63;
          runbit = lastb & 0x40;
        } else {
          mask = 0x01;
        }
        --j;
        continue; // restart
      }
      if (c) {
        *p = color;
      }
      p++;
    }
  }
}

// print text with font size = 1
static void print(int x, int y, int col, const char* text) {
  int x0 = x;
  while (*text) {
    char c = *text++;
    if (c == '\r') continue;
    if (c == '\n') {
      x = x0;
      y += 10;
      continue;
    }
    /*
    if (x + 8 > DISPLAY_WIDTH) {
        x = x0;
        y += 10;
    }
    */
    if (c < ' ') c = '?';
    if (c >= 0x7f) c = '?';
    c -= ' ';
    printch(x, y, col, &font8[c * 6]);
    x += 6;
  }
}

// Print text with font size = 4
static void print4(int x, int y, int color, const char* text) {
  while (*text) {
    char c = *text++;
    c -= ' ';
    printch4(x, y, color, &font8[c * 6]);
    x += CHAR4_KERNED_WIDTH;
    if (x + CHAR4_KERNED_WIDTH > DISPLAY_WIDTH) {
      // Next char won't fit.
      return;
    }
  }
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static void draw_screen(uint8_t const* fb) {
  uint8_t const* p = fb;
  for (int y = 0; y < DISPLAY_WIDTH; ++y) {
    uint8_t cc[DISPLAY_HEIGHT * 2];
    uint32_t dst = 0;
    for (int x = 0; x < DISPLAY_HEIGHT; ++x) {
      uint16_t color = palette[*p++ & 0xf];
      cc[dst++] = color >> 8;
      cc[dst++] = color & 0xff;
    }

    board_display_draw_line(y, cc, sizeof(cc));
  }
}

// draw color bar
static void drawBar(int y, int h, int color) {
  for (int x = 0; x < DISPLAY_WIDTH; ++x) {
    memset(frame_buf + x * DISPLAY_HEIGHT + y, color, h);
  }
}

// draw drag & drop screen
void screen_draw_drag(void) {
  drawBar(0, 52, COLOR_GREEN);
  drawBar(52, 55, COLOR_BLUE);
  drawBar(107, 14, COLOR_ORANGE);

  // Center UF2_PRODUCT_NAME and UF2_VERSION_BASE.
  int name_x = (DISPLAY_WIDTH - CHAR4_KERNED_WIDTH * (int) strlen(DISPLAY_TITLE)) / 2;
  print4(name_x >= 0 ? name_x : 0, 5, COLOR_WHITE, DISPLAY_TITLE);

  int version_x = (DISPLAY_WIDTH - 6 * (int) strlen(UF2_VERSION_BASE)) / 2;
  print(version_x >= 0 ? version_x : 0, 40, COLOR_PURPLE, UF2_VERSION_BASE);

  // TODO the reset should be center as well
  print(23, 110, 1, "circuitpython.org");

#define DRAG 70
#define DRAGX 10
  printicon(DRAGX + 20, DRAG + 5, COLOR_WHITE, fileLogo);
  printicon(DRAGX + 66, DRAG, COLOR_WHITE, arrowLogo);
  printicon(DRAGX + 108, DRAG, COLOR_WHITE, pendriveLogo);
  print(10, DRAG - 12, COLOR_WHITE, "firmware.uf2");
  print(90, DRAG - 12, COLOR_WHITE, UF2_VOLUME_LABEL);

  draw_screen(frame_buf);
}

#endif
