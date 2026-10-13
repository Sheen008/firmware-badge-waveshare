// MIXED text rendering, Dingos Medium + bitmap emojis (emoji_font.h) -
// SHARED firmware / emulator. Text arrives as UTF-8 (Setup webapp):
// ASCII runs go through mdPrint, known emojis are drawn as 18x18 bitmaps
// (pre-blended on white: to be used on light backgrounds, the message
// pill), everything else (variation selectors, ZWJ, unknown emojis,
// non-transliterated accents) is silently ignored.
// Depends on: canvas, mdPrint/mdTextW, emoji_font.h.
#pragma once
#include "emoji_font.h"

static const EmjGlyph *emjFind(uint32_t cp)
{
  for (int i = 0; i < EMJ_N; i++)
    if (EMJ_GLYPHS[i].cp == cp)
      return &EMJ_GLYPHS[i];
  return nullptr;
}

// decodes the UTF-8 codepoint at position *s and advances *s
static uint32_t mxDecode(const char **s)
{
  const uint8_t *p = (const uint8_t *)*s;
  uint32_t cp = *p;
  int n = 0;
  if (cp >= 0xF0) { cp &= 7; n = 3; }
  else if (cp >= 0xE0) { cp &= 15; n = 2; }
  else if (cp >= 0xC0) { cp &= 31; n = 1; }
  (*s)++;
  while (n-- && ((uint8_t)**s & 0xC0) == 0x80)
    cp = (cp << 6) | ((uint8_t)*(*s)++ & 63);
  return cp;
}

#ifndef UI_TS
#define UI_TS 1.0f // uniform UI text scale (1.0 = 360 badge; 1.30 on 466 AMOLED)
#endif
#define MX_EMJ_ADV (EMJ_S + 2)

static int mxTextW(const char *s)
{
  char run[64];
  int w = 0, rl = 0;
  while (*s)
  {
    if ((uint8_t)*s < 0x80)
    {
      if (rl < 63)
        run[rl++] = *s;
      s++;
      continue;
    }
    run[rl] = 0;
    w += mdTextW(run);
    rl = 0;
    uint32_t cp = mxDecode(&s);
    if (emjFind(cp))
      w += (int)(MX_EMJ_ADV * UI_TS);
  }
  run[rl] = 0;
  return w + mdTextW(run);
}

// y = top of the Dingos Medium line (emojis are centered on it)
static void mxPrint(int x, int y, const char *s, uint16_t col)
{
  char run[64];
  int rl = 0;
  while (*s)
  {
    if ((uint8_t)*s < 0x80)
    {
      if (rl < 63)
        run[rl++] = *s;
      s++;
      continue;
    }
    run[rl] = 0;
    mdPrint(x, y, run, col);
    x += mdTextW(run);
    rl = 0;
    uint32_t cp = mxDecode(&s);
    const EmjGlyph *g = emjFind(cp);
    if (!g)
      continue;
    int es = (int)(EMJ_S * UI_TS);
    int ey = y + (int)(7 * UI_TS) - es / 2; // approx vertical center
    for (int gy = 0; gy < es; gy++)
      for (int gx = 0; gx < es; gx++)
        if (g->mask[gy * EMJ_S / es] & (1u << (gx * EMJ_S / es)))
          canvas->drawPixel(x + gx, ey + gy,
                            g->px[(gy * EMJ_S / es) * EMJ_S + gx]);
    x += (int)(MX_EMJ_ADV * UI_TS);
  }
  run[rl] = 0;
  mdPrint(x, y, run, col);
}
