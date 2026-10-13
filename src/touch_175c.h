// Minimal CST9217 capacitive touch driver for the Waveshare 1.75C board.
// Self-contained port of the essential parts of lewis-he's TouchDrvCST92xx
// (SensorLib, MIT): 16-bit-register I2C protocol at 0x5A.
//   init:  reset (TP_RST), enter command mode {0xD1,0x01}, read resolution
//   poll:  write {0xD0,0x00} -> read 15 bytes -> write ack {0xD0,0x00,0xAB}
//          point 0 = bytes[0..3]: evt = b0&0x0F (0x06 = pressed),
//          x = (b1<<4)|(b3>>4), y = (b2<<4)|(b3&0x0F)
// Assumes defined before inclusion: TP_RST, IIC_SDA, IIC_SCL, Serial0.
#pragma once
#include <Wire.h>

#define TP_ADDR 0x5A
#define TP_REG_READ 0xD000
#define TP_ACK 0xAB
#define TP_EVT_PRESS 0x06

static bool tpOk = false;
static int tpResX = W, tpResY = H;

static bool tpReadRegs(uint16_t reg, uint8_t *buf, int n)
{
  Wire.beginTransmission(TP_ADDR);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) // repeated start
    return false;
  if (Wire.requestFrom((uint8_t)TP_ADDR, (uint8_t)n) != n)
    return false;
  for (int i = 0; i < n; i++)
    buf[i] = Wire.read();
  return true;
}

static bool tpWriteReg(uint8_t reg, uint8_t v)
{
  // NOTE: single-byte register + value, 2 bytes total (matches SensorLib's
  // writeRegister(uint8_t, uint8_t) -- the command-mode entry is {0xD1,0x01})
  Wire.beginTransmission(TP_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static void tpInit()
{
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);
  delay(10);
  digitalWrite(TP_RST, HIGH);
  delay(50);
  Wire.begin(IIC_SDA, IIC_SCL, 400000UL);
  delay(30);
  uint8_t b[4];
  for (int retry = 0; retry < 3 && !tpOk; retry++)
  {
    tpWriteReg(0xD1, 0x01); // enter command mode
    delay(10);
    if (tpReadRegs(0xD1F8, b, 4)) // panel resolution of the touch frame
    {
      tpResX = (b[1] << 8) | b[0];
      tpResY = (b[3] << 8) | b[2];
      tpOk = tpResX > 100 && tpResY > 100;
    }
    if (!tpOk)
      delay(50);
  }
  Serial0.printf("touch: res %dx%d %s\n", tpResX, tpResY, tpOk ? "OK" : "FAIL");
}

// Polls the panel: returns 1 with the first finger position in native touch
// coordinates (mapping to panel coords is done by the caller), 0 if released.
// Contact = evt 0x06 strictly (proven on this panel); brief dropouts during
// fast swipes are handled by the gesture debounce in touchPollButtons.
static int tpReadPoint(int16_t *x, int16_t *y)
{
  if (!tpOk)
    return 0;
  uint8_t buf[15] = {0};
  if (!tpReadRegs(TP_REG_READ, buf, 15))
    return 0;
  // ack the read so the chip re-arms its point buffer
  {
    Wire.beginTransmission(TP_ADDR);
    Wire.write(TP_REG_READ >> 8);
    Wire.write(TP_REG_READ & 0xFF);
    Wire.write(TP_ACK);
    Wire.endTransmission();
  }
  if (buf[6] != TP_ACK)
    return 0;
  int n = buf[5] & 0x7F;
  if (n < 1)
    return 0;
  const uint8_t *d = buf; // point 0 lives in the first 4 bytes
  if ((d[0] & 0x0F) != TP_EVT_PRESS)
    return 0;
  *x = (int16_t)((d[1] << 4) | (d[3] >> 4));
  *y = (int16_t)((d[2] << 4) | (d[3] & 0x0F));
  return 1;
}
