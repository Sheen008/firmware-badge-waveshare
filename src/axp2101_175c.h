// Minimal AXP2101 PMIC driver for the Waveshare 1.75C board (I2C 0x34).
// Self-contained port of the few registers we need from lewis-he's
// XPowersLib (MIT):
//   STATUS1 (0x00) bit3 = battery connected
//   STATUS2 (0x01) bits7:5 = 1 charging / 2 discharging
//   batt voltage: regs 0x34/0x35, H5L8 -> millivolts
//   fuel gauge percent: reg 0xA4 (-1 when no battery)
// Assumes Wire is already started (touch_175c.h does it) and Serial0.
#pragma once
#include <Wire.h>

#define AXP_ADDR 0x34

static bool axpOk = false;

static bool axpReadRegs(uint8_t reg, uint8_t *buf, int n)
{
  Wire.beginTransmission(AXP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return false;
  if (Wire.requestFrom((uint8_t)AXP_ADDR, (uint8_t)n) != n)
    return false;
  for (int i = 0; i < n; i++)
    buf[i] = Wire.read();
  return true;
}

static uint8_t axpReadReg(uint8_t reg)
{
  uint8_t v = 0;
  axpReadRegs(reg, &v, 1);
  return v;
}

static void axpInit()
{
  // chip id: AXP2101 answers 0x4A on reg 0x03
  uint8_t id = 0;
  for (int retry = 0; retry < 3 && !axpOk; retry++)
  {
    if (axpReadRegs(0x03, &id, 1) && (id == 0x4A || id == 0x49))
      axpOk = true;
    else
      delay(20);
  }
  Serial0.printf("axp2101: %s (id 0x%02x)\n", axpOk ? "OK" : "NOT FOUND", id);
}

static bool axpBattConnected() { return axpOk && (axpReadReg(0x00) >> 3) & 1; }
static bool axpCharging() { return axpOk && (axpReadReg(0x01) >> 5) == 0x01; }

static uint16_t axpBattMv()
{
  if (!axpOk || !axpBattConnected())
    return 0;
  uint8_t b[2];
  if (!axpReadRegs(0x34, b, 2))
    return 0;
  return (uint16_t)(((b[0] & 0x1F) << 8) | b[1]); // H5L8, 1 mV/LSB
}

// fuel gauge: -1 = no battery
static int axpBatteryPercent()
{
  if (!axpOk || !axpBattConnected())
    return -1;
  return (int)axpReadReg(0xA4);
}

// ---- PWR button (wired to the AXP2101 POWERON pin) ----
// Edge IRQs latched in INTSTS2 (0x49): bit1 = press (falling edge),
// bit0 = release (rising edge). Waveshare maps PWR to the PMIC, so there
// is no GPIO: we poll the latched edges. Software semantics: short press
// = exit to menu (pwrExitMs timestamp, consumer checks age), hold
// >= 1.5 s = power off (the PMIC still hard-cuts at ~4 s as a brick rescue).
static uint32_t pwrDownAt = 0;      // 0 = up
static volatile bool pwrConfirmFlag = false; // short press = global confirm
static uint32_t pwrConfirmMs = 0;   // timestamp: games treat it as "quit"

static void axpPwrInit()
{
  if (!axpOk)
    return;
  uint8_t en = axpReadReg(0x41); // INTEN2
  en |= 0x03;                    // enable POWERON rising + falling edge IRQs
  Wire.beginTransmission(AXP_ADDR);
  Wire.write(0x41);
  Wire.write(en);
  Wire.endTransmission();
}

static void axpPollPwr()
{
  if (!axpOk)
    return;
  uint8_t st = axpReadReg(0x49); // INTSTS2
  if (!st)
    return;
  if (st & 0x02)
    pwrDownAt = millis(); // press (falling edge)
  if (st & 0x01)
  { // release (rising edge)
    if (pwrDownAt)
    {
      if (millis() - pwrDownAt > 60)
      {
        pwrConfirmFlag = true;
        pwrConfirmMs = millis();
      }
      pwrDownAt = 0;
    }
  }
  // clear the latched status bits (write 1 to clear)
  Wire.beginTransmission(AXP_ADDR);
  Wire.write(0x49);
  Wire.write(st);
  Wire.endTransmission();
}
