// ASYNCHRONOUS QSPI screen flush for the Waveshare 1.75C (CO5300, 466x466).
// Port of dma_flush.h (ping-pong internal buffers, byte-swap in flight,
// queued transactions) to the ESP32-S3 SPI3 host in QSPI mode, replicating
// the Arduino_ESP32QSPI wire protocol exactly:
//   register write c (+ up to 4 param bytes): cmd 0x02 (quad), addr = c<<8
//   pixel stream: cmd 0x32, addr 0x003C00, QIO data phase, chunked
// Transactions complete IN QUEUE ORDER: a simple queued/done sequence
// counter is enough to know when a ping-pong buffer is reusable.
// CS is hardware-driven per transaction (spics_io_num).
// Call dmafInit() ONCE after canvas->begin(): the pins move to SPI3 and
// the Arduino_GFX bus (SPI2) must never be used again afterwards (same
// trick as dma_flush.h).
// Assumes defined before inclusion: LCD_CS/LCD_SCLK/LCD_SDIO0..3,
// SPI_FREQ, W/H, Serial0.
#pragma once
#include <driver/spi_master.h>

#define DMAF_CHUNK_PX 16380 // pixels per transaction (~32 KB IDF DMA cap)
#define DMAF_QUEUE 8
// CO5300 demo offsets (col_offset1=6, row_offset1=0 in the Waveshare
// example constructor) -- the panel RAM is 472 wide, active area shifted
#define DMAF_XSTART 6
#define DMAF_YSTART 0

static spi_device_handle_t dmafDev = nullptr;
static uint16_t *dmafPing[2] = {nullptr, nullptr};
static spi_transaction_t dmafTrans[DMAF_QUEUE];
static int dmafHead = 0, dmafInFlight = 0;
static uint32_t dmafSeqQueued = 0, dmafSeqDone = 0;
static uint32_t dmafBufSeq[2] = {0, 0}; // last queued transaction of each buffer

static void dmafReapOne()
{
  spi_transaction_t *done;
  spi_device_get_trans_result(dmafDev, &done, portMAX_DELAY);
  dmafInFlight--;
  dmafSeqDone++;
}

static spi_transaction_t *dmafNextTrans()
{
  if (dmafInFlight >= DMAF_QUEUE)
    dmafReapOne();
  spi_transaction_t *t = &dmafTrans[dmafHead];
  dmafHead = (dmafHead + 1) % DMAF_QUEUE;
  memset(t, 0, sizeof(*t));
  return t;
}

static void dmafQueue(spi_transaction_t *t)
{
  spi_device_queue_trans(dmafDev, t, portMAX_DELAY);
  dmafInFlight++;
  dmafSeqQueued++;
}

// register write, no data: cmd 0x02 (quad cmd+addr), addr = c<<8
static void dmafCmd(uint8_t c)
{
  spi_transaction_t *t = dmafNextTrans();
  t->flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
  t->cmd = 0x02;
  t->addr = ((uint32_t)c) << 8;
  t->length = 0;
  dmafQueue(t);
}

// register write with up to 4 data bytes (CASET/PASET params)
static void dmafData4(uint8_t c, const uint8_t *d, int n) // n <= 4
{
  spi_transaction_t *t = dmafNextTrans();
  t->flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_USE_TXDATA;
  t->cmd = 0x02;
  t->addr = ((uint32_t)c) << 8;
  memcpy(t->tx_data, d, n);
  t->length = 8 * n;
  dmafQueue(t);
}

static void dmafWindow(int x, int y, int w, int h)
{
  x += DMAF_XSTART;
  y += DMAF_YSTART;
  uint8_t ca[4] = {(uint8_t)(x >> 8), (uint8_t)x,
                   (uint8_t)((x + w - 1) >> 8), (uint8_t)(x + w - 1)};
  uint8_t ra[4] = {(uint8_t)(y >> 8), (uint8_t)y,
                   (uint8_t)((y + h - 1) >> 8), (uint8_t)(y + h - 1)};
  dmafData4(0x2A, ca, 4); // CASET
  dmafData4(0x2B, ra, 4); // PASET
  dmafCmd(0x2C);          // RAMWR
}

// copy + byte swap 2 pixels at a time (the panel expects MSB first)
static void dmafSwapCopy(uint16_t *dst, const uint16_t *src, int npx)
{
  const uint32_t *s = (const uint32_t *)src;
  uint32_t *d = (uint32_t *)dst;
  int n = npx >> 1;
  for (int i = 0; i < n; i++)
  {
    uint32_t v = s[i];
    d[i] = ((v >> 8) & 0x00FF00FFu) | ((v << 8) & 0xFF00FF00u);
  }
  if (npx & 1)
    dst[npx - 1] = __builtin_bswap16(src[npx - 1]);
}

// Sends a rectangle (src with stride in pixels). waitEnd = true: blocks
// until the last byte (partial flushes, power-off); false: returns as soon
// as the last chunk is queued (the rest goes out over DMA while the next
// frame is being rendered).
static void dmafFlush(int x, int y, int w, int h, const uint16_t *src,
                      int stride, bool waitEnd)
{
  dmafWindow(x, y, w, h);
  int rowsPerChunk = DMAF_CHUNK_PX / w;
  if (rowsPerChunk < 1)
    rowsPerChunk = 1;
  int b = 0, row = 0;
  while (row < h)
  {
    int rows = h - row < rowsPerChunk ? h - row : rowsPerChunk;
    int npx = rows * w;
    while (dmafSeqDone < dmafBufSeq[b]) // buffer still on the wire
      dmafReapOne();
    if (stride == w)
      dmafSwapCopy(dmafPing[b], src + row * stride, npx);
    else
      for (int r = 0; r < rows; r++)
        dmafSwapCopy(dmafPing[b] + r * w, src + (row + r) * stride, w);
    spi_transaction_t *t = dmafNextTrans();
    t->flags = SPI_TRANS_MODE_QIO; // cmd 0x32, addr 0x003C00: RAM write continue
    t->cmd = 0x32;
    t->addr = 0x003C00;
    t->length = (size_t)npx * 16;
    t->tx_buffer = dmafPing[b];
    dmafQueue(t);
    dmafBufSeq[b] = dmafSeqQueued;
    b ^= 1;
    row += rows;
  }
  if (waitEnd)
    while (dmafInFlight)
      dmafReapOne();
}

// Simple blocking command (0x28 display off / 0x10 sleep in at power-off)
static void dmafCmdBlocking(uint8_t cmd)
{
  dmafCmd(cmd);
  while (dmafInFlight)
    dmafReapOne();
}

// Call ONCE after canvas->begin(): switches the pins over to SPI3 (QSPI);
// the Arduino_GFX bus (SPI2) must never be used again afterwards.
static bool dmafInit()
{
  spi_bus_config_t bus = {};
  bus.mosi_io_num = LCD_SDIO0;
  bus.miso_io_num = LCD_SDIO1;
  bus.sclk_io_num = LCD_SCLK;
  bus.quadwp_io_num = LCD_SDIO2;
  bus.quadhd_io_num = LCD_SDIO3;
  bus.max_transfer_sz = DMAF_CHUNK_PX * 2 + 8;
  if (spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK)
  {
    Serial0.println("dmaf: spi_bus_initialize failed");
    return false;
  }
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = SPI_FREQ;
  dev.mode = 0;
  dev.command_bits = 8;  // every transaction: 8-bit cmd phase
  dev.address_bits = 24; // ... then 24-bit address phase
  dev.spics_io_num = LCD_CS;
  dev.flags = SPI_DEVICE_HALFDUPLEX;
  dev.queue_size = DMAF_QUEUE;
  if (spi_bus_add_device(SPI3_HOST, &dev, &dmafDev) != ESP_OK)
  {
    Serial0.println("dmaf: spi_bus_add_device failed");
    return false;
  }
  for (int i = 0; i < 2; i++)
  {
    dmafPing[i] = (uint16_t *)heap_caps_malloc(DMAF_CHUNK_PX * 2, MALLOC_CAP_DMA);
    if (!dmafPing[i])
    {
      Serial0.println("dmaf: DMA buffer alloc failed");
      return false;
    }
  }
  Serial0.println("dmaf: async QSPI DMA flush active (SPI3)");
  return true;
}
