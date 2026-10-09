// Copyright Sandeep Mistry, Mark Qvist and Jacob Eva.
// Licensed under the MIT license.

#include "Boards.h"

#if MODEM == LR1121
#include "lr11xx.h"

#if MCU_VARIANT == MCU_ESP32
  #if MCU_VARIANT == MCU_ESP32 and !defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "soc/rtc_wdt.h"
  #endif
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

#define OP_GET_STATUS_11X           0x0100
#define OP_GET_VERSION_11X          0x0101
#define OP_WRITE_BUFFER_11X         0x0109
#define OP_READ_BUFFER_11X          0x010A
#define OP_CLEAR_RX_BUFFER_11X      0x010B
#define OP_CALIBRATE_11X            0x010F
#define OP_SET_REG_MODE_11X         0x0110
#define OP_CALIBRATE_IMAGE_11X      0x0111
#define OP_SET_DIO_RF_SWITCH_11X    0x0112
#define OP_SET_DIO_IRQ_PARAMS_11X   0x0113
#define OP_CLEAR_IRQ_11X            0x0114
#define OP_SET_SLEEP_11X            0x011B
#define OP_SET_STANDBY_11X          0x011C
#define OP_GET_RANDOM_11X           0x0120
#define OP_RX_BUFFER_STATUS_11X     0x0203
#define OP_PACKET_STATUS_11X        0x0204
#define OP_CURRENT_RSSI_11X         0x0205
#define OP_RX_11X                   0x0209
#define OP_TX_11X                   0x020A
#define OP_RF_FREQ_11X              0x020B
#define OP_PACKET_TYPE_11X          0x020E
#define OP_MODULATION_PARAMS_11X    0x020F
#define OP_PACKET_PARAMS_11X        0x0210
#define OP_TX_PARAMS_11X            0x0211
#define OP_RX_TX_FALLBACK_MODE_11X  0x0213
#define OP_PA_CONFIG_11X            0x0215
#define OP_RX_BOOSTED_11X           0x0227
#define OP_LORA_SYNC_WORD_11X       0x022B

#define IRQ_TX_DONE_MASK_11X        (1UL << 2)
#define IRQ_RX_DONE_MASK_11X        (1UL << 3)
#define IRQ_PREAMBLE_DET_MASK_11X   (1UL << 4)
#define IRQ_HEADER_VALID_MASK_11X   (1UL << 5)
#define IRQ_HEADER_ERROR_MASK_11X   (1UL << 6)
#define IRQ_PAYLOAD_CRC_ERROR_MASK_11X (1UL << 7)
#define IRQ_ALL_MASK_11X            0xFFFFFFFFUL

#define MODE_STDBY_RC_11X           0x00
#define MODE_STDBY_XOSC_11X         0x01
#define FALLBACK_STDBY_RC_11X       0x01
#define PACKET_TYPE_LORA_11X        0x02
#define SYNC_WORD_PRIVATE_11X       0x12

#define VERSION_TYPE_LR1121         0x03
#define VERSION_TYPE_LR1121_TRX     0xF3

#define FREQ_MIN_11X                150E6
#define FREQ_MAX_11X                960E6
#define IMAGE_CALIB_WINDOW_11X      4E6

#define MAX_PKT_LENGTH 255

#ifndef LR11XX_DIO_AS_RF_SWITCH
  #define LR11XX_DIO_AS_RF_SWITCH false
#endif
#ifndef LR11XX_USE_DCDC
  #define LR11XX_USE_DCDC false
#endif

extern SPIClass SPI;

lr11xx::lr11xx() :
  _spiSettings(16E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN), _busy(LORA_DEFAULT_BUSY_PIN),
  _frequency(0),
  _calibratedFrequency(0),
  _txp(0),
  _sf(0x07),
  _bw(0x04),
  _cr(0x01),
  _ldro(0x00),
  _packetIndex(0),
  _preambleLength(18),
  _implicitHeaderMode(0),
  _payloadLength(0),
  _crcMode(1),
  _rxLen(0),
  _rxOffset(0),
  _hw(0),
  _type(0),
  _fwMajor(0),
  _fwMinor(0),
  _packet{},
  _txbuf{},
  _preinit_done(false),
  _rx(false),
  _onReceive(NULL)
{ setTimeout(0); }

void ISR_VECT lr11xx::waitOnBusy(uint32_t timeout_ms) {
  if (_busy != -1) {
    unsigned long time = millis();
    while (digitalRead(_busy) == HIGH) {
      if (millis() - time >= timeout_ms) { break; }
    }
  }
}

void ISR_VECT lr11xx::writeCmd(uint16_t opcode, const uint8_t *params, uint8_t size) {
  waitOnBusy();
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_ss, LOW);
  SPI.transfer(opcode >> 8);
  SPI.transfer(opcode & 0xFF);
  for (int i = 0; i < size; i++) { SPI.transfer(params[i]); }
  digitalWrite(_ss, HIGH);
  SPI.endTransaction();
}

void ISR_VECT lr11xx::readCmd(uint16_t opcode, const uint8_t *params, uint8_t size, uint8_t *out, uint16_t out_size) {
  writeCmd(opcode, params, size);
  waitOnBusy();
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_ss, LOW);
  SPI.transfer(0x00); // Stat1, discarded
  for (int i = 0; i < out_size; i++) { out[i] = SPI.transfer(0x00); }
  digitalWrite(_ss, HIGH);
  SPI.endTransaction();
}

uint32_t ISR_VECT lr11xx::getIrq() {
  uint8_t buf[6] = {0};
  waitOnBusy();
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_ss, LOW);
  SPI.transfer(OP_GET_STATUS_11X >> 8);
  SPI.transfer(OP_GET_STATUS_11X & 0xFF);
  for (int i = 2; i < 6; i++) { buf[i] = SPI.transfer(0x00); }
  digitalWrite(_ss, HIGH);
  SPI.endTransaction();
  return ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
}

void ISR_VECT lr11xx::clearIrq(uint32_t mask) {
  uint8_t buf[4] = { (uint8_t)(mask >> 24), (uint8_t)(mask >> 16), (uint8_t)(mask >> 8), (uint8_t)mask };
  writeCmd(OP_CLEAR_IRQ_11X, buf, 4);
}

void lr11xx::setDioIrqParams(uint32_t irq1, uint32_t irq2) {
  uint8_t buf[8] = {
    (uint8_t)(irq1 >> 24), (uint8_t)(irq1 >> 16), (uint8_t)(irq1 >> 8), (uint8_t)irq1,
    (uint8_t)(irq2 >> 24), (uint8_t)(irq2 >> 16), (uint8_t)(irq2 >> 8), (uint8_t)irq2
  };
  writeCmd(OP_SET_DIO_IRQ_PARAMS_11X, buf, 8);
}

void lr11xx::reset(void) {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
  }
  delay(300); // BUSY stays high ~230-270 ms after reset
  waitOnBusy(3000);
}

bool lr11xx::preInit() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

  // Second radio footprint on this board shares the SPI bus; keep it deselected
  // pin_cs2 is -1 on boards without it (ESP32-C3: GPIO11-17 are flash pins)
  if (pin_cs2 >= 0) {
    pinMode(pin_cs2, OUTPUT);
    digitalWrite(pin_cs2, HIGH);
  }

  if (_busy != -1) { pinMode(_busy, INPUT); }

  SPI.begin(pin_sclk, pin_miso, pin_mosi, pin_cs);
  gpio_pullup_en((gpio_num_t)pin_miso);

  reset();

  bool found = false;
  long start = millis();
  while (((millis() - start) < 2000) && (millis() >= start)) {
    uint8_t ver[4] = {0};
    readCmd(OP_GET_VERSION_11X, NULL, 0, ver, 4);
    _hw = ver[0]; _type = ver[1]; _fwMajor = ver[2]; _fwMinor = ver[3];
    // 0xF3 is the LR1121 transceiver image (ELRS-flashed); 0xDF is the bootloader and is rejected
    if (_type == VERSION_TYPE_LR1121 || _type == VERSION_TYPE_LR1121_TRX) { found = true; break; }
    delay(100);
  }
  if (!found) { return false; }

  _preinit_done = true;
  return true;
}

int lr11xx::begin(long frequency) {
  if (frequency < FREQ_MIN_11X || frequency > FREQ_MAX_11X) { return 0; }

  if (!_preinit_done) { if (!preInit()) { return false; } }
  else { reset(); }

  standby();
  uint8_t fallback = FALLBACK_STDBY_RC_11X;
  writeCmd(OP_RX_TX_FALLBACK_MODE_11X, &fallback, 1);
  clearIrq(IRQ_ALL_MASK_11X);

  uint8_t calibrate = 0x3F;
  writeCmd(OP_CALIBRATE_11X, &calibrate, 1);
  delay(5);
  waitOnBusy();

  uint8_t packet_type = PACKET_TYPE_LORA_11X;
  writeCmd(OP_PACKET_TYPE_11X, &packet_type, 1);

  #if LR11XX_DIO_AS_RF_SWITCH
    const uint8_t rfsw[8] = LR11XX_RFSW_CFG;
    writeCmd(OP_SET_DIO_RF_SWITCH_11X, rfsw, 8);
  #endif

  #if LR11XX_USE_DCDC
    uint8_t dcdc = 0x01;
    writeCmd(OP_SET_REG_MODE_11X, &dcdc, 1);
  #endif

  uint8_t boosted = 0x01;
  writeCmd(OP_RX_BOOSTED_11X, &boosted, 1);

  setSyncWord(SYNC_WORD_PRIVATE_11X);
  setDioIrqParams(IRQ_RX_DONE_MASK_11X, IRQ_ALL_MASK_11X);

  _calibratedFrequency = 0;
  setFrequency(frequency);
  setTxPower(2);
  enableCrc();
  setModulationParams(_sf, _bw, _cr, _ldro);
  refreshPacketParams();

  return 1;
}

void lr11xx::end() { sleep(); SPI.end(); _preinit_done = false; }

int lr11xx::beginPacket(int implicitHeader) {
  standby();
  _implicitHeaderMode = implicitHeader ? 1 : 0;
  _payloadLength = 0;
  return 1;
}

int lr11xx::endPacket() {
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
  writeCmd(OP_WRITE_BUFFER_11X, _txbuf, _payloadLength);
  uint8_t timeout[3] = {0}; // Single TX, no timeout
  writeCmd(OP_TX_11X, timeout, 3);

  bool timed_out = false;
  uint32_t w_timeout = millis()+LORA_MODEM_TIMEOUT_MS;
  while ((millis() < w_timeout) && ((getIrq() & IRQ_TX_DONE_MASK_11X) == 0)) { yield(); }
  if (!(millis() < w_timeout)) { timed_out = true; }

  clearIrq(IRQ_TX_DONE_MASK_11X);
  if (timed_out) { return 0; } else { return 1; }
}

static unsigned long preamble_detected_at = 0;
extern long lora_preamble_time_ms;
extern long lora_header_time_ms;
static bool false_preamble_detected = false;

bool lr11xx::dcd() {
  uint32_t irq = getIrq();
  uint32_t now = millis();

  bool header_detected = false;
  bool carrier_detected = false;

  if ((irq & IRQ_HEADER_VALID_MASK_11X) != 0) { header_detected = true; carrier_detected = true; }

  if ((irq & IRQ_PREAMBLE_DET_MASK_11X) != 0) {
    carrier_detected = true;
    if (preamble_detected_at == 0) { preamble_detected_at = now; }
    if (now - preamble_detected_at > lora_preamble_time_ms + lora_header_time_ms) {
      preamble_detected_at = 0;
      if (!header_detected) { false_preamble_detected = true; }
      clearIrq(IRQ_PREAMBLE_DET_MASK_11X);
    }
  }

  if (false_preamble_detected) { lr11xx_modem.receive(); false_preamble_detected = false; }
  return carrier_detected;
}

uint8_t lr11xx::currentRssiRaw() {
  uint8_t byte = 0;
  readCmd(OP_CURRENT_RSSI_11X, NULL, 0, &byte, 1);
  return byte;
}

int ISR_VECT lr11xx::currentRssi() {
  uint8_t byte = 0;
  readCmd(OP_CURRENT_RSSI_11X, NULL, 0, &byte, 1);
  return -(int(byte)) / 2;
}

uint8_t lr11xx::packetRssiRaw() {
  uint8_t buf[3] = {0};
  readCmd(OP_PACKET_STATUS_11X, NULL, 0, buf, 3);
  return buf[2];
}

int ISR_VECT lr11xx::packetRssi() {
  uint8_t buf[3] = {0};
  readCmd(OP_PACKET_STATUS_11X, NULL, 0, buf, 3);
  return -buf[0] / 2;
}

int ISR_VECT lr11xx::packetRssi(uint8_t pkt_snr_raw) { return packetRssi(); }

uint8_t ISR_VECT lr11xx::packetSnrRaw() {
  uint8_t buf[3] = {0};
  readCmd(OP_PACKET_STATUS_11X, NULL, 0, buf, 3);
  return buf[1];
}

float ISR_VECT lr11xx::packetSnr() {
  uint8_t buf[3] = {0};
  readCmd(OP_PACKET_STATUS_11X, NULL, 0, buf, 3);
  return float((int8_t)buf[1]) * 0.25;
}

long lr11xx::packetFrequencyError() { return 0; }

size_t lr11xx::write(uint8_t byte) { return write(&byte, sizeof(byte)); }
size_t lr11xx::write(const uint8_t *buffer, size_t size) {
  if ((_payloadLength + size) > MAX_PKT_LENGTH) { size = MAX_PKT_LENGTH - _payloadLength; }
  memcpy(_txbuf + _payloadLength, buffer, size);
  _payloadLength = _payloadLength + size;
  return size;
}

int ISR_VECT lr11xx::available() { return _rxLen - _packetIndex; }

int ISR_VECT lr11xx::read() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t req[2] = { (uint8_t)_rxOffset, (uint8_t)_rxLen };
    readCmd(OP_READ_BUFFER_11X, req, 2, _packet, _rxLen);
    writeCmd(OP_CLEAR_RX_BUFFER_11X, NULL, 0);
  }

  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

int lr11xx::peek() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t req[2] = { (uint8_t)_rxOffset, (uint8_t)_rxLen };
    readCmd(OP_READ_BUFFER_11X, req, 2, _packet, _rxLen);
    writeCmd(OP_CLEAR_RX_BUFFER_11X, NULL, 0);
  }

  return _packet[_packetIndex];
}

void lr11xx::flush() { }

void lr11xx::onReceive(void(*callback)(int)) {
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    // DIO9 (IRQ pin) carries RX_DONE only; DIO11 is unrouted but keeps the other flags readable via GetStatus
    setDioIrqParams(IRQ_RX_DONE_MASK_11X, IRQ_ALL_MASK_11X);

    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    attachInterrupt(digitalPinToInterrupt(_dio0), lr11xx::onDio0Rise, RISING);

  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
  }
}

void lr11xx::receive(int size) {
  if (size > 0) {
    _implicitHeaderMode = 1;
    _payloadLength = size;
  } else { _implicitHeaderMode = 0; }
  refreshPacketParams();

  uint8_t mode[3] = {0xFF, 0xFF, 0xFF}; // Continuous mode
  writeCmd(OP_RX_11X, mode, 3);
  _rx = true;
}

void lr11xx::standby() {
  uint8_t byte = MODE_STDBY_XOSC_11X;
  writeCmd(OP_SET_STANDBY_11X, &byte, 1);
  _rx = false;
}

void lr11xx::sleep() {
  uint8_t buf[5] = {0};
  writeCmd(OP_SET_SLEEP_11X, buf, 5);
  _rx = false;
}

void lr11xx::enableTCXO() { }
void lr11xx::disableTCXO() { }

void lr11xx::setTxPower(int level, int outputPin) {
  // Always the HP PA: the LP PA would need a different matching path than this board has
  uint8_t pa_buf[4] = {0x01, 0x01, 0x04, 0x07};
  writeCmd(OP_PA_CONFIG_11X, pa_buf, 4);

  if (level > 22) { level = 22; }
  else if (level < -9) { level = -9; }

  uint8_t tx_buf[2] = { (uint8_t)(int8_t)level, 0x02 }; // 48 us ramp
  writeCmd(OP_TX_PARAMS_11X, tx_buf, 2);

  _txp = level;
}

uint8_t lr11xx::getTxPower() { return _txp; }

void lr11xx::calibrateImage(long frequency) {
  float mhz = frequency / 1E6;
  uint8_t buf[2] = {
    (uint8_t)floor((mhz - 4.0 - 1.0) / 4.0),
    (uint8_t)ceil((mhz + 4.0 + 1.0) / 4.0)
  };
  bool was_rx = _rx;
  standby();
  writeCmd(OP_CALIBRATE_IMAGE_11X, buf, 2);
  waitOnBusy();
  if (was_rx) { receive(_implicitHeaderMode ? _payloadLength : 0); }
  _calibratedFrequency = frequency;
}

void lr11xx::setFrequency(long frequency) {
  // The sketch stores magic values here and reads them back, so always cache
  _frequency = frequency;
  if (frequency < FREQ_MIN_11X || frequency > FREQ_MAX_11X) { return; }

  if (_calibratedFrequency == 0 || labs(frequency - _calibratedFrequency) > IMAGE_CALIB_WINDOW_11X) {
    calibrateImage(frequency);
  }

  uint32_t freq = (uint32_t)frequency;
  uint8_t buf[4] = { (uint8_t)(freq >> 24), (uint8_t)(freq >> 16), (uint8_t)(freq >> 8), (uint8_t)freq };
  writeCmd(OP_RF_FREQ_11X, buf, 4);
}

uint32_t lr11xx::getFrequency() { return _frequency; }

void lr11xx::setSpreadingFactor(int sf) {
  if (sf < 5)       { sf = 5; }
  else if (sf > 12) { sf = 12; }
  _sf = sf;

  handleLowDataRate();
  setModulationParams(sf, _bw, _cr, _ldro);
}

long lr11xx::getSignalBandwidth() {
  switch (_bw) {
    case 0x01: return 15.6E3;
    case 0x02: return 31.25E3;
    case 0x03: return 62.5E3;
    case 0x04: return 125E3;
    case 0x05: return 250E3;
    case 0x06: return 500E3;
    case 0x08: return 10.4E3;
    case 0x09: return 20.8E3;
    case 0x0A: return 41.7E3;
  }
  return 0;
}

extern bool lora_low_datarate;
void lr11xx::handleLowDataRate() {
  if ( long( (1<<_sf) / (getSignalBandwidth()/1000)) > 16)
         { _ldro = 0x01; lora_low_datarate = true;  }
    else { _ldro = 0x00; lora_low_datarate = false; }
}

void lr11xx::setSignalBandwidth(long sbw) {
  // No 7.8 kHz on LR11xx
  if (sbw <= 10.4E3)       { _bw = 0x08; }
  else if (sbw <= 15.6E3)  { _bw = 0x01; }
  else if (sbw <= 20.8E3)  { _bw = 0x09; }
  else if (sbw <= 31.25E3) { _bw = 0x02; }
  else if (sbw <= 41.7E3)  { _bw = 0x0A; }
  else if (sbw <= 62.5E3)  { _bw = 0x03; }
  else if (sbw <= 125E3)   { _bw = 0x04; }
  else if (sbw <= 250E3)   { _bw = 0x05; }
  else                     { _bw = 0x06; }

  handleLowDataRate();
  setModulationParams(_sf, _bw, _cr, _ldro);
}

void lr11xx::setCodingRate4(int denominator) {
  if (denominator < 5) { denominator = 5; }
  else if (denominator > 8) { denominator = 8; }
  int cr = denominator - 4;
  _cr = cr;
  setModulationParams(_sf, _bw, cr, _ldro);
}

void lr11xx::setPreambleLength(long preamble_symbols) {
  _preambleLength = preamble_symbols;
  refreshPacketParams();
}

void lr11xx::setSyncWord(uint16_t sw) {
  // Same hardcoded private network word as sx126x (0x1424 on SX126x)
  uint8_t sync = SYNC_WORD_PRIVATE_11X;
  writeCmd(OP_LORA_SYNC_WORD_11X, &sync, 1);
}

void lr11xx::setPins(int ss, int reset, int dio0, int busy) {
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _busy = busy;
}

void lr11xx::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro) {
  uint8_t buf[4] = { sf, bw, cr, (uint8_t)ldro };
  writeCmd(OP_MODULATION_PARAMS_11X, buf, 4);
}

void lr11xx::setPacketParams(long preamble_symbols, uint8_t headermode, uint8_t payload_length, uint8_t crc) {
  uint8_t buf[6];
  buf[0] = uint8_t((preamble_symbols & 0xFF00) >> 8);
  buf[1] = uint8_t((preamble_symbols & 0x00FF));
  buf[2] = headermode;
  buf[3] = payload_length;
  buf[4] = crc;
  buf[5] = 0x00; // standard IQ
  writeCmd(OP_PACKET_PARAMS_11X, buf, 6);
}

void lr11xx::refreshPacketParams() {
  setPacketParams(_preambleLength, _implicitHeaderMode, _implicitHeaderMode ? _payloadLength : MAX_PKT_LENGTH, _crcMode);
}

void ISR_VECT lr11xx::handleDio0Rise() {
  uint32_t irq = getIrq();
  clearIrq(irq);

  bool header_failed = (irq & IRQ_HEADER_ERROR_MASK_11X) && !(irq & IRQ_HEADER_VALID_MASK_11X);
  if ((irq & IRQ_RX_DONE_MASK_11X) && !(irq & IRQ_PAYLOAD_CRC_ERROR_MASK_11X) && !header_failed) {
    uint8_t rxbuf[2] = {0};
    readCmd(OP_RX_BUFFER_STATUS_11X, NULL, 0, rxbuf, 2);
    _rxLen = rxbuf[0];
    _rxOffset = rxbuf[1];
    _packetIndex = 0;
    if (_onReceive) { _onReceive(_rxLen); }
  }
}

void ISR_VECT lr11xx::onDio0Rise() { lr11xx_modem.handleDio0Rise(); }
void lr11xx::setSPIFrequency(uint32_t frequency) { _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0); }
void lr11xx::enableCrc() { _crcMode = 1; refreshPacketParams(); }
void lr11xx::disableCrc() { _crcMode = 0; refreshPacketParams(); }
void lr11xx::explicitHeaderMode() { _implicitHeaderMode = 0; refreshPacketParams(); }
void lr11xx::implicitHeaderMode() { _implicitHeaderMode = 1; refreshPacketParams(); }
byte lr11xx::random() {
  uint8_t buf[4] = {0};
  readCmd(OP_GET_RANDOM_11X, NULL, 0, buf, 4);
  return buf[3];
}

lr11xx lr11xx_modem;

#endif
