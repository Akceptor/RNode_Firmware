// Copyright Sandeep Mistry, Mark Qvist and Jacob Eva.
// Licensed under the MIT license.

#ifndef LR11XX_H
#define LR11XX_H

#include <Arduino.h>
#include <SPI.h>
#include "Modem.h"

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2
#define LORA_DEFAULT_BUSY_PIN  -1
#define LORA_MODEM_TIMEOUT_MS 20E3

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

class lr11xx : public Stream {
public:
  lr11xx();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int parsePacket(int size = 0);
  int packetRssi();
  int packetRssi(uint8_t pkt_snr_raw);
  int currentRssi();
  uint8_t packetRssiRaw();
  uint8_t currentRssiRaw();
  uint8_t packetSnrRaw();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));

  void receive(int size = 0);
  void standby();
  void sleep();
  void reset(void);

  bool preInit();
  uint8_t getTxPower();
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  uint32_t getFrequency();
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  long getSignalBandwidth();
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long preamble_symbols);
  void setSyncWord(uint16_t sw);
  bool dcd();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  void waitOnBusy(uint32_t timeout_ms = 100);
  void setPacketParams(long preamble_symbols, uint8_t headermode, uint8_t payload_length, uint8_t crc);
  void setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro);

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN, int busy = LORA_DEFAULT_BUSY_PIN);
  void setSPIFrequency(uint32_t frequency);

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();
  static void onDio0Rise();

  void handleLowDataRate();
  void refreshPacketParams();

  void writeCmd(uint16_t opcode, const uint8_t *params, uint8_t size);
  void readCmd(uint16_t opcode, const uint8_t *params, uint8_t size, uint8_t *out, uint16_t out_size);
  uint32_t getIrq();
  void clearIrq(uint32_t mask);
  void setDioIrqParams(uint32_t irq1, uint32_t irq2);
  void calibrateImage(long frequency);

private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _busy;
  long _frequency;
  long _calibratedFrequency;
  int _txp;
  uint8_t _sf;
  uint8_t _bw;
  uint8_t _cr;
  uint8_t _ldro;
  int _packetIndex;
  int _preambleLength;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  int _rxLen;
  int _rxOffset;
  uint8_t _hw;
  uint8_t _type;
  uint8_t _fwMajor;
  uint8_t _fwMinor;
  uint8_t _packet[255];
  uint8_t _txbuf[255];
  bool _preinit_done;
  bool _rx;
  void (*_onReceive)(int);
};

extern lr11xx lr11xx_modem;

#endif
