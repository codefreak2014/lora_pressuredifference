#include "loraService.h"

#include <lmic.h>
#include <hal/hal.h>
#include <LoRa.h>
#include <SPI.h>
#include <ArduinoJson.h>

#define DEVICEID_SIZE 8
#define FIRMWARE_VERSION 1

uint8_t lora_data[LORA_DATA_SIZE];

bool bLoRa_available = false;

uint64_t u64DeviceID = *((uint64_t *)NRF_FICR->DEVICEID);
// byte LoRaAddress[8] = {byte(u64DeviceID), byte(u64DeviceID >> 8), byte(u64DeviceID >> 16), byte(u64DeviceID >> 24), byte(u64DeviceID >> 32), byte(u64DeviceID >> 40), byte(u64DeviceID >> 48), byte(u64DeviceID >> 56)};

/**LORA SETTINGS*/
// static const u1_t PROGMEM DEVEUI[8] = {byte(u64DeviceID >> 56), byte(u64DeviceID >> 48), byte(u64DeviceID >> 40), byte(u64DeviceID >> 32), byte(u64DeviceID >> 24), byte(u64DeviceID >> 16), byte(u64DeviceID >> 8), byte(u64DeviceID)};
// static const u1_t PROGMEM DEVEUI[8] = {0x9E, 0xB0, 0xAF, 0x1F, 0x3E, 0xBD, 0xD0, 0xAB};
static const u1_t PROGMEM DEVEUI[8] = {0xAB, 0xD0, 0xBD, 0x3E, 0x1F, 0xAF, 0xB0, 0x9E};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// static const u1_t PROGMEM APPEUI[8] = {0xFF, 0xB3, 0xFF, 0x7E, 0xFF, 0x05, 0xFF, 0xBB};
static const u1_t PROGMEM APPEUI[8] = {0xBB, 0xFF, 0x05, 0xFF, 0x7E, 0xFF, 0xB3, 0xFF};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = {0x44, 0xFF, 0xB1, 0xFF, 0xE9, 0xFF, 0x0F, 0xFF, 0x26, 0xEF, 0x5E, 0x56, 0xC6, 0x48, 0xB4, 0xE3};
// static const u1_t PROGMEM APPKEY[16] = {0xE3, 0xB4, 0x48, 0xC6, 0x56, 0x5E, 0xEF, 0x26, 0xFF, 0x0F, 0xFF, 0xE9, 0xFF, 0xB1, 0xFF, 0x44};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

void onEvent(ev_t ev);
void do_send(osjob_t *j);
void sendLora();

const lmic_pinmap lmic_pins = {
    .nss = PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PIN_LORA_NRESET,
    .dio = {PIN_LORA_DIO0, PIN_LORA_DIO1, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 0, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 0,
};

int initLora()
{
  LoRa.setPins(PIN_SPI_NSS, PIN_LORA_NRESET, PIN_LORA_DIO0); // Set CS, Reset, IRQ pin
  LoRa.setSPIFrequency(1.25E5);                              // Use lower SPI frequency 125kHz = 8E6/64

  if (!LoRa.begin(868100000))
  {

    Serial2.println("LoRa init failed. Check your connections.");
    bLoRa_available = false;
    return 0;
  }
  else
  {
    bLoRa_available = true;
    LoRa.idle();
    Serial2.println("********** LoRa init succeeded. **********");
    delay(100);
    os_init();

    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF12;
    LMIC_setDrTxpow(DR_SF7, 14);
    return 1;
  }
}

void onEvent(ev_t ev)
{
  Serial2.println("Event received");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial2.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial2.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial2.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial2.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial2.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial2.println(F("EV_JOINED"));
    {
      u4_t netid = 0;
      devaddr_t devaddr = 0;
      u1_t nwkKey[16];
      u1_t artKey[16];
      LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
    }
    // Disable link check validation (automatically enabled
    // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial2.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial2.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial2.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    Serial2.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial2.println(F("Received ack"));
    break;
  case EV_LOST_TSYNC:
    Serial2.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial2.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    Serial2.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial2.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial2.println(F("EV_LINK_ALIVE"));
    break;
  case EV_SCAN_FOUND:
    Serial2.println(F("EV_SCAN_FOUND"));
    break;
  case EV_TXSTART:
    Serial2.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial2.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial2.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;
  default:
    Serial2.print(F("Unknown event: "));
    Serial2.println((unsigned)ev);
    break;
  }
}

void sendLora()
{
  do_send(&sendjob);
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial2.println(F("OP_TXRXPEND, not sending..."));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lora_data, sizeof(lora_data) - 1, 0);
    Serial2.println(F("Packet queued"));
    delay(1000);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setLoraData(uint8_t data[LORA_DATA_SIZE])
{
  memcpy(lora_data, data, sizeof(lora_data));
}
