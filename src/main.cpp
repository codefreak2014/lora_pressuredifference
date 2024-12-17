/*******************************************************************************
 * Copyright (c) 2023 Arestech
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 *******************************************************************************/

#include <Arduino.h>
#include "variant.h"
#include "Adafruit_TinyUSB.h"
#include "bluetoothService.h"
#include "loraService.h"
#include <lmic.h>
#include <Timeout.h>
#include "SDP.h"
#include "Wire.h"

#include <SPI.h>
//#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define INFO_Y 50
#define USELORASEND 1
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SDP_Controller sdp_controller;

//uint8_t data[LORA_DATA_SIZE];

// Sanomatyyppiä varten
enum MessageType {
    NORMAL = 0x00,    // Normaali tilatieto
    POWER_CHANGE = 0x01, // PowerStatus muutos
    ALERT = 0x02      // Hälytystila
};

float alertThreshold = 0.0;
char comparisonOperator = '>';
uint32_t transmissionInterval = 10 * 60 * 1000; // Annetaan minuutteina -> muunnos millisekunteihin
int powerStatus = 1;

char idkey[7];
const uint32_t ledpinarray[] = {PIN_LED1, PIN_LED2, PIN_LED3};
// Alaslinkin käsittelijä

void onDownlinkMessage(uint8_t *payload, uint8_t length) {
    if (length < 6) {
        Serial2.println("Virhe: alaslinkkiviesti on liian lyhyt!");
        return;
    }

    // Puretaan tiedot alaslinkistä:
    // [0-3]: Hälytysraja (float, IEEE 754 -muodossa)
    // [4]: Vertailuoperaattori (1 = '>', 2 = '<')
    // [5-6]: Lähetyssykli (uint16_t, minuutteina)

    // Hälytysraja
    memcpy(&alertThreshold, payload, sizeof(float));

    // Vertailuoperaattori
    if (payload[4] == 1) {
        comparisonOperator = '>';
    } else if (payload[4] == 2) {
        comparisonOperator = '<';
    } else {
        Serial2.println("Virhe: tuntematon vertailuoperaattori!");
        return;
    }

    // Lähetyssykli
    transmissionInterval = (payload[5] << 8) | payload[6];

    // Tulostetaan päivitykset
    Serial2.println("Alaslinkki vastaanotettu:");
    Serial2.print("  Hälytysraja: ");
    Serial2.println(alertThreshold);
    Serial2.print("  Vertailuoperaattori: ");
    Serial2.println(comparisonOperator);
    Serial2.print("  Lähetyssykli: ");
    Serial2.print(transmissionInterval);
    Serial2.println(" minuuttia");
}

void packPayload(uint8_t *payload, uint8_t type, float press, float temp, int powerStatus) {
    // Sanomatyyppi
    payload[0] = type; // Ensimmäinen tavu = sanomatyyppi
    
    // Skaalataan painemitta välille [-1024 ... 1024], pakataan int16_t-muotoon
    int16_t press_scaled = (int16_t)(press * 100); // Kerrotaan 100:lla tarkkuuden säilyttämiseksi
    
    // Skaalataan lämpötila välille [-50 ... 85], pakataan int8_t-muotoon
    int8_t temp_scaled = (int8_t)(temp * 2); // Kerrotaan 2:lla tarkkuuden säilyttämiseksi

    // Tehotilan tallennus 8-bittiseen muotoon
    uint8_t power_scaled = (uint8_t)powerStatus;

    // Rakennetaan payload
    payload[1] = press_scaled >> 8;   // Paineen MSB
    payload[2] = press_scaled & 0xFF; // Paineen LSB
    payload[3] = temp_scaled;         // Lämpötila
    payload[4] = power_scaled;        // Tehotila
}

void oledDisplay(int csq, char toprow[], char text[], int valueTextSize, int externalPower) {
  char connection_strength[6];
  char idkeyinfo[30];
  sprintf( connection_strength, "s:%d",csq);
  
  display.clearDisplay();
  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);
  if (sizeof(idkey) > 1) {
    if (strcmp(idkey,"null")!=0) {
      sprintf( idkeyinfo, "ID: %s - PWR:%d", idkey, externalPower);
      display.setCursor(5,1);
      display.println(idkeyinfo);
    }
  }
  
  display.setCursor(5,INFO_Y);
  display.println(toprow);
  #ifndef USE_WIFI
  display.setCursor(98, INFO_Y);
  display.print(connection_strength);  
  #endif
  if (valueTextSize > 1) {
    display.setTextSize(valueTextSize); // Draw 1X-scale text
    display.setCursor(10, 20);
  } else {
    display.setCursor(0, 20);
  }
  display.println(text);
  display.display();
  delay(10);
}

void setup()
{
    char display_data[96];
    char display_header[64];
    uint8_t payload[LORA_DATA_SIZE];

    Serial2.begin(115200);
    while (!Serial2)
        delay(10);
    Serial2.flush();
    delay(1000);
    Serial2.println("Serial2 started");
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    digitalWrite(PIN_LED1,  HIGH);    
    // Relay control pins
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_PWR_STATUS, INPUT);

    Wire.begin();
    sprintf( idkey, "BDD0AC");
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 
      Serial2.println(F("SSD1306 allocation failed"));
    }
    display.clearDisplay();
    sprintf( display_header, "DONE!");
    sprintf( display_data, "Display OK");
    oledDisplay(99, display_header,display_data, 3, powerStatus);
    digitalWrite(PIN_LED1, LOW);
    //    bluetoothInit();
    //    bluetoothStartAdv();
    digitalWrite(PIN_LED2,  HIGH);
    #ifdef USELORASEND
    sprintf( display_data, "Set LORA");
    oledDisplay(99, display_header,display_data, 3, powerStatus);
    if (!initLora())
    {
      sprintf( display_data, "LoRa init fail!");
      oledDisplay(99, display_header,display_data, 1, powerStatus);
      Serial2.println("LoRa initialization failed");
    };
    #endif
    digitalWrite(PIN_LED2,  LOW);
    delay(1000);
    digitalWrite(PIN_LED3,  HIGH);
    sdp_controller.begin();
    sdp_controller.getAddress();
    sdp_controller.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_NONE);
    sprintf( display_data, "Sensor set!");
    oledDisplay(99, display_header,display_data, 1, powerStatus);
    delay(1000);
    digitalWrite(PIN_LED3,  LOW);
    float press = sdp_controller.getDiffPressure();
    float temp = sdp_controller.getTemperature();
    sprintf( display_data, "Values read");
    oledDisplay(99, display_header,display_data, 1, powerStatus);

    packPayload(payload, 0x00, press, temp, powerStatus);

    //Serial2.println("Lähetetään data:");
    //Serial2.println((char *)data);
    sprintf( display_data, "Send data.");
    oledDisplay(99, display_header,display_data, 1, powerStatus);
    #ifdef USELORASEND
    setLoraData(payload);
    sendLora();
    #endif
    sprintf( display_data, "All done!");
    oledDisplay(99, display_header,display_data, 1, powerStatus);

}

void loop()
{ 
    static uint32_t lastMeasureMillis, lastSendMillis;
    static int lastPowerStatus = 1;
    static int ledpin=0;
    char display_data[96];
    static char display_header[64];
    
    if (millis() - lastMeasureMillis > 4000) {
        powerStatus = digitalRead(PIN_PWR_STATUS);
        uint32_t ledStatus = digitalRead(ledpinarray[ledpin]);
        digitalWrite(ledpinarray[ledpin],  !ledStatus);
        //uint32_t relayStatus = digitalRead(PIN_RELAY1);
        //digitalWrite(PIN_RELAY1,  !relayStatus);
        //relayStatus = digitalRead(PIN_RELAY2);
        //digitalWrite(PIN_RELAY2,  !relayStatus);
        ledpin++;
        if (ledpin > 2) ledpin=0;
        float press = sdp_controller.getDiffPressure();
        float temp = sdp_controller.getTemperature();
        lastMeasureMillis = millis();

        sprintf( display_data, "%.1f",press);
        sprintf( display_header, "temp: %.1f", temp);
        oledDisplay(99, display_header,display_data, 3, powerStatus);
        if (millis() - lastSendMillis > transmissionInterval) { 
          uint8_t payload[LORA_DATA_SIZE];
          packPayload(payload, 0x00, press, temp, powerStatus);            
          setLoraData(payload);
          sendLora();
          lastSendMillis = millis();
          lastPowerStatus = powerStatus;
        }
    }
    os_runloop_once();

}
