/*******************************************************************************
 * Copyright (c) 2023 Arestech
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 *******************************************************************************/

#include <Arduino.h>
#include <variant.h>
#include "Adafruit_TinyUSB.h"
#include "bluetoothService.h"
#include "loraService.h"
#include <lmic.h>
#include <Timeout.h>
#include "SDP.h"
#include "Wire.h"

#include <SPI.h>
#include <Wire.h>
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

uint8_t data[LORA_DATA_SIZE];
float alertThreshold = 0.0;
char comparisonOperator = '>';
uint16_t transmissionInterval = 10; // Minuutteina, oletusarvo 10
char idkey[7];

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

void oledDisplay(int csq, char toprow[], char text[], int valueTextSize) {
  char connection_strength[6];
  char idkeyinfo[30];
  sprintf( connection_strength, "s:%d",csq);
  
  display.clearDisplay();
  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(SSD1306_WHITE);
  if (sizeof(idkey) > 1) {
    if (strcmp(idkey,"null")!=0) {
      sprintf( idkeyinfo, "ID: %s", idkey);
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

    Serial2.begin(115200);
    while (!Serial2)
        delay(10);
    Serial2.flush();
    delay(1000);
    Serial2.println("Serial2 started");

    Wire.begin();
    sprintf( idkey, "123456");
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);

    //    bluetoothInit();
    //    bluetoothStartAdv();
    #ifdef USELORASEND
    if (!initLora())
    {
        Serial2.println("LoRa initialization failed");
    };
    #endif
    digitalWrite(PIN_LED1,  HIGH);
    sdp_controller.begin();
    sdp_controller.getAddress();
    sdp_controller.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_NONE);
    digitalWrite(PIN_LED2,  HIGH);
    delay(1000);
    digitalWrite(PIN_LED3,  HIGH);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 
      Serial2.println(F("SSD1306 allocation failed"));
    }
    digitalWrite(PIN_LED3,  LOW);
    display.clearDisplay();
    sprintf( display_header, "DONE!");
    sprintf( display_data, "Display OK");
    oledDisplay(99, display_header,display_data, 3);
    double press = sdp_controller.getDiffPressure();
    double temp = sdp_controller.getTemperature();

    char temp_buffer[LORA_DATA_SIZE];
    digitalWrite(PIN_LED2,  LOW);

    snprintf(temp_buffer, LORA_DATA_SIZE, "%f;%f", press, temp);
    Serial2.println(temp_buffer);
    memcpy(data, temp_buffer, LORA_DATA_SIZE);

    Serial2.println("Lähetetään data:");
    Serial2.println((char *)data);
    #ifdef USELORASEND
    setLoraData(data);
    sendLora();
    #endif
    digitalWrite(PIN_LED1, LOW);

}

void loop()
{ 
    static uint32_t lastMeasureMillis;
    char display_data[96];
    static char display_header[64];
    
    //os_runloop_once();
    if (millis() - lastMeasureMillis > 4000) {
        uint32_t ledStatus = digitalRead(PIN_LED3);
        digitalWrite(PIN_LED3,  !ledStatus);
        
        double press = sdp_controller.getDiffPressure();
        double temp = sdp_controller.getTemperature();
        lastMeasureMillis = millis();
    
        char temp_buffer[LORA_DATA_SIZE];

        snprintf(temp_buffer, LORA_DATA_SIZE, "%f;%f", press, temp);
        Serial2.println(temp_buffer);
        sprintf( display_data, "%.1f",press);
        sprintf( display_header, "temp: %.1f", temp);
        oledDisplay(99, display_header,display_data, 3);
    }

}
