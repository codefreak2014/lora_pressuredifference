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

SDP_Controller sdp_controller;

uint8_t data[LORA_DATA_SIZE];
float alertThreshold = 0.0;
char comparisonOperator = '>';
uint16_t transmissionInterval = 10; // Minuutteina, oletusarvo 10

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


void setup()
{
    Serial2.begin(115200);
    while (!Serial2)
        delay(10);
    Serial2.flush();
    delay(1000);
    Serial2.println("Serial2 started");

    Wire.begin();

    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);

    //    bluetoothInit();
    //    bluetoothStartAdv();

    if (!initLora())
    {
        Serial2.println("LoRa initialization failed");
    };

    sdp_controller.begin();
    sdp_controller.getAddress();
    sdp_controller.startContinuousMeasurement(SDP_TEMPCOMP_DIFFERENTIAL_PRESSURE, SDP_AVERAGING_NONE);

    delay(1000);

    double press = sdp_controller.getDiffPressure();
    double temp = sdp_controller.getTemperature();

    char temp_buffer[LORA_DATA_SIZE];
    digitalWrite(PIN_LED1, HIGH);
    snprintf(temp_buffer, LORA_DATA_SIZE, "%f;%f", press, temp);
    Serial2.println(temp_buffer);
    memcpy(data, temp_buffer, LORA_DATA_SIZE);

    Serial2.println("Lähetetään data:");
    Serial2.println((char *)data);
    setLoraData(data);
    sendLora();
    digitalWrite(PIN_LED1, LOW);
    delay(5000);
    digitalWrite(PIN_LED2, LOW);
    delay(5000);
    digitalWrite(PIN_LED3, LOW);
    delay(5000);
    digitalWrite(PIN_LED1, HIGH);
    delay(5000);
    digitalWrite(PIN_LED2, HIGH);
    delay(5000);
    digitalWrite(PIN_LED3, HIGH);
    delay(5000);
}

void loop()
{
    os_runloop_once();
}
