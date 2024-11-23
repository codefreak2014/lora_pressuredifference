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

    snprintf(temp_buffer, LORA_DATA_SIZE, "%f;%f", press, temp);
    Serial2.println(temp_buffer);
    memcpy(data, temp_buffer, LORA_DATA_SIZE);

    Serial2.println("Lähetetään data:");
    Serial2.println((char *)data);
    setLoraData(data);
    sendLora();
}

void loop()
{
    os_runloop_once();
}
