#ifndef _LORASERVICE_H
#define _LORASERVICE_H

#include <stdint.h>

#define LORA_DATA_SIZE 7

int initLora();
void sendLora();
void setLoraData(uint8_t data[LORA_DATA_SIZE]);

#endif