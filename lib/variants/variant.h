/*
 Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
 Copyright (c) 2016 Sandeep Mistry All right reserved.
 Copyright (c) 2018, Adafruit Industries (adafruit.com)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_PCA10056_
#define _VARIANT_PCA10056_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF
// #define USE_LFRC // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  // Number of pins defined in PinDescription array

#define UNUSED_GPIO 27

#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#define PIN_LED1 (26) // R
#define PIN_LED2 (6)  // G
#define PIN_LED3 (4)  // B

// Defined unused pin to LED_BLUE and LED_BUILDIN to avaid compile errors from depending libraries
#define LED_BLUE (UNUSED_GPIO)
#define LED_BUILTIN (UNUSED_GPIO)

#define LED_STATE_ON 0 // State when LED is litted

/*
 * Buttons
 */

/*
 * Analog pins
 */
#define PIN_A0 (2)
#define PIN_A1 (0xff)
#define PIN_A2 (0xff)
#define PIN_A3 (0xff)
#define PIN_A4 (0xff)
#define PIN_A5 (0xff)
#define PIN_A6 (0xff)
#define PIN_A7 (0xff)

  static const uint8_t A0 = PIN_A0;
  static const uint8_t A1 = PIN_A1;
  static const uint8_t A2 = PIN_A2;
  static const uint8_t A3 = PIN_A3;
  static const uint8_t A4 = PIN_A4;
  static const uint8_t A5 = PIN_A5;
  static const uint8_t A6 = PIN_A6;
  static const uint8_t A7 = PIN_A7;
#define ADC_RESOLUTION 14

/*
 * Serial interfaces
 */

/*
 * SPI Interface (LoRa)
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO (29)
#define PIN_SPI_MOSI (31)
#define PIN_SPI_SCK (47)
#define PIN_SPI_NSS (34)

/*
 * Serial interfaces (GPIO1 & GPIO2)
 */
#define PIN_SERIAL1_TX (33)
#define PIN_SERIAL1_RX (9)

// For testing purposes
#define PIN_SERIAL2_RX (UNUSED_GPIO)
#define PIN_SERIAL2_TX (PIN_GPIO1)

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (24)
#define PIN_WIRE_SCL (22)

#define PIN_WIRE_SDA2 (15)
#define PIN_WIRE_SCL2 (13)

  /*
   * IOs
   */

#define PIN_LORA_DIO0 (12)
#define PIN_LORA_DIO1 (PIN_GPIO2)
#define PIN_LORA_NRESET (42)
#define PIN_PWR_STATUS (36)
#define PIN_CHR_NINT (38)
#define PIN_RELAY1 (8)
#define PIN_RELAY2 (41)
#define PIN_GPIO1 (20)
#define PIN_GPIO2 (45)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
