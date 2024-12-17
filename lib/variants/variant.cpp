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

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"
const uint32_t g_ADigitalPinMap[] =
    {
        // D0 .. D31
        46, // D46 is P1.14
        47, // D47 is P1.15
        2,  // D2  is P0.02  (-> AIN0)
        3,  // D3  is P0.03
        4,  // D4  is P0.04
        5,  // D5  is P0.05
        6,  // D6  is P0.06
        7,  // D7  is P0.07
        8,  // D8  is P0.08
        9,  // D9  is P0.09
        10, // D10 is P0.10
        11, // D11 is P0.11
        12, // D12 is P0.12
        13, // D13 is P0.13
        14, // D14 is P0.14
        15, // D15 is P0.15
        16, // D16 is P0.16
        17, // D17 is P0.17  (!RESET, to !RESET via this pin)
        18, // D18 is P0.18  (!RESET)
        19, // D19 is P0.19
        20, // D20 is P0.20
        21, // D21 is P0.21
        22, // D22 is P0.22
        23, // D23 is P0.23
        24, // D24 is P0.24
        25, // D25 is P0.25
        26, // D26 is P0.26
        27, // D27 is P0.27
        28, // D28 is P0.28
        29, // D29 is P0.29
        30, // D30 is P0.30
        31, // D31 is P0.31

        // D32 .. 47
        32, // D32 is P1.00
        33, // D33 is P1.01
        34, // D34 is P1.02
        35, // D35 is P1.03
        36, // D36 is P1.04
        37, // D37 is P1.05
        38, // D38 is P1.06
        39, // D39 is P1.07
        40, // D40 is P1.08
        41, // D41 is P1.09
        42, // D47 is P1.10
        43, // D43 is P1.11
        44, // D44 is P1.12
        45, // D45 is P1.13
        46, // D46 is P1.14
        47, // D47 is P1.15
};

void initVariant()
{
}
