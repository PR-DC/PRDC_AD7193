/**
 * test_AD7193.ino - Test AD7193
 * Author: Milos Petrasinovic <mpetrasinovic@pr-dc.com>
 * PR-DC, Republic of Serbia
 * info@pr-dc.com
 * 
 * --------------------
 * Copyright (C) 2021 PR-DC <info@pr-dc.com>
 *
 * This file is part of PRDC_AD7193.
 *
 * PRDC_AD7193 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * PRDC_AD7193 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PRDC_AD7193.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


// Library
// --------------------
// PRDC AD7193
// Author: PRDC
#include <PRDC_AD7193.h>

// Define variables
// --------------------

// SPI communications
PRDC_AD7193 AD7193;

// setup function
// --------------------
void setup() { 
  // Communication settings
  Serial.begin(1000000);
  AD7193.setSPI(SPI);

  if(!AD7193.begin(PIN_SPI_SS, PIN_SPI_MISO)) {
    Serial.println(F("AD7193 initialization failed!"));
  }
}

// loop function
// --------------------
void loop(){
  Serial.println(AD7193.checkID());
  delay(500);
}