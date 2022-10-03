/**
 * PRDC_AD7193.h - Analog Devices AD7193 ADC Library
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
 
#ifndef _PRDC_AD7193_H_
#define _PRDC_AD7193_H_

#include "Arduino.h"
#include <SPI.h>

// AD71983 Library debug
// #define DEBUG_AD7193

// SPI communication settings
#define AD7193_DEFAULT_SPI_FREQUENCY 1000000
#define AD7193_DEFAULT_SPI SPI
#define AD7193_DEFAULT_CS PIN_SPI_SS
#define AD7193_DEFAULT_MISO PIN_SPI_MISO

// AD7193 Register Map
#define AD7193_REG_COMM         0 // Communications Register (WO, 8-bit) 
#define AD7193_REG_STAT         0 // Status Register         (RO, 8-bit) 
#define AD7193_REG_MODE         1 // Mode Register           (RW, 24-bit 
#define AD7193_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD7193_REG_DATA         3 // Data Register           (RO, 24/32-bit) 
#define AD7193_REG_ID           4 // ID Register             (RO, 8-bit) 
#define AD7193_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit) 
#define AD7193_REG_OFFSET       6 // Offset Register         (RW, 24-bit 
#define AD7193_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

// Communications Register Bit Designations (AD7193_REG_COMM)
#define AD7193_COMM_WEN         (1 << 7)           // Write Enable. 
#define AD7193_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD7193_COMM_READ        (1 << 6)           // Read Operation. 
#define AD7193_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address. 
#define AD7193_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

// Status Register Bit Designations (AD7193_REG_STAT)
#define AD7193_STAT_RDY         (1 << 7) // Ready.
#define AD7193_STAT_ERR         (1 << 6) // ADC error bit.
#define AD7193_STAT_NOREF       (1 << 5) // Error no external reference. 
#define AD7193_STAT_PARITY      (1 << 4) // Parity check of the data register. 
#define AD7193_STAT_CH3         (1 << 3) // Channel 3. 
#define AD7193_STAT_CH2         (1 << 2) // Channel 2. 
#define AD7193_STAT_CH1         (1 << 1) // Channel 1. 
#define AD7193_STAT_CH0         (1 << 0) // Channel 0. 

// Mode Register Bit Designations (AD7193_REG_MODE)
#define AD7193_MODE_SEL(x)      (((x) & 0x7) << 21) // Operation Mode Select.
#define AD7193_MODE_DAT_STA     (1 << 20)           // Status Register transmission.
#define AD7193_MODE_CLKSRC(x)   (((x) & 0x3) << 18) // Clock Source Select.
#define AD7193_MODE_AVG(x)      (((x) & 0x3) << 16) // Fast settling filter.
#define AD7193_MODE_SINC3       (1 << 15)           // SINC3 Filter Select.
#define AD7193_MODE_SINC4       (0 << 15)           // SINC4 Filter Select.
#define AD7193_MODE_ENPAR       (1 << 13)           // Parity Enable.
#define AD7193_MODE_CLKDIV      (1 << 12)           // Clock divide by 2 (AD7190/2 only).
#define AD7193_MODE_SCYCLE      (1 << 11)           // Single cycle conversion (zero latency).
#define AD7193_MODE_REJ60       (1 << 10)           // 50/60Hz notch filter.
#define AD7193_MODE_NO_REJ60    (0 << 10)           // No 50/60Hz notch filter.
#define AD7193_MODE_RATE(x)     ((x) & 0x3FF)       // Filter Update Rate Select.

// Mode Register: AD7193_MODE_SEL(x) options
#define AD7193_MODE_CONT                0 // Continuous Conversion Mode.
#define AD7193_MODE_SINGLE              1 // Single Conversion Mode.
#define AD7193_MODE_IDLE                2 // Idle Mode.
#define AD7193_MODE_PWRDN               3 // Power-Down Mode.
#define AD7193_MODE_CAL_INT_ZERO        4 // Internal Zero-Scale Calibration.
#define AD7193_MODE_CAL_INT_FULL        5 // Internal Full-Scale Calibration.
#define AD7193_MODE_CAL_SYS_ZERO        6 // System Zero-Scale Calibration.
#define AD7193_MODE_CAL_SYS_FULL        7 // System Full-Scale Calibration.

// Mode Register: AD7193_MODE_CLKSRC(x) options
#define AD7193_CLK_EXT_MCLK1_2          0 // External crystal. The external crystal
                                          // is connected from MCLK1 to MCLK2.
#define AD7193_CLK_EXT_MCLK2            1 // External Clock applied to MCLK2 
#define AD7193_CLK_INT                  2 // Internal 4.92 MHz clock. 
                                          // Pin MCLK2 is tristated.
#define AD7193_CLK_INT_CO               3 // Internal 4.92 MHz clock. The internal
                                          // clock is available on MCLK2.

// Mode Register: AD7193_MODE_AVG(x) options
#define AD7193_AVG_NONE                 0 // No averaging (fast settling mode disabled).
#define AD7193_AVG_BY_2                 1 // Average by 2.
#define AD7193_AVG_BY_8                 2 // Average by 8.
#define AD7193_AVG_BY_16                3 // Average by 16.

// Configuration Register Bit Designations (AD7193_REG_CONF)
#define AD7193_CONF_CHOP        (1 << 23)            // CHOP enable.
#define AD7193_CONF_NO_CHOP     (0 << 23)            // CHOP disable.
#define AD7193_CONF_REFSEL      (1 << 20)            // REFIN2 Reference Select.
#define AD7193_CONF_REFSEL      (0 << 20)            // REFIN1 Reference Select.
#define AD7193_CONF_PSEUDO      (1 << 18)            // Pseudo differential analog inputs.
#define AD7193_CONF_DIFF        (0 << 18)            // Differential analog inputs.
#define AD7193_CONF_CHAN(x)     (((x) & 0x3FF) << 8) // Channel select.
#define AD7193_CONF_BURN        (1 << 7)             // Burnout current enable.
#define AD7193_CONF_NO_BURN     (0 << 7)             // Burnout current disable.
#define AD7193_CONF_REFDET      (1 << 6)             // Reference detect enable.
#define AD7193_CONF_BUF         (1 << 4)             // Buffered Mode Enable.
#define AD7193_CONF_NO_BUF      (0 << 4)             // Buffered Mode Disable.
#define AD7193_CONF_UNIPOLAR    (1 << 3)             // Unipolar/Bipolar Enable.
#define AD7193_CONF_GAIN(x)     ((x) & 0x7)          // Gain Select.

// Configuration Register: AD7193_CONF_CHAN(x) options
//                            Pseudo Bit = 0           Pseudo Bit = 1
#define AD7193_CH_0      0 // AIN1(+) - AIN2(-);       AIN1 - AINCOM
#define AD7193_CH_1      1 // AIN3(+) - AIN4(-);       AIN2 - AINCOM
#define AD7193_CH_2      2 // AIN5(+) - AIN6(-);       AIN3 - AINCOM
#define AD7193_CH_3      3 // AIN7(+) - AIN8(-);       AIN4 - AINCOM
#define AD7193_CH_4      4 // AIN1(+) - AIN2(-);       AIN5 - AINCOM
#define AD7193_CH_5      5 // AIN3(+) - AIN4(-);       AIN6 - AINCOM
#define AD7193_CH_6      6 // AIN5(+) - AIN6(-);       AIN7 - AINCOM
#define AD7193_CH_7      7 // AIN7(+) - AIN8(-);       AIN8 - AINCOM
#define AD7193_CH_TEMP   8 //           Temperature sensor
#define AD7193_CH_SHORT  9 // AIN2(+) - AIN2(-);       AINCOM(+) - AINCOM(-) 

// Configuration Register: AD7193_CONF_GAIN(x) options
//                                 ADC Input Range (5 V Reference)
#define AD7193_CONF_GAIN_1		0 // Gain 1    +-2.5 V
#define AD7193_CONF_GAIN_8		3 // Gain 8    +-312.5 mV
#define AD7193_CONF_GAIN_16		4 // Gain 16   +-156.2 mV
#define AD7193_CONF_GAIN_32		5 // Gain 32   +-78.125 mV
#define AD7193_CONF_GAIN_64		6 // Gain 64   +-39.06 mV
#define AD7193_CONF_GAIN_128	7 // Gain 128  +-19.53 mV

// ID Register Bit Designations (AD7193_REG_ID)
#define ID_AD7193               0x2
#define AD7193_ID_MASK          0x0F

// GPOCON Register Bit Designations (AD7193_REG_GPOCON)
#define AD7193_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD7193_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD7193_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD7193_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD7193_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD7193_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD7193_GPOCON_P0DAT     (1 << 0) // P0 state

class PRDC_AD7193 {
  public:
    PRDC_AD7193();
    
    void setSPIFrequency(uint32_t);
    void setSPI(SPIClass&);
  
    bool begin(void);
    bool begin(uint8_t, uint8_t);
    void end(void);
    void reset(void);
    void setClockMode(uint8_t);
    void setRate(uint32_t);
    void setFilter(uint32_t);
    void enableNotchFilter(bool);
    void enableChop(bool);
    void enableBuffer(bool);
    bool checkID(void);
    void waitReady(void);
    void setPower(uint8_t);
    void channelSelect(uint8_t);
    void calibrate(uint8_t, uint8_t);
    void rangeSetup(uint8_t, uint8_t);
    uint32_t singleConversion();
    uint32_t continuousReadAverage(uint32_t);
    void continuousRead(uint32_t, uint32_t*);
    float temperatureRead(void);
    float rawToVolts(uint32_t, float);
    void printAllRegisters(void);
    
  private:
    SPISettings _spiSettings;
    SPIClass* _spi;
    uint8_t _CS;
    uint8_t _MISO;
    
    uint8_t _clock_mode = AD7193_CLK_INT;
    uint32_t _rate = 0x060;
    
    uint8_t _polarity = 0;
    uint8_t _gain = AD7193_CONF_GAIN(AD7193_CONF_GAIN_1);
    uint32_t _filter = AD7193_MODE_SINC4;
    uint32_t _notch_filter = AD7193_MODE_REJ60;
    uint32_t _chop = AD7193_CONF_NO_CHOP;
    uint32_t _buf = AD7193_CONF_NO_BUF;
    uint32_t _burnout = AD7193_CONF_NO_BURN;
    uint8_t _channel = AD7193_CH_0;
    
    void pinInit(void);
    void beginTransaction(void);
    void endTransaction(void);
    uint32_t getRegister(uint8_t, uint8_t);
    uint32_t getSingleRegister(uint8_t, uint8_t);
    void setRegister(uint8_t, uint32_t, uint8_t);
    void setSingleRegister(uint8_t, uint32_t, uint8_t);
    void updateConf(void);
};
#endif // _PRDC_AD7193_H_