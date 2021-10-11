#ifndef __DRV_INA219_H
#define __DRV_INA219_H

#include "drv_com.h"

// #define INA_ADDRESS     0x40    // 1000000 (A0=GND, A1=GND)
//#define INA_ADDRESS     0x41    // 1000001 (A0=VCC, A1=GND)
//#define INA_ADDRESS     0x44    // 1000100 (A0=GND, A1=VCC)
#define INA_ADDRESS     0x45    // 1000101 (A0=VCC, A1=VCC)


#define INA219_REG_CONFIGURATION    0x00
#define INA219_REG_SHUNT_VOLTAGE    0x01
#define INA219_REG_BUS_VOLTAGE      0x02
#define INA219_REG_POWER            0x03
#define INA219_REG_CURRENT          0x04
#define INA219_REG_CALIBRATION      0x05

/* Configuration Register */
#define CONFIG_RESET                        0x8000 // Reset Bit

#define CONFIG_BVOLTAGERANGE_MASK           0x2000
#define CONFIG_BVOLTAGERANGE_16V            0x0000 // 0-16V Range
#define CONFIG_BVOLTAGERANGE_32V            0x2000 // 0-32V Range

// gain bits
#define CONFIG_GAIN_MASK                    0x1800
#define CONFIG_GAIN_1_40MV                  0x0000 // Gain 1, 40mV Range
#define CONFIG_GAIN_2_80MV                  0x0800 // Gain 2, 80mV Range
#define CONFIG_GAIN_4_160MV                 0x1000 // Gain 4, 160mV Range
#define CONFIG_GAIN_8_320MV                 0x1800 // Gain 8, 320mV Range

// bus ADC resolution bits
#define CONFIG_BADCRES_9BIT                 0x0000 // 9-bit bus res = 0..511
#define CONFIG_BADCRES_10BIT                0x0080 // 10-bit bus res = 0..1023
#define CONFIG_BADCRES_11BIT                0x0100 // 11-bit bus res = 0..2047
#define CONFIG_BADCRES_12BIT                0x0180 // 12-bit bus res = 0..4097
#define CONFIG_BADCRES_MASK                 0x0780 // Bus ADC Resolution Mask

// shunt ADC resolution bits
#define CONFIG_SADCRES_9BIT_1S_84US         0x0000 // 1 x 9-bit shunt sample
#define CONFIG_SADCRES_10BIT_1S_148US       0x0008 // 1 x 10-bit shunt sample
#define CONFIG_SADCRES_11BIT_1S_276US       0x0010 // 1 x 11-bit shunt sample
#define CONFIG_SADCRES_12BIT_1S_532US       0x0018 // 1 x 12-bit shunt sample
#define CONFIG_SADCRES_12BIT_2S_1060US      0x0048 // 2 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_4S_2130US      0x0050 // 4 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_8S_4260US      0x0058 // 8 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_16S_8510US     0x0060 // 16 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_32S_17MS       0x0068 // 32 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_64S_34MS       0x0070 // 64 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_12BIT_128S_69MS      0x0078 // 128 x 12-bit shunt samples averaged together
#define CONFIG_SADCRES_MASK                 0x0078 // Shunt ADC Resolution and Averaging Mask

// operating mode bits
#define CONFIG_MODE_POWERDOWN               0x0000
#define CONFIG_MODE_SVOLT_TRIGGERED         0x0001
#define CONFIG_MODE_BVOLT_TRIGGERED         0x0002
#define CONFIG_MODE_SANDBVOLT_TRIGGERED     0x0003
#define CONFIG_MODE_ADCOFF                  0x0004
#define CONFIG_MODE_SVOLT_CONTINUOUS        0x0005
#define CONFIG_MODE_BVOLT_CONTINUOUS        0x0006
#define CONFIG_MODE_SANDBVOLT_CONTINUOUS    0x0007
#define CONFIG_MODE_MASK                    0x0007 // Operating Mode Mask

#ifdef __cplusplus
 extern "C" {
#endif

void    ina219_init(void);
int16_t getShuntVoltage_raw();

//void setCalibration_32V_2A(void);
//void setCalibration_16V_6A(void);
//void setCalibration_32V_1A(void);
//void setCalibration_16V_400mA(void);`
//float getBusVoltage_V(void);
//float getShuntVoltage_mV(void);
//int16_t getCurrent_mA(void);
//float getPower_mW(void);
//
//bool getCurrentAmps(float * amps);

void drv_ina219_test();
/*
 * https://github.com/mindThomas/STM32-libraries/blob/master/Drivers/INA219/INA219.h
 */

#ifdef __cplusplus
}
#endif

#endif /* ___DRV_INA219_H */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
