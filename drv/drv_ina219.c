#include "drv_ina219.h"
#include "drv_i2c.h"

#include "drv_delay.h"

void setCalibration_32V_2A(void);

void ina219_init(void)
{
    i2c_init();
    setCalibration_32V_2A();
}

/**************************************************************************/
/*!
    @brief  Configures to INA219 to be able to measure up to 32V and 2A
            of current.  Each unit of current corresponds to 100uA, and
            each unit of power corresponds to 2mW. Counter overflow
            occurs at 3.2A.
    @note   These calculations assume a 0.1 ohm resistor is present
*/
/**************************************************************************/
void setCalibration_32V_2A(void)
{
	uint16_t  _calValue = 0;
	uint16_t  _currentDivider_mA = 0;
	uint16_t  _powerMultiplier_mW = 0;

    // By default we use a pretty huge range for the input voltage,
    // which probably isn't the most appropriate choice for system
    // that don't use a lot of power.  But all of the calculations
    // are shown below if you want to change the settings.  You will
    // also need to change any relevant register settings, such as
    // setting the VBUS_MAX to 16V instead of 32V, etc.

    // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
    // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
    // RSHUNT = 0.1               (Resistor value in ohms)

    // 1. Determine max possible current
    // MaxPossible_I = VSHUNT_MAX / RSHUNT
    // MaxPossible_I = 3.2A

    // 2. Determine max expected current
    // MaxExpected_I = 2.0A

    // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
    // MinimumLSB = MaxExpected_I/32767
    // MinimumLSB = 0.000061              (61uA per bit)
    // MaximumLSB = MaxExpected_I/4096
    // MaximumLSB = 0,000488              (488uA per bit)

    // 4. Choose an LSB between the min and max values
    //    (Preferrably a roundish number close to MinLSB)
    // CurrentLSB = 0.0001 (100uA per bit)

    // 5. Compute the calibration register
    // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
    // Cal = 4096 (0x1000)


    _calValue = 4096;

    // 6. Calculate the power LSB
    // PowerLSB = 20 * CurrentLSB
    // PowerLSB = 0.002 (2mW per bit)

    // 7. Compute the maximum current and shunt voltage values before overflow
    //
    // Max_Current = Current_LSB * 32767
    // Max_Current = 3.2767A before overflow
    //
    // If Max_Current > Max_Possible_I then
    //    Max_Current_Before_Overflow = MaxPossible_I
    // Else
    //    Max_Current_Before_Overflow = Max_Current
    // End If
    //
    // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
    // Max_ShuntVoltage = 0.32V
    //
    // If Max_ShuntVoltage >= VSHUNT_MAX
    //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
    // Else
    //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
    // End If

    // 8. Compute the Maximum Power
    // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
    // MaximumPower = 3.2 * 32V
    // MaximumPower = 102.4W

    // Set multipliers to convert raw current/power values
    _currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
    _powerMultiplier_mW = 2;     // Power LSB = 1mW per bit (2/1)

    // Set Calibration register to 'Cal' calculated above
    i2c_write_register( INA219_REG_CALIBRATION, _calValue, sizeof(_calValue));

    // Set Config register to take into account the settings above
    uint16_t config = CONFIG_BVOLTAGERANGE_16V |
                    CONFIG_GAIN_8_320MV |
                    CONFIG_BADCRES_12BIT |
                    CONFIG_SADCRES_12BIT_1S_532US |
                    CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    i2c_write_register( INA219_REG_CONFIGURATION, config, sizeof(config));
}


/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
    @return the raw bus voltage reading
*/
/**************************************************************************/
int16_t getBusVoltage_raw() {
    uint8_t value[2] = {0};
    uint16_t raw_data = 0;

    i2c_read_registers(INA219_REG_BUS_VOLTAGE, &value[0], 2);

    raw_data = value[0] << 8 | value[1];

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((raw_data >> 3) * 4);
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
    @return the raw shunt voltage reading
*/
/**************************************************************************/
int16_t getShuntVoltage_raw()
{
	uint8_t value[2] = {0};

	/*TODO: read status register */
    i2c_read_registers(INA219_REG_SHUNT_VOLTAGE, &value[0], 2);

    return (int16_t) ( (value[0] << 8 | value[1]) / 10.0 );
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
    @return the bus voltage converted to volts
*/
/**************************************************************************/
float getBusVoltage_V() {
    int16_t value = getBusVoltage_raw();
    return value * 0.001;
}

static float vbus = 0;
void drv_ina219_test() {
	vbus = getBusVoltage_V();
    delay_ms(10);
}
/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
