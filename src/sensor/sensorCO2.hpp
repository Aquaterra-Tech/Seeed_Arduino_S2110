#ifndef _SENSOR_CO2_H
#define _SENSOR_CO2_H
#include "sensorClass.hpp"
#include <SensirionI2CScd4x.h>
#include <Wire.h>

class sensorCO2 : public sensorClass
{
public:
    sensorCO2() : sensorClass("CO2"){};
    ~sensorCO2(){};

    uint16_t init(uint16_t reg);
    bool connected();
    bool sample();

    enum
    {
        CO2,
        HUMIDITY,
        TEMPERATURE,
        MAX = 0
    };

private:
    SensirionI2CScd4x _scd4x; // IIC
};

uint16_t sensorCO2::init(uint16_t reg)
{
    uint16_t t_reg = reg;
    for (uint16_t i = 0; i < sensorCO2::MAX; i++)
    {
        sensorClass::reg_t value;
        value.addr = t_reg;
        value.type = sensorClass::regType_t::REG_TYPE_S32_ABCD;
        value.value.s32 = 0;
        m_valueVector.emplace_back(value);
        t_reg += sensorClass::valueLength(value.type);
    }

    GROVE_SWITCH_IIC;

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    uint32_t timeout = 5000;

    Wire.begin();

    _scd4x.begin(Wire);

    if (_scd4x.stopPeriodicMeasurement())
    {
        _connected = false;
        return false;
    }

    if (_scd4x.getSerialNumber(serial0, serial1, serial2))
    {
        _connected = false;
        return false;
    }

    if (_scd4x.startPeriodicMeasurement())
    {
        _connected = false;
        return false;
    }

    _connected = true;

    return t_reg - reg;
}

bool sensorCO2::sample()
{
    GROVE_SWITCH_IIC;
    uint16_t error = 0;
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;

    error = _scd4x.readMeasurement(co2, temperature, humidity);

    if (error == 0)
    {
        m_valueVector[CO2].value.s32 = co2 * SCALE;
        m_valueVector[HUMIDITY].value.s32 = humidity * SCALE;
        m_valueVector[TEMPERATURE].value.s32 = temperature * SCALE;
    }

    return true;
}

bool sensorCO2::connected()
{
    return _connected;
}

#endif