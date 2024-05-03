#ifndef _SENSOR_BUILDER_CLASS_H
#define _SENSOR_BUILDER_CLASS_H

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include "sensorClass.hpp"
#include <map>

#define SENSOR_BUILDER_DEF_BAUD 9600
#define SENSOR_BUILDER_DEF_SLAVE 1
#define SENSOR_BUILDER_DEF_MAC_ADDR 0x00000001
#define SENSOR_BUILDER_MAJOR 1 // max: 31
#define SENSOR_BUILDER_MINOR 0 // max: 31
#define SENSOR_BUILDER_BUILD 1 // max: 63
#define SENSOR_BUILDER_DEF_VERSION (((SENSOR_BUILDER_MAJOR << 11) & 0xF800) | ((SENSOR_BUILDER_MINOR << 6) & 0x7C0) | (SENSOR_BUILDER_BUILD & 0x3F)) // major(5bit) minor(5bit) build(6bit) - 1.0.1

#define SENSOR_BUILDER_DEF_VALUE 0x0000

class SensorBuilderClass
{
private:
    /* data */
    uint8_t _slave;
    uint32_t _baudrate;
    uint32_t _mac_addr;
    uint16_t _regs;
    bool _i2c_available = false;

    std::map<uint16_t, sensorClass *> m_sensorMap;

    enum
    {
        REG_ADDR = 0,
        REG_BAUD = 2,
        REG_VERSION = 3,
    };

public:
    SensorBuilderClass() : _regs(4){};
    ~SensorBuilderClass(){};
    uint16_t addSensor(sensorClass *sensor);
    // bool removeSensor(sensorClass *sensor);
    void check_grove(void);
    bool begin(uint8_t slave = SENSOR_BUILDER_DEF_SLAVE, uint32_t mac_addr = SENSOR_BUILDER_DEF_MAC_ADDR, uint32_t baudrate = SENSOR_BUILDER_DEF_BAUD);
    int poll();
    uint16_t size();
};

void SensorBuilderClass::check_grove()
{
    // Check if an analog type sensor is connected
    GROVE_SWITCH_ADC;
    pinMode(SENSOR_ANALOG_PIN, OUTPUT);
    digitalWrite(SENSOR_ANALOG_PIN, HIGH);
    delay(10);
    pinMode(SENSOR_ANALOG_PIN, INPUT);
    // check i2c sensor
    _i2c_available = (digitalRead(SENSOR_ANALOG_PIN) == HIGH);
    if (!_i2c_available)
    {
        _i2c_available = (analogRead(SENSOR_ANALOG_PIN) > 100 && analogRead(SENSOR_ANALOG_E_PIN) > 100);
    }
}

bool SensorBuilderClass::begin(uint8_t slave, uint32_t mac_addr, uint32_t baudrate)
{

    _slave = slave;
    _baudrate = baudrate;
    _mac_addr = mac_addr;

    // start the Modbus RTU server, with (slave) id 1
    RS485.setDelays(5000, 5000);
    if (!ModbusRTUServer.begin(_slave, _baudrate))
    {
        Serial.println("Failed to start Modbus RTU Client!");
        while (1)
            ;
    }

    Serial.print("Version: ");
    Serial.println(SENSOR_BUILDER_DEF_VERSION, HEX);

    Serial.print("regs: ");
    Serial.println(_regs);

    ModbusRTUServer.configureInputRegisters(0x00, _regs);
    ModbusRTUServer.configureHoldingRegisters(0x00, _regs);

    ModbusRTUServer.inputRegisterWrite(SensorBuilderClass::REG_ADDR, (uint16_t)(_mac_addr >> 16));
    ModbusRTUServer.inputRegisterWrite(SensorBuilderClass::REG_ADDR + 1, (uint16_t)(_mac_addr & 0x0000FFFF));
    ModbusRTUServer.holdingRegisterWrite(SensorBuilderClass::REG_ADDR, (uint16_t)(_mac_addr >> 16));
    ModbusRTUServer.holdingRegisterWrite(SensorBuilderClass::REG_ADDR + 1, (uint16_t)(_mac_addr & 0x0000FFFF));

    ModbusRTUServer.inputRegisterWrite(SensorBuilderClass::REG_BAUD, _baudrate / 100);
    ModbusRTUServer.holdingRegisterWrite(SensorBuilderClass::REG_BAUD, _baudrate / 100);

    ModbusRTUServer.inputRegisterWrite(SensorBuilderClass::REG_VERSION, SENSOR_BUILDER_DEF_VERSION);
    ModbusRTUServer.holdingRegisterWrite(SensorBuilderClass::REG_VERSION, SENSOR_BUILDER_DEF_VERSION);

    /* skip head information */
    for (uint16_t i = 4; i < _regs; i += 1)
    {
        ModbusRTUServer.inputRegisterWrite(i, (uint16_t)(SENSOR_BUILDER_DEF_VALUE >> 16));
        ModbusRTUServer.holdingRegisterWrite(i, (uint16_t)(SENSOR_BUILDER_DEF_VALUE >> 16));
    }

    return true;
}

int SensorBuilderClass::poll()
{
    for (auto iter = m_sensorMap.begin(); iter != m_sensorMap.end(); ++iter)
    {
        sensorClass *sensor = (sensorClass *)iter->second;

        if (!sensor->connected())
        {
            continue;
        }

        sensor->sample();

        auto m_measureValue = sensor->getMeasureValue();
        Serial.println(sensor->name().c_str());
        for (auto m_iter = m_measureValue.begin(); m_iter != m_measureValue.end(); ++m_iter)
        {
            Serial.print(m_iter->addr);
            Serial.print(':');
            switch (m_iter->type)
            {
            case sensorClass::regType_t::REG_TYPE_U16_AB:
                Serial.println(m_iter->value.u16);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, m_iter->value.u16);
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, m_iter->value.u16);
                break;
            case sensorClass::regType_t::REG_TYPE_S16_AB:
                Serial.println(m_iter->value.s16);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, m_iter->value.s16);
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, m_iter->value.s16);
                break;
            case sensorClass::regType_t::REG_TYPE_U32_ABCD:
                Serial.println(m_iter->value.u32);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.u32 >> 16));
                ModbusRTUServer.inputRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.u32 & 0x0000FFFF));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.u32 >> 16));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.u32 & 0x0000FFFF));
                break;
            case sensorClass::regType_t::REG_TYPE_S32_ABCD:
                Serial.println(m_iter->value.s32);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.s32 >> 16));
                ModbusRTUServer.inputRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.s32 & 0x0000FFFF));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.s32 >> 16));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.s32 & 0x0000FFFF));
                break;
            case sensorClass::regType_t::REG_TYPE_U32_CDAB:
                Serial.println(m_iter->value.u32);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.u32 & 0x0000FFFF));
                ModbusRTUServer.inputRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.u32 >> 16));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.u32 & 0x0000FFFF));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.u32 >> 16));
                break;
            case sensorClass::regType_t::REG_TYPE_S32_CDAB:
                Serial.println(m_iter->value.s32);
                ModbusRTUServer.inputRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.s32 & 0x0000FFFF));
                ModbusRTUServer.inputRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.s32 >> 16));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr, (uint16_t)(m_iter->value.s32 & 0x0000FFFF));
                ModbusRTUServer.holdingRegisterWrite(m_iter->addr + 1, (uint16_t)(m_iter->value.s32 >> 16));
                break;
            default:
                break;
            }
        }
        Serial.println();
    }
    Serial.println("================");

    return ModbusRTUServer.poll();
}

uint16_t SensorBuilderClass::size()
{
    return m_sensorMap.size();
}

uint16_t SensorBuilderClass::addSensor(sensorClass *_sensor)
{
    uint16_t regs = 0;

    for (std::map<uint16_t, sensorClass *>::const_iterator iter = m_sensorMap.begin(); iter != m_sensorMap.end(); ++iter)
    {
        if (iter->second == _sensor)
        {
            return false;
        }
    }

    m_sensorMap.insert(std::pair<uint16_t, sensorClass *>(m_sensorMap.size(), _sensor));

    regs = _sensor->init(_regs, _i2c_available);

    _regs += regs;

    return regs;
}

// bool SensorBuilderClass::removeSensor(sensorClass *_sensor)
// {
//     for (std::map<uint16_t, sensorClass *>::iterator iter = m_sensorMap.begin(); iter != m_sensorMap.end(); ++iter)
//     {
//         if (iter->second == _sensor)
//         {
//             iter = m_sensorMap.erase(iter);
//             return true;
//         }
//     }

//     return false;
// }

#endif
