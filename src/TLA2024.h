#pragma once

#include <Arduino.h>
#include <Wire.h>

class TLA2024 {
public:

    enum OperatingMode {
        OP_CONTINUOUS = 0,
        OP_SINGLE = 1
    };

    enum DataRate {
        DR_128SPS = 0x0,
        DR_250SPS = 0x1,
        DR_490SPS = 0x2,
        DR_920SPS = 0x3,
        DR_1600SPS = 0x4,
        DR_2400SPS = 0x5,
        DR_3300SPS = 0x6,
    };

    enum FullScaleRange {
        FSR_6_144V = 0x0,
        FSR_4_096V = 0x1,
        FSR_2_048V = 0x2,
        FSR_1_024V = 0x3,
        FSR_0_512V = 0x4,
        FSR_0_256V = 0x5,
    };

    enum MultiplexerConfig {
        MUX_AIN0_AIN1 = 0x0,
        MUX_AIN0_AIN3 = 0x1,
        MUX_AIN1_AIN3 = 0x2,
        MUX_AIN2_AIN3 = 0x3,
        MUX_AIN0_GND = 0x4,
        MUX_AIN1_GND = 0x5,
        MUX_AIN2_GND = 0x6,
        MUX_AIN3_GND = 0x7,
    };

    TLA2024();
    TLA2024(TwoWire *devI2C);

    /*
      Initializes I2C bus
      returns:
         true - adc responds with correct default conf
         false - otherwise
    */
    bool begin();
    bool begin(uint8_t address);

    // Resets device to default configuration
    void reset();

    // Restores device to last config
    void restore();

    /*
      Get analog reading for current multiplexer setting
    */
    float analogRead();

    /*
      Convenient method to get analog reading of a channel compared to GND
    */
    float analogRead(uint8_t channel);

    /*
      Sets the Full Scale Range of the ADC
    */
    void setFullScaleRange(FullScaleRange range);

    /*
      Set multiplexer configuration
    */
    void setMuxConfig(MultiplexerConfig option);

    /*
      Continous conversion or single shot
    */
    void setOperatingMode(OperatingMode mode);

    /*
      Set data rate setting
    */
    void setDataRate(DataRate rate);

    /*
      Convenient method to read actual voltage (in volt) respecting the full scale range
    */
    float voltageRead(uint8_t channel);

    float getCurrentFullRangeVoltage();

private:

    TwoWire *devI2C_ = NULL;
    uint8_t addr = 0x49;
    uint8_t convReg_ = 0x00;
    uint8_t confReg_ = 0x01;

    // this is default conf.
    uint16_t initConf_ = 0x8583;
    uint16_t savedConf_ = 0x8583;
    OperatingMode currentMode_ = OP_SINGLE;
    FullScaleRange currentFSR_val_ = FSR_2_048V;

    union I2C_data {
        uint8_t packet[2];
        uint16_t value;
    } data;

    /*
      A generic read from mem_addr.
    */
    uint16_t read(uint8_t mem_addr);

    /*
      We only write to the configuration register.
      out_data is the 16 bits of conf_regs

      Should always return 2

      Saves data to current_conf
    */
    int write(uint16_t data);
};

