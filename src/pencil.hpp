#ifndef PENCIL_HPP
#define PENCIL_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <thread>
#include "dataCache.hpp"

#define ADC_ADDR 0x48
#define CONFIG_REG 0x01
#define CONVERSION_REG 0x00

#define CFG_HIGH 0xC0
#define CFG_LOW 0x83

class GT2
{
public:
    GT2(int size);
    ~GT2();
    int openI2C();
    int readVoltage(); // Return voltage in millivolts
    int getLatestVoltage();
private:
    int adc;
    double FSR = 6.144;
    dataCache<int> voltageCache;
};

#endif