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
#include <iomanip>
#include <sstream>
#include "dataCache.hpp"

#define ADC_ADDR 0x48
#define CONFIG_REG 0x01
#define CONVERSION_REG 0x00

#define CFG_HIGH 0xC0
#define CFG_LOW 0x83
#define Z_THRESH 5

#define MIN_BIT 5485
#define MAX_BIT 26763
#define MIN_DIST 0.0
#define MAX_DIST 12.97

struct PencilReading
{
    int raw;
    double millimeters;
    bool flag;
};

class GT2
{
public:
    GT2(int size);
    ~GT2();
    int openI2C();
    void readRaw();
    PencilReading getLatestReading();
    int convertToMillivolts(int bits);
    int convertToMilliamps(int bits);
    double convertToMillimeters(int bits);
    std::string JSONOutput();
    
private:
    int adc;
    double FSR = 6.144;
    double min_ma = 4.0;
    double max_ma = 20.0;
    double max_mm = 12.0;
    dataCache<PencilReading> cache;
};

#endif
