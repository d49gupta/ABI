#include "pencil.hpp"

GT2::GT2(int size) : voltageCache(size)
{
    this->adc = openI2C();

    uint8_t config[] = {CONFIG_REG, CFG_HIGH, CFG_LOW};
    if (write(this->adc, config, 3) != 3) 
        std::cerr << "Failed to write to config register" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t conv_reg = CONV_REG_ADDR;
    if (write(this->adc, &conv_reg, 1) != 1) 
        std::cerr << "Failed to set conversion register pointer" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

GT2::~GT2()
{
    close(this->adc);
}

int GT2::openI2C()
{
    int file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) 
    {
        std::cerr << "Failed to open I2C bus\n";
        exit(1);
    }
    if (ioctl(file, I2C_SLAVE, ADC_ADDR) < 0) 
    {
        std::cerr << "Failed to connect to device at addr 0x" << std::hex << ADC_ADDR << "\n";
        exit(1);
    }
    return file;
}

void GTS::readVoltage()
{
    uint8_t data[2];
    if (read(this->adc, data, 2) != 2) 
    {
        std::cerr << "Failed to read conversion data\n";
        return -1;
    }

    int16_t raw_adc = (data[0] << 8) | data[1];
    if (raw_adc & 0x8000) 
        raw_adc -= 0x10000;

    double voltage = (raw_adc * this->FSR) / 32768.0;
    this->voltageCache.enqueue(static_cast<int>(voltage * 1000));
}

GT2::getLatestVoltage()
{
    return this->voltageCache.newestValue();
}