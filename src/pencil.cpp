#include "pencil.hpp"

// Opoens I2C bus and configs the ADC chip
GT2::GT2(int size) : cache(size)
{
    this->adc = openI2C();

    uint8_t config[] = {CONFIG_REG, CFG_HIGH, CFG_LOW};
    if (write(this->adc, config, 3) != 3) 
        std::cerr << "Failed to write to config register" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t conv_reg = CONVERSION_REG;
    if (write(this->adc, &conv_reg, 1) != 1) 
        std::cerr << "Failed to set conversion register pointer" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// Closes I2C file descriptor
GT2::~GT2()
{
    close(this->adc);
}

// Sts I2C slave address and returns file descriptor
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

// Reads 2 bytes from ADC, reconstructs signed 16-bit int, converts to mm, 
// sets a contact flag if distance exceeds z-threshold
void GT2::readRaw()
{
    PencilReading reading;
    uint8_t data[2];
    if (read(this->adc, data, 2) != 2) 
    {
        std::cerr << "Failed to read conversion data\n";
    }

    int16_t raw_adc = (data[0] << 8) | data[1];
    if (raw_adc & 0x8000) 
        raw_adc -= 0x10000;


    reading.raw = static_cast<int>(raw_adc);
    reading.millimeters = convertToMillimeters(raw_adc);
    reading.flag = !(std::abs(reading.millimeters) < Z_THRESH);
    this->cache.enqueue(reading);
}

// converts raw ADC bits to millivolts
int GT2::convertToMillivolts(int bits)
{
    double voltage = (bits * this->FSR) / 32768.0;
    return static_cast<int>(voltage * 1000);
}

// converts raw ADC bits to milliamps
int GT2::convertToMilliamps(int bits)
{
    double current = ((bits) / 32767.0) * (this->max_ma - this->min_ma) + this->min_ma;
    return static_cast<int>(current);
}

// Linear interpolation mapping ADC bits to physical displacements
double GT2::convertToMillimeters(int bits)
{
    double distance = (double)(bits - MIN_BIT) * (MAX_DIST - MIN_DIST) / (MAX_BIT - MIN_BIT);    
    return distance;
}

// Returns most recent PencilReading from cache
PencilReading GT2::getLatestReading()
{
    return this->cache.newestValue();
}

// Serialize the latest reading into JSON string with raw displacement (mm), and flag fields.
// return is what is published to pecil/reading MQTT topic
std::string GT2::JSONOutput()
{
    std::stringstream ss;
    PencilReading reading = getLatestReading();
    ss << "{"
       << "\"raw\": " << reading.raw << ", "
       << "\"millimeters\": " << std::fixed << std::setprecision(4) << reading.millimeters << ", "
       << "\"flag\": " << std::boolalpha << reading.flag
       << "}";
    std::cout<<reading.millimeters<<std::endl;

    return ss.str();
}
