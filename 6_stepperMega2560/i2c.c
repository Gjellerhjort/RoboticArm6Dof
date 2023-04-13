#define SCL_PIN PD0 // SCL pin of I2C interface
#define SDA_PIN PD1 // SDA pin of I2C interface

#define PCF8572_ADDR 0x20

// Define the AS5600 sensor I2C address
#define AS5600_SENSOR_ADDR 0x36

// Define the AS5600 sensor command for reading the angle measurement
#define AS5600_ANGLE_CMD 0x0C

void i2c_init() { //set up the clock speed for the I2C/TWI
  DDRD |= (1 << SCL_PIN) | (1 << SDA_PIN); // Set SCL and SDA pins as outputs
  PORTD |= (1 << SCL_PIN) | (1 << SDA_PIN); // Set SCL and SDA pins high
  TWSR = 0x00;
  TWBR = (F_CPU / 100000UL - 16) / 2;
}

void i2c_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Send start condition
    while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag to be set
}

void i2c_stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // Send stop condition
    while (TWCR & (1 << TWSTO)); // Wait for TWSTO flag to be cleared
}

void i2c_write(uint8_t data) {
    TWDR = data; // Load data into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag to be set
}

uint8_t i2c_read(uint8_t ack) {
    if (ack) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
    } else {
        TWCR = (1 << TWINT) | (1 << TWEN); // Read byte and send NACK
    }
    while (!(TWCR & (1 << TWINT))); // Wait for TWINT flag to be set
    return TWDR; // Return received data
}

void PCF8572_init()
{
    i2c_start(); // Send start condition
    i2c_write(PCF8572_ADDR << 1 | 0x00); // Send slave address with write bit
    i2c_write(0xFF); // Set all pins on the IO expander to inputs
    i2c_stop(); // Send stop condition
}

uint8_t PCF8572_read()
{
    uint8_t data;
    i2c_start(); // Send start condition
    i2c_write(SLAVE_ADDRESS << 1 | 0x01); // Send slave address with read bit
    data = i2c_read(0); // Read the data from the IO expander with NACK
    i2c_stop(); // Send stop condition
    return data;
}

void TCA9548A_selectBUS(uint8_t BUS)
{
    i2c_start();
    i2c_write((BUS<<1));
    i2c_read();
    i2c_stop();
}

uint16_t AS5600_read()
{
    uint16_t angle;
    i2c_start(); // Send start condition
    i2c_write(SLAVE_ADDRESS << 1 | 0x00); // Send slave address with write bit
    i2c_write(ANGLE_REG_ADDRESS); // Send address of the angle register
    i2c_stop(); // Send stop condition
    i2c_start(); // Send start condition
    i2c_write(SLAVE_ADDRESS << 1 | 0x01); // Send slave address with read bit
    data = i2c_read(0); // Read the data from the IO expander with NACK
    i2c_stop(); // Send stop condition
}