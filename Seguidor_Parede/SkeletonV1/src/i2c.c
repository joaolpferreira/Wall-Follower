#include "i2c.h"


void i2c_init(void)
{
	TWBR = 0xFF;
}

void i2c_start_condition(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
}

void i2c_stop_condition(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_send_byte(unsigned char byte)
{
	TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
}

void i2c_send_packet(unsigned char value, unsigned char address)
{
	i2c_start_condition();
	i2c_send_byte(address);
	i2c_send_byte(value);
	i2c_stop_condition();
}

unsigned char i2c_recv_byte(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while ((TWCR & (1 << TWINT)) == 0);
    return TWDR;
}

unsigned char i2c_recv_last_byte(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
    return TWDR;
}