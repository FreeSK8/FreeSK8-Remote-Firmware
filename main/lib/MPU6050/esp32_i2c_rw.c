/**
 * @file esp32_i2c_rw.c
 * 
 * @author
 * Gabriel Boni Vicari (133192@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 *
 * @copyright 2012 Jeff Rowberg
 *
 * @brief I2C Read/Write functions for ESP32 ESP-IDF.
 */

#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32_i2c_rw.h"

#include "esp-i2c.h"
#define I2C_NUM (I2C_NUM_0)


int8_t esp32_i2c_read_bytes
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t size,
	uint8_t* data
)
{
	if (i2c_master_read_slave_reg(I2C_NUM, device_address, register_address, data, size) == ESP_OK)
	{
		return size;
	}

	return 0;
}

int8_t esp32_i2c_read_byte
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t* data
)
{
	return (esp32_i2c_read_bytes(device_address, register_address, 1, data));
}

int8_t esp32_i2c_read_bits
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_start,
	uint8_t size,
	uint8_t* data
)
{
	uint8_t bit;
	uint8_t count;

	if ((count = esp32_i2c_read_byte(device_address, register_address, &bit))) {
		uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);

		bit &= mask;
		bit >>= (bit_start - size + 1);
		*data = bit;
	}

	return (count);
}

int8_t esp32_i2c_read_bit
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t* data
)
{
	uint8_t bit;
	uint8_t count = esp32_i2c_read_byte(device_address, register_address, &bit);

	*data = bit & (1 << bit_number);

	return (count);
}

bool esp32_i2c_write_bytes
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t size,
	uint8_t* data
)
{
	if(i2c_master_write_slave_reg(I2C_NUM, device_address, register_address, data, size) == ESP_OK)
	{
		return true;
	}

	return false;
}

bool esp32_i2c_write_byte
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t data
)
{
	if(i2c_master_write_slave_reg(I2C_NUM, device_address, register_address, &data, 1) == ESP_OK)
	{
		return true;
	}

	return false;
}

bool esp32_i2c_write_bits
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_start,
	uint8_t size,
	uint8_t data
)
{
	uint8_t bit = 0;

	if (esp32_i2c_read_byte(device_address, register_address, &bit) != 0) {
		uint8_t mask = ((1 << size) - 1) << (bit_start - size + 1);
		data <<= (bit_start - size + 1);
		data &= mask;
		bit &= ~(mask);
		bit |= data;
		return (esp32_i2c_write_byte(device_address, register_address, bit));
	}
	else
		return (false);
}

bool esp32_i2c_write_bit
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t bit_number,
	uint8_t data
)
{
	uint8_t bit;

	esp32_i2c_read_byte(device_address, register_address, &bit);

	if (data != 0)
		bit = (bit | (1 << bit_number));
	else
		bit = (bit & ~(1 << bit_number));

	return (esp32_i2c_write_byte(device_address, register_address, bit));
}

int8_t esp32_i2c_write_word
(
	uint8_t device_address,
	uint8_t register_address,
	uint8_t data
)
{
	uint8_t data_1[] = {(uint8_t) (data >> 8), (uint8_t) (data & 0xFF)};

	esp32_i2c_write_bytes(device_address, register_address, 2, data_1);

	return (1);
}
