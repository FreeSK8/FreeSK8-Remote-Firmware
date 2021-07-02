#include "ADS1015.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp-i2c.h"
#include "esp_log.h"

#define ESP_INTR_FLAG_DEFAULT 0

void IRAM_ATTR ads1015_gpio_isr_handler(void* arg)
{
    // ADC Conversion is ready to be read
    xSemaphoreGiveFromISR(_semaphore, NULL);
}


uint16_t ADS1015_readRegister(uint8_t i2cAddress, uint8_t reg)
{
    uint8_t data[2];
	i2c_master_read_slave_reg(I2C_PORT_NUM, i2cAddress, reg, &data[0], 2);

	return ((uint16_t)data[0] << 8) | (data[1] & 0xFF);
}

esp_err_t ADS1015_writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value)
{
    uint16_t tosend = byte_swap(value);
	return i2c_master_write_slave_reg(I2C_PORT_NUM, i2cAddress, reg, (uint8_t*)&tosend, 2);
}


void ADS1015_init()
{
    _address = ADS1015_ADDRESS;

    gpio_config_t io_conf;
    //interrupt on falling edge
    io_conf.intr_type = (gpio_int_type_t) GPIO_PIN_INTR_NEGEDGE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO33
    io_conf.pin_bit_mask =  (1ULL << PIN_NUM_ADC_READY);
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t) 0;
    //enable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t) 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    /*
    The ALERT/RDY pin can also be configured as a conversion ready pin. Set the most-significant bit of the
    Hi_thresh register to 1 and the most-significant bit of Lo_thresh register to 0 to enable the pin as a conversion
    ready pin.
    */
    ADS1015_writeRegister(_address, ADS1015_REG_POINTER_HITHRESH, 0x8000);
    ADS1015_writeRegister(_address, ADS1015_REG_POINTER_LOWTHRESH, 0x7FFF);

    _semaphore = xSemaphoreCreateCounting(1,0);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add((gpio_num_t)PIN_NUM_ADC_READY, ads1015_gpio_isr_handler, NULL);
}


uint16_t ADS1015_readADC_SingleEnded(uint8_t channel)
{

    // Start with default values
    uint16_t config = ADS1015_REG_CONFIG_CQUE_1CONV    | // Enable comparator, 1 conv
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;    // Single-shot mode (default)
                    

    // Set PGA/voltage range
    config |= (uint16_t) GAIN_TWOTHIRDS;

    // Set single-ended input channel
    switch (channel)
    {
        case (0):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
        break;
        case (1):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
        break;
        case (2):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
        break;
        case (3):
        config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
        break;
    }

    // Set 'start single-conversion' bit
    config |= ADS1015_REG_CONFIG_OS_SINGLE;

    //printf("Pin state before: %i\n", gpio_get_level((gpio_num_t)PIN_NUM_ADC_READY));
    // Write config register to the ADC
    ADS1015_writeRegister(_address, ADS1015_REG_POINTER_CONFIG, config);

    //Will wait for fallling edge
    // Wait for the conversion to complete, read ready signal
    if (xSemaphoreTake(_semaphore, ADS1015_CONVERSIONDELAY / portTICK_RATE_MS) != pdTRUE)
    {
        ESP_LOGE(__FUNCTION__, "ERROR ADS1015::readADC_SingleEnded : could not wait for adc semaphore after interrupt.\n");
    }

    // Read the conversion results
    // Shift 12-bit results right 4 bits for the ADS1015
    return ADS1015_readRegister(_address, ADS1015_REG_POINTER_CONVERT) >> 4;
}


