/*
    I2C driver for the ESP8266 
    Copyright (C) 2014 Rudy Hardeman (zarya) 

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"

#include "i2c.h"

void ICACHE_FLASH_ATTR
i2c_sda(uint8 state)
{
    // os_printf("SDA %d\n", state);
    if (state)
        gpio_output_set(1 << I2C_SDA_PIN, 0, 1 << I2C_SDA_PIN, 0);
    else
        gpio_output_set(0, 1 << I2C_SDA_PIN, 1 << I2C_SDA_PIN, 0);
}

void ICACHE_FLASH_ATTR
i2c_sck(uint8 state)
{
    // os_printf("SCK %d\n", state);
    if (state)
        gpio_output_set(1 << I2C_SCK_PIN, 0, 1 << I2C_SCK_PIN, 0);
    else
        gpio_output_set(0, 1 << I2C_SCK_PIN, 1 << I2C_SCK_PIN, 0);
}

void ICACHE_FLASH_ATTR
i2c_init(void)
{
    uint8 i;

    ETS_GPIO_INTR_DISABLE();

    PIN_FUNC_SELECT(I2C_SDA_MUX, I2C_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_SCK_MUX, I2C_SCK_FUNC);

    GPIO_REG_WRITE(
        GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SDA_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SDA_PIN))) |
            GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );

    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_SDA_PIN));

    GPIO_REG_WRITE(
        GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SCK_PIN)),
        GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_SCK_PIN))) |
            GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)
    );

    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_SCK_PIN));

    ETS_GPIO_INTR_ENABLE();

    i2c_sda(1);
    i2c_sck(1);

    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(1);
    os_delay_us(I2C_SLEEP_TIME);

    for (i = 0; i < 28; i++) {
        i2c_sck(0);
        os_delay_us(I2C_SLEEP_TIME);
        i2c_sck(1);
        os_delay_us(I2C_SLEEP_TIME);
    }

    return;
}

void ICACHE_FLASH_ATTR
i2c_start(void)
{
    i2c_sda(1);
    i2c_sck(1);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);
}

void ICACHE_FLASH_ATTR
i2c_stop(void)
{
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(1);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(1);
    os_delay_us(I2C_SLEEP_TIME);
}

void ICACHE_FLASH_ATTR
i2c_send_ack(uint8 state)
{
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);

    i2c_sda((state?0:1));

    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(1);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);

    i2c_sda(1);
    os_delay_us(I2C_SLEEP_TIME);
}

uint8 ICACHE_FLASH_ATTR
i2c_check_ack(void)
{
    uint8 ack, k;

    // os_printf("CHECK ACK BEGIN\n");

    i2c_sda(1);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(1);
    os_delay_us(I2C_SLEEP_TIME);

    for (k = 0; k < 10; k++) {
        if (i2c_read_sck())
            break;
        os_delay_us(I2C_SLEEP_TIME);
        // os_printf("SLAVE HOLDING SCL LOW\n");
    }

    ack = i2c_read();
    // os_printf("SDA READ %d\n", ack);

    os_delay_us(I2C_SLEEP_TIME);
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);
    i2c_sda(0);
    os_delay_us(I2C_SLEEP_TIME);

    // os_printf("CHECK ACK END %d\n", ack?0:1);

    return (ack?0:1);
}

uint8 ICACHE_FLASH_ATTR
i2c_read_byte(void)
{
    uint8 data = 0;
    uint8 data_bit;
    uint8 i;

    // os_printf("READ BEGIN\n");

    i2c_sda(1);

    for (i = 0; i < 8; i++)
    {
        os_delay_us(I2C_SLEEP_TIME);
        i2c_sck(0);
        os_delay_us(I2C_SLEEP_TIME);

        i2c_sck(1);
        os_delay_us(I2C_SLEEP_TIME);

        data_bit = i2c_read();
        os_delay_us(I2C_SLEEP_TIME);

        data_bit <<= (7 - i);
        data |= data_bit;
    }
    i2c_sck(0);
    os_delay_us(I2C_SLEEP_TIME);

    // os_printf("READ END %d\n", data);

    return data;
}

void ICACHE_FLASH_ATTR
i2c_write_byte(uint8 data)
{
    sint8 i;

    // os_printf("WRITE BEGIN %u\n", data);

    os_delay_us(I2C_SLEEP_TIME);

    for (i = 7; i >= 0; i--) {
        i2c_sda((data >> i) & 1);
        os_delay_us(I2C_SLEEP_TIME);
        i2c_sck(1);
        os_delay_us(I2C_SLEEP_TIME);
        i2c_sck(0);
        os_delay_us(I2C_SLEEP_TIME);
    }

    // os_printf("WRITE END\n");
}