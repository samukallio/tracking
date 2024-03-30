#include "ets_sys.h"
#include "osapi.h"
#include "os_type.h"
#include "gpio.h"

#include "user_interface.h"

#include "i2c.h"

static os_timer_t mpu9250_timer;

const uint8 MPU9250_ADDR_READ = (0x68 << 1) | 1;
const uint8 MPU9250_ADDR_WRITE = (0x68 << 1) | 0;

const uint8 AK8963_ADDR_READ = (0x0C << 1) | 1;
const uint8 AK8963_ADDR_WRITE = (0x0C << 1) | 0;

uint8 mag_adj_x, mag_adj_y, mag_adj_z;

struct vec
{
    sint16 x, y, z;
};

void mpu9250_read_sensors(sint16 *temp, struct vec *acc, struct vec *rot, struct vec *mag)
{
    uint8 ack;
    uint16 reg;

    i2c_start();

    i2c_write_byte(MPU9250_ADDR_WRITE);
    ack = i2c_check_ack();
    i2c_write_byte(0x3B);
    ack = i2c_check_ack();
    i2c_start();
    i2c_write_byte(MPU9250_ADDR_READ);
    ack = i2c_check_ack();

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    acc->x = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    acc->y = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    acc->z = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    *temp = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    rot->x = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(1);
    rot->y = *(sint16*)&reg;

    reg = i2c_read_byte() << 8;
    i2c_send_ack(1);
    reg |= i2c_read_byte();
    i2c_send_ack(0);
    rot->z = *(sint16*)&reg;

    i2c_stop();

    // Read AK8963 magnetometer registers.
    i2c_start();

    i2c_write_byte(AK8963_ADDR_WRITE);
    ack = i2c_check_ack();
    i2c_write_byte(0x03);
    ack = i2c_check_ack();
    i2c_start();
    i2c_write_byte(AK8963_ADDR_READ);
    ack = i2c_check_ack();

    reg = i2c_read_byte();
    i2c_send_ack(1);
    reg |= i2c_read_byte() << 8;
    i2c_send_ack(1);
    mag->x = *(sint16*)&reg;

    reg = i2c_read_byte();
    i2c_send_ack(1);
    reg |= i2c_read_byte() << 8;
    i2c_send_ack(1);
    mag->y = *(sint16*)&reg;

    reg = i2c_read_byte();
    i2c_send_ack(1);
    reg |= i2c_read_byte() << 8;
    i2c_send_ack(0);
    mag->z = *(sint16*)&reg;

    i2c_stop();

    mag->x = ((((sint32)mag->x) * ((sint32)mag_adj_x - 128)) / 256) + ((sint32)mag->x);
    mag->y = ((((sint32)mag->y) * ((sint32)mag_adj_y - 128)) / 256) + ((sint32)mag->y);
    mag->z = ((((sint32)mag->z) * ((sint32)mag_adj_z - 128)) / 256) + ((sint32)mag->z);

    // Acknowledge AK8963 measurement.
    i2c_start();
    i2c_write_byte(AK8963_ADDR_WRITE);
    ack = i2c_check_ack();
    i2c_write_byte(0x09);
    ack = i2c_check_ack();
    i2c_start();
    i2c_write_byte(AK8963_ADDR_READ);
    ack = i2c_check_ack();
    i2c_read_byte();
    i2c_send_ack(0);
    i2c_stop();
}

void mpu9250_timer_func(void *arg)
{
    sint16 temp;
    struct vec acc, rot, mag;

    mpu9250_read_sensors(&temp, &acc, &rot, &mag);

    // os_printf("ACC %d %d %d\n", acc.x, acc.y, acc.z);
    // os_printf("MAG %d %d %d\n", mag.x, mag.y, mag.z);
    // os_printf("GYR %d %d %d\n", rot.x, rot.y, rot.z);

    os_printf("%d %d %d %d %d %d %d %d %d\n",
        acc.x, acc.y, acc.z,
        mag.x, mag.y, mag.z,
        rot.x, rot.y, rot.z);
}

void mpu9250_write(uint8 device, uint8 addr, uint8 data)
{
    uint8 ack;

    i2c_start();
    i2c_write_byte(device << 1);
    ack = i2c_check_ack();
    i2c_write_byte(addr);
    ack = i2c_check_ack();
    i2c_write_byte(data);
    ack = i2c_check_ack();
    i2c_stop();
}

void mpu9250_init()
{
    uint8 ack;

    uint8 i;

    // Power management.
    mpu9250_write(0x68, 0x6B, 0x00);
    mpu9250_write(0x68, 0x6C, 0x00);

    // Gyroscope scale +1000 dps, no low-pass filter.
    mpu9250_write(0x68, 0x1A, 0x00);
    mpu9250_write(0x68, 0x1B, 0x13); /* 0x13 = +-1000dps, no DLPF */

    // Accelerometer scale +-4g, no low-pass filter.
    mpu9250_write(0x68, 0x1C, 0x08); /* 0x08 = +-4g */
    mpu9250_write(0x68, 0x1D, 0x08); /* 0x02 = DLPF 92Hz, 0x08 = No DLPF */

    // I2C bypass enable.
    mpu9250_write(0x68, 0x37, 0x02);

    // Compass - continuous measurement mode at 100Hz.
    mpu9250_write(0x0C, 0x0A, 0x16);

    // Read sensitivity adjustment values.
    i2c_start();
    i2c_write_byte(AK8963_ADDR_WRITE);
    ack = i2c_check_ack();
    i2c_write_byte(0x10);
    ack = i2c_check_ack();
    i2c_start();
    i2c_write_byte(AK8963_ADDR_READ);
    ack = i2c_check_ack();
    mag_adj_x = i2c_read_byte();
    i2c_send_ack(1);
    mag_adj_y = i2c_read_byte();
    i2c_send_ack(1);
    mag_adj_z = i2c_read_byte();
    i2c_send_ack(0);
    i2c_stop();

    os_delay_us(50000);

    os_printf("MPU9250 init done\n");
}

void user_init()
{
    os_timer_disarm(&mpu9250_timer);
    os_timer_setfn(&mpu9250_timer, mpu9250_timer_func, NULL);
    os_timer_arm(&mpu9250_timer, 20, 1);

    uart_div_modify(0, UART_CLK_FREQ / 115200);
    wifi_set_opmode(NULL_MODE);

    i2c_init();

    os_printf("\n\n\n\n\n");
    os_delay_us(50000);

    mpu9250_init();
}
