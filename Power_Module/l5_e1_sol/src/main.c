#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#define ICM42688_REG_PWR_MGMT_0     0x1F  // Power Management
#define ICM42688_REG_GYRO_CONFIG0   0x4F  // Gyro Configuration

#define ICM42688_REG_GYRO_Y_OUT_H   0x27  // Gyro Y-axis High Byte
#define ICM42688_REG_GYRO_Y_OUT_L   0x28  // Gyro Y-axis Low Byte
#define ICM42688_REG_RESET          0x4B  // Device Reset
#define ICM42688_RESET_VALUE        0x01  // Reset value
#define ICM42688_REG_WHO_AM_I       0x75  // WHO_AM_I register
#define ICM42688_WHO_AM_I_VALUE     0x47  // Expected WHO_AM_I value for ICM42688

#define ICM42688_GYRO_LOW_NOISE_MODE 0x0F // Enable Low-Noise Mode for Gyro
#define ICM42688_GYRO_1000DPS        0x02 // 1000 dps FSR

// Get SPI device from devicetree
static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

static struct spi_config spi_cfg = {
    .frequency = 1000000,  // 1MHz SPI speed
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0
};

static int icm42688_write(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[2] = {reg, value};
    struct spi_buf tx = {.buf = tx_buf, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};

    return spi_write(spi_dev, &spi_cfg, &tx_set);
}

static int icm42688_read(uint8_t reg, uint8_t *value) {
    uint8_t tx_buf[2] = {reg | 0x80, 0x00}; // Read operation (MSB = 1)
    uint8_t rx_buf[2] = {0};

    struct spi_buf tx = {.buf = tx_buf, .len = 2};
    struct spi_buf rx = {.buf = rx_buf, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret == 0) {
        *value = rx_buf[1]; // Read data is in the second byte
    }
    return ret;
}

static int icm42688_verify_comms(void) {
    uint8_t who_am_i = 0;
    int ret;

    ret = icm42688_read(ICM42688_REG_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        printk("Failed to read WHO_AM_I register\n");
        return ret;
    }

    if (who_am_i != ICM42688_WHO_AM_I_VALUE) {
        printk("Unexpected WHO_AM_I value: 0x%02X\n", who_am_i);
        return -EIO;
    }

    printk("ICM42688 communication verified, WHO_AM_I = 0x%02X\n", who_am_i);
    return 0;
}

static int icm42688_init(void) {
    int ret;

    // Reset the sensor
    ret = icm42688_write(ICM42688_REG_RESET, ICM42688_RESET_VALUE);
    if (ret < 0) {
        printk("Failed to reset ICM42688\n");
        return ret;
    }

    k_sleep(K_MSEC(10)); // Wait for the sensor to reset

    // Power up the sensor
    ret = icm42688_write(ICM42688_REG_PWR_MGMT_0, ICM42688_GYRO_LOW_NOISE_MODE);
    if (ret < 0) {
        printk("Failed to power up ICM42688\n");
        return ret;
    }

    // Configure the gyroscope
    ret = icm42688_write(ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_1000DPS);
    if (ret < 0) {
        printk("Failed to configure ICM42688 gyroscope\n");
        return ret;
    }

    return 0;
}

void main(void) {
    int ret;

    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready!\n");
        //return;
    }

    printk("Initializing ICM-42688-P...\n");

    ret = icm42688_init();
    if (ret >= 0) {
        printk("ICM42688 initialization success\n");
        //return;
    } else{
        printk("ICM42688 initialization failed\n");
    }


    ret = icm42688_verify_comms();
    if (ret >= 0) {
        printk("ICM42688 communication verification success\n");
        return;
    } else{
        printk("ICM42688 communication verification failed\n");
    }



    while (1) {



        uint8_t gyro_y_h, gyro_y_l;
        int16_t gyro_y;

        icm42688_read(ICM42688_REG_GYRO_Y_OUT_H, &gyro_y_h);
        icm42688_read(ICM42688_REG_GYRO_Y_OUT_L, &gyro_y_l);

        // Combine high and low bytes
        gyro_y = ((int16_t)gyro_y_h << 8) | gyro_y_l;

        //printk("Gyro Y-axis: %d\n", gyro_y);
        k_msleep(100);
    }
}
