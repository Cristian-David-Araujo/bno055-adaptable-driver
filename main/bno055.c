/**
 * @file bno055.c
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bno055.h"

void BNO055_Init(BNO055_t *bno055, uint8_t gpio_tx, uint8_t gpio_rx)
{
    bno055->operation_mode = CONFIGMODE;

    // Initialize the UART
    uart_init(&bno055->uart_config, 115200, 1024, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);

    uint8_t data = BNO055_PAGE_ZERO;

    // Write the default page as zero
    BN055_Write(bno055, BNO055_PAGE_ID_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
    // bno055->page_id = BNO055_PAGE_ZERO;

    // Read the chip ID
    BNO055_Read(bno055, BNO055_CHIP_ID_ADDR, &bno055->chip_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read accelerometer revision ID
    BNO055_Read(bno055, BNO055_ACCEL_REV_ID_ADDR, &bno055->accel_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read magnetometer revision ID
    BNO055_Read(bno055, BNO055_MAG_REV_ID_ADDR, &bno055->mag_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read gyroscope revision ID
    BNO055_Read(bno055, BNO055_GYRO_REV_ID_ADDR, &bno055->gyro_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read bootloader revision ID
    BNO055_Read(bno055, BNO055_BL_REV_ID_ADDR, &bno055->bl_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read software revision ID
    BNO055_Read(bno055, BNO055_SW_REV_ID_LSB_ADDR, &bno055->sw_rev_id, BNO055_LSB_MSB_READ_LENGTH);

    // Read page ID
    BNO055_Read(bno055, BNO055_PAGE_ID_ADDR, &bno055->page_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Print the data read from the BNO055 sensor
    printf("\n----------------- BNO055 Sensor Data -----------------\n");
    printf("Chip ID: %02X\n", bno055->chip_id);
    printf("Accelerometer revision ID: %02X\n", bno055->accel_rev_id);
    printf("Magnetometer revision ID: %02X\n", bno055->mag_rev_id);
    printf("Gyroscope revision ID: %02X\n", bno055->gyro_rev_id);
    printf("Bootloader revision ID: %02X\n", bno055->bl_rev_id);
    printf("Software revision ID: %02X%02X\n", bno055->sw_rev_id[0], bno055->sw_rev_id[1]);
    printf("Page ID: %02X\n", bno055->page_id);
 
}

int8_t BN055_Write(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (len + 4 > 128) { // Check for buffer overflow
        printf("Error: Data length exceeds buffer size\n");
        return -2; // Error: Buffer overflow
    }

    printf("Write operation\n");
    // The structure for write data is as follows:
    // | Start Byte | Opertation | Register | Length | Data 1 | Data 2 | ... | Data N |

    uint8_t buffer[128];
    buffer[0] = 0xAA; // Start byte
    buffer[1] = 0x00; // Write operation
    buffer[2] = reg;  // Register
    buffer[3] = len;  // Length

    // Copy data into the buffer manually
    for (uint8_t i = 0; i < len; i++) {
        buffer[4 + i] = data[i];
    }

    // Set the last position of the buffer to NULL
    buffer[len + 4] = NULL;

    // Print the buffer
    printf("Buffer: ");
    for (uint8_t i = 0; i < len + 4; i++) {
        printf("%02X ", buffer[i]);
    }

    uart_write(&bno055->uart_config, buffer, len + 4);

    if (BNO055_CheckAck(bno055) == BNO055_SUCCESS)
    {
        printf("Write operation successful\n");
        return BNO055_SUCCESS;
    } else {
        printf("Write operation failed\n");
    }

    return BNO055_ERROR;


}

int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (len + 4 > 128) { // Check for buffer overflow
        printf("Error: Data length exceeds buffer size\n");
        return -2; // Error: Buffer overflow
    }

    // The structure for read data is as follows:
    // | Start Byte | Opertation | Register | Length |

    uint8_t buffer_tx[4];
    buffer_tx[0] = 0xAA; // Start byte
    buffer_tx[1] = 0x01; // Read operation
    buffer_tx[2] = reg; // Register
    buffer_tx[3] = len; // Length

    // Print the buffer
    printf("Buffer TX: ");
    for (uint8_t i = 0; i < 4; i++) {
        printf("%02X ", buffer_tx[i]);
    }

    // Send the read operation to the BNO055 sensor
    uart_write(&bno055->uart_config, buffer_tx, 4);

    // Read the data from the BNO055 sensor
    uint8_t buffer_rx[128];
    uart_read(&bno055->uart_config, buffer_rx, 128, 5);

    // Print the buffer
    printf("Buffer RX: ");
    for (uint8_t i = 0; i < len + 2; i++) {
        printf("%02X ", buffer_rx[i]);
    }

    uint8_t length = 0;
    // Check if the read operation was successful
    if (buffer_rx[0] == BNO055_READ_SUCCESS) {
        length = buffer_rx[1];
        for (uint8_t i = 0; i < length; i++) {
            data[i] = buffer_rx[2 + i];
        }
    }
    else if (buffer_rx[0] == BNO055_ACK_VALUE)
    {
        printf("Error: Read operation failed\n");
        return -3; // Error: Read operation failed
    }
    
    return BNO055_SUCCESS;
}

int8_t BNO055_CheckAck(BNO055_t *bno055)
{
    uint8_t buffer[2];
    // Check if the write operation was successful with the ACK
    uart_read(&bno055->uart_config, buffer, 2, 5);

    if (buffer[0] == BNO055_ACK_VALUE) {
        switch (buffer[1])
        {
        case BNO055_WRITE_SUCCESS:
            return BNO055_SUCCESS;
            break;
        case BNO055_WRITE_FAIL:
            return BNO055_ERROR;
            break;
        default:
            return BNO055_ERROR;
            break;
        }
    }
    return BNO055_ERROR;

}
