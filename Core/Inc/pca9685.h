// PCA9685 I2C Communication Protocol Header File
#pragma once
#include "stm32f4xx_hal.h"


//Standard Includes
#include <stdint.h>
#include <stdbool.h>

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */

// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */





typedef struct {
    uint8_t i2c_address;     /**< The I2C address of the PCA9685 device */
    uint8_t mode1;            /**< Current value of Mode Register 1 */
    uint8_t mode2;            /**< Current value of Mode Register 2 */
    uint8_t prescale;         /**< Prescaler value for PWM frequency */
    uint8_t subaddress[1];    /**< Subaddresses for the device */
} PCA9685_t;

//Function Prototypes:


/**
 * @brief Initializes the PCA9685 module
 * @return true if initialization was successful, false otherwise
 */

bool PCA9685_Init();

/**
 * @brief Sets the PWM frequency for the PCA9685
 * @param pca9685 Pointer to the PCA9685 device instance
 * @param frequency The desired PWM frequency in Hz
 * @return true if the operation was successful, false otherwise
 */
bool PCA9685_SetPWMFrequency(uint16_t frequency);



/**
 * @brief Writes data to a specific register of an I2C device.
 * @param hi2c Pointer to the I2C handle.
 * @param device_address 7-bit I2C address of the device (without the R/W bit).
 * @param register_address Address of the register to write to.
 * @param data Pointer to the data to be written.
 * @param size Size of the data in bytes.
 * @return HAL status (HAL_OK on success, error otherwise).
 */
HAL_StatusTypeDef PCA9685_Write(I2C_HandleTypeDef *hi2c, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);



/**
 * @brief Reads data from a specific register of an I2C device.
 * @param hi2c Pointer to the I2C handle.
 * @param device_address 7-bit I2C address of the device (without the R/W bit).
 * @param register_address Address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param size Size of the data to be read in bytes.
 * @return HAL status (HAL_OK on success, error otherwise).
 */
HAL_StatusTypeDef PCA9685_Read (I2C_HandleTypeDef *hi2c, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t size);


/**
 * @brief Sets the PWM output of one of the PCA9685 pins.
 * @param hi2c Pointer to the I2C handle.
 * @param device_address 7-bit I2C address of the PCA9685 device.
 * @param num PWM output pin number (0 to 15).
 * @param on Point in the 4096-part cycle to turn the PWM output ON (0 to 4095).
 * @param off Point in the 4096-part cycle to turn the PWM output OFF (0 to 4095).
 * @return HAL status (HAL_OK on success, error otherwise).
 */
HAL_StatusTypeDef setPWM(uint8_t num, uint16_t on, uint16_t off);


HAL_StatusTypeDef PCA9685_LED_Status(uint8_t MemAddress, uint8_t memAddSize);