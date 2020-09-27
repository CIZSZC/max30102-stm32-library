/*
 * max30102_drv.h
 *
 *  Created on: 13 сент. 2020 г.
 *      Author: Dmitry Ses
 */

#ifndef INC_MAX30102_DRV_H_
#define INC_MAX30102_DRV_H_
#include "stm32f1xx_hal.h"


#define MAX30102_I2C_ADDRESS 0xAE;
#define MAX30102_PART_ID 0x15

/*Status registers addresses*/
#define INT_STATUS_1_REG 0x00
#define INT_STATUS_2_REG 0x01
#define INT_ENABLE_1_REG 0x02
#define INT_ENABLE_2_REG 0x03
#define FIFO_WR_PTR_REG  0x04
#define OVF_COUNTER_REG  0x05
#define FIFO_RD_PTR_REG  0x06
#define FIFO_DATA_REG    0x07
/*Configuration registers addresses*/
#define FIFO_CONFIG_REG 0x08
#define MODE_CONFIG_REG 0x09
#define SPO2_CONFIG_REG 0x0A
#define LED_PULSE_AMPLITUDE_RED_REG 0x0C
#define LED_PULSE_AMPLITUDE_IR_REG 0x0D
#define M_LED_CTRL_2_1_REG 0x11
#define M_LED_CTRL_4_3_REG 0x12
/*Die temperature registers addresses*/
#define DIE_TEMP_INT_REG   0x1F
#define DIE_TEMP_FRACT_REG 0x20
#define DIE_TEMP_CONFIG_REG 0x21
/*Part ID registers addresses*/
#define REVISION_ID_REG 0xFE
#define PART_ID_REG 0xFF

/*Interrupt status 1 bits*/
#define A_FULL  (1<<7)  //
#define PPG_RDY (1<<6)
#define ALC_OVF (1<<5)
#define PWR_RDY (1<<0)
/*Interrupt status 2 bits*/
#define DIE_TEMP_RDY (1<<1)
/*Interrupt enable 1 bits*/
#define A_FULL_EN  (1<<7)
#define PPG_RDY_EN (1<<6)
#define ALC_OVF_EN (1<<5)
/*Interrupt enable 2 bits*/
#define DIE_TEMP_RDY_EN (1<<1)

#define FIFO_WR_PTR_MAX 0x0F //Maximum FIFO write pointer value
#define OVF_COUNTER_MAX 0x0F //Maximum over flow counter value
#define FIFO_RD_PTR_MAX 0x0F //Maximum FIFO read pointer value
#define FIFO_DATA_MAX 0xFF   //Maximum FIFO data register value

/*FIFO configuration bits*/
/*Sample averaging*/
#define SMP_AVE_1 0x00
#define SMP_AVE_2 0x20
#define SMP_AVE_4 0x40
#define SMP_AVE_8 0x60
#define SMP_AVE_16 0x80
#define SMP_AVE_32 0xA0
#define FIFO_ROLLOVER_EN 0x10
#define FIFO_ROLLOVER_DIS 0x00
#define FIFO_A_FULL_MAX 0x0F

/*Mode configuration bits*/
#define SHDN 0x80
#define RESET 0x40
#define HR_MODE 0x02
#define SPO2_MODE 0x03
#define MULTILED_MODE 0x0

/*SPO2 configuration bits*/
/*SPO2 range control*/
#define SPO2_ADC_RGE_1 0x00   //LSB 7.81pA, full scale 2048pA
#define SPO2_ADC_RGE_2 0x20   //LSB 15.63pA, full scale 4096pA
#define SPO2_ADC_RGE_3 0x40   //LSB 31.25pA, full scale 8192pA
#define SPO2_ADC_RGE_4 0x60   //LSB 62.5pA, full scale 16384pA
/*SPO2 sample rate*/
#define SPO2_SR_50SPS 0x00
#define SPO2_SR_100SPS 0x04
#define SPO2_SR_200SPS 0x08
#define SPO2_SR_400SPS 0x0C
#define SPO2_SR_800SPS 0x10
#define SPO2_SR_1000SPS 0x14
#define SPO2_SR_1600SPS 0x18
#define SPO2_SR_3200SPS 0x1C
/*LED pulse width control and ADC resolution*/
#define LED_PW_69 0x00
#define LED_PW_118 0x01
#define LED_PW_215 0x02
#define LED_PW_411 0x03

/*LED pulse amplitude*/
#define LED_PULSE_AMPLITUDE_MIN 0x00 //0mA
#define LED_PULSE_AMPLITUDE_MAX 0xFF //51mA

/*Multi-LED control mode*/
#define SLOT_1_NONE 0x00
#define SLOT_1_LED_RED 0x01
#define SLOT_1_LED_IR 0x2
#define SLOT_3_NONE 0x00
#define SLOT_3_LED_RED 0x01
#define SLOT_3_LED_IR 0x2
#define SLOT_2_NONE 0x00
#define SLOT_2_LED_RED 0x10
#define SLOT_2_LED_IR 0x20
#define SLOT_4_NONE 0x00
#define SLOT_4_LED_RED 0x10
#define SLOT_4_LED_IR 0x20

/*temperature config register bits*/
#define TEMP_ENABLE 0x01

/**
 * \brief pulse oximeter configuration structure
 */
typedef struct{
    uint8_t fifoConfig;
    uint8_t modeConfig;
    uint8_t spo2Config;
    uint8_t ledPulseAmplRed;
    uint8_t ledPulseAmplIr;
    uint8_t multiLed_2_1;
    uint8_t multiLed_4_3;
    uint8_t address;
    I2C_HandleTypeDef *hi2c;
} MAX30102_HandleTypeDef;

/**
 * \brief Read register from specific address
 *
 * \return HAL_Status
 */
HAL_StatusTypeDef po_read_reg(MAX30102_HandleTypeDef *hpo, uint8_t *regBuff, uint8_t regAddr);

/**
 * \brief Write data to specific register
 *
 * \return HAL_Status
 */
HAL_StatusTypeDef po_write_reg(MAX30102_HandleTypeDef *hpo, uint8_t regData, uint8_t regAddr);

/**
 * \brief Send Configuration to max30102
 *
 * \return HAL_Status
 */
HAL_StatusTypeDef po_init(MAX30102_HandleTypeDef *hpo);

HAL_StatusTypeDef po_get_temperature(MAX30102_HandleTypeDef *hpo, int16_t *temp_2_comp);

HAL_StatusTypeDef po_print_reg(MAX30102_HandleTypeDef *hpo, uint8_t regAddr);

HAL_StatusTypeDef po_print_dump(MAX30102_HandleTypeDef *hpo);
#endif /* INC_MAX30102_DRV_H_ */
