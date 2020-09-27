/*
 * max30102_dpv.c
 *
 *  Created on: 18 сент. 2020 г.
 *      Author: Dmitry Ses
 */

#include "max30102_drv.h"
#include "utils.h"

HAL_StatusTypeDef po_read_reg(MAX30102_HandleTypeDef *hpo, uint8_t *regBuff, uint8_t regAddr){
    HAL_StatusTypeDef retCode = HAL_OK;
    CHECK_PTR(hpo);
    CHECK_PTR(hpo->hi2c);
    CHECK_PTR(regBuff);
    CHECK_SUCCESS(HAL_I2C_Master_Transmit(hpo->hi2c, hpo->address, &regAddr, 1, HAL_MAX_DELAY));
    CHECK_SUCCESS(HAL_I2C_Master_Receive(hpo->hi2c, hpo->address, regBuff, 1, HAL_MAX_DELAY));

    exit:
    return retCode;
}

HAL_StatusTypeDef po_write_reg(MAX30102_HandleTypeDef *hpo, uint8_t regData, uint8_t regAddr){
    HAL_StatusTypeDef retCode = HAL_OK;
    uint8_t buff [2] = {regAddr, regData};
    CHECK_PTR(hpo);
    CHECK_SUCCESS(HAL_I2C_Master_Transmit(hpo->hi2c, hpo->address, buff, 2, HAL_MAX_DELAY));

    exit:
    return retCode;
}

#define MAX_30102_PART_ID_MISMATCH 0

HAL_StatusTypeDef po_init(MAX30102_HandleTypeDef *hpo){
    HAL_StatusTypeDef retCode = HAL_OK;
    CHECK_PTR(hpo);
    CHECK_PTR(hpo->hi2c);
    uint8_t partID = 0;
    CHECK_SUCCESS(po_write_reg(hpo, RESET, MODE_CONFIG_REG));
    CHECK_SUCCESS(po_read_reg(hpo, &partID, PART_ID_REG));  //read partID
    if(partID != MAX30102_PART_ID){ // Check partID
        retCode = HAL_ERROR;
        goto exit;
    }
    CHECK_SUCCESS(po_write_reg(hpo, hpo->fifoConfig, FIFO_CONFIG_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->modeConfig, MODE_CONFIG_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->spo2Config, SPO2_CONFIG_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->ledPulseAmplRed, LED_PULSE_AMPLITUDE_RED_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->ledPulseAmplIr, LED_PULSE_AMPLITUDE_IR_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->multiLed_2_1, M_LED_CTRL_2_1_REG));
    CHECK_SUCCESS(po_write_reg(hpo, hpo->multiLed_4_3, M_LED_CTRL_4_3_REG));
    exit:
    return retCode;
}

HAL_StatusTypeDef po_get_temperature(MAX30102_HandleTypeDef *hpo, int16_t *temp_2_comp){
    HAL_StatusTypeDef retCode = HAL_OK;
    int8_t intTemp = 0;
    int8_t fractTemp = 0;
    uint8_t intStatusReg = 0;
    CHECK_PTR(hpo);
    CHECK_PTR(temp_2_comp);

    po_write_reg(&hpo, TEMP_ENABLE, DIE_TEMP_CONFIG_REG);
    do{
        po_read_reg(&hpo, &intStatusReg, INT_STATUS_2_REG);
    } while(intStatusReg == 0);
    po_read_reg(&hpo, &intTemp, DIE_TEMP_INT_REG);
    po_read_reg(&hpo, &fractTemp, DIE_TEMP_FRACT_REG);
    *temp_2_comp = (int16_t)((intTemp<<4)|fractTemp);
    exit:
    return retCode;
}

HAL_StatusTypeDef po_print_reg(MAX30102_HandleTypeDef *hpo, uint8_t regAddr){
    HAL_StatusTypeDef retCode = HAL_OK;
    uint8_t readReg = 0;
    CHECK_PTR(hpo);
    CHECK_PTR(hpo->hi2c);
    CHECK_SUCCESS(po_read_reg(hpo, &readReg, regAddr));
    switch (regAddr) {
    case INT_STATUS_1_REG:
        printf("INT_STATUS_1_REG: %d\r\n", readReg);
        break;
    case INT_STATUS_2_REG:
        printf("INT_STATUS_2_REG: %d\r\n", readReg);
        break;
    case INT_ENABLE_1_REG:
        printf("INT_ENABLE_1_REG: %d\r\n", readReg);
        break;
    case INT_ENABLE_2_REG:
        printf("INT_ENABLE_2_REG: %d\r\n", readReg);
        break;
    case FIFO_WR_PTR_REG:
        printf("FIFO_WR_PTR_REG: %d\r\n", readReg);
        break;
    case OVF_COUNTER_REG:
        printf("OVF_COUNTER_REG: %d\r\n", readReg);
        break;
    case FIFO_RD_PTR_REG:
        printf("FIFO_RD_PTR_REG: %d\r\n", readReg);
        break;
    case FIFO_DATA_REG:
        printf("FIFO_DATA_REG: %d\r\n", readReg);
        break;
    case FIFO_CONFIG_REG:
        printf("FIFO_CONFIG_REG: %d\r\n", readReg);
        break;
    case MODE_CONFIG_REG:
        printf("MODE_CONFIG_REG: %d\r\n", readReg);
        break;
    case SPO2_CONFIG_REG:
        printf("SPO2_CONFIG_REG: %d\r\n", readReg);
        break;
    case LED_PULSE_AMPLITUDE_IR_REG:
        printf("LED_PULSE_AMPLITUDE_IR_REG: %d\r\n", readReg);
        break;
    case LED_PULSE_AMPLITUDE_RED_REG:
        printf("LED_PULSE_AMPLITUDE_RED_REG: %d\r\n", readReg);
        break;
    case M_LED_CTRL_2_1_REG:
        printf("M_LED_CTRL_2_1_REG: %d\r\n", readReg);
        break;
    case M_LED_CTRL_4_3_REG:
        printf("M_LED_CTRL_4_3_REG: %d\r\n", readReg);
        break;
    case DIE_TEMP_INT_REG:
        printf("DIE_TEMP_INT_REG: %d\r\n", readReg);
        break;
    case DIE_TEMP_FRACT_REG:
        printf("DIE_TEMP_FRACT_REG: %d\r\n", readReg);
        break;
    case DIE_TEMP_CONFIG_REG:
        printf("DIE_TEMP_CONFIG_REG: %d\r\n", readReg);
        break;
    case REVISION_ID_REG:
        printf("REVISION_ID_REG: %d\r\n", readReg);
        break;
    case PART_ID_REG:
        printf("PART_ID_REG:: %d\r\n", readReg);
        break;
    default:
        printf("INVALID REG ADDRESS!\r\n");
        break;
    }
    exit:
    return retCode;
}

HAL_StatusTypeDef po_print_dump(MAX30102_HandleTypeDef *hpo){
    HAL_StatusTypeDef retCode = HAL_OK;
    CHECK_PTR(hpo);
    CHECK_PTR(hpo->hi2c);
    printf("_____MAX30102 memory dump_____\r\n");
    po_print_reg(hpo, INT_STATUS_1_REG);
    po_print_reg(hpo, INT_STATUS_2_REG);
    po_print_reg(hpo, INT_ENABLE_1_REG);
    po_print_reg(hpo, INT_ENABLE_2_REG);
    po_print_reg(hpo, FIFO_WR_PTR_REG);
    po_print_reg(hpo, OVF_COUNTER_REG);
    po_print_reg(hpo, FIFO_RD_PTR_REG);
    po_print_reg(hpo, FIFO_DATA_REG);
    po_print_reg(hpo, FIFO_CONFIG_REG);
    po_print_reg(hpo, MODE_CONFIG_REG);
    po_print_reg(hpo, SPO2_CONFIG_REG);
    po_print_reg(hpo, LED_PULSE_AMPLITUDE_IR_REG);
    po_print_reg(hpo, LED_PULSE_AMPLITUDE_RED_REG);
    po_print_reg(hpo, M_LED_CTRL_2_1_REG);
    po_print_reg(hpo, M_LED_CTRL_4_3_REG);
    po_print_reg(hpo, DIE_TEMP_INT_REG);
    po_print_reg(hpo, DIE_TEMP_FRACT_REG);
    po_print_reg(hpo, DIE_TEMP_CONFIG_REG);
    po_print_reg(hpo, REVISION_ID_REG);
    po_print_reg(hpo, PART_ID_REG);
    printf("______________________________\r\n");
    exit:
    return retCode;
}
