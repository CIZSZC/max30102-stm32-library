/*
 * max30102_drv.h
 *
 *  Created on: 13 сент. 2020 г.
 *      Author: Дима Сесь
 */

#ifndef INC_MAX30102_DRV_H_
#define INC_MAX30102_DRV_H_

/*Status registers addresses*/
#define INT_STATUS_1 0x00
#define INT_STATUS_2 0x01
#define INT_ENABLE_1 0x02
#define INT_ENABLE_2 0x03
#define FIFO_WR_PTR  0x04
#define OVF_CNT      0x05
#define FIFO_RD_PTR  0x06
#define FIFO_DATA    0x07
/*Configuration registers addresses*/
#define FIFO_CONFIG 0x08
#define MODE_CONFIG 0x09
#define SPO2_CONFIG 0x0A
#define LED_PULSE_AMPLITUDE_1 0x0C
#define LED_PULSE_AMPLITUDE_2 0x0D
#define M_LED_CTRL_1_2 0x11
#define M_LED_CTRL_3_4 0x12
/*Die temperature registers addresses*/
#define DIE_TEMP_INT   0x1F
#define DIE_TEMP_FRACT 0x20
#define DIE_TEMP_CONFIG 0x21
/*Part ID registers addresses*/
#define REVISION_ID 0xFE
#define PART_ID 0xFF

/*Interrupt status 1 bits*/
#define A_FULL  (1<<7)
#define PPG_RFY (1<<6)
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


#endif /* INC_MAX30102_DRV_H_ */
