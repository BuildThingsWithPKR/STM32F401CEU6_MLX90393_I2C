/*
 * mlx90393.h
 *
 *  Created on: Apr 6, 2024
 *      Author: PKumars'
 */

#ifndef INC_MLX90393_H_
#define INC_MLX90393_H_

#include "main.h"
#include "stdio.h"
#include "string.h"

typedef struct {
	int16_t X_axis;			// value from X axis
	int16_t Y_axis;			// value from Y axis
	int16_t Z_axis;			// value from Z axis
	int16_t temperature;	// temperature
}MLX_data;

typedef struct {
	uint8_t gain;
	uint8_t osr;
	uint8_t dig_filter;
	uint8_t res_x;
	uint8_t res_y;
	uint8_t res_z;
	uint16_t offset_x;
	uint16_t offset_y;
	uint16_t offset_z;
	uint16_t field_intensity;
}calibration_settings;

typedef enum mlx90393_filter {
  MLX90393_FILTER_0,
  MLX90393_FILTER_1,
  MLX90393_FILTER_2,
  MLX90393_FILTER_3,
  MLX90393_FILTER_4,
  MLX90393_FILTER_5,
  MLX90393_FILTER_6,
  MLX90393_FILTER_7,
} mlx90393_filter_t;

/** Oversampling settings for CONF3 register. */
typedef enum mlx90393_oversampling {
  MLX90393_OSR_0,
  MLX90393_OSR_1,
  MLX90393_OSR_2,
  MLX90393_OSR_3,
} mlx90393_oversampling_t;

/** Gain settings for CONF1 register. */
typedef enum mlx90393_gain {
  MLX90393_GAIN_5X = (0x00),
  MLX90393_GAIN_4X,
  MLX90393_GAIN_3X,
  MLX90393_GAIN_2_5X,
  MLX90393_GAIN_2X,
  MLX90393_GAIN_1_67X,
  MLX90393_GAIN_1_33X,
  MLX90393_GAIN_1X
} mlx90393_gain_t;

/** Resolution settings for CONF3 register. */
typedef enum mlx90393_resolution {
  MLX90393_RES_16,
  MLX90393_RES_17,
  MLX90393_RES_18,
  MLX90393_RES_19,
} mlx90393_resolution_t;


typedef struct {
	float x;
	float y;
	float z;
}calculated_data;


//#define MLX90393_ADDR 				(0x0C << 1)	// sensor address
#define MLX90393_ADDR		(0x18 << 1) //MLX90393xLW-ABA-014-RE 0x06 0x18, 0x19, 0x1A, 0x1B
#define X_AXIS_MASK					0x02		// mask to OR with RM (the mask | RM) command to read data from X axis
#define Y_AXIS_MASK					0x04		// mask to OR with RM (the mask | RM) command to read data from Y axis
#define Z_AXIS_MASK					0x08		// mask to OR with RM (the mask | RM) command to read data from Z axis


#define CMD_START_SINGLE 			0x30		// Start Single Measurement Mode
#define CMD_READ_MEASUREMENT 		0x40		// Read Measurement
#define CMD_READ_REGISTER 			0x50		// Read Register
#define CMD_WRITE_REGISTER 			0x60		// Write Register
#define CMD_EXIT 					0x80		// Exit Mode
#define CMD_MEM_RECALL 				0xD0		// Memory Recall
#define CMD_MEM_STORE 				0xE0		// Memory Store
#define CMD_RST 					0xF0		// Reset


/* MLX90393 MEMORY MAP */

/*	REGISTER || BIT15 | BIT14 | BIT13 | BIT12 | BIT11 | BIT10 | BIT9 | BIT8 | BIT7 | BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0 |
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 * 	__0x00h__||________________ANA_RESERVED_LOW______________________|_BIST_|Z-SER_|______GAIN_SEL______|_________HALL_CONF_________|
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 * 	__0x01h__||TRG_INT|__COMMON_MODE__|WOC_DIF|EXT_TRG|TCMP_EN|_____BURST_SEL(xyzt)_______|_________BURST_DATA_RATE(BDR)____________|
 * 	_________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x02h__||_______|_______|_______|______OSR2_____|_____RES_Z____|_____RES_Y___|_____RES_X___|_____DIG_FILT_______|_____OSR_____|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x03h__||_________________________SENS_TC_HT__________________________|______________________SENS_TC_LT_______________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x04h__||_____________________________________________________OFFSET_X________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x05h__||_____________________________________________________OFFSET_Y________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x06h__||_____________________________________________________OFFSET_Z________________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x07h__||___________________________________________________WOXY_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x08h__||____________________________________________________WOZ_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x09h__||____________________________________________________WOT_THRESHOLD____________________________________________________|
 *  _________||_______|_______|_______|_______|_______|_______|______|______|______|______|______|______|______|______|______|______|
 *  __0x0Ah__||                                                                                                                     |
 *     ...   ||                                                         FREE                                                        |
 *  __0x1Fh__||_____________________________________________________________________________________________________________________|
 */


#define HALLCONF_MASK				0x000F
#define HALLCONF_START_BIT			0
#define GAIN_SEL_MASK				0x0070
#define GAIN_SEL_START_BIT			4
#define TCMP_EN_MASK				0x0400
#define TCMP_EN_START_BIT			10
#define COMM_MODE_MASK				0x6000
#define COMM_MODE_START_BIT			13
#define OSR_MASK					0x0003
#define OSR_START_BIT				0
#define DIG_FILT_MASK				0x001C
#define DIG_FILT_START_BIT			2
#define RES_X_MASK					0x0060
#define RES_X_START_BIT				5
#define RES_Y_MASK					0x0180
#define RES_Y_START_BIT				7
#define RES_Z_MASK					0x0600
#define RES_Z_START_BIT				9
#define OSR2_MASK					0x1800
#define OSR2_START_BIT				11
#define SENS_TC_LT_MASK				0xFF
#define SENS_TC_LT_START_BIT		0
#define SENS_TC_HT_MASK				0xFF
#define SENS_TC_HT_START_BIT		8
#define OFFSET_X					0x04			// 16 bit long
#define OFFSET_Y					0x05			// 16 bit long
#define OFFSET_Z					0x06			// 16 bit long
#define WOXY_THRESHOLD				0x07			// 16 bit long
#define WOZ_THRESHOLD				0x08			// 16 bit long
#define WOT_THRESHOLD				0x09			// 16 bit long
#define REG_00						0x00
#define REG_01						0x01
#define REG_02						0x02
#define REG_03						0x03
#define REG_04						0x04
#define REG_05						0x05
#define REG_06						0x06
#define REG_07						0x07
#define REG_08						0x08
#define REG_09						0x09

#define ERROR_BIT_MASK				0x10
// addresses from 0x0A to 0x1F are free for customer content

void read_measure_X(MLX_data *data_struct);		// read X axis measure
void read_measure_Y(MLX_data *data_struct);		// read X axis measure
void read_measure_Z(MLX_data *data_struct);		// read X axis measure
void store_data_from_gui_to_struct(uint8_t *data, calibration_settings *data_struct);

uint8_t get_gain(void);
uint8_t get_osr(void);// get gain
uint8_t get_filter(void);
void set_gain(uint8_t gain);							// set gain
void set_resolution_X(uint8_t res);						// set resolution of X axis
void set_resolution_Y(uint8_t res);						// set resolution of Y axis
void set_resolution_Z(uint8_t res);						// set resolution of Z axis
void set_filter(uint8_t conf);
void set_osr(uint8_t osr); 								// Magnetic sensor ADC oversampling ratio
uint8_t get_resolution_X(void);							// get resolution of X axis
uint8_t get_resolution_Y(void);							// get resolution of Y axis
uint8_t get_resolution_Z(void);

uint16_t get_offset_X(void);
uint16_t get_offset_Y(void);
uint16_t get_offset_Z(void);
void set_offset_X(uint16_t offset_X);
void set_offset_Y(uint16_t offset_Y);
void set_offset_Z(uint16_t offset_Z);

void set_multipliers(float* x, float* y, float* z);

#endif /* INC_MLX90393_H_ */
