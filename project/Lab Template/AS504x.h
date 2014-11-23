/// @file 	AS504x.h
/// @brief	AS504x driver form STM32 chip family
/// @author	Brian Cairl
#ifndef AS504x_H
#define AS504x_H 1

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "defines.h"
#include "tm_stm32f4_spi.h"

#ifndef 	AS504x_D_ANGLE_MAX
#define 	AS504x_D_ANGLE_MAX					90.0f
#endif

#ifndef		AS504x_ANGLE_BUFFER_SIZE
#define		AS504x_ANGLE_BUFFER_SIZE		10UL
#endif

#ifndef 	AS504x_VELOCITY_LPFG
#define 	AS504x_VELOCITY_LPFG				0.1f
#endif

typedef enum pinname_x
{
	AS504x_CLK = 0,
	AS504x_CS  = 1,
	AS504x_IN	= 2
} AS504x_pinname_t;
#define PIN_HI(p)			GPIO_WriteBit(p.port,p.pin,Bit_SET)
#define PIN_LO(p)			GPIO_WriteBit(p.port,p.pin,Bit_RESET)
#define PIN_RD(p)			GPIO_ReadInputDataBit(p.port,p.pin)


typedef struct diopin_x
{
	uint32_t 			rcc;
	GPIO_TypeDef* port;
	uint16_t 			pin;
} AS504x_diopin_t;


typedef struct AS504x_x
{
	AS504x_diopin_t			clk;
	AS504x_diopin_t			cs;
	AS504x_diopin_t			in;
	
	uint16_t 						ios;
	uint32_t 						packet;
	
	uint32_t 						position;
	uint32_t 						status;
	///									b1	RES
	///									b2	Decrement
	///									b3	Increment
	///									b4	Linearity Alarm
	///									b5	Cordic Overflow Errpr
	///									b6	Chip startup finished

	float 							velocity;
	float 							angles[AS504x_ANGLE_BUFFER_SIZE];
	uint32_t 						angle_itr;
} AS504x_t;

/// Some convinience macros for accessing AS504x_t
#define AS504x_DECREMENT(enc)			((enc).status&2UL)
#define AS504x_INCREMENT(enc)			((enc).status&4UL)
#define AS504x_LIN_ALARM(enc)			((enc).status&8UL)
#define AS504x_DATA_ERROR(enc)		((enc).status&16UL)
#define AS504x_READY(enc)					((enc).status&32UL)
#define AS504x_BAD_RANGE(enc)			(AS504x_DECREMENT(enc)&&AS504x_INCREMENT(enc))

#define AS504x_CURRENT_ANGLE(enc)	(enc).angles[((enc).angle_itr)%AS504x_ANGLE_BUFFER_SIZE]


/// @brief 	Used to setup clock, input and chip-select pins
///	@param 	enc		encoder support structure
///	@param	rcc		corresponds to RCC_AHB1Periph_GPIOx
///	@param	port	corresponds to GPIOx
///	@param	pin		pin number
/// @param 	name 	pin alias [ AS504x_CLK | AS504x_CS | AS504x_IN ]
void AS504x_SetPin(
	AS504x_t* 						enc,
	uint32_t 							rcc,
	GPIO_TypeDef*					port,
	uint16_t							pin,
	AS504x_pinname_t			name
);


/// @brief 	Zeros out support structure values; readies sensor for you.
///	@param 	enc		encoder support structure
void AS504x_Init(
	AS504x_t* 						enc
);


/// @brief 	Readsback data packet, grabs position and statues
///	@param 	enc		encoder support structure
void AS504x_Read(
	AS504x_t* 						enc
);


/// @brief  Updates velocity estimate
///					Should have something timing the encoder updates for this (uniform sampling)
///	@param 	enc		encoder support structure
void AS504x_UpdateVelocity(
	AS504x_t* 						enc
);


#endif