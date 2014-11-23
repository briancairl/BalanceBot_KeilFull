/// @file 	AS504x.c
/// @brief	AS504x driver form STM32 chip family
/// @author	Brian Cairl

#include "AS504x.h"

#define AS504x_SCALE_ANGLE(x)	((x)*(0.08789f))
#define AS504x_ANGLE_MASK			((uint32_t)(262080UL))
#define AS504x_STATS_MASK			((uint32_t)(63UL))

extern uint32_t msTicks;


/// This is, of course, ghetto as hell, but ya know, wtvr.
void wait()
{
	uint32_t cycles = 1000UL;
	while(cycles--);
}



void AS504x_SetPin(
	AS504x_t* 						enc,
	uint32_t 							rcc,
	GPIO_TypeDef*					port,
	uint16_t							pin,
	AS504x_pinname_t			name
){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable clock for CS port */
	RCC_AHB1PeriphClockCmd(rcc,  ENABLE);
	
	switch(name)
	{
		case AS504x_CS:
			enc->cs.pin = pin;
			enc->cs.port= port;
			enc->cs.rcc = rcc;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
			break;
		case AS504x_CLK:
			enc->clk.pin = pin;
			enc->clk.port= port;
			enc->clk.rcc = rcc;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
			break;	
		case AS504x_IN:
			enc->in.pin = pin;
			enc->in.port= port;
			enc->in.rcc = rcc;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
			break;		
		default:
			abort();
	}
			
	/* Configure CS pin */
	GPIO_InitStruct.GPIO_Pin 			= pin;
	GPIO_InitStruct.GPIO_OType 		= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 		= GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed 		= GPIO_Speed_25MHz;
	
	/* GPIO-G Init */
	GPIO_Init( port, &GPIO_InitStruct);
}



void AS504x_Init(
	AS504x_t* 		enc
){
	enc->velocity = 0.0f;
	enc->position	= 0UL;
	enc->ios 			= 0U;
	enc->packet 	= 0U;
	enc->angle_itr= 0UL;
	
	/// Start Devices
	PIN_LO(enc->cs);
	wait();
	PIN_LO(enc->clk);
	wait();
}



void AS504x_Read(
	AS504x_t* 		enc
){
	uint32_t itr;
	
	/// Zero out data
	enc->packet = 0UL;
	enc->ios 		= 0U;
	
	// Queue Transfer
	PIN_HI(enc->cs);
	wait();
	PIN_HI(enc->clk);
	wait();
		
	// Start Transfer
	PIN_LO(enc->cs);
	wait();
	PIN_LO(enc->clk);
	wait();
	
	// Get Bits (18 bits)
	for( itr = 0UL; itr < 18; itr++ )
	{
			PIN_HI(enc->clk);
			wait();

			enc->ios 		= PIN_RD(enc->in);
			enc->packet	= ((enc->packet<<1U)|(1U&enc->ios));

			PIN_LO(enc->clk);
			wait();
	}

	// Mask out status and position
	enc->status 	= (enc->packet&AS504x_STATS_MASK);
	enc->position	= ((enc->packet&AS504x_ANGLE_MASK)>>6U); 
	
	// Update real position
	if(!AS504x_DATA_ERROR(*enc))
		enc->angles[(enc->angle_itr++)%AS504x_ANGLE_BUFFER_SIZE] = AS504x_SCALE_ANGLE(enc->position);
}



void AS504x_UpdateVelocity(
	AS504x_t* 		enc
)
{
	uint32_t 	itr,jtr1,jtr2;
	uint32_t 	inc = AS504x_ANGLE_BUFFER_SIZE-1UL;
	float 		diff_sum = 0.0f;
	float 		diff;
	
	/// Calulate deiscrete differentials
	for( itr = 1UL; itr < AS504x_ANGLE_BUFFER_SIZE; itr++ )
	{
		jtr1 = (enc->angle_itr+itr)	 %AS504x_ANGLE_BUFFER_SIZE;
		jtr2 = (enc->angle_itr+itr-1)%AS504x_ANGLE_BUFFER_SIZE;
		
		diff = 	enc->angles[jtr1]-enc->angles[jtr2];
		
		/// Reject spikes
		if(fabsf(diff) < AS504x_D_ANGLE_MAX )
			diff_sum += diff;
		else
			--inc;
	}
	
	/// Only update if enough were accepted
	if(inc>2UL)
		enc->velocity += AS504x_VELOCITY_LPFG*((diff_sum/(float)(inc)) - enc->velocity);
}
