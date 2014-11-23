#include "MC339xx.h"
#include "math.h"

#if(!MC339xx_DUAL_MOTOR_QUICK_SETUP)
MC339xx_PWM_Group_t PE9_PWM 	= { GPIOE, RCC_AHB1Periph_GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1, GPIO_Pin_9,  TIM1, TIM_OC1Init, TIM_OC1PreloadConfig };
MC339xx_PWM_Group_t PE11_PWM 	= { GPIOE, RCC_AHB1Periph_GPIOE, GPIO_PinSource11, GPIO_AF_TIM1, GPIO_Pin_11, TIM1, TIM_OC2Init, TIM_OC2PreloadConfig };
MC339xx_PWM_Group_t PE13_PWM 	= { GPIOE, RCC_AHB1Periph_GPIOE, GPIO_PinSource13, GPIO_AF_TIM1, GPIO_Pin_13, TIM1, TIM_OC3Init, TIM_OC3PreloadConfig };
MC339xx_PWM_Group_t PE14_PWM 	= { GPIOE, RCC_AHB1Periph_GPIOE, GPIO_PinSource14, GPIO_AF_TIM1, GPIO_Pin_14, TIM1, TIM_OC4Init, TIM_OC4PreloadConfig };

MC339xx_PWM_Group_t PD12_PWM 	= { GPIOD, RCC_AHB1Periph_GPIOD, GPIO_PinSource12, GPIO_AF_TIM4, GPIO_Pin_12, TIM4, TIM_OC1Init, TIM_OC1PreloadConfig };
MC339xx_PWM_Group_t PD13_PWM 	= { GPIOD, RCC_AHB1Periph_GPIOD, GPIO_PinSource13, GPIO_AF_TIM4, GPIO_Pin_13, TIM4, TIM_OC2Init, TIM_OC2PreloadConfig };
MC339xx_PWM_Group_t PD14_PWM 	= { GPIOD, RCC_AHB1Periph_GPIOD, GPIO_PinSource14, GPIO_AF_TIM4, GPIO_Pin_14, TIM4, TIM_OC3Init, TIM_OC3PreloadConfig };
MC339xx_PWM_Group_t PD15_PWM 	= { GPIOD, RCC_AHB1Periph_GPIOD, GPIO_PinSource15, GPIO_AF_TIM4, GPIO_Pin_15, TIM4, TIM_OC4Init, TIM_OC4PreloadConfig };


void MC339xx_PWM_Group_Init( MC339xx_PWM_Group_t* g )
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	
	/// Init GPIO
	GPIO_PinAFConfig(g->port, g->pin_source, g->pin_af);

	GPIO_InitStruct.GPIO_Pin 		= g->pin_n;
	GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_100MHz;
	
	GPIO_Init(g->port, &GPIO_InitStruct);	
	

	/// Init Timer-base
	RCC_APB1PeriphClockCmd(g->pin_rcc_periph, ENABLE);

	TIM_BaseStruct.TIM_Prescaler 					= 0;
	TIM_BaseStruct.TIM_CounterMode 				= TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period 						= 8399; /* 10kHz PWM */
	TIM_BaseStruct.TIM_ClockDivision 			= TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter 	= 0;
			
	TIM_TimeBaseInit(g->timer, &TIM_BaseStruct);
	TIM_Cmd(g->timer, ENABLE);

}


void MC339xx_PWM_Group_Write( MC339xx_PWM_Group_t* g, uint16_t pulse_length )
{
		TIM_OCInitTypeDef TIM_OCStruct;

		TIM_OCStruct.TIM_OCMode 			= TIM_OCMode_PWM2;
		TIM_OCStruct.TIM_OutputState 	= TIM_OutputState_Enable;
		TIM_OCStruct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
		TIM_OCStruct.TIM_Pulse 				= pulse_length;	

		g->pin_init_fn(g->timer,&TIM_OCStruct);	
		g->pin_prel_fn(g->timer, TIM_OCPreload_Enable);
}


void Init_MC339xx( 
	MC339xx_t* 						motor, 
	MC339xx_PWM_Group_t* 	pwm_R,
	MC339xx_PWM_Group_t* 	pwm_L
){
	motor->group[0U] = (MC339xx_PWM_Group_t*)pwm_R;
	motor->group[1U] = (MC339xx_PWM_Group_t*)pwm_L;
	
	MC339xx_PWM_Group_Init(motor->group[0U]);
	MC339xx_PWM_Group_Init(motor->group[1U]);
}


void Cmd_MC339xx( MC339xx_t* motor, float speed )
{	
		uint16_t pulse_length;
	
		if(speed > +1.0f)
			speed = 1.0f;
		else
		if(speed < -1.0f)
			speed =-1.0f;
		
		pulse_length = (8339U*fabsf(speed)) + 1U;
	
		if(speed>0.0f)
		{
			MC339xx_PWM_Group_Write(motor->group[0U],pulse_length);
			MC339xx_PWM_Group_Write(motor->group[1U],0U);
		}
		else
		{
			MC339xx_PWM_Group_Write(motor->group[0U],0U);
			MC339xx_PWM_Group_Write(motor->group[1U],pulse_length);
		}
}
#else

	void Init_Dual_MC339xx(void)
	{
		GPIO_InitTypeDef 				GPIO_InitStruct;
		TIM_TimeBaseInitTypeDef TIM_BaseStruct;
		
		/* Clock for GPIOD */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		
		/* Alternating functions for pins */
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
		
		/* Set pins */
		GPIO_InitStruct.GPIO_Pin 		= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_100MHz;
		
		GPIO_Init(GPIOD, &GPIO_InitStruct);
		
			
		/* Enable clock for TIM4 */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_BaseStruct.TIM_Prescaler = 0;
		
		/* Count up */
		TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

		TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
		TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_BaseStruct.TIM_RepetitionCounter = 0;
			
		/* Initialize TIM4 */
    	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
		
		/* Start count on TIM4 */
    	TIM_Cmd(TIM4, ENABLE);
	}
	

	void Cmd_Dual_MC339xx( float speed_L, float speed_R )
	{
		TIM_OCInitTypeDef TIM_OCStruct;

		uint16_t 
			pulse_length_L, 
			pulse_length_R;

		TIM_OCStruct.TIM_OCMode 			= TIM_OCMode_PWM2;
		TIM_OCStruct.TIM_OutputState 	= TIM_OutputState_Enable;		
		TIM_OCStruct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
		
		
		if(speed_L > +1.0f)	speed_L = 1.0f;
		else
		if(speed_L < -1.0f)	speed_L =-1.0f;
		
		pulse_length_L = (8339U*fabsf(speed_L)) + 1U;
	
		if(speed_L>0)
		{
			TIM_OCStruct.TIM_Pulse = pulse_length_L; /* 25% duty cycle */
			TIM_OC1Init(TIM4, &TIM_OCStruct);
			TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
			
			TIM_OCStruct.TIM_Pulse = 0; /* 50% duty cycle */
			TIM_OC2Init(TIM4, &TIM_OCStruct);
			TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else
		{
			TIM_OCStruct.TIM_Pulse = 0; /* 25% duty cycle */
			TIM_OC1Init(TIM4, &TIM_OCStruct);
			TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
			TIM_OCStruct.TIM_Pulse = pulse_length_L; /* 50% duty cycle */
			TIM_OC2Init(TIM4, &TIM_OCStruct);
			TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}

		
		if(speed_R > +1.0f)	speed_R = 1.0f;
		else
		if(speed_R < -1.0f)	speed_R =-1.0f;
		
		pulse_length_R = (8339U*fabsf(speed_R)) + 1U;		
		
		if(speed_R>0)
		{
			TIM_OCStruct.TIM_Pulse = pulse_length_R;
			TIM_OC3Init(TIM4, &TIM_OCStruct);
			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
			
			TIM_OCStruct.TIM_Pulse = 0;
			TIM_OC4Init(TIM4, &TIM_OCStruct);
			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}
		else
		{
			TIM_OCStruct.TIM_Pulse = 0;
			TIM_OC3Init(TIM4, &TIM_OCStruct);
			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
			
			
			TIM_OCStruct.TIM_Pulse = pulse_length_R;
			TIM_OC4Init(TIM4, &TIM_OCStruct);
			TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		}

	}


#endif