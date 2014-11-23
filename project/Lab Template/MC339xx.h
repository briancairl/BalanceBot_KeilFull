#ifndef MC339xx_H
#define MC339xx_H 1

#include <stdint.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#ifndef MC339xx_DUAL_MOTOR_QUICK_SETUP
#define MC339xx_DUAL_MOTOR_QUICK_SETUP 1
#endif

#if(!MC339xx_DUAL_MOTOR_QUICK_SETUP)

	typedef void(*MC339xx_InitialFn)(TIM_TypeDef*, TIM_OCInitTypeDef*);
	typedef void(*MC339xx_PreloadFn)(TIM_TypeDef*, uint16_t);

	typedef struct MC339xx_PWM_Group_x
	{
		GPIO_TypeDef*								port;
		uint32_t 										pin_rcc_periph;
		uint8_t 										pin_source;
		uint8_t 										pin_af;
		uint16_t 										pin_n;
		
		TIM_TypeDef*								timer;
		MC339xx_InitialFn						pin_init_fn;
		MC339xx_PreloadFn						pin_prel_fn;
	} MC339xx_PWM_Group_t;


	typedef struct MC339xx_x 
	{
		MC339xx_PWM_Group_t*				group[2U];
	} MC339xx_t;



	void Init_MC339xx( 
		MC339xx_t* 									motor, 
		MC339xx_PWM_Group_t* 	pwm_R,
		MC339xx_PWM_Group_t* 	pwm_L
	);

	void Cmd_MC339xx( MC339xx_t* motor, float speed );
		
	extern MC339xx_PWM_Group_t PE9_PWM;
	extern MC339xx_PWM_Group_t PE11_PWM;
	extern MC339xx_PWM_Group_t PE13_PWM;
	extern MC339xx_PWM_Group_t PE14_PWM;
		
	extern MC339xx_PWM_Group_t PD12_PWM;
	extern MC339xx_PWM_Group_t PD13_PWM;
	extern MC339xx_PWM_Group_t PD14_PWM;
	extern MC339xx_PWM_Group_t PD15_PWM;
	
#else//MC339xx_DUAL_MOTOR_QUICK_SETUP==1
	
	/////////////////////////////////////////
	/// FIXED-MODE Setup									///
	/////////////////////////////////////////
	///	LEFT-MOTOR												///
	/// ----------------------------------///
	/// TIM4-CH1 : PD-12	(In-1)					///
	/// TIM4-CH2 : PD-13	(In-2)					///
	/////////////////////////////////////////
	///	RIGHT-MOTOR												///
	/// ----------------------------------///
	/// TIM4-CH3 : PD-14	(In-1)					///
	/// TIM4-CH4 : PD-15	(In-2)					///
	/////////////////////////////////////////
	///
	///
	///
	///
	///--------------------------------------
	///	 M2FB				NC
	///	nM2SF				NC
	///	 M2D1				GND
	///	nM2D2				3v3/5v0 
	///	 M2IN1			{PD-12}
	///	 M2IN2			{PD-13}
	///	 INV				GND
	///	 SLEW				GND
	///	 EN					3v3/5v0 [or DIO]
	///	 M1FB				NC
	///	nM1SF				NC
	///	 M1D1				GND
	///	nM1D2				NC
	///	 M1IN1			{PD-14}
	///	 M1IN2			{PD-15}
	///	 VDD				3v3/5v0
	///	 GND				GND
	///  VIN				NC
	///--------------------------------------
	
	void Init_Dual_MC339xx(void);
	
	void Cmd_Dual_MC339xx( float speed_L, float speed_R );
	
#endif//MC339xx_DUAL_MOTOR_QUICK_SETUP

#endif