#ifndef	SYS_H
#define SYS_H

#define 	M_PI 					3.1416f
#define 	M_PI1_2				(M_PI/2.f)
#define 	M_PI1_4				(M_PI/4.f)
#define 	M_PI1_8				(M_PI/8.f)
#define 	M_PI1_16			(M_PI/16.f)
#define 	M_PI1_32			(M_PI/32.f)
#define 	M_PI1_64			(M_PI/64.f)

#define		FRESH_BATT_GAINS	0

#if(FRESH_BATT_GAINS)
#define CMD_G 	0.165f
#define K_P 		412.00f
#define K_D 		105.00f
#define K_I			 76.00f
#else
#define CMD_G 	0.175f
#define K_P 		412.00f
#define K_D 		105.00f
#define K_I			 81.00f
#endif

#include <stdint.h>

#include "stm32f4xx.h"
#include "LSM9DS0.h"		// imu
#include "AS504x.h"			// mag. encoder
#include "MC339xx.h"		// motor drivers


#define 	ARM_MATH_CM4 									1
#include 	<arm_math.h>

#define		TEST_OFF											0
#define		TEST_GYRO											1
#define		TEST_ACCL											2
#define		TEST_ENCODER									3

#define 	TESTING												TEST_OFF

#define 	SYS_MSG_HEADER								"SYS"
#define 	SYS_NCALIB_DELAY							100UL

#define 	PHY_BEAM_LEN									20.0f
#define 	PHY_ACC_OFF										10.0f
#define 	PHY_GYRO_OFF									10.0f

#define 	SYS_KP												1.0f
#define 	SYS_KD												1.0f
#define 	SYS_KDD												1.0f
#define 	SYS_KDI												1.0f
#define 	SYS_KDDI											1.0f


#define 	SYS_NCALIB_SAMPLES 						1000
#define 	SYS_BUFLEN 										10
#define 	SYS_FIR_NTAPS									10
#define 	SYS_TPS_BASE_F								1000.0f
#define 	SYS_F_TICKS(hz)								(uint32_t)(SYS_TPS_BASE_F/(float32_t)(hz))


#define 	SYS_BV(n)											(1<<n)
#define 	SYS_SET_FLAG(reg,n)						reg|=SYS_BV(n)
#define 	SYS_CLR_FLAG(reg,n)						reg&=~SYS_BV(n)
#define 	SYS_TIMED_ITEM_IMPL						uint32_t 	uticks;			\
																				uint32_t 	flags;			\
																				uint32_t 	period;
																				
#define 	SYS_ITEM_INIT(item_p)					item_p->uticks = item_p->flags = item_p->period = 0UL;
#define 	SYS_TRIGGER_UPDATE(item)			( (Sys.Time.ticks[tNow]-Sys.##item##.uticks) >= Sys.##item##.period) 
#define 	SYS_TRIGGER_RESET(item)				Sys.##item##.uticks = Sys.Time.ticks[tNow]
#define 	SYS_SENSOR_FLAG_ERROR(item)		SYS_SET_FLAG(Sys.##item##.flags,DevError)
#define 	SYS_SENSOR_FLAG_UPDATE(item)	SYS_SET_FLAG(Sys.##item##.flags,DoUpdate); SYS_CLR_FLAG(Sys.##item##.flags,DoIdle)
#define 	SYS_SENSOR_FLAG_IDLE(item)		SYS_SET_FLAG(Sys.##item##.flags,DoIdle);	 SYS_CLR_FLAG(Sys.##item##.flags,DoUpdate)	
#define 	SYS_SENSOR_FLAG_RESET(item)		Sys.##item##.flags = 0x0	


void 		__SysInitTime();
void 		__SysInitIMU();
void 		__SysInitMotor();

void 		__SysUpdateIMU();
void 		__SysUpdateController();
void 		__SysUpdateTime();
void 		__SysUpdateMotor();

void SysLoop();

#define SYS_INIT_IMPL	\
				__SysInitTime(); 					\
				__SysInitIMU();						\
				__SysInitMotor();					\
											

#define SYS_LOOP_IMPL	\
				__SysUpdateIMU();					\
				__SysUpdateController();	\
				__SysUpdateTime();				\
				__SysUpdateMotor();

typedef enum
{
	nX = 0U,
	nY = 1U,
	nZ = 2U
} Axis;


typedef enum
{
	tNow 	= 0U,
	tPrev 	= 1U
} Instances;


typedef enum 
{
	DoUpdate = 0U,
	DoIdle 	 = 1U,
	DevError = 2U,
	RngError = 3U,
} Flags;

#endif