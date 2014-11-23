#include <stdio.h>
#include <stdlib.h>
#include <math.h>






#include "sys.h"

/// CONNECTIONS
/////////////////////
/// PD10 : CS 				<-- LSM-CSXM
/// PB12 : CS 				<-- LSM-CSG
/// PB13 : SPI2-SCK		<-- LSM-SCK(SCL)
/// PB14 : SPI2-MISO	<-- LSM-MISO
/// PB15 : SPI2-MOSI	<-- LSM-MOSI(SDA)


#define READ_MODE 3
#define DEG(x)	(x/3.14f)*180.0f

volatile uint32_t msTicks = 0;                      /* counts 1ms timeTicks       */


// initialize the system tick 
void InitSystick(void){
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

static uint32_t 	msPrev = 0;

#if((TESTING==TEST_GYRO)||(TESTING==TEST_ACCL))
static LSM9DS0_t 	imu, *imu_p = &imu;
#endif

static AS504x_t		encoder[2UL];


int32_t main(void)
{
	
		/// System Inits
	SystemInit();
	InitSystick();
	
	/// IMU Inits
	#if((TESTING==TEST_GYRO)||(TESTING==TEST_ACCL))
	
	LSM9DS0_Init( imu_p, MODE_SPI, 0x00, 0x00 );
	LSM9DS0_begin( imu_p, NULL );
	
	#elif(TESTING==TEST_ENCODER)
	
	AS504x_Init( encoder );
	AS504x_SetPin( encoder, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_15, AS504x_IN );
	AS504x_SetPin( encoder, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_14, AS504x_CS );
	AS504x_SetPin( encoder, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_13, AS504x_CLK );
	
	#elif(TESTING==TEST_OFF)
	
	//SYS_INIT_IMPL;
	
	#endif
  
	#if(TESTING==TEST_OFF)
		
	SysLoop();
		
	
	#else
	while(1)
	{
		#if(TESTING==TEST_ENCODER)
		
		if( (msTicks-msPrev) > 10UL )
		{	msPrev= msTicks;
			
			AS504x_Read(encoder);
		
			AS504x_UpdateVelocity(encoder);
		
			printf("P: %f, V : %f\n", AS504x_CURRENT_ANGLE(encoder[0]) , encoder->velocity/10.0f );
		}
		
		#elif(TESTING==TEST_GYRO)
		LSM9DS0_readGyro(imu_p);
		
		printf("%d, %d, %d\n",
			imu_p->gx,
			imu_p->gy,
			imu_p->gz
		);
		
		#elif(TESTING==TEST_ACCL)
		LSM9DS0_readAccel(imu_p);
		
		printf("%d, %d, %d\n",
			imu_p->ax,
			imu_p->ay,
			imu_p->az
		);
		#endif
	}
	#endif
}
