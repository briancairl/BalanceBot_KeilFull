#include "sys.h"
#define USING_HOME_BREW_PID 0
#define COMP_FILT_USE_PREFILT 0

typedef struct
{
	float32_t		state;
	float32_t		omega;
	float32_t		sigma;
	float32_t		var;
} EKF1f;

void Init_EFK1f( EKF1f* ekf, float32_t var )
{
	ekf->omega 	= 0.f;
	ekf->sigma 	= 100.f;
	ekf->state 	= 0.f;
	ekf->var 		= var;
}

void Update_EKF1f( EKF1f* ekf, float32_t* in )
{
	ekf->omega  = ekf->sigma/( ekf->sigma + ekf->var );
	ekf->state += ekf->omega*( *in - ekf->state );
	ekf->sigma  = (1.f-ekf->omega)*ekf->sigma;
	
	if(ekf->sigma<5e-3f)
		ekf->sigma = 5e-4f;
}


typedef struct 
{
	EKF1f				acc_pre_filter;
	EKF1f				gyr_pre_filter;
	float32_t		output;
} CompFilt1f;



#define COMP_FILT_ACC_SEN		2.00f
#define COMP_FILT_ACC_MAX		9.81f
#define	COMP_FILT_ACC_BIAS	0.0098f


void Init_CompFilt1f_x(
		CompFilt1f* est, 
		float32_t		acc_x_variance,
		float32_t		acc_y_variance,
		float32_t		acc_z_variance,
		float32_t		gyr_x_variance
){
	est->output = 0.f;

	float32_t	acc_xz_mag 		= sqrtf( powf(acc_x_variance,2.0f) + powf(acc_z_variance,2.0f) ); 
	float32_t	acc_var				= atan2f( acc_y_variance, acc_xz_mag );

	Init_EFK1f(&est->acc_pre_filter,acc_var);
	Init_EFK1f(&est->gyr_pre_filter,gyr_x_variance);
}


void Update_CompFilt1f_x( 
	CompFilt1f* est, 
	float32_t*	ax,
	float32_t*	ay,
	float32_t*	az,
	float32_t*	gx,
	float32_t*	dt
){
		/// Use accelerometer axies to form primitive orientation estimate
		float32_t	acc_xz_mag 		= sqrtf( powf(*ax,2.0f) + powf(*az,2.0f) ); 
		float32_t	acc_pitch			=-copysignf( atan2f( *ay, acc_xz_mag ), *ay );



#if(COMP_FILT_USE_PREFILT)
		/// Prefilter the accelerometer estimate
		Update_EKF1f( &est->acc_pre_filter, &acc_pitch );
		
		/// Prefilter the gyro reading
		Update_EKF1f( &est->gyr_pre_filter, gx );
	
		/// Update complementary filter
		est->output = (COMP_FILT_ACC_BIAS)*est->acc_pre_filter.state + (1.f-COMP_FILT_ACC_BIAS)*(est->output + est->gyr_pre_filter.state*(*dt));
#else
		est->output = (COMP_FILT_ACC_BIAS)*acc_pitch + (1.f-COMP_FILT_ACC_BIAS)*(est->output + (*gx)*(*dt));
#endif
}



typedef struct
{
	float32_t		gain;
	float32_t 	in;
	float32_t		out;
	float32_t		scaling;
	float32_t		data[20U];
	uint32_t 	data_itr;
	float32_t 	bias;
	uint32_t 		bias_n;
	float32_t 	var;
	uint32_t 		var_n;
	
} StatSen1f;



void Init_StatSen1f( StatSen1f* lfs, float32_t gain, float32_t scaling )
{	
	lfs->out 			= 0.f;
	
	lfs->bias 		= 0.f;
	lfs->bias_n 	= 0UL;
	
	lfs->var			= 0.f;
	lfs->var_n		= 0UL;
	
	lfs->gain 		= gain;
	lfs->scaling	= scaling;
	
	lfs->data_itr = 0UL;
}



void Update_StatSen1f( StatSen1f* lfs, int16_t* data )
{
	uint32_t itr;
	lfs->out = 0.f;
	
	lfs->data[lfs->data_itr++] = *data;
	
	lfs->data_itr %= 20UL;
	
	for( itr = 0; itr < 20UL; itr++ )
			lfs->out += lfs->data[itr];
	
	lfs->out *= lfs->scaling/20.f;
}


void NewBias_StatSen1f( StatSen1f* lfs )
{
	lfs->bias 	= 0.0f;
	lfs->bias_n	= 0UL;
}


void SetBias_StatSen1f( StatSen1f* lfs, int16_t* data )
{
	lfs->bias += lfs->scaling*((float32_t)*data);
	++lfs->bias_n;
}


void KeepBias_StatSen1f( StatSen1f* lfs )
{
	if(lfs->bias_n)
		lfs->bias/=((float32_t)(lfs->bias_n));
}


void NewVar_StatSen1f( StatSen1f* lfs )
{
	lfs->var 		= 0.0f;
	lfs->var_n	= 0UL;
}


void SetVar_StatSen1f( StatSen1f* lfs, int16_t* data )
{
	lfs->var += powf( ( lfs->scaling*((float32_t)*data) - lfs->bias ), 2.0f );
	++lfs->var_n;
}


void KeepVar_StatSen1f( StatSen1f* lfs )
{	
	if(lfs->var_n>1UL)
		lfs->var /=((float32_t)(lfs->var_n-1UL));
}


/*****************************************************************************************************/


static struct SYS_x
{
	struct TIME_x
	{
		uint32_t 						ticks[2U];
		float32_t 					time;
	} Time;	
	
	struct IMU_x
	{	SYS_TIMED_ITEM_IMPL
		StatSen1f						gyr_out[3U];
		StatSen1f						acc_out[3U];		
		LSM9DS0_t						hw_base;
	} Imu;
	
	struct MOTORS_x
	{	SYS_TIMED_ITEM_IMPL
		AS504x_t						enc_hw_base[2UL];
		float32_t						omega_l;
		float32_t						omega_r;
	} Motors;
	
	struct CMD_x
	{	SYS_TIMED_ITEM_IMPL
		float32_t 					dt;
		CompFilt1f					pitch_estimate;
		float32_t						pitch_setpoint;
		float32_t						d_pitch_estimate;
		float32_t						output;
		arm_pid_instance_f32	control_base;	
	} Cmd;
	
} Sys;



void __SysInitIMU()
{
	uint32_t itr, jtr;
	
	LSM9DS0_Init (&Sys.Imu.hw_base,MODE_SPI,0,0);
	LSM9DS0_begin(&Sys.Imu.hw_base, NULL );
	
	for( itr = 0UL; itr < 3UL; itr++ ) 
	{
		Init_StatSen1f(Sys.Imu.acc_out+itr,0.8f,(Sys.Imu.hw_base.aRes));
		Init_StatSen1f(Sys.Imu.gyr_out+itr,0.8f,(Sys.Imu.hw_base.gRes)/1.1f);
		
		NewBias_StatSen1f	(Sys.Imu.acc_out+itr);
		NewBias_StatSen1f	(Sys.Imu.gyr_out+itr);
		
		NewVar_StatSen1f	(Sys.Imu.acc_out+itr);
		NewVar_StatSen1f	(Sys.Imu.gyr_out+itr);
	}
	
	/// Accumulate Samples
	for( itr = 0; itr < SYS_NCALIB_SAMPLES; itr++ )
	{
		LSM9DS0_readAccel(&Sys.Imu.hw_base);
	
		SetBias_StatSen1f(Sys.Imu.acc_out+0UL,&Sys.Imu.hw_base.ax);
		SetBias_StatSen1f(Sys.Imu.acc_out+1UL,&Sys.Imu.hw_base.ay);
		SetBias_StatSen1f(Sys.Imu.acc_out+2UL,&Sys.Imu.hw_base.az);
	
		LSM9DS0_readGyro (&Sys.Imu.hw_base);
	
		SetBias_StatSen1f(Sys.Imu.gyr_out+0UL,&Sys.Imu.hw_base.gx);
		SetBias_StatSen1f(Sys.Imu.gyr_out+1UL,&Sys.Imu.hw_base.gy);
		SetBias_StatSen1f(Sys.Imu.gyr_out+2UL,&Sys.Imu.hw_base.gz);
	}
	
	/// Generate Sample Bias
	for( itr = 0UL; itr < 3UL; itr++ ) 
	{
		KeepBias_StatSen1f(Sys.Imu.acc_out+itr);
		KeepBias_StatSen1f(Sys.Imu.gyr_out+itr);
	}
	
	
	/// Accumulate Samples
	for( itr = 0; itr < SYS_NCALIB_SAMPLES; itr++ )
	{
		LSM9DS0_readAccel(&Sys.Imu.hw_base);
	
		SetVar_StatSen1f(Sys.Imu.acc_out+0UL,&Sys.Imu.hw_base.ax);
		SetVar_StatSen1f(Sys.Imu.acc_out+1UL,&Sys.Imu.hw_base.ay);
		SetVar_StatSen1f(Sys.Imu.acc_out+2UL,&Sys.Imu.hw_base.az);
	
		LSM9DS0_readGyro (&Sys.Imu.hw_base);
	
		SetVar_StatSen1f(Sys.Imu.gyr_out+0UL,&Sys.Imu.hw_base.gx);
		SetVar_StatSen1f(Sys.Imu.gyr_out+1UL,&Sys.Imu.hw_base.gy);
		SetVar_StatSen1f(Sys.Imu.gyr_out+2UL,&Sys.Imu.hw_base.gz);
	}
	
	/// Generate Sample Variance
	for( itr = 0UL; itr < 3UL; itr++ ) 
	{
		KeepVar_StatSen1f(Sys.Imu.acc_out+itr);
		KeepVar_StatSen1f(Sys.Imu.gyr_out+itr);
	}
	
	/// Print-out statistical information
	const char* axis = "XYZ";
	for( itr = 0UL; itr < 3UL; itr++ ) 
		printf( "ACC-%c | M : %f | S : %f\n", axis[itr], Sys.Imu.acc_out[itr].bias , Sys.Imu.acc_out[itr].var );
	
	for( itr = 0UL; itr < 3UL; itr++ ) 
		printf( "GYR-%c | M : %f | S : %f\n", axis[itr], Sys.Imu.gyr_out[itr].bias , Sys.Imu.gyr_out[itr].var	);

	
	/// Callibrate goal set-pooint
	float32_t avg_setpoint = 0.f;
	for( itr = 0UL; itr < 10000UL; itr++ )
	{
		__SysUpdateIMU();
		avg_setpoint += Sys.Cmd.pitch_estimate.output;
	}
	Sys.Cmd.pitch_setpoint = avg_setpoint/10000.f;
	
	printf("SET POINT TARGET %f\n",Sys.Cmd.pitch_setpoint);
}


void __SysUpdateIMU()
{
	float32_t mdt;
	if(SYS_TRIGGER_UPDATE(Imu))
	{		SYS_TRIGGER_RESET(Imu);
		
		/// Read Accelerometer
		LSM9DS0_readAccel(&Sys.Imu.hw_base);
		{
			Update_StatSen1f(Sys.Imu.acc_out+0UL,&Sys.Imu.hw_base.ax);
			Update_StatSen1f(Sys.Imu.acc_out+1UL,&Sys.Imu.hw_base.ay);
			Update_StatSen1f(Sys.Imu.acc_out+2UL,&Sys.Imu.hw_base.az);
		}
		
		/// Read Gyroscope
		LSM9DS0_readGyro (&Sys.Imu.hw_base);
		{
			Update_StatSen1f(Sys.Imu.gyr_out+0UL,&Sys.Imu.hw_base.gx);
			Update_StatSen1f(Sys.Imu.gyr_out+1UL,&Sys.Imu.hw_base.gy);
			Update_StatSen1f(Sys.Imu.gyr_out+2UL,&Sys.Imu.hw_base.gz);
		}
		
		
		///@todo Fix gyro-scalling hard define
		Sys.Cmd.d_pitch_estimate = (Sys.Imu.gyr_out[nX].out-Sys.Imu.gyr_out[nX].bias);///10.f;
		if( fabs(Sys.Cmd.d_pitch_estimate) > 5.0f )
			Sys.Cmd.d_pitch_estimate = copysignf(5.f,Sys.Cmd.d_pitch_estimate);
		
		mdt = Sys.Cmd.dt*(1000.f/(float32_t)Sys.Imu.period);
		/// Update Pitch estimate
		Update_CompFilt1f_x( 
			&Sys.Cmd.pitch_estimate, 
			&Sys.Imu.acc_out[nX].out,
			&Sys.Imu.acc_out[nY].out,
			&Sys.Imu.acc_out[nZ].out,
			&Sys.Cmd.d_pitch_estimate,
			&Sys.Cmd.dt
		);
	}
}


/* Encoder Pin Setup
 * 
 * U1
 * ----------------------
 * PIN 	|		W.COL. 	|	FN	
 * ----------------------
 * PE4	|			G			|	CS
 * PE2 	|			P			| CLK
 * PE0	|			B			| SD
 *
 * U2
 * ----------------------
 * PIN 	|		W.COL. 	|	FN	
 * ----------------------
 * PD5	|			B			| SD
 * PD3	|			P			| CLK
 * PD1	|	 		G			| CS
 *
 */
#define L_ENCODER 	(Sys.Motors.enc_hw_base + 0U)
#define R_ENCODER 	(Sys.Motors.enc_hw_base + 1U)
void __SysInitMotors()
{
	#if(USING_ENCODERS)
	AS504x_Init( L_ENCODER );
	AS504x_SetPin( L_ENCODER, RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_0, AS504x_IN );
	AS504x_SetPin( L_ENCODER, RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_4, AS504x_CS );
	AS504x_SetPin( L_ENCODER, RCC_AHB1Periph_GPIOE, GPIOE, GPIO_Pin_2, AS504x_CLK );
	
	AS504x_Init( R_ENCODER );
	AS504x_SetPin( R_ENCODER, RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_5, AS504x_IN );
	AS504x_SetPin( R_ENCODER, RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_1, AS504x_CS );
	AS504x_SetPin( R_ENCODER, RCC_AHB1Periph_GPIOD, GPIOD, GPIO_Pin_3, AS504x_CLK );
	#endif
	
	Sys.Motors.omega_l = Sys.Motors.omega_r = 0.0f;
	
	Init_Dual_MC339xx();
}


void __SysUpdateMotors()
{
	if(SYS_TRIGGER_UPDATE(Motors))
	{	SYS_TRIGGER_RESET(Motors);
		
		#if(USING_ENCODERS)
		AS504x_Read(L_ENCODER);
		AS504x_UpdateVelocity(L_ENCODER);
		
		AS504x_Read(R_ENCODER);
		AS504x_UpdateVelocity(R_ENCODER);		
		#endif
		
		Cmd_Dual_MC339xx(Sys.Motors.omega_l/60.f,Sys.Motors.omega_r/60.f);
	}
}



void __SysInitCmd()
{
	Sys.Cmd.dt 							= 0.f;
	Sys.Cmd.pitch_setpoint	= -0.01f;
	
	/// Initialize Auto-PID
	Sys.Cmd.control_base.Kp = K_P*CMD_G;
	Sys.Cmd.control_base.Kd = K_D*CMD_G;
	Sys.Cmd.control_base.Ki = K_I*CMD_G;
	
	#if(USING_HOME_BREW_PID==0)
	arm_pid_init_f32(&Sys.Cmd.control_base,1U);
	#endif
	
	/// Should be called after sample variances have been acquired!
	Init_CompFilt1f_x( 
		&Sys.Cmd.pitch_estimate,
		 Sys.Imu.acc_out[nX].var,
		 Sys.Imu.acc_out[nY].var,
		 Sys.Imu.acc_out[nZ].var,
		 Sys.Imu.gyr_out[nX].var
	);
}





static float32_t cmd_out = 0.f;

void __SysUpdateCmd()
{		
	static float32_t d_error = 0.f, int_error = 0.f;
	float32_t error;
	
	
	if(SYS_TRIGGER_UPDATE(Cmd))
	{	
	/// Generate new DT estimate
	SYS_TRIGGER_RESET(Cmd);		

		
		#if(USING_HOME_BREW_PID)
		
		d_error += 0.1f*(Sys.Cmd.d_pitch_estimate-d_error);
		
		/// Caluclate Pitching error
		error 	= (Sys.Cmd.pitch_estimate.output-Sys.Cmd.pitch_setpoint);
				
		

		
		/// Kill integral term if error is small
		if( fabs(int_error) < I_E_MAX )
			int_error += error*.01f;
		else
			int_error = copysignf(I_E_MAX,int_error);
		
		/// Do PID
		#define DEADBAND  0.1f
	
		Sys.Cmd.output = 
		CMD_G*(
			K_P*error 		+ 
			K_D*d_error 	+
			K_I*int_error
		);
		//if( fabsf(Sys.Cmd.output) < DEADBAND )
		//{
		//		Sys.Cmd.output 	= copysignf(DEADBAND*powf(Sys.Cmd.output/DEADBAND,8.f),Sys.Cmd.output);
		//}
		//if(error < 0.f)
		//		Sys.Cmd.output *= 1.2f;
		
		#else

		/// Caluclate Pitching error
		error 	= (Sys.Cmd.pitch_estimate.output-Sys.Cmd.pitch_setpoint);
		
				//printf("%f %f\n",error,d_error);

		Sys.Cmd.output = 0.95f*(arm_pid_f32(&Sys.Cmd.control_base,error)-Sys.Cmd.output);
		if(fabsf(Sys.Cmd.output) > 120.f )
		{
			arm_pid_reset_f32(&Sys.Cmd.control_base);
			
			
			Sys.Cmd.output = copysignf(120.f,Sys.Cmd.output);
		}
		#endif

		
		/// For now, just try to balance
		if(error > M_PI1_4 )
		{
			Sys.Motors.omega_l = 
			Sys.Motors.omega_r = 0.f;
		}
		else
		{
			Sys.Motors.omega_l = 
			Sys.Motors.omega_r = 
				Sys.Cmd.output;
		}		
	}
}


/// Count system ticks
void SysTick_Handler()
{
	++Sys.Time.ticks[tNow];
}


void SysLoop()
{
	__SysInitCmd();
	__SysInitIMU();
	__SysInitMotors();

	Sys.Imu.period 		= SYS_F_TICKS(1000.f);
	Sys.Cmd.period 		= SYS_F_TICKS(500.f);
	Sys.Motors.period = SYS_F_TICKS(100.f);
		for(;;)
	{
		Sys.Cmd.dt 						+= 0.25f*( (float32_t)(Sys.Time.ticks[tNow]-Sys.Time.ticks[tPrev])/1000.f - Sys.Cmd.dt);
		Sys.Time.ticks[tPrev]  = Sys.Time.ticks[tNow];

		__SysUpdateIMU();
		__SysUpdateCmd();
		__SysUpdateMotors();
	}
}