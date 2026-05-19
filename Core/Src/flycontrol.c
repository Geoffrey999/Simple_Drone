#include "flycontrol.h"
#include "filter.h"
#include <string.h>

#define RAD_TO_DEG (57.29577951308232f)

static float PID_Update(PID *pid, float error, float dt)
{
	pid->integral += error * dt;

	if (pid->integral_max > 0.0f)
	{
		if (pid->integral > pid->integral_max) 
		{
			pid->integral = pid->integral_max;
		}
		else if (pid->integral < -pid->integral_max)
		{
			pid->integral = -pid->integral_max;
		}
	}

	float derivative = (error - pid->prev_error) / dt;
	
	float alpha = pid->dterm_lpf;
	if (alpha <= 0.0f || alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	pid->dterm_filtered = alpha * derivative + (1.0f - alpha) * pid->dterm_filtered;
	pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->dterm_filtered;
	pid->prev_error = error;
	return pid->output;
}

static void PID_Reset(PID *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->output = 0.0f;
	pid->dterm_filtered = 0.0f;
}

void FlyController_Init(FlyController *fc)
{
	memset(fc, 0, sizeof(FlyController));

	fc->roll_outer.kp = 7.0f;
	fc->roll_outer.ki = 0.6f;
	fc->roll_outer.kd = 0.12f;
	fc->roll_outer.integral_max = 600.0f;
	fc->roll_outer.dterm_lpf = 0.16f;

	fc->pitch_outer.kp = 7.0f;
	fc->pitch_outer.ki = 0.6f;
	fc->pitch_outer.kd = 0.12f;
	fc->pitch_outer.integral_max = 600.0f;
	fc->pitch_outer.dterm_lpf = 0.16f;
 
	fc->yaw_outer.kp = 2.0f;
	fc->yaw_outer.ki = 0.1f;
	fc->yaw_outer.kd = 0.02f;
	fc->yaw_outer.integral_max = 200.0f;
	fc->yaw_outer.dterm_lpf = 1.0f;

	fc->roll_inner.kp = 10.2f;
	fc->roll_inner.ki = 0.3f;//0.6
	fc->roll_inner.kd = 0.05f;
	fc->roll_inner.integral_max = 300.0f;
	fc->roll_inner.dterm_lpf = 0.12f;

	fc->pitch_inner.kp = 10.2f;
	fc->pitch_inner.ki = 0.3f;
	fc->pitch_inner.kd = 0.05f;  
	fc->pitch_inner.integral_max = 300.0f;
	fc->pitch_inner.dterm_lpf = 0.12f;

	fc->yaw_inner.kp = 6.0f;
	fc->yaw_inner.ki = 0.0f;
	fc->yaw_inner.kd = 0.06f;
	fc->yaw_inner.integral_max = 800.0f;
	fc->yaw_inner.dterm_lpf = 0.12f;
	
	fc->altitude.kp = 160.0f;
	fc->altitude.ki = 2.0f;
	fc->altitude.kd = 5200.0f;
	fc->altitude.integral_max = 300.0f;
	fc->altitude.dterm_lpf = 0.08f;

	fc->target_roll = 0.0f;
	fc->target_pitch = 0.0f;
	fc->target_yaw = 0.0f;
	fc->throttle = 0.0f;
	fc->target_altitude = 0.0f;
	fc->current_altitude = 0.0f;
	fc->prev_altitude = 0.0f;
	
	fc->motor1 = MOTOR_MIN;
	fc->motor2 = MOTOR_MIN;
	fc->motor3 = MOTOR_MIN;
	fc->motor4 = MOTOR_MIN;
}

void FlyController_Update(FlyController *fc, ICM42605_Data *imu_data)
{
	float dt = imu_data->sample_period;
	if (dt <= 0.0f) {
			dt = 0.001f;
	}
	
	float current_roll = imu_data->angle.roll;
	float current_pitch = imu_data->angle.pitch;
	float current_yaw = imu_data->angle.yaw;
	
	float gyro_x = imu_data->gyro_x * RAD_TO_DEG;
	float gyro_y = imu_data->gyro_y * RAD_TO_DEG;
	float gyro_z = imu_data->gyro_z * RAD_TO_DEG;
	
	float roll_error = fc->target_roll - current_roll;
	float pitch_error = fc->target_pitch - current_pitch;
	float yaw_error = fc->target_yaw - current_yaw;
	
	if (yaw_error > 180.0f) {
			yaw_error -= 360.0f;
	} else if (yaw_error < -180.0f) {
			yaw_error += 360.0f;
	}
	
	float roll_outer_output = PID_Update(&fc->roll_outer, roll_error, dt);
	float pitch_outer_output = PID_Update(&fc->pitch_outer, pitch_error, dt);
	float yaw_outer_output = PID_Update(&fc->yaw_outer, yaw_error, dt);
	
	float roll_inner_error = roll_outer_output - gyro_y;
	float pitch_inner_error = pitch_outer_output + gyro_x;
	float yaw_inner_error = yaw_outer_output + gyro_z;
	
	float roll_output = PID_Update(&fc->roll_inner, roll_inner_error, dt);
	float pitch_output = PID_Update(&fc->pitch_inner, pitch_inner_error, dt);
	float yaw_output = PID_Update(&fc->yaw_inner, yaw_inner_error, dt);
	
	float altitude_output = 0.0f;
	static float saved_altitude_output = 0.0f;
	
	if (fc->flying_status == HORIZON_FLYING)
	{
		saved_altitude_output = 0.0f;

	}
	else
	{
		float altitude_error = fc->target_altitude - fc->current_altitude;
		if(altitude_error>0.6f)altitude_error=0.6f;
		if(altitude_error<-0.6f)altitude_error=-0.6f;
		saved_altitude_output = PID_Update(&fc->altitude, altitude_error, 0.02f);
		fc->prev_altitude = fc->current_altitude;
		altitude_output = saved_altitude_output;		 
	}

	float final_throttle = fc->throttle + altitude_output;
	
	if(imu_data->tilt_total < 60.0f)
	{
		final_throttle = 1/cosf(imu_data->tilt_total/RAD_TO_DEG) * final_throttle;
	}

	if((fabsf(imu_data->accel_z-0.995f)<0.1f)&&(fabsf(imu_data->tilt_total)<2.0f)&&(fc->flying_status == ALTITUDE_HOLD_FLYING))
	{
		Update_Hover_Throttle(fc,final_throttle);
	}

	
	float m1 = final_throttle + pitch_output - roll_output + yaw_output;
	float m2 = final_throttle - pitch_output - roll_output - yaw_output;
	float m3 = final_throttle + pitch_output + roll_output - yaw_output;
	float m4 = final_throttle - pitch_output + roll_output + yaw_output;	

	if (m1 < MOTOR_MIN) m1 = MOTOR_MIN;
	if (m1 > MOTOR_MAX) m1 = MOTOR_MAX;
	if (m2 < MOTOR_MIN) m2 = MOTOR_MIN;
	if (m2 > MOTOR_MAX) m2 = MOTOR_MAX;
	if (m3 < MOTOR_MIN) m3 = MOTOR_MIN;
	if (m3 > MOTOR_MAX) m3 = MOTOR_MAX;
	if (m4 < MOTOR_MIN) m4 = MOTOR_MIN;
	if (m4 > MOTOR_MAX) m4 = MOTOR_MAX;		
	
	fc->motor1 = (uint16_t)m1;
	fc->motor2 = (uint16_t)m2;
	fc->motor3 = (uint16_t)m3;
	fc->motor4 = (uint16_t)m4;		

}

void FlyController_SetTarget(FlyController *fc, float roll, float pitch, float yaw, float throttle)
{
	fc->target_roll = roll;
	fc->target_pitch = pitch;
	fc->target_yaw = yaw;
	
	if (throttle < MOTOR_MIN)
	{
		fc->throttle = MOTOR_MIN;
	}
	else if (throttle > MOTOR_MAX)
	{
		fc->throttle = MOTOR_MAX;
	}
	else
	{
		fc->throttle = throttle;
	}
}

void FlyController_Reset(FlyController *fc)
{
	PID_Reset(&fc->roll_outer);
	PID_Reset(&fc->roll_inner);
	PID_Reset(&fc->pitch_outer);
	PID_Reset(&fc->pitch_inner);
	PID_Reset(&fc->yaw_outer);
	PID_Reset(&fc->yaw_inner);
	PID_Reset(&fc->altitude);
	
	fc->target_roll = 0.0f;
	fc->target_pitch = 0.0f;
	fc->target_yaw = 0.0f;
	fc->target_altitude = 0.0f;
	
	fc->motor1 = MOTOR_MIN;
	fc->motor2 = MOTOR_MIN;
	fc->motor3 = MOTOR_MIN;
	fc->motor4 = MOTOR_MIN;
}

void FlyController_SetTargetAltitude(FlyController *fc, float target_altitude)
{
   fc->target_altitude = target_altitude;
}

void FlyController_SetCurrentAltitude(FlyController *fc, float current_altitude)
{
   fc->current_altitude = current_altitude;
}

void Update_Hover_Throttle(FlyController *fc,float hover_throttle)
{
	static LowPassFilter_t hover_throttle_filter;
	static uint8_t lpf_init_flag=0;
	if(lpf_init_flag==0)
	{
		LowPass_Init(&hover_throttle_filter,0.08f);
		lpf_init_flag = 1;
	}
	fc->hover_throttle = LowPass_Update(&hover_throttle_filter,hover_throttle);
}

float Throttle_Voltage_Compensation(uint16_t bat_percentage)
{
	float compensated_throttle;
//	compensated_throttle = (ALT_STABLE_REF_VOLTAGE / (ALT_STABLE_REF_VOLTAGE + (BATTERY_STEP * (bat_percentage - ALT_STABLE_REF_VOLTAGE_PERCENTAGE))) * ALT_STABLE_REF_THROTTLE)*3.0f;
	compensated_throttle = (((THO_H+THO_L)/2) + ((THO_H-THO_L)*(2*bat_percentage-VOL_H-VOL_L))/(2*(VOL_H-VOL_L))) * THROTTLE_TO_PWM_MULTIPLE_SCALE;
	return compensated_throttle;
}
