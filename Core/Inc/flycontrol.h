#ifndef __FLYCONTROL_H__
#define __FLYCONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "icm42605.h"

#define MOTOR_MIN 0
#define MOTOR_MAX 4200
#define	THROTTLE_TO_PWM_MULTIPLE_SCALE	3.0f
#define 	THO_H		360.0f	//ALT_STABLE_REF_THROTTLE_HIGH
#define 	VOL_H		60.0f		//ALT_STABLE_REF_VOLTAGE_PERCENTAGE_HIGH
#define 	THO_L		439.0f	//ALT_STABLE_REF_THROTTLE_LOW
#define 	VOL_L		26.0f		//ALT_STABLE_REF_VOLTAGE_PERCENTAGE_LOW

#define ALT_STABLE_REF_THROTTLE							537.0f
#define ALT_STABLE_REF_VOLTAGE							3.648f
#define ALT_STABLE_REF_VOLTAGE_PERCENTAGE		31.0f
#define	BATTERY_STEP												0.008f

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
		float integral_max;
    float prev_error;
    float output;
    float dterm_lpf;
    float dterm_filtered;
} PID;

typedef enum{
	LANDED=0,
	LANDING,
	ANGLE_FLYING,
	ALTITUDE_HOLD_FLYING
}Drone_Flying_Status;

typedef struct {
    PID roll_outer;
    PID roll_inner;
    PID pitch_outer;
    PID pitch_inner;
    PID yaw_outer;
    PID yaw_inner;
    PID altitude;
    
    float target_roll;
    float target_pitch;
    float target_yaw;
    float throttle;
    float target_altitude;
    float current_altitude;
    float prev_altitude;
	
		float hover_throttle;
		Drone_Flying_Status flying_status;
    
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
} FlyController;

void FlyController_Init(FlyController *fc);
void FlyController_Update(FlyController *fc, ICM42605_Data *imu_data);
void FlyController_SetTarget(FlyController *fc, float roll, float pitch, float yaw, float throttle);
void FlyController_SetTargetAltitude(FlyController *fc, float target_altitude);
void FlyController_SetCurrentAltitude(FlyController *fc, float current_altitude);
void FlyController_Reset(FlyController *fc);
float Throttle_Voltage_Compensation(uint16_t bat_percentage);
void Update_Hover_Throttle(FlyController *fc,float hover_throttle);
#ifdef __cplusplus
}
#endif

#endif
