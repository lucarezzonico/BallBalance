#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <motors_speed.h>
#include <sensors/imu.h>

#define	MOTORS_SPEED_THD_SIZE			256
#define FREQUENCY						0.1							//kHz
#define STALL_ANGLE						(M_PI * 25/180) // 25° (angle de décrochage)
//#define	ERROR_ACC_THRESHOLD				0.3
#define	ACC_THRESHOLD					1.2
//#define	D								0.17
#define	G								9.81
//#define	MOTORS_SPEED_LIMIT_MS			0.13
#define	KI_ACC							4 //pow(MOTORS_SPEED_LIMIT_MS,2)/(2*D*sin(STALL_ANGLE)*G) / FREQUENCY 			// 5
//#define ERROR_YAW_ANGLE_THRESHOLD		(M_PI * 5/180) // 5°
//#define ERROR_PITCH_ANGLE_THRESHOLD		(M_PI * 5/180) // 5°
//#define PITCH_ANGLE_THRESHOLD			(M_PI * 0.2/180) // 0.2°
#define KP_ROTATION						(MOTOR_SPEED_LIMIT/(sin(STALL_ANGLE)*G/1.5))
#define KI_ROTATION 					0.1 / FREQUENCY	//must not be zero
#define KD_ROTATION						10 * FREQUENCY
#define ERROR_ROTATION_THRESHOLD		0.3
#define ROTATION_THRESHOLD				sqrt(2)*ERROR_ROTATION_THRESHOLD
#define MAX_SUM_ERROR 					(MOTOR_SPEED_LIMIT/KI_ROTATION)


//float get_yaw_angle(float *accel){
//	/*
//	* Quadrant:
//	*       FRONT
//	*       ####
//	*    #    0   #
//	*  #            #
//	* #PI/2 TOP -PI/2#
//	* #     VIEW     #
//	*  #            #
//	*    # PI|-PI #
//	*       ####
//	*       BACK
//	*/
//
//	// angle in rad with 0 being the front of the e-puck2 (-Y axis of the IMU)
//	// see quadrant above
//	float yaw_angle = atan2(accel[X_AXIS], -accel[Y_AXIS]);
//	if(yaw_angle > M_PI){
//		yaw_angle = -2 * M_PI + yaw_angle;
//	}
//	return yaw_angle;
//}

//float get_pitch_angle(float *accel){
//	/*
//	* Quadrant:
//	* 		#############################
//	* 		#		   -PI/2			#
//	* 		#							#
//	* FRONT # 0		 SIDE VIEW	 -PI|PI # BACK
//	* 		#							#
//	* 		#			PI/2			#
//	* 		#############################
//	*/
//
//	// angle in rad with 0 being the front of the e-puck2 (-Y axis of the IMU)
//	// see quadrant above
//	float pitch_angle = M_PI/2 - atan2(-accel[Z_AXIS], -accel[Y_AXIS]);
//	if(pitch_angle > M_PI){
//		pitch_angle = -2 * M_PI + pitch_angle;
//	}
//	return pitch_angle;
//}

// Integrate acceleration given by the imu to get the speed of the robot on a slope
float speed_sum_acceleration(float *accel){

	static float speed_acceleration = 0;

	speed_acceleration += (-accel[Y_AXIS]) * KI_ACC;

	if(fabs(speed_acceleration) < ACC_THRESHOLD){
		speed_acceleration = 0;
	}

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(speed_acceleration > MOTOR_SPEED_LIMIT){
		speed_acceleration = MOTOR_SPEED_LIMIT;
	} else if (speed_acceleration < -MOTOR_SPEED_LIMIT){
		speed_acceleration = -MOTOR_SPEED_LIMIT;
	}

	return speed_acceleration;
}

// PI regulator to regulate the direction angle of the robot with the imu
float imu_rotation_regulator(float *accel){

	float error = 0;
	float rotation_regulation = 0;

	static float sum_error = 0, previous_error;

	previous_error = error;
	error = accel[X_AXIS];

	//due to imprecise measure
	if (fabs(error) < ERROR_ROTATION_THRESHOLD)
		return 0;

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	} else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	if (sqrt(pow(accel[Y_AXIS],2)+pow(accel[X_AXIS],2)) > ROTATION_THRESHOLD)
	{
		rotation_regulation = KP_ROTATION  * error + KI_ROTATION  * sum_error + KD_ROTATION * (error - previous_error);		// KI = ???

		if (accel[Y_AXIS] > 0)
			return -rotation_regulation;
		else
			return rotation_regulation;
	}
	else
		return 0;
}

//motors control thread
static THD_WORKING_AREA(waMotors_speed, MOTORS_SPEED_THD_SIZE);
static THD_FUNCTION(Motors_speed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    static imu_msg_t imu_values;

    //create a pointer to the array for shorter name
    float *accel = imu_values.acceleration;

	systime_t time;

    float rotation_regulation = 0;
    float speed_acceleration = 0;
//    float target_angle = 0;
//    float yaw_angle = 0;

    while(1)
    {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	speed_acceleration = speed_sum_acceleration(accel);
/*
    	yaw_angle = get_yaw_angle(accel);
		if (fabs(yaw_angle) < M_PI/2){
			target_angle = 0;
		} else if (yaw_angle >= M_PI/2){
			target_angle = M_PI;
		} else {
			target_angle = -M_PI;
		}
*/

    	rotation_regulation = imu_rotation_regulator(accel);


    	right_motor_set_speed((int16_t)(speed_acceleration + rotation_regulation));
    	left_motor_set_speed((int16_t)(speed_acceleration - rotation_regulation));

    	//100Hz
    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

//start the thread
void motors_speed_start(void)
{
	chThdCreateStatic(waMotors_speed, sizeof(waMotors_speed), NORMALPRIO, Motors_speed, NULL);
}
