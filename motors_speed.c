#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <motors_speed.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>

#define	MOTORS_SPEED_THD_SIZE			512
#define	IMU_MEAN_VALUE_THD_SIZE			512
#define	FREQUENCY						50		//50Hz
#define NB_SAMPLE						5
#define STALL_ANGLE						(M_PI * 25/180) // 25° (angle de décrochage)
#define	ERROR_ACC_THRESHOLD				0.5
#define	ACC_THRESHOLD					10
#define	BRAKING							pow(0.99,100/FREQUENCY)
//#define	D								0.17
#define	G								9.81
//#define	MOTORS_SPEED_LIMIT_MS			0.13
#define	KI_ACC							4 * 100/FREQUENCY //pow(MOTORS_SPEED_LIMIT_MS,2)/(2*D*sin(STALL_ANGLE)*G) / FREQUENCY 			// 5
//#define ERROR_YAW_ANGLE_THRESHOLD		(M_PI * 5/180) // 5°
//#define ERROR_PITCH_ANGLE_THRESHOLD		(M_PI * 5/180) // 5°
//#define PITCH_ANGLE_THRESHOLD			(M_PI * 0.2/180) // 0.2°
#define KP_ROTATION						(MOTOR_SPEED_LIMIT/(sin(STALL_ANGLE)*G/2))
#define KI_ROTATION 					1 * 100/FREQUENCY	//must not be zero
#define KD_ROTATION						10 * FREQUENCY/100
//#define ROTATION_THRESHOLD				sqrt(2)*ERROR_ACC_THRESHOLD
#define MAX_SUM_ERROR 					(MOTOR_SPEED_LIMIT/KI_ROTATION)
#define ROTATION_CORRECTION				200
#define	PROX_LIMIT						500

enum{
	IR1,
	IR2,
	IR3,
	IR4,
	IR5,
	IR6,
	IR7,
	IR8
};

static float mean_accel[NB_AXIS];

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
float speed_sum_acceleration(unsigned int *delta){

	static float speed_acceleration = 0, previous_speed;

	previous_speed = speed_acceleration;

	if (fabs(mean_accel[Y_AXIS]) > ERROR_ACC_THRESHOLD){
		speed_acceleration += (-mean_accel[Y_AXIS]) * KI_ACC;
	} else {
		if (fabs(speed_acceleration) > ACC_THRESHOLD)
			speed_acceleration *= BRAKING;
		else
			speed_acceleration = 0;
	}

	//if speed_acceleration changes sign, we set the speed to 0 for 10ms
	if (speed_acceleration*previous_speed < 0)
		speed_acceleration = 0;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(speed_acceleration > MOTOR_SPEED_LIMIT){
		speed_acceleration = MOTOR_SPEED_LIMIT;
	} else if (speed_acceleration < -MOTOR_SPEED_LIMIT){
		speed_acceleration = -MOTOR_SPEED_LIMIT;
	}

	if ((speed_acceleration > 0 && (delta[IR1] > PROX_LIMIT || delta[IR2] > PROX_LIMIT || delta[IR7] > PROX_LIMIT || delta[IR8] > PROX_LIMIT)) ||
		(speed_acceleration < 0 && (delta[IR4] > PROX_LIMIT || delta[IR5] > PROX_LIMIT)))
		speed_acceleration = 0;

	return speed_acceleration;
}

// PID regulator to regulate the direction angle of the robot with the imu
float imu_rotation_regulator(void){

	float error = 0;
	float rotation_regulation = 0;

	static float sum_error = 0, previous_error;

	previous_error = error;
	error = mean_accel[X_AXIS];

	//due to imprecise measure
	if (fabs(error) < ERROR_ACC_THRESHOLD)
	{
		error = 0;
		sum_error = 0;
		previous_error = 0;
	}

	if (mean_accel[Y_AXIS] > 0)
		error = -error;

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	} else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

//	if (sqrt(pow(accel[Y_AXIS],2)+pow(accel[X_AXIS],2)) > ROTATION_THRESHOLD)
//	{
//		rotation_regulation = KP_ROTATION  * error + KI_ROTATION  * sum_error + KD_ROTATION * (error - previous_error);		// KI = ???
//
//		if (accel[Y_AXIS] > 0)
//			return -rotation_regulation;
//		else
//			return rotation_regulation;
//	}
//	else
//		return 0;

	rotation_regulation = KP_ROTATION  * error + KI_ROTATION  * sum_error + KD_ROTATION * (error - previous_error);

	return rotation_regulation;
}

//motors control thread
static THD_WORKING_AREA(waMotors_speed, MOTORS_SPEED_THD_SIZE);
static THD_FUNCTION(Motors_speed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

//    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
//    static imu_msg_t imu_values;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    static proximity_msg_t prox_values;

    //create a pointer to the array for shorter name
//    float *accel = imu_values.acceleration, *previous_accel = NULL;
    unsigned int *delta = prox_values.delta;

	systime_t time;

    float rotation_regulation = 0;
    float speed_acceleration = 0;
//    float target_angle = 0;
//    float yaw_angle = 0;

    while(true)
    {
    	time = chVTGetSystemTime();

//    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));		//250Hz
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));	//100Hz

    	speed_acceleration = speed_sum_acceleration(delta);

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

    	rotation_regulation = imu_rotation_regulator();

    	if ((speed_acceleration > 0 && delta[IR3] > PROX_LIMIT) ||
    		(speed_acceleration < 0 && delta[IR6] > PROX_LIMIT))
    		rotation_regulation += ROTATION_CORRECTION;
    	else if ((speed_acceleration > 0 && delta[IR6] > PROX_LIMIT) ||
    			 (speed_acceleration < 0 && delta[IR3] > PROX_LIMIT))
    		rotation_regulation -= ROTATION_CORRECTION;

    	right_motor_set_speed((int16_t)(speed_acceleration + rotation_regulation));
    	left_motor_set_speed((int16_t)(speed_acceleration - rotation_regulation));

    	chprintf((BaseSequentialStream *)&SD3, "mean_Ax = %f		mean_Ay = %f		\r\n\n", mean_accel[X_AXIS], mean_accel[Y_AXIS]);

//    	if (previous_accel && previous_accel[Y_AXIS]*accel[Y_AXIS] < 0)
//    		chThdSleepMilliseconds(100/FREQUENCY);
//    	else
    		//50Hz
    		chThdSleepUntilWindowed(time, time + MS2ST(1000/FREQUENCY));
    }
}

static THD_WORKING_AREA(waImu_mean_value, IMU_MEAN_VALUE_THD_SIZE);
static THD_FUNCTION(Imu_mean_value, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    static imu_msg_t imu_values;

    float *accel = imu_values.acceleration;

    static float accel_sample[NB_AXIS][NB_SAMPLE];

	float sum_x_axis = 0, sum_y_axis = 0, sum_z_axis = 0;

	uint8_t counter;

    for (uint8_t i = 0; i < NB_AXIS; i++)
    {
    	for (counter = 0; counter < NB_SAMPLE; counter++)
    		accel_sample[i][counter] = 0;
    }

    counter = 0;

    while(true)
        {
        	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));		//250Hz

        	sum_x_axis -= accel_sample[X_AXIS][counter];
        	sum_y_axis -= accel_sample[Y_AXIS][counter];
        	sum_z_axis -= accel_sample[Z_AXIS][counter];

        	accel_sample[X_AXIS][counter] = accel[X_AXIS];
        	accel_sample[Y_AXIS][counter] = accel[Y_AXIS];
        	accel_sample[Z_AXIS][counter] = accel[Z_AXIS];

        	sum_x_axis += accel_sample[X_AXIS][counter];
        	sum_y_axis += accel_sample[Y_AXIS][counter];
        	sum_z_axis += accel_sample[Z_AXIS][counter];

        	mean_accel[X_AXIS] = sum_x_axis/NB_SAMPLE;
        	mean_accel[Y_AXIS] = sum_y_axis/NB_SAMPLE;
        	mean_accel[Z_AXIS] = sum_z_axis/NB_SAMPLE;

        	counter++;

        	if (counter >= NB_SAMPLE)
        		counter = 0;
        }
}

//start the thread
void motors_speed_start(void)
{
	chThdCreateStatic(waMotors_speed, sizeof(waMotors_speed), NORMALPRIO, Motors_speed, NULL);
	chThdCreateStatic(waImu_mean_value, sizeof(waImu_mean_value), NORMALPRIO, Imu_mean_value, NULL);
}
