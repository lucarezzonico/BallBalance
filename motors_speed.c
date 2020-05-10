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

#define	G							9.81													//m/s²
#define	MOTORS_SPEED_THD_SIZE		256
#define	IMU_MEAN_VALUE_THD_SIZE		512
#define	FREQUENCY					50														//Hz
#define NB_SAMPLE					5														//number of sample to calculate the mean value of the imu
#define STALL_ANGLE					M_PI * 25/180 											//25°
#define	ERROR_ACC_THRESHOLD			0.5														//0.5 m/s²
#define	SPEED_THRESHOLD				10														//10 step/s
#define	BRAKING						pow(0.99,100/FREQUENCY)									//multiply the speed by 0.99 each time it passes through the thread
#define	KI_ACC						(MOTOR_SPEED_LIMIT/(sin(STALL_ANGLE)*G/1.5)) / FREQUENCY//we have computed the coefficient at 100Hz
#define KP_ROTATION					MOTOR_SPEED_LIMIT / (sin(STALL_ANGLE)*G/2)				//we set the maximum speed at half of the stall slope
#define KI_ROTATION 				1 * 100/FREQUENCY										//we have computed the coefficients at 100Hz
#define KD_ROTATION					10 * FREQUENCY/100
#define MAX_SUM_ERROR 				MOTOR_SPEED_LIMIT/KI_ROTATION
#define ROTATION_CORRECTION			200
#define	PROX_LIMIT					500														//500 corresponds to a reasonable distance to see the effect; 800 corresponds to the limit

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

//a table with the mean acceleration on the 3 axis
static float mean_accel[NB_AXIS];

//integrate the acceleration given by the imu to get the speed of the robot on a slope
float speed_sum_acceleration(unsigned int *delta){

	static float speed = 0;

	//due to imprecise measure
	//we subtract the current mean acceleration because forward direction is -y
	if (fabs(mean_accel[Y_AXIS]) > ERROR_ACC_THRESHOLD)
		speed -= mean_accel[Y_AXIS] * KI_ACC;
	else
	{
		//if the e-puck had a speed and we put it at the horizontal, the e-puck slowly brakes
		if (fabs(speed) > SPEED_THRESHOLD)
			speed *= BRAKING;
		else
			speed = 0;
	}

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if (speed > MOTOR_SPEED_LIMIT)
		speed = MOTOR_SPEED_LIMIT;
	else if (speed < -MOTOR_SPEED_LIMIT)
		speed = -MOTOR_SPEED_LIMIT;

	//check for the presence of a wall in front or back of the e_puck when e_puck goes respectively forward or backward
	if ((speed > 0 && (delta[IR1] > PROX_LIMIT || delta[IR8] > PROX_LIMIT)) ||
		(speed < 0 && ((delta[IR4] > PROX_LIMIT && (delta[IR5] > PROX_LIMIT || delta[IR3] < PROX_LIMIT)) ||
				       (delta[IR5] > PROX_LIMIT && (delta[IR4] > PROX_LIMIT || delta[IR6] < PROX_LIMIT)))))
		speed = 0;

	return speed;
}

//PID regulator to regulate the direction angle of the robot with the imu
float imu_rotation_regulator(void){

	float error = 0;

	static float sum_error = 0, previous_error;

	previous_error = error;						//simulate the derivative term
	error = mean_accel[X_AXIS];					//we want to put the acceleration on x axis to 0

	//due to imprecise measure
	if (fabs(error) < ERROR_ACC_THRESHOLD)
	{
		error = 0;
		sum_error = 0;
		previous_error = 0;
	}

	//if the e_puck goes backward the rotation is inverted
	if (mean_accel[Y_AXIS] > 0)
		error = -error;

	sum_error += error;							//simulate the integrative term

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if (sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if (sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

	return KP_ROTATION  * error + KI_ROTATION  * sum_error + KD_ROTATION * (error-previous_error);
}

/* check for the presence of a wall on the sides of the e_puck
 * if the speed is too high, we put the speed on rotation
 * ROTATION_CORRECTION impose a minimum rotation speed when the e_puck is slow */
float obstacle_detection (float speed, unsigned int *delta){

	if ((speed > 0 && (delta[IR3] > PROX_LIMIT || delta[IR2] > PROX_LIMIT)) ||
		(speed < 0 && (delta[IR6] > PROX_LIMIT || (delta[IR5] > PROX_LIMIT && delta[IR4] < PROX_LIMIT))))
		return ROTATION_CORRECTION + fabs(speed);
	else if ((speed > 0 && (delta[IR6] > PROX_LIMIT || delta[IR7] > PROX_LIMIT)) ||
			 (speed < 0 && (delta[IR3] > PROX_LIMIT || (delta[IR4] > PROX_LIMIT && delta[IR5] < PROX_LIMIT))))
		return -(ROTATION_CORRECTION + fabs(speed));
	else
		return 0;
}

//motors control thread
static THD_WORKING_AREA(waMotors_speed, MOTORS_SPEED_THD_SIZE);
static THD_FUNCTION(Motors_speed, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    static proximity_msg_t prox_values;

    //create a pointer for shorter name to get proximity values
    unsigned int *delta = prox_values.delta;

	systime_t time;

    float rotation_regulation = 0;
    float speed = 0;

    while(true)
    {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));	//100Hz

    	speed = speed_sum_acceleration(delta);

    	rotation_regulation = imu_rotation_regulator() + obstacle_detection(speed, delta);

    	right_motor_set_speed((int16_t)(speed + rotation_regulation));
    	left_motor_set_speed((int16_t)(speed - rotation_regulation));

    	//FREQUENCY [Hz]
    	chThdSleepUntilWindowed(time, time + S2ST(1/FREQUENCY));
    }
}

//calculate the mean value of the imu on NB_SAMPLE values
static THD_WORKING_AREA(waImu_mean_value, IMU_MEAN_VALUE_THD_SIZE);
static THD_FUNCTION(Imu_mean_value, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    static imu_msg_t imu_values;

    //create a pointer for shorter name to get acceleration values
    float *accel = imu_values.acceleration;

    //cyclic table to store samples of the acceleration on the 3 axis
    static float accel_sample[NB_AXIS][NB_SAMPLE];

	float sum_x_axis = 0, sum_y_axis = 0, sum_z_axis = 0;

	uint8_t counter = 0;

    for (uint8_t i = 0; i < NB_AXIS; i++)
    {
    	for (counter = 0; counter < NB_SAMPLE; counter++)
    		accel_sample[i][counter] = 0;
    }

    counter = 0;

    while(true)
    {
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));		//250Hz

        //subtract oldest acceleration values
        sum_x_axis -= accel_sample[X_AXIS][counter];
        sum_y_axis -= accel_sample[Y_AXIS][counter];
        sum_z_axis -= accel_sample[Z_AXIS][counter];

        //update acceleration values
        accel_sample[X_AXIS][counter] = accel[X_AXIS];
        accel_sample[Y_AXIS][counter] = accel[Y_AXIS];
        accel_sample[Z_AXIS][counter] = accel[Z_AXIS];

        //add updated acceleration values
        sum_x_axis += accel_sample[X_AXIS][counter];
        sum_y_axis += accel_sample[Y_AXIS][counter];
        sum_z_axis += accel_sample[Z_AXIS][counter];

        //compute average
        mean_accel[X_AXIS] = sum_x_axis/NB_SAMPLE;
        mean_accel[Y_AXIS] = sum_y_axis/NB_SAMPLE;
        mean_accel[Z_AXIS] = sum_z_axis/NB_SAMPLE;

        counter++;

        //manage cyclic table
        if (counter >= NB_SAMPLE)
        	counter = 0;
    }
}

//start the threads
void motors_speed_start(void){

	chThdCreateStatic(waMotors_speed, sizeof(waMotors_speed), NORMALPRIO, Motors_speed, NULL);
	chThdCreateStatic(waImu_mean_value, sizeof(waImu_mean_value), NORMALPRIO, Imu_mean_value, NULL);
}
