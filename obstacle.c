#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include <main.h>

#include <motors.h>
#include <sensors/proximity.h>

#include <obstacle.h>

#define ROTATION_THRESHOLD			10
#define ROTATION_COEFF				2
#define ERROR_ANGLE_THRESHOLD		(M_PI * 10/180) // 10°
#define POSITION_ERROR_THRESHOLD	5
#define GOAL_DISTANCE 				500
#define KP							60
#define KI 							3	//must not be zero
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)

#define PROX_SENSOR_LIN_A			-9
#define PROX_SENSOR_LIN_B			72

#define PROX_MAX_VAL				3800
#define PROX_MIN_VAL				0

#define FAR_FROM_OBSTACLE			70	// more than 60[mm] is not detected

#define OBSTACLE_IN_CONTACT			0.5 // closer than 5[mm] is considered touching the obstacle

//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE				53.5f    // [mm]
#define PERIMETER_EPUCK				(M_PI * WHEEL_DISTANCE)
#define RADIUS_EPUCK				35 // [mm]

#define ANGLE_IR6_IR7				(M_PI * 41/180) // 41°
#define GOAL_ANGLE					(M_PI * 49/180) // 49°

//semaphore
static BSEMAPHORE_DECL(obstacle_ready_sem, TRUE);


// functions to control the motors
void motor_stop(void) {
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

// move the robot at a certain speed
// shift forwards if speed > 0
// shift backwards if speed < 0
void shift(int16_t speed) {
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

// rotate the robot for a time duration at a certain speed
// rotate left if speed > 0
// rotate right if speed < 0
void rotate(int16_t duration, int16_t speed) {
	right_motor_set_speed(speed);
	left_motor_set_speed(-speed);

	chThdSleepMilliseconds(duration);

	motor_stop();
}

// get the linearized distance [mm] from any proximity sensor
int16_t obstacle_dist(int8_t ir_sensor_index) {
	if (get_calibrated_prox(ir_sensor_index) > PROX_MIN_VAL){
		return (int16_t)(PROX_SENSOR_LIN_A*log(get_calibrated_prox(ir_sensor_index)) + PROX_SENSOR_LIN_B);
	} else {
		return FAR_FROM_OBSTACLE;
	}
}

/*******************
 * Obstacle Thread *
 *******************/

void obstacle_detection(void){
	int16_t closest_ir_value = 0;
	int8_t closest_ir_index = IR1;

	/* detect which proximity sensor is the closest to an obstacle */
	for(int i = IR1; i < NB_IR ; i++){
		if (get_calibrated_prox(i) > closest_ir_value){
			closest_ir_value = get_calibrated_prox(i);
			closest_ir_index = i;
		}
	}

//	/* slow down when an obstacle is detected */
//	if (closest_ir_value > 500){
//		if ((closest_ir_index == IR1) || (closest_ir_index == IR2) ||
//			(closest_ir_index == IR3) || (closest_ir_index == IR4)){
//			// aligner vers IR3
//
//			while (IR3 < 500){
//				right_motor_set_speed(200);
//				left_motor_set_speed(-200);
//			}
//		} else if ((closest_ir_index == IR5) || (closest_ir_index == IR6) ||
//				   (closest_ir_index == IR7) || (closest_ir_index == IR8)){
//			// aligner vers IR6
//			right_motor_set_speed(-200);
//			left_motor_set_speed(200);
//		}
//	}

	/* slow down when an obstacle is detected */
	if (closest_ir_value > 500){
		right_motor_set_speed(200);
		left_motor_set_speed(-200);
	} else if (closest_ir_value > 250){
		right_motor_set_speed(200);
		left_motor_set_speed(200);
	} else if (closest_ir_value > 100){
		right_motor_set_speed(400);
		left_motor_set_speed(400);
	} else {
		right_motor_set_speed(600);
		left_motor_set_speed(600);
	}


}

float print_sin_beta(void){
	int16_t a = obstacle_dist(IR7) + RADIUS_EPUCK;
	int16_t b = obstacle_dist(IR6) + RADIUS_EPUCK;

	// thm du cos et du sin pour trouver l'angle que forme IR7 avec le mur
	float sin_beta = ((b * sin(ANGLE_IR6_IR7)) / sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(ANGLE_IR6_IR7)));

	int16_t c = 10;
	return b*sin(ANGLE_IR6_IR7);
}

//simple PI regulator to regulate the distance between the robot and the obstacle
int16_t obstacle_prox_regulator(int16_t goal_sin_beta){

	int16_t error = 0;
	int16_t speed_regulation = 0;

	static int16_t sum_error = 0;

	int16_t a = obstacle_dist(IR6) + RADIUS_EPUCK;
	int16_t b = obstacle_dist(IR7) + RADIUS_EPUCK;

	// thm du cos et du sin pour trouver l'angle que forme IR7 avec le mur
	int16_t sin_beta = (int16_t)((b * sin(ANGLE_IR6_IR7)) / sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(ANGLE_IR6_IR7)));

	error = sin_beta - goal_sin_beta;

	if(fabs(error) < ERROR_ANGLE_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed_regulation = KP * error + KI * sum_error;

	return speed_regulation;
}

static THD_WORKING_AREA(waObstacle, 256);
static THD_FUNCTION(Obstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

//    systime_t time;

    int16_t speed_regulation = 0;

    while(1){
//    	time = chVTGetSystemTime();



    	/* detect obstacle */
//    	obstacle_detection();

//    	chprintf((BaseSequentialStream *)&SD3, "distance of IR3 = %d   \r\n\n", obstacle_dist(IR3));

    	//computes the speed to give to the motors
    	speed_regulation = obstacle_prox_regulator((int16_t)(0.755));

		//applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(600 + speed_regulation);
		left_motor_set_speed(600 - speed_regulation);

    	//100Hz
//    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}













float print_current_angle(void){
	int16_t a = obstacle_dist(IR7) + RADIUS_EPUCK;
	int16_t b = obstacle_dist(IR6) + RADIUS_EPUCK;

	// thm du cos et du sin pour trouver l'angle que forme IR7 avec le mur
	float current_angle = asin((a * sin(ANGLE_IR6_IR7)) / sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(ANGLE_IR6_IR7)));

	return current_angle;
}

// PI regulator to regulate the angle between the robot and the wall
int16_t pi_angle_regulator(float current_angle, float target_angle){

	float error_angle = 0;
	float angle_regulation = 0;

	static float sum_error = 0;

	error_angle = current_angle - target_angle;

	if(fabs(error_angle) < ERROR_ANGLE_THRESHOLD){
		return 0;
	}

	sum_error += error_angle;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	angle_regulation = KP * error_angle;// + KI * sum_error;

	return angle_regulation;
}

static THD_WORKING_AREA(waangle_regulator, 256);
static THD_FUNCTION(angle_regulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    float angle_regulation = 0;

    float current_angle = M_PI/2;
    float target_angle = M_PI/2;

    while(1){


    	right_motor_set_speed(-400);
    	left_motor_set_speed(400);

    	int16_t a = obstacle_dist(IR7) + RADIUS_EPUCK;
    	int16_t b = obstacle_dist(IR6) + RADIUS_EPUCK - 1;
    	// thm du cos et du sin pour trouver l'angle que forme IR7 avec le mur
    	current_angle = (asin((a * sin(ANGLE_IR6_IR7)) / sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(ANGLE_IR6_IR7))));

    	float error_angle = M_PI/2 - current_angle;

    	if(fabs(error_angle) < ERROR_ANGLE_THRESHOLD){
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    	} else {
    		right_motor_set_speed(-400);
    		left_motor_set_speed(400);
    	}

    	//computes the speed to give to the motors
//    	angle_regulation = pi_angle_regulator(current_angle, target_angle);

		//applies the speed from the PI regulator and the correction for the rotation
//		right_motor_set_speed(-angle_regulation);
//		left_motor_set_speed(angle_regulation);

    	//100Hz
//    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void angle_regulator_start(void){
	chThdCreateStatic(waangle_regulator, sizeof(waangle_regulator), NORMALPRIO, angle_regulator, NULL);
}
