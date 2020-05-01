#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include <motors.h>
#include <sensors/imu.h>
#include <sensors/proximity.h>

#include <chprintf.h>

#include <obstacle.h>

#include <motors_speed.h>

//#define NB_SAMPLES_IMU_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

int main(void)
{
	// system initialization
    halInit();
    chSysInit();
    mpu_init();




    // starting sensors
    imu_start();
    proximity_start();

    // communication initialization
    serial_start();

    motors_init();

    // inits the inter process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

// ?????

//    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//    proximity_msg_t prox_values;

    //wait 2 seconds to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);


    //calibrate the imu
    //imu_compute_offset(imu_topic, NB_SAMPLES_IMU_OFFSET);

    //calibrate the proximity sensors
//    calibrate_ir();

    //calibrate the imu
    calibrate_acc();

    motors_speed_start();

//    right_motor_set_speed(600);
//    left_motor_set_speed(600);

//  pi_regulator_start();
//    obstacle_start();

//    right_motor_set_speed(MOTOR_SPEED_LIMIT);
//    left_motor_set_speed(-MOTOR_SPEED_LIMIT);

//    angle_regulator_start();

    /* Infinite loop. */
    while (1) {
    	//wait for new measures to be published
//    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

//    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

//    	chprintf((BaseSequentialStream *)&SD3, "calibrated of IR3 = %d   \r\n\n", get_calibrated_prox(5));

//    	chprintf((BaseSequentialStream *)&SD3, "dist of IR3 = %d		\r\n\n", obstacle_dist(5));


    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
