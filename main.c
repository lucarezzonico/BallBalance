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
#include <motors_speed.h>

#include <chprintf.h>
#include <obstacle.h>

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

    //wait 2 seconds to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);


    //calibrate the proximity sensors
    calibrate_ir();

    //calibrate the imu
    calibrate_acc();

    // start control of motors speed with imu and proximity sensors
    motors_speed_start();


//  pi_regulator_start();
//    obstacle_start();
//    angle_regulator_start();

    /* Infinite loop. */
    while (1) {
    	//wait for new measures to be published

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
