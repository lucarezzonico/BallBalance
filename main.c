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

#define SLEEP_TIME			2000		//ms

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	//system initialization
    halInit();
    chSysInit();
    mpu_init();

    //starting sensors
    imu_start();
    proximity_start();

    //inits the motors
    motors_init();

    //inits the inter process communication bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //wait 2 seconds to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(SLEEP_TIME);

    //calibrate the imu
    calibrate_acc();

    //start control of motors speed with imu and proximity sensors
    motors_speed_start();

    /* Infinite loop. */
    while (true)
    {
    	//waits 1 second
        chThdSleepMilliseconds(SLEEP_TIME);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
