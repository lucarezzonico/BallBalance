#ifndef MOTORS_SPEED_H
#define MOTORS_SPEED_H

/** Robot wide IPC bus. */
//extern messagebus_t bus;

//extern parameter_namespace_t parameter_root;

float get_yaw_angle(float *accel);
float get_pitch_angle(float *accel);

//start the motors' speed regulation thread
void motors_speed_start(void);

#endif /* MOTORS_SPEED_H */
