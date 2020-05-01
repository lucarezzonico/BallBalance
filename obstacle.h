#ifndef OBSTACLE_H
#define OBSTACLE_H

#define IR1     0
#define IR2     1
#define IR3     2
#define IR4     3
#define IR5     4
#define IR6     5
#define IR7		6
#define IR8     7
#define NB_IR   8

/* function to detect an obstacle */
void obstacle_detection(void);

/* start the obstacle thread */
void obstacle_start(void);

int16_t obstacle_dist(int8_t ir_sensor_index);
float print_sin_beta(void);
float print_current_angle(void);

/* start the PI-regulator thread */
void pi_regulator_start(void);

#endif /* OBSTACLE_H */
