#ifndef COMMUNICATION_H
#define COMMUNICATION_H

bool flag_communication_init;
bool flag_communication_connect;

/**
 * [communication_init description]
 * @return [description]
 */
int communication_init(void (*handler)(int num));

/**
 * send system state
 */
void communication_system_state_send(void);

/**
 * send parameter
 */
void communication_parameter_send(void);

/**
 * send imu data
 */
void communication_imu_send();

/**
 * send gps data
 */
void communication_gps_send();

/**
 * receive message from computer or telephone
 * @return  >0: receive buf's length; -1: err; 0: tcp is closed
 */
void communication_receive(void);





#endif
