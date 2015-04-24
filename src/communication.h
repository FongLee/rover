#ifndef COMMUNICATION_H
#define COMMUNICATION_H

bool flag_communication_init;


int communication_init(char *ip);
void communication_system_state_send(void);
void communication_parameter_send(void);
void communication_imu_send();

void communication_gps_send();

void communication_receive(void);





#endif
