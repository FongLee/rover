#ifndef AP_CONTROL_H
#define AP_CONTROL_H
extern int channel_steer;
extern int channel_throttle;
extern bool flag_control_init;
extern int flag_control_mode;
int flag_control_avoid;
int control_init();
void control_close();
void setting_moto(int throttle);
void moto_control(int throttle, int steering);


#endif
