#ifndef RTPSEND_H
#define RTPSEND_H

#include <ortp/ortp.h>   
#include <signal.h>   
#include <stdlib.h>        
#include <sys/types.h>   
#include <sys/time.h>   
#include <stdio.h>   
#include "capture.h"
 #define TIME_STAMP_INC  160    
#define MAX_RTP_PKT_LENGTH 1400
#define DefaultTimestampIncrement 90000/25 
#define SERVER_IP "192.168.1.111"
#define RTP_PORT  3020
  // 时间戳    
  uint32_t g_user_ts;  
RtpSession * pRtpSession ; 
FILE*fp_h264;
FILE* fp_sdp;
 RtpSession * rtpInit(char * ipStr,int port);
int rtpSend(RtpSession *session, unsigned char *buffer, int len);    
 int rtpExit(RtpSession *session) ;
 int camera_init(char *ipStr);

#endif