

   /// COPYRIGHT NOTICE      
    // Copyright (c) 2009, 华中科技大学ticktick Group      
   /// All rights reserved.       
   ///       
   /// @file    ortpSend.c        
   /// @brief   ortpSend的测试      
   ///      
   /// 本文件示例使用ortp库进行rtp数据包的发送     
   ///       
  /// @version 1.0         
  /// @author  tickTick     
  /// @date    2010/07/07       
  /// @E-mail  lujun.hust@gmail.com      
  ///      
  /// 修订说明：创建文件      
  //////////////////////////////////////////////////////////////////////////       
     
  #include <ortp/ortp.h>   
  #include <signal.h>   
  #include <stdlib.h>   
     
  #ifndef _WIN32    
  #include <sys/types.h>   
  #include <sys/time.h>   
  #include <stdio.h>   
  #endif   
     
  // 时间戳增量  
  #define TIME_STAMP_INC  160    
  #define BYTES_PER_COUNT 65535  
  // 时间戳    
  uint32_t g_user_ts;    
   
  /**  初始化     
   *       
   *   主要用于对ortp以及其它参数进行初始化    
   *   @param:  char * ipStr 目的端IP地址描述串     
  *   @param:  iint port 目的端RTP监听端口     
   *   @return:  RtpSession * 返回指40   向RtpSession对象的指41   针,如果为NULL，42    则初始化失败     
   *   @note:       
   */     
  RtpSession * rtpInit(char * ipStr,int port)    
  {    
     // Rtp会话对象     
     RtpSession *session;    
      char *ssrc;    
          
      // 时间戳初始化     
      g_user_ts = 0;    
     
      // ortp的一些基本初始化操作    
      ortp_init();    
      ortp_scheduler_init();    
      // 创建新的rtp会话对象   
      session=rtp_session_new(RTP_SESSION_SENDONLY);      
          
      rtp_session_set_scheduling_mode(session,1);    
      rtp_session_set_blocking_mode(session,1);    
      // 设置远程RTP客户端的的IP和监听端口（即本rtp数据包的发送目的地址）    
      rtp_session_set_remote_addr(session,ipStr,port);    
      // 设置负载类型     
      rtp_session_set_payload_type(session,0);    
          
      // 获取同68    步源标69   识     
      ssrc=getenv("SSRC");    
      if (ssrc!=NULL)     
      {    
          printf("using SSRC=%i.\n",atoi(ssrc));    
          rtp_session_set_ssrc(session,atoi(ssrc));    
      }    
          
      return session;    
  }    
     
  /**  发送rtp数据包     
   *       
   *   主要用于发送rtp数据包     
   *   @param:  RtpSession *session RTP会话对象的指85   针     
   *   @param:  const char *buffer 要发送的数据的缓冲区地址      
   *   @param: int len 要发送的数据长度     
   *   @return:  int 实际发送的数据包数目     
   *   @note:     如果要发送的数据包长度大于BYTES_PER_COUNT，90 本函数内部会进行分包处理     
   */   
 int rtpSend(RtpSession *session,const char *buffer, int len)    
  {    
      int curOffset = 0;    
      int sendBytes = 0;    
      int clockslide=500;     
      // 发送包的个数   
      int sendCount = 0;    
     
     ortp_message("send data len %i\n ",len);    
    
     // 是否全部发送完毕     
     while(curOffset < len )    
     {    
     // 如果需要发送的数据长度小于等于阙值，106    则直接发送   
         if( len <= BYTES_PER_COUNT )    
         {    
             sendBytes = len;    
         }    
         else   
         {    
            // 如果当前偏移 + 阈值 小于等于 总长度，114 则发送阈值大小的数据   
             if( curOffset + BYTES_PER_COUNT <= len )    
             {    
                 sendBytes = BYTES_PER_COUNT;    
             }    
             // 否则就发送剩余长度的数据   
            else   
             {    
                 sendBytes = len - curOffset;    
             }    
         }    
             
         ortp_message("send data bytes %i\n ",sendBytes);    
             
         rtp_session_send_with_ts(session,(char *)(buffer+curOffset),sendBytes,g_user_ts);    
             
         // 累加   
         sendCount ++;    
         curOffset += sendBytes;                     
         g_user_ts += TIME_STAMP_INC;    
         
         // 发送一定数据包后休眠一会   
         if (sendCount%10==0)     
         {    
             usleep(20000);    
         }       
    }    
     return 0;    
 }    
    
 /**  结束ortp的发送，145  释放资源     
  *      
  *   @param:  RtpSession *session RTP会话对象的指148  针     
  *   @return:  0表示成功     
  *   @note:         
  */   
 int rtpExit(RtpSession *session)    
 {    
     g_user_ts = 0;    
         
     rtp_session_destroy(session);    
     ortp_exit();    
     ortp_global_stats_display();    
    
     return 0;    
 }    
    
 // 主函数，164  进行测试   
 int main()    
 {    
     // 待发送的数据缓冲区   
     char * pBuffer = "123445356234134234532523654323413453425236244123425234";    
         
     RtpSession * pRtpSession = NULL;    
     // 向（192.201.0.51，172   8000）目的地址发送rtp包   
     pRtpSession = rtpInit("192.168.1.111",8000);    
     if(pRtpSession==NULL)    
     {    
         printf("error rtpInit");    
         return 0;    
     }    
         
     // 循环发送   
     while(1)    
     {    
         if( rtpSend(pRtpSession,pBuffer,20) != 0)    
         {    
             printf("error rtpInit");    
            break;    
        }    
         usleep(10000);    
         printf("sleep");    
     }    
         
     // 退出   
     rtpExit(pRtpSession);    
         
     return 0;    
 }   
