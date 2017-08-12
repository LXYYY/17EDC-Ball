#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <pthread.h>
#include "mutex.h"
#include "cvclass.h"

class SerialPortClass
{
private:
   int serial_fd = -1;
   fd_set serial_fd_set;
   struct FrameR_S
  {
      unsigned char Header_1=0x11;
      unsigned char Header_2=0x12;
      unsigned char ID;
      float Data;
      unsigned char Tail_1=0xFF;
      unsigned char Tail_2=0xFE;
//      unsigned char SumCheck;
  }__attribute__ ((packed)) Frame;
   enum FrameFormatIndex
   {
           Header_1 = 0,
           Header_2,
           ID,
           Byte0,
           Byte1,
           Byte2,
           Byte3,
           Tail_1,
           Tail_2,
//           SumCheck,
           FrameLength
   };
   float pBuff[FrameLength];
public:
   enum MissionStageE
   {
       Standby=-1,
       Steering0,
       Reversing,
       Steering1,
       Moving1,
       Steering2,
       Moving2,
       Steering3,
   };
   enum MissionE
   {
       StandBy=-1,
       Park=0,
       Change,
       Return
   };
   enum IDSet
   {
       P0_Y_OFS=0,
       P0_X_OFS,

       HOME_X,
       HOME_Y,

       MISSION_ID,
       MISSION_STAGE,

       Car_X,
       Car_Y,
       Car_YAW,

       START,

       CV_READY,
       CV_START,

       TARG_ID,
       MsgLength
   };
   float Msg[MsgLength];
   char* name;
   SerialPortClass();
   SerialPortClass(char *port_str);
   int SerialOpen(char *port_str);
   int SerialStart(int baud_rate);
   int SerialClose(void);
   int SerialConfig(int baudrate,char data_bits,char parity_bits,char stop_bits);
   int SerialWrite(unsigned char *buf,int len);
   int SerialRead(unsigned char *buf,int len);
   int SerialSendMsg(unsigned char id,float data);
   int FrameAnalysis(unsigned char buf);
   static void* SerialReadThread(void* arg);
   int SerialPortThreadStart(void);
};
extern SerialPortClass serialPort;
#endif // SERIALPORT_H
