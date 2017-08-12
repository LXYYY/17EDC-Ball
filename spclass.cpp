#include "spclass.h"
#include <iostream>
using namespace std;

SerialPortClass serialPort;

SerialPortClass::SerialPortClass(void)
{

}

SerialPortClass::SerialPortClass(char *port_str)
{
    name=port_str;
}

int SerialPortClass::SerialOpen(char *port_str)
{
    name=port_str;
    serial_fd = open(name, O_RDWR | O_NOCTTY);  //block mode
    if(serial_fd < 0)
    {
        printf("%s,%d:ERROR\n",__func__,__LINE__);
        return -1;
    }
    return 0;
}

int SerialPortClass::SerialClose(void)
{
    close(serial_fd);
    serial_fd = -1;
    return 0;
}

int SerialPortClass::SerialConfig(int baudrate,char data_bits,char parity_bits,char stop_bits)
{
    int st_baud[]=
    {
        B4800,
        B9600,
        B19200,
        B38400,
        B57600,
        B115200,
        B230400
    };
    int std_rate[]=
    {
        4800,
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        1000000,
        1152000,
        3000000,
    };

    int i,j;
    struct termios newtio, oldtio;
    /* save current port parameter */
    if (tcgetattr(serial_fd, &oldtio) != 0)
    {
        printf("%s,%d:ERROR\n",__func__,__LINE__);
        return -1;
    }
    bzero(&newtio, sizeof(newtio));

    /* config the size of char */
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    /* config data bit */
    switch (data_bits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /* config the parity bit */
    switch (parity_bits)
    {
        /* odd */
    case 'O':
    case 'o':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        break;
        /* even */
    case 'E':
    case 'e':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
        /* none */
    case 'N':
    case 'n':
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* config baudrate */
    j = sizeof(std_rate)/4;
    for(i = 0;i < j;i ++)
    {
        if(std_rate[i] == baudrate)
        {
            /* set standard baudrate */
            cfsetispeed(&newtio, st_baud[i]);
            cfsetospeed(&newtio, st_baud[i]);
            break;
        }
    }
    /* config stop bit */
    if( stop_bits == 1 )
       newtio.c_cflag &=  ~CSTOPB;
    else if ( stop_bits == 2 )
       newtio.c_cflag |=  CSTOPB;

    /* config waiting time & min number of char */
    newtio.c_cc[VTIME]  = 1;
    newtio.c_cc[VMIN] = 1;

    /* using the raw data mode */
    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag  &= ~OPOST;

    /* flush the hardware fifo */
    tcflush(serial_fd,TCIFLUSH);

    /* activite the configuration */
    if((tcsetattr(serial_fd,TCSANOW,&newtio))!=0)
    {
        printf("%s,%d:ERROR\n",__func__,__LINE__);
        return -1;
    }
    return 0;
}

int SerialPortClass::SerialStart(int baud_rate)
{
    if(SerialConfig(baud_rate,8,'N',1) == 0)
    {
        FD_ZERO(&serial_fd_set);
        FD_SET(serial_fd, &serial_fd_set);
        return serial_fd;
    }
    return -1;
}

int SerialPortClass::SerialWrite(unsigned char *buf,int len)
{
    return write(serial_fd,buf,len);
}

int SerialPortClass::SerialRead(unsigned char *buf,int len)
{
    int saved = 0;
    int ret = -1;

    if( buf == NULL)
        return -1;
    else
    {
        for(; saved < len ;)
        {
            ret = read(serial_fd,buf + saved,len - saved);
            if(ret > 0)
            {
                saved += ret;
            }
            else
            {
                break;
            }
        }
        return saved;
    }
}

union BitConverterU
{
    unsigned char byte[4];
    float value;
} BitConverter;

int SerialPortClass::SerialSendMsg(unsigned char id,float data)
{
    Frame.ID=id;
    Frame.Data=data;
//    BitConverter.value=data;
//    Frame.SumCheck=BitConverter.byte[2]+BitConverter.byte[3];
    SerialWrite((unsigned char*)&Frame,sizeof(Frame));
    SerialWrite((unsigned char*)&Frame,sizeof(Frame));
}

void *SerialPortClass::SerialReadThread(void *arg)
{
    int ret;
    unsigned int depth,len;
    unsigned char buf[64];
    int i;
    while(1)
    {
        ret = select(FD_SETSIZE, &serialPort.serial_fd_set, (fd_set *)0, (fd_set *)0,(struct timeval *) 0);
        if (ret < 1)
        {
            printf("%s,%d,ERROR\n", __func__, __LINE__);
            FD_ZERO(&serialPort.serial_fd_set);
            FD_SET(serialPort.serial_fd, &serialPort.serial_fd_set);
            continue;
        }

        ioctl(serialPort.serial_fd, FIONREAD, &depth);
        if(depth > 0)
        {
            len = depth > sizeof(buf) ? sizeof(buf) : depth;
            ret = read(serialPort.serial_fd,buf,len);
            for(i = 0; i < ret; i ++)
            {
                //TODO...call protocol decode function
                //printf("%02X\n",buf[i]);
                serialPort.FrameAnalysis(buf[i]);
            }
        }
    }
    return NULL;
}



int SerialPortClass::FrameAnalysis(unsigned char buf)
{
    for(int i=0;i<FrameLength-1;i++)
    {
        pBuff[i]=pBuff[i+1];
    }
    pBuff[FrameLength-1]=buf;

    if (pBuff[Header_1] == 0x11
            && pBuff[Header_2] == 0x12
            && pBuff[Tail_1] == 0xFF
            && pBuff[Tail_2] == 0xFE)
    {
//        if(pBuff[SumCheck]==pBuff[Byte2]+pBuff[Byte3])
        {
            unsigned char id = pBuff[ID];
            BitConverter.byte[0] = pBuff[Byte0];
            BitConverter.byte[1] = pBuff[Byte1];
            BitConverter.byte[2] = pBuff[Byte2];
            BitConverter.byte[3] = pBuff[Byte3];

            if(id<=MsgLength)
            {
                Msg[id] = BitConverter.value;
                //                cout<<(int)id<<"  "<<BitConverter.value<<endl;
                pthread_mutex_lock(&mutex);
                if(id==MISSION_ID)
                {
                    switch((int)BitConverter.value)
                    {
                    case Park:
                        cvClass.setHome();
                    case Change:
                        SerialSendMsg(HOME_Y,cvClass.car.pos.y);
                        SerialSendMsg(HOME_Y,cvClass.car.pos.y);
                        SerialSendMsg(HOME_Y,cvClass.car.pos.y);
                        cvClass.car.pos_home=cvClass.car.pos;
                        cvClass.car.preV=cvClass.car.tmpV;
                        cvClass.ifChooseTarg=true;
                        break;
                    case Return:
                        cvClass.targetID=4;
                        cvClass.dest=cvClass.home;
                        break;
                    }
                }
                if(id==CV_START)
                {
                    SerialSendMsg(START,1);
                    cvClass.reset();
                    cvClass.ifUpdateEdge=true;
                }
                pthread_mutex_unlock(&mutex);
            }
        }
    }
}

int SerialPortClass::SerialPortThreadStart(void)
{
    pthread_t A_ARR;

    if(pthread_create(&A_ARR,0,SerialReadThread,NULL)!=0)
    {
        return -1;
    }
    return 0;
}

