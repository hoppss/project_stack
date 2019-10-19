#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "nav_goal_lh/current_gps.h"
#include "getgps.h"
// #include "nav_goal/getgps.h"


#define GPS_LEN 512

int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int gps_analyse(char *buff,GPRMC *gps_date);
int gps_get();

int set_serial(int fd,int nSpeed,int nBits,char nEvent,int nStop)
{
    struct termios newttys1,oldttys1;

    /*保存原有串口配置*/
    if(tcgetattr(fd,&oldttys1)!=0)
    {
        perror("Setupserial 1");
        return -1;
    }
    bzero(&newttys1,sizeof(newttys1));
    newttys1.c_cflag|=(CLOCAL|CREAD ); /*CREAD 开启串行数据接收，CLOCAL并打开本地连接模式*/

    newttys1.c_cflag &=~CSIZE;/*设置数据位*/
    /*数据位选择*/
    switch(nBits)
    {
        case 7:
            newttys1.c_cflag |=CS7;
            break;
        case 8:
            newttys1.c_cflag |=CS8;
            break;
    }
    /*设置奇偶校验位*/
    switch( nEvent )
    {
        case '0':  /*奇校验*/
            newttys1.c_cflag |= PARENB;/*开启奇偶校验*/
            newttys1.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
            newttys1.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/
            break;
        case 'E':/*偶校验*/
            newttys1.c_cflag |= PARENB; /*开启奇偶校验  */
            newttys1.c_iflag |= ( INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/
            newttys1.c_cflag &= ~PARODD;/*启用偶校验*/
            break;
        case 'N': /*无奇偶校验*/
            newttys1.c_cflag &= ~PARENB;
            break;
    }
    /*设置波特率*/
    switch( nSpeed )
    {
        case 2400:
            cfsetispeed(&newttys1, B2400);
            cfsetospeed(&newttys1, B2400);
            break;
        case 4800:
            cfsetispeed(&newttys1, B4800);
            cfsetospeed(&newttys1, B4800);
            break;
        case 9600:
            cfsetispeed(&newttys1, B9600);
            cfsetospeed(&newttys1, B9600);
            break;
        case 115200:
            cfsetispeed(&newttys1, B115200);
            cfsetospeed(&newttys1, B115200);
            break;
        default:
            cfsetispeed(&newttys1, B9600);
            cfsetospeed(&newttys1, B9600);
            break;
    }
    /*设置停止位*/
    if( nStop == 1)/*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
    {
        newttys1.c_cflag &= ~CSTOPB;/*默认为一位停止位； */
    }
    else if( nStop == 2)
    {
        newttys1.c_cflag |= CSTOPB;/*CSTOPB表示送两位停止位*/
    }

    /*设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
    newttys1.c_cc[VTIME] = 0;/*非规范模式读取时的超时时间；*/
    newttys1.c_cc[VMIN]  = 0; /*非规范模式读取时的最小字符数*/
    tcflush(fd ,TCIFLUSH);/*tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来 */

    /*激活配置使其生效*/
    if((tcsetattr( fd, TCSANOW,&newttys1))!=0)
    {
        perror("com set error");
        return -1;
    }

    return 0;
}


int gps_analyse (char *buff,GPRMC *gps_data)
{
    char *ptr = NULL;

    if(gps_data==NULL)
    {
        return -1;
    }

    if(strlen(buff)<10)
    {
        return -1;
    }

    if(NULL==(ptr=strstr(buff,"$GPRMC")))
    {
        printf("$GPRMC error!");
        return -1;
    }

    // std::cout<<"buff is: "<<std::endl<<ptr<<std::endl;

    if( !(sscanf(ptr,"$GPRMC,,%c,,,,,,,,,,*",&(gps_data->pos_state))) )
    {
        sscanf(ptr,"$GPRMC,%d.%*[^,],%c,,,,,,,,,,*",
        &(gps_data->time),&(gps_data->pos_state));
    }

    if( gps_data->pos_state == 'A' )
    {
        sscanf(ptr,"$GPRMC,%d.%*[^,],%c,%f,N,%f,E,%f,%f,%f,,,%c,*",
        &(gps_data->time),&(gps_data->pos_state),&(gps_data->latitude),&(gps_data->longitude),
        &(gps_data->speed),&(gps_data->direction),&(gps_data->date),&(gps_data->mode));
//        std::cout<<"=============================================================="<<std::endl;
//        std::cout<< "latitude: "<<gps_data->latitude<<"  longitude: "<<gps_data->longitude<<std::endl;
//        std::cout<<"=============================================================="<<std::endl;
        gps_data->latitude = int(gps_data->latitude/100) + std::fmod( gps_data->latitude, 100 ) / 60;
        gps_data->longitude = int(gps_data->longitude/100) + fmod( gps_data->longitude, 100 ) / 60;

//        std::cout<<"=============================================================="<<std::endl;
//        std::cout<< "jisuanhou: "<<gps_data->latitude<<"  jisuanhou: "<<gps_data->longitude<<std::endl;
//        std::cout<<"=============================================================="<<std::endl;


        // std::cout<<"mod: "<<fmod( gps_data->longitude, 100 ) / 60<<std::endl;

        // gps_data->latitude = int(gps_data->latitude) + std::fmod( gps_data->latitude, 100 ) / 60;
        // gps_data->longitude = int(gps_data->longitude) + fmod( gps_data->longitude, 100 ) / 60;
    }
    else
    {
        gps_data->latitude = 0;
        gps_data->longitude = 0;
    }
    
    return 0;
}


int gps_get ()
{
    int fd = 0;
    int nread = 0;

    char gps_buff[GPS_LEN];
    char *dev_name = "/dev/gpsusb";

    fd = open(dev_name,O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0)
    {
        printf("open gpsusb failed.\n");
        return -1;
    }

    set_serial( fd,115200,8,'N',1);

    sleep(2);
    nread = read(fd,gps_buff,sizeof(gps_buff));
    if(nread<0)
    {
        printf("read GPS date error!!\n");
        return -2;
    }

    memset(&gprmc, 0 , sizeof(gprmc));
    gps_analyse(gps_buff,&gprmc);

    close(fd);
    return 0;
}


int main( int argc, char **argv )
{
    ros::init(argc, argv, "getgps");
    ros::NodeHandle n;
    ros::Publisher gps_pub = n.advertise<nav_goal_lh::current_gps>("curr_gps",1000);
    ros::Rate loop_rate(10);

    while( ros::ok() )
    {
        gps_get ();
        nav_goal_lh::current_gps msg;
        gprmc.longitude = gprmc.longitude;
        gprmc.latitude = gprmc.latitude;
        msg.EE =  gprmc.longitude;
        msg.NN =  gprmc.latitude;
        // gprmc.pos_state;

        std::cout<<"pos_state is: "<<gprmc.pos_state<<std::endl;
        std::cout<<"longitude is: "<<gprmc.longitude<<std::endl;
        std::cout<<"latitude is: "<<gprmc.latitude<<std::endl;

        gps_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
