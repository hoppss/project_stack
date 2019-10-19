//
// Created by fs on 19-8-22.
//

#ifndef __GETGPS_H__
#define __GETGPS_H__

typedef unsigned int UINT;
typedef int BYTE;
typedef long int WORD;

typedef struct __gprmc__
{
    UINT time;
    char pos_state;
    float latitude;
    float longitude;
    float speed;
    float direction;
    UINT date;
    float declination;
    char dd;
    char mode;
}GPRMC;

extern int gps_analysis(char *buff,GPRMC *gps_date);
extern int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
GPRMC gprmc;

#endif

