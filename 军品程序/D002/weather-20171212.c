// weather 2016-03-25 15:00 V7.3
// source from "weather-20150914.c"
//

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "../include/at91_ioctl.h"
#include <math.h>
#include <time.h>
#include <pthread.h>
#include "can.h"

#define AD7792	1
//#define CANSTAT	1

#define UART0   "/dev/ttyAT1"
#define UART1   "/dev/ttyAT2"
#define UART2   "/dev/ttyAT3"
#define SERIAL0 "/dev/ttyS0"
#define SERIAL1 "/dev/ttyS1"
#define SERIAL2 "/dev/ttyS2"
#define SERIAL3 "/dev/ttyS3"
#define CAN_DEV "/dev/mcpcan"
#define DEV_PIO "/dev/pio0"
#define DEV_AD  "/dev/ad7792"
#define DEV_TC0 "/dev/tc0"
#define DEV_TC1 "/dev/tc1"
#define DEV_TC2 "/dev/tc2"
#define DEV_FIQ "/dev/fiq"
//#define DEV_RTC "/dev/rtc"

#define TARH	1
#define ZZ090310 1

#define A	0.00097392
#define B	2.363857
#define C	-246.122453
#define PI	3.141592654
#define WDJJ	40

#define SQYA 10.79574
#define SQYB -5.028
#define SQYC 0.000150475
#define SQYD -8.2969
#define SQYE 0.00042873
#define SQYF 4.76955
#define SQYG 0.78614
#define SXDWD 273.16	 
#define LDWDA 7.69
#define LDWDB 243.92
#define BHSQY0 6.1078

#define	P	0	
#define	T0	1
#define	TD	2
#define U	3
#define WD	4
#define WS	5
#define	RAT 	6
#define	RAW	7
#define VI 	8


#define NOQC 9		// mei you jian cha
#define RI   0		// zheng que
#define SUS  1		// cun yi
#define WR   2		// cuo wu
#define NOT  3		// bu yi zhi
#define JY   4		// jiao yan guo de
#define MI   8		// que shi

#define BYTEQCPS	20
#define BYTEQCPM    30

#define	TCHD	10
#define BYTEDZTN	14
#define BYTEDZSTN	32
#define BYTEDZRAN	11
#define BYTEDZSENSI 8

#define TEN		10
#define HUNDRED	100
#define WRONG	9999

#define ZZ090317 1
//#define ZZ150914 1

#define R485_STA_CTL1   1
#define TX  1
#define RX  0

// New
#define TAG	211
#define CNT_FZSJ	255

typedef enum {
        PIO_INP_IDX0 = 0,
        PIO_OUT_IDX0 = 8,
        PIO_LDO_CTRL = 16,  //LDO_CTL output
        PIO_LVL_INP1 = 17,  //LEVEL_IN1
        PIO_LVL_INP2 = 18,  //LEVEL_IN2
        PIO_ADX_SEL0 = 19,
        PIO_ADX_SEL1,
        PIO_ADX_SEL2,
        PIO_ADX_SEL3,
        PIO_ADX_EN,
        PIO_ADY_EN,
        PIO_WDT_CTL,
	    PIO_WDT_INP,
        PIO_RS485_CTL1,
        PIO_RS485_CTL2,
} at91_pio_idx;

at91_pio_arg arg;
at91_pio_arg argwd;

static int speed_arr[] = {B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1800, B1200, B600, B300};
static int name_arr[]  = {230400,  115200,  57600,  38400,  19200,  9600,  4800,  2400,  1800,  1200,  600,  300};

static float threshold[9] = {400,-75,-80,0,0,0,0,0,0};
static float ceiling[9] = {1100,80,50,100,360,60,999,999,64000};
static float suspirate[9] = {0.3,2,4,5,360,20,999,999,8000};
static int cost[9] = {65535,9999,9999,9999,9999,9999,9999,9999,65535}; 

static float instthre[9] = {400,-75,-80,0,0,0,0,0,0};
static float instceil[9] = {1100,80,50,100,360,60,999,999,64000};
static float instsusp[9] = {0.5,3,4,10,360,10,999,999,5000};
static float instwrong[9] = {2,5,5,15,360,20,999,999,10000};
static int instcost[9] = {65535,9999,9999,9999,9999,9999,9999,9999,65535};

static int aver=0, tag=0;
static int fd_tc0=-1, fd_tc1=-1, fd_can=-1, fd_pio=-1, fd_ttyS0=-1, fd_ttyS1=-1, fd_ttyS2=-1, fd_485=-1;
static int fs3s[2][12], fs1m[2][60], fx1m[2][60], fs2m120[2][120], fx2m120[2][120];
static int sec, min, hour, mday, mon, year, suffix, secary, minary;
static char beijintime[12];
static int samp45d[2][30], sampsd[2][30];
static int fzyl=0, fzyltag=0, xsyl=0; 
static int fzcc=0, fzcctag=0, xscc=0;
static int sampfs1mmax=-1, sampfx1mmax=0;
static char qiya[14], njd[100];
static int visi=0, visi10min=64000, visitime=0, visilasttime=0, visisame=0, visi10mintag=0;
static long pres;

static int temp45d=0, humi=0;
static char sampt[126], samprh[126], samppp[156], sampwd[246], sampws[966];
static char senstate[11]={"111111111\r\n"};
static int senst = 0;

static int fddz;
static char dztn[BYTEDZTN], dzstn[BYTEDZSTN], dzran[BYTEDZRAN], dzsensi[BYTEDZSENSI];
static int dzt0=0, dzt1=0, dzt2=0, dzt3=0;
static int dztg=0, dzst0=0, dzst1=0, dzst2=0, dzst3=0, dzst4=0, dzst5=0, dzst6=0, dzst7=0, dzst8=0;
static int dzrat=0, dzraw=0, dzu=0;
static int dzgr=0, dzdr=0;

static int fdstat=-1, fdsenco=-1;

static int sampdlwd[2][30], sampdlsd[2][30], njd10min[2][10];
static float wsbl0=0.2315, wsbl1=0.0495, ws1bl0=0.2315, ws1bl1=0.0495, ratbl=0.1, rat1bl=0.5;

static int jd28f=0, jd184=0, jd284=0, jd186=0, jd286=0, jd386=0;

static int tqxx_js, tqxx_sc, tqxx_nj, tqxx_ld, tqxx_qt;

void fiq_handle(void)
	{
//    usleep(1);
    printf("fiq_handle\n");
	}

int set_speed(int fd, int speed)
	{
    int   i;
    int   status;
    struct termios   Opt;

    tcgetattr(fd, &Opt);

    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
        {
        if (speed == name_arr[i])
        	{
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0)
            	{
                perror("tcsetattr fd");
                return -1;
            	}
            tcflush(fd,TCIOFLUSH);
            return 0;
        	}
    	}
    return -1;
	}

int set_other_attribute(int fd, int databits, int stopbits, int parity, int vt)
    {
    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    	{
        perror("SetupSerial 1");
        return -1;
    	}
    options.c_cflag &= ~CSIZE;
    switch (databits) 
    	{
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return -1;
    	}
    switch (parity)
    	{
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;   /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
        case 1:
            options.c_cflag |= (PARODD | PARENB); 
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
        case 2:
            options.c_cflag |= PARENB;     /* Enable parity */
            options.c_cflag &= ~PARODD;    
            options.c_iflag |= INPCK;      /* Disnable parity checking */
            break;
        case 'S':
        case 's':  /*as no parity*/
        case 0:
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return -1;
    	}
    
    switch (stopbits)
    	{
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return -1;
    	}
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = vt; //150; /* 15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
//      options.c_oflag = 0;

    if (tcsetattr(fd,TCSANOW,&options) != 0)
    	{
        perror("SetupSerial 3");
        return -1;
    	}
                                                                                                                                               
#if 0
    tcgetattr(fd, &options);
    printf("c_iflag: %x\rc_oflag: %x\n", options.c_iflag, options.c_oflag);
    printf("c_cflag: %x\nc_lflag: %x\n", options.c_cflag, options.c_lflag);
    printf("c_line: %x\nc_cc[VTIME]: %d\nc_cc[VMIN]: %d\n", options.c_line, options.c_cc[VTIME], options.c_cc[VMIN]);
#endif
    return 0;
    }

static void rs485clt(int fd, int pin, int sta)
    {
    unsigned char databuf[50];
    databuf[0] = pin;
    databuf[1] = sta;
    write(fd, databuf, 2);
    }

static int qclimit(int num, float ceiling, float threshold)
    {
    int tag=NOQC;
    tag = (num>ceiling || num<threshold)?WR:RI;
	if(num>ceiling)
		senst = 2;
	else if(num<threshold)
		senst = 3;
	else
		senst = 0;
    return(tag);
    }
                                                                                                                                               
static int qcsampchangerate(int num, int cost, float suspirate)
    {
    int tag=NOQC, absvalue;
    absvalue = abs(num-cost);
    tag = (absvalue>=suspirate)?SUS:RI;
    return(tag);
    }
                                                                                                                                               
static int qcinstchangerate(int num, int cost, int suspirate, int wrongrate)
    {
    int tag=NOQC, absvalue;
    absvalue = abs(num-cost);
    if(absvalue>=wrongrate)
        tag = WR;
    else
        tag = (absvalue>=suspirate)?SUS:RI;
    return(tag);
    }

static int qcsamp(int ret, int x, int beilv, int chushi)
	{
	int tag=NOQC;
	tag = qclimit(ret, ceiling[x]*beilv, threshold[x]*beilv);
    if(tag == RI)
    	{
        if(cost[x] == chushi)
        	cost[x] = ret;
        else
            {
            tag = qcsampchangerate(ret, cost[x], suspirate[x]*beilv);
//            if(ret == RI)
//            	cost[x] = ret;
            }
		cost[x] = ret;
        }
	senstate[x] = senst + '0';
	return(tag);
	}

static int qcinstance(int ret, int x, int beilv, int chushi)
    {
	int tag=NOQC;
    tag = qclimit(ret, instceil[x]*beilv, instthre[x]*beilv);
    if(tag == RI)
		{
	    if(instcost[x] == chushi)
       		instcost[x] = ret;
		else
			{
     		tag = qcinstchangerate(ret, instcost[x], instsusp[x]*beilv, instwrong[x]*beilv);
//			if(ret == RI)
//				instcost[x] = ret;
			}
		instcost[x] = ret;
		}
	return(tag);
    }


static void average(int *samp, int n)
    {
    long int sampsum=0, sampcnt=0, sampaver=0, num=0;
    int j;
    for(j=0;j<n;j++)
        {
//  	if(samp[1][j] == RI)
		if(*(samp+n+j) == RI)
        	{
	    	sampsum += *(samp+j);
//      	sampsum += samp[0][j];
        	sampcnt++;
			num++;
        	}
		else if(*(samp+n+j) == NOQC)
			num++;
        }
//	printf("sum=%ld,cnt=%d\n", sampsum, sampcnt);
//    if(sampcnt >= n*2/3)
    if(num>=n*2/3 && sampcnt>=n*1/3)
        {
        sampaver = 10*sampsum/sampcnt;
		if(sampaver >= 0)
			aver = (sampaver+5) / 10;
		else
			aver = (sampaver-5) / 10;
        tag = RI;
        }
    else
        {
        aver = WRONG;
        tag = MI;
        }
    }

static void averagefs(int *samp, int n)
    {
    long int sampsum=0, sampcnt=0, sampaver=0;
    int j;
    for(j=0;j<n;j++)
        {
//        if(samp[1][j] == RI)
        if(*(samp+n+j) == RI)
            {
            sampsum += *(samp+j);
//            sampsum += samp[0][j];
            sampcnt++;
            }
        }
//        printf("sum=%ld,cnt=%d\n", sampsum, sampcnt);
    if(sampcnt >= n*3/4)
        {
        sampaver = 10*sampsum/sampcnt;
		aver = (sampaver+5) / 10;
        tag = RI;
        }
    else
        {
        aver = WRONG;
        tag = MI;
        }
    }

static void averagefx(int *samp, int n)
    {
    double sampsum=0, xaver=0, yaver=0;
    int sampcnt=0, cum=0, i, j;
//	long int longaver=0;
    double longaver=0;

//    for(i=0;i<2;i++)
//	{
//	for(j=0;j<60;j++)
//	    printf("%d;",*(samp+i*60+j));
//	}

    for(j=0;j<n;j++)
		{
        if(*(samp+n+j) == RI)
            {
	    	sampsum += sin(*(samp+j) * PI / 180);
	    	sampcnt++;
//            printf("xsum=%f\n",sampsum);
            }
		}
    if(sampcnt >= n*3/4)
		{
    	xaver = sampsum / sampcnt;
        sampsum = 0;
		sampcnt = 0;
//		printf("xaver=%f\n",xaver);
		}
    else
		{
        aver = WRONG;
        tag = MI;	
		goto averfzend;
		}

    for(j=0;j<n;j++)
        {
        if(*(samp+n+j) == RI)
            {
            sampsum += cos(*(samp+j) * PI / 180);
            sampcnt++;
//            printf("ysum=%f\n",sampsum);
            }
        }
    if(sampcnt >= n*3/4)
        {
        yaver = sampsum / sampcnt;
//        printf("yaver=%f\n",yaver);
        }
    else
        {
        aver = WRONG;
        tag = MI;
        goto averfzend;;
        }
//  printf("%d,%d\n",xaver,yaver);
    if(yaver == 0)
		{
		if(xaver > 0)
			aver = 90;
		else
			aver = 270;
		}
    else
		{
    	if(xaver>0 && yaver>0)    									cum=0;
    	else if((xaver>0 && yaver<0) || (xaver<0 && yaver<0))		cum=180;
    	else if(xaver<0 && yaver>0)       							cum=360;
//    	longaver = atan(xaver/yaver) * 180 * 10 / PI + cum * 10 + 5;
//    	aver = longaver / 10;
        longaver = atan(xaver/yaver) * 180.0 / PI + cum;
//		printf("longaver=%f	",longaver);
        aver = longaver + 0.5;
//		printf("aver=%d\n",aver);
		}
    tag = RI;

averfzend:
    usleep(1);
//    printf("winddir1m=%d\n",winddir1m);
    }

static void asciitemp(char *string, int aver, int n)
    {
    if(aver >= 0)
		{
        string[n] = ' ';
        string[n+1] = aver / 100 + '0';
        string[n+2] = (aver - aver/100*100) / 10 + '0';
        string[n+3] = aver % 10 + '0';
        if(string[n+1] == '0')
	    	{
            string[n+1] = ' ';
            if(string[n+2] == '0')
                {
                string[n+2] = ' ';
				}
            }
        }
    else
        {
		aver = 0 - aver;
        string[n] = '-';
        string[n+1] = aver / 100+'0';
        string[n+2] = (aver - aver/100*100) / 10 + '0';
        string[n+3] = aver % 10 + '0';
        if(string[n+1] == '0')
            {
            string[n] = ' ';
            string[n+1] = '-';
            if(string[n+2] == '0')
            	{
                string[n+1] = ' ';
                string[n+2] = '-';
            	}
	    	}
        }
    }

static void asciibyte4(char *string, int aver, int n)
    {
    if(aver < 0)
	aver = 0;
    string[n] = aver / 1000 + '0';
    string[n+1] = (aver - aver/1000*1000) / 100 + '0';
    string[n+2] = (aver - aver/100*100) / 10 + '0';
    string[n+3] = aver % 10 + '0';
    if(string[n] == '0')
        {
        string[n] = ' ';
        if(string[n+1] == '0')
            {
            string[n+1] = ' ';
	    	if(string[n+2] == '0')
            	{
            	string[n+2] = ' ';
				}
            }
        }
    }

static void asciibyte5(char *string, int aver, int n)
    {
    if(aver < 0)
        aver = 0;
    string[n] = aver / 10000 + '0';
    string[n+1] = (aver - aver/10000*10000) / 1000 + '0';
    string[n+2] = (aver - aver/1000*1000) / 100 + '0';
    string[n+3] = (aver - aver/100*100) / 10 + '0';
    string[n+4] = aver % 10 + '0';
    if(string[n] == '0')
        {
        string[n] = ' ';
        if(string[n+1] == '0')
            {
            string[n+1] = ' ';
            if(string[n+2] == '0')
                {
                string[n+2] = ' ';
            	if(string[n+3] == '0')
                	{
                	string[n+3] = ' ';
					}
                }
            }
        }
    }

static void tagnotRI(char *string, int n)
    {
    string[n] = '/';
    string[n+1] = '/';
    string[n+2] = '/';
    string[n+3] = '/';
    }

static void tagMIbyte5(char *string, int n)
    {
    string[n] = '/';
    string[n+1] = '/';
    string[n+2] = '/';
    string[n+3] = '/';
	string[n+4] = '/';
    }

static void tagNbyte4(char *string, int n)
    {
    string[n] = '-';
    string[n+1] = '-';
    string[n+2] = '-';
    string[n+3] = '-';
    }

static void tagNbyte5(char *string, int n)
    {
    string[n] = '-';
    string[n+1] = '-';
    string[n+2] = '-';
    string[n+3] = '-';
    string[n+4] = '-';
    }

static void stringmove(char *from, int i, char *to, int j, int k)
    {
    for( ;k>0;k--)
        {
        to[j]=from[i];
        i++;
        j++;
        }
    }

static void twoarrayinit(int *string, int m, int n)
    {
    int i,j;
    for(i=0;i<m;i++)
		{
		for(j=0;j<n;j++)
	    	*(string+i*n+j) = NOQC;
		}
    }

static void timeconvert(char *string, long set)
    {
    time_t timep;
    struct tm *p;
    time(&timep);
    p = localtime(&timep);
    p->tm_year = (string[0] - '0') * 10 + (string[1] - '0') + 100;
    p->tm_mon  = (string[2] - '0') * 10 + (string[3] - '0') - 1;
    p->tm_mday = (string[4] - '0') * 10 + (string[5] - '0');
    p->tm_hour = (string[6] - '0') * 10 + (string[7] - '0');
    p->tm_min  = (string[8] - '0') * 10 + (string[9] - '0');
    p->tm_sec  = (string[10] - '0') * 10 + (string[11] - '0');
    timep = mktime(p);
    timep = timep + set;
    p = localtime(&timep);
    string[0] = (p->tm_year - 100) / 10 + '0';
    string[1] = (p->tm_year - 100) % 10 + '0';
    string[2] = (p->tm_mon + 1) / 10 + '0';
    string[3] = (p->tm_mon + 1) % 10 + '0';
    string[4] = (p->tm_mday) / 10 + '0';
    string[5] = (p->tm_mday) % 10 + '0';
    string[6] = (p->tm_hour) / 10 + '0';
    string[7] = (p->tm_hour) % 10 + '0';
    string[8] = (p->tm_min) / 10 + '0';
    string[9] = (p->tm_min) % 10 + '0';
    string[10] = (p->tm_sec) / 10 + '0';
    string[11] = (p->tm_sec) % 10 + '0';
    }

static void* pthread_raw(void *t)
    {
    char rawbuff[20];
    char c;
    int i;
    char fzccbuff[3], xsccbuff[4];
//  int fzcc, xscc;
	int fdlog;

    while(1)
        {
        read(fd_ttyS0, &c, 1);
        if(c>='0' && c<='2')
            {
            rawbuff[0] = c;
            for(i=1;i<18 && c!=0x0A;i++)
                {
                read(fd_ttyS0, &c, 1);
                rawbuff[i] = c;
                }
            rawbuff[i] = '\0';
            printf("raw=%s",rawbuff);
            stringmove(rawbuff, 5, fzccbuff, 0, sizeof(fzccbuff));
            fzccbuff[3] = '\0';
            fzcc = atof(fzccbuff);
//			fzcc += atof(fzccbuff);
            fzcctag = qclimit(fzcc, ceiling[RAW]*TEN, threshold[RAW]*TEN);
//            if(fzcctag == RI)
//                xscc += fzcc;
/*
            if((fdlog = open("/tmp/write.txt",O_WRONLY|O_CREAT|O_APPEND)) < 1)
	            usleep(1);
            else
              	{
				write(fdlog, beijintime, sizeof(beijintime));
              	write(fdlog, rawbuff, i);
				close(fdlog);
              	}
*/
			sleep(50);
            }
        }
    }

static void* njdpthreadlandun(void *t)
    {
    int i=0, j;
    char c, njd[60], njdvalue[5], tqxxvalue[2];
	int fdlog;
	int njdlast, njdlast10, tqxxlast; 
	int njdtag=0;
	int jdnjd=0;

    while(1)
        {
        read(fd_ttyS1, &c, 1);
        if(c == '!')
            {
            for(i=0;i<30 && c!=0x0A;i++)
                {
                read(fd_ttyS1, &c, 1);
                njd[i] = c;
                }
            njd[i] = '\0';
            printf("Visibility:%s",njd);
            stringmove(njd, 5, njdvalue, 0, sizeof(njdvalue));
            njdvalue[5] = '\0';
			for(i=0;i<5;i++)
				{
				if(njdvalue[i]<'0' || njdvalue[i]>'9')
					njdtag = 1;
				else
					njdtag = 0;
				}
//            visi = atof(njdvalue);
			njdlast = atof(njdvalue);
			if(njdlast>=10 && njdlast<=50000 && njdtag==0)
				visi = njdlast;
//            visi10min = visi;
			njd10min[0][jdnjd] = visi;
			njd10min[1][jdnjd] = RI;
			if(jdnjd >= 9)
				jdnjd = 0;
			else
				jdnjd++;
			average(*njd10min, 10);
			visi10min = aver;
			visi10mintag = tag;
			printf("visi=%d,visi10min=%d,tag=%d\n",visi,visi10min,visi10mintag);			
			stringmove(njd, 11, tqxxvalue, 0, sizeof(tqxxvalue));
			tqxxvalue[2] = '\0';
			tqxxlast = atof(tqxxvalue);
			printf("tqxxlast = %d\n", tqxxlast);
			switch(tqxxlast)
				{
				case 30:
					tqxx_js = 0;
                    tqxx_sc = 1;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
					break;
                case 10:
                    tqxx_js = 0;
                    tqxx_sc = 2;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
                case 4:
                    tqxx_js = 0;
                    tqxx_sc = 8;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
                case 61:
				case 62:
				case 63:
                    tqxx_js = 1;
                    tqxx_sc = 0;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
                case 64:
                case 65:
                case 66:
                    tqxx_js = 6;
                    tqxx_sc = 0;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
                case 71:
                case 72:
                case 73:
                    tqxx_js = 4;
                    tqxx_sc = 0;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
                case 75:
                    tqxx_js = 11;
                    tqxx_sc = 0;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
                    break;
				default:
					tqxx_js = 0;
                    tqxx_sc = 0;
                    tqxx_nj = 0;
                    tqxx_ld = 0;
                    tqxx_qt = 0;
				}
			sleep(50);
            }
        }
    }

static void* qiyapthread(void *t)
    {
	char c;
	int i;
    for(;;)
        {
        read(fd_ttyS2, &c, 1);
		qiya[0] = c;
//		printf("%c",c);
        for(i=1; i<14 && c!='\n'; i++)
        	{
        	read(fd_ttyS2, &c, 1);
            qiya[i] = c;
//			printf("%c",c);
            }
//        read(fd_ttyS2, qiya, sizeof(qiya));
		qiya[13] = c;
        qiya[14]='\0';
//		printf("%s\n",qiya);
		pres = 10 * atof(qiya);
//		printf("qy=%ld\n",pres);
        asciibyte5(samppp, pres, 4+suffix*5);
        if(suffix<29)
	        asciibyte5(samppp, pres, 4+(suffix+1)*5);
        samppp[154] = '\r';
        samppp[155] = '\n';
		usleep(800000);
//		fflush(stdout);
        }
    }

static void* tc0fs(void *t)
    {
    int ret, fsdata[4]={0};
    int fs1=0, fs2=0, fs3=0, intfs3s=0;
    int tc0tag=MI;
    int gray, winddir;
    int graycode[7]={0};
    int i, n=0;
	int wdset;

    ioctl(fd_tc0, IOCTL_TC_START, 4);

    while(1)
	{
		{
        for (i=0; i<7; i++) 
	    	{
            argwd.pin_idx = i;
            ioctl(fd_pio, IOCTL_PIO_GETSTA, &argwd);
	    	graycode[i] = argwd.pin_sta;
//     		printf("PIN %d, VAL = %d\n", arg.pin_idx, arg.pin_sta);
            }

    	gray = graycode[6]*64 +graycode[5]*32 +graycode[4]*16 +graycode[3]*8 +graycode[2]*4 +graycode[1]*2 +graycode[0];
    	switch(gray)
        {
        case 0:  winddir = 0;   break;
        case 1:  winddir = 3;   break;
        case 2:  winddir = 8;   break;
        case 3:  winddir = 6;   break;
        case 4:  winddir = 20;  break;
        case 5:  winddir = 17;  break;
        case 6:  winddir = 11;  break;
        case 7:  winddir = 14;  break;
        case 8:  winddir = 42;  break;
        case 9:  winddir = 39;  break;
        case 10: winddir = 34;  break;
        case 11: winddir = 37;  break;
        case 12: winddir = 23;  break;
        case 13: winddir = 25;  break;
        case 14: winddir = 31;  break;
        case 15: winddir = 28;  break;
        case 16: winddir = 87;  break;
        case 17: winddir = 84;  break;
        case 18: winddir = 79;  break;
        case 19: winddir = 82;  break;
        case 20: winddir = 68;  break;
        case 21: winddir = 70;  break;
        case 22: winddir = 76;  break;
        case 23: winddir = 73;  break;
        case 24: winddir = 45;  break;
        case 25: winddir = 48;  break;
        case 26: winddir = 53;  break;
        case 27: winddir = 51;  break;
        case 28: winddir = 65;  break;
        case 29: winddir = 62;  break;
        case 30: winddir = 56;  break;
        case 31: winddir = 59;  break;
        case 32: winddir = 177; break;
        case 33: winddir = 174; break;
        case 34: winddir = 169; break;
        case 35: winddir = 172; break;
        case 36: winddir = 158; break;
        case 37: winddir = 160; break;
        case 38: winddir = 166; break;
        case 39: winddir = 163; break;
        case 40: winddir = 135; break;
        case 41: winddir = 138; break;
        case 42: winddir = 143; break;
        case 43: winddir = 141; break;
        case 44: winddir = 155; break;
        case 45: winddir = 152; break;
        case 46: winddir = 146; break;
        case 47: winddir = 149; break;
        case 48: winddir = 90;  break;
        case 49: winddir = 93;  break;
        case 50: winddir = 98;  break;
        case 51: winddir = 96;  break;
        case 52: winddir = 110; break;
        case 53: winddir = 107; break;
        case 54: winddir = 101; break;
        case 55: winddir = 104; break;
        case 56: winddir = 132; break;
        case 57: winddir = 129; break;
        case 58: winddir = 124; break;
        case 59: winddir = 127; break;
        case 60: winddir = 113; break;
        case 61: winddir = 115; break;
        case 62: winddir = 121; break;
        case 63: winddir = 118; break;
        case 64: winddir = 357; break;
        case 65: winddir = 354; break;
        case 66: winddir = 349; break;
        case 67: winddir = 352; break;
        case 68: winddir = 338; break;
        case 69: winddir = 340; break;
        case 70: winddir = 346; break;
        case 71: winddir = 343; break;
        case 72: winddir = 315; break;
        case 73: winddir = 318; break;
        case 74: winddir = 323; break;
        case 75: winddir = 321; break;
        case 76: winddir = 335; break;
        case 77: winddir = 332; break;
        case 78: winddir = 326; break;
        case 79: winddir = 329; break;
        case 80: winddir = 270; break;
        case 81: winddir = 273; break;
        case 82: winddir = 278; break;
        case 83: winddir = 276; break;
        case 84: winddir = 290; break;
        case 85: winddir = 287; break;
        case 86: winddir = 281; break;
        case 87: winddir = 284; break;
        case 88: winddir = 312; break;
        case 89: winddir = 309; break;
        case 90: winddir = 304; break;
        case 91: winddir = 307; break;
        case 92: winddir = 293; break;
        case 93: winddir = 295; break;
        case 94: winddir = 301; break;
        case 95: winddir = 298; break;
        case 96: winddir = 180; break;
        case 97: winddir = 183; break;
        case 98: winddir = 188; break;
        case 99: winddir = 186; break;
        case 100: winddir = 200; break;
        case 101: winddir = 197; break;
        case 102: winddir = 191; break;
        case 103: winddir = 194; break;
        case 104: winddir = 222; break;
        case 105: winddir = 219; break;
        case 106: winddir = 214; break;
        case 107: winddir = 217; break;
        case 108: winddir = 203; break;
        case 109: winddir = 205; break;
        case 110: winddir = 211; break;
        case 111: winddir = 208; break;
        case 112: winddir = 267; break;
        case 113: winddir = 264; break;
        case 114: winddir = 259; break;
        case 115: winddir = 262; break;
        case 116: winddir = 248; break;
        case 117: winddir = 250; break;
        case 118: winddir = 256; break;
        case 119: winddir = 253; break;
        case 120: winddir = 225; break;
        case 121: winddir = 228; break;
        case 122: winddir = 233; break;
        case 123: winddir = 231; break;
        case 124: winddir = 245; break;
        case 125: winddir = 242; break;
        case 126: winddir = 236; break;
        case 127: winddir = 239; break;
        default: winddir = -1;
        }

        fx1m[0][secary] = winddir;
        fx1m[1][secary] = qclimit(winddir, ceiling[WD], threshold[WD]);
//		printf("winddirection=%d,tag=%d\n",winddir,tag);
//		printf("pthread:%d,%d,%d\n",secary,fx1m[0][secary],fx1m[1][secary]);
        asciibyte4(sampwd, winddir, 4+secary*4);
		if(secary<59)
			asciibyte4(sampwd, winddir, 4+(secary+1)*4);
        sampwd[244] = '\r';
        sampwd[245] = '\n';

#ifndef ZZ090310
        if(secary == 59)
            wdset = secary+(min%2)*60;
        else
            wdset = secary+(minary%2)*60;
//      printf("1m=%d,  min=%d, 2m=%d\n",secary,min,wdset);
        fx2m120[0][wdset] = winddir;
        fx2m120[1][wdset] = fx1m[1][secary];

        if(secary == 1)
            {
            fx1m[0][2] = fx1m[0][1];
            fx1m[1][2] = fx1m[1][1];
            fx2m120[0][wdset+1] = fx2m120[0][wdset];
            fx2m120[1][wdset+1] = fx2m120[1][wdset];
            }
#endif
		}	// if(n == 0)

        ioctl(fd_tc0, IOCTL_TC_GETCNT, &fsdata);
		for(n=0;n<8;n++)
			{
			fs3s[0][n] = fs3s[0][n+4];
        	fs3s[1][n] = fs3s[1][n+4];
			} 
//		printf("wsbl1=%f,wsbl0=%f\n",wsbl1,wsbl0);
		for(n=0;n<4;n++)
			{
//			printf("WS=%d\n",fsdata[n]);	
			if(fsdata[n] == 0)
				ret = 0;
			else
				ret = (fsdata[n]*4*wsbl1*100 + wsbl0*100 + 5) / 10;
//				ret = (fsdata[n]*4*4.93 + 24 + 5) / 10;
        	fs3s[0][n+8] = ret;
			fs3s[1][n+8] = qcsamp(ret, WS, TEN, 9999);
//			printf("fsdata[n]=%d,ret=%d,tag=%d\n",fsdata[n]*4,fs3s[0][n+8],fs3s[1][n+8]);
			asciibyte4(sampws, ret, 4+secary*16+n*4);
        	if(secary<59)
            	asciibyte4(sampws, ret, 4+(secary+1)*16+n*4);
			sampws[964] = '\r';
        	sampws[965] = '\n';
        
			if(fs3s[1][n+8]==RI && ret>sampfs1mmax)
				{
				sampfs1mmax = ret;
				sampfx1mmax = fx1m[0][secary];
				}
			}
		fs1m[0][secary] = fs3s[0][11];
		fs1m[1][secary] = fs3s[1][11];
        fs2m120[0][secary+(minary%2)*60] = fs3s[0][11];
        fs2m120[1][secary+(minary%2)*60] = fs3s[1][11];
//   	printf("sec=%d, min=%d\n",sec, min);
//		printf("winddirection=%d\n",winddir);
//		sleep(1);
		usleep(800000);
		}
    }


static void* canRev(void *t)
	{
    CanData data;
    int candata, ccxuan, cc1, cc2, cc3;
	int ccshijian = 61;
	double gtjs1=0, gtjs2=0, gtjs3=0, gt01=0, gt02=0, gt03=0, gt1=0, gt2=0, gt3=0;
	int i;
	int voltage;
	int fbcandata;
	char statclim[37]={"STATCLIM 0 120 0 0316 0 0 - - - - -\r\n"};
    char statradi[37]={"STATRADI 0 120 0 0312 0 0 - - - - -\r\n"};
    char stateath[37]={"STATEATH 0 120 0 0311 0 0 - - - - -\r\n"};
	char statintl[39]={"STATINTL_1 0 120 0 0308 0 0 - - - - -\r\n"};
	char statccsw[4];
    int n, ret, sret, winddir, wdset;

    for(;;)
	{
        read(fd_can, &data, sizeof(CanData));

/*
        printf("Receive CAN package:\n");
        printf("ID  = %x\n", data.id);
        printf("FF  = %d\n", data.IsExt);
	printf("RTR = %d\n", data.rxRTR);
        printf("DAT = ");
        for(i=0;i<data.dlc;i++)
            printf("%x ", data.data[i]);
	printf("sec=%d\n",sec);
        printf("\n\n");
*/

	switch(data.id)
	{
/*
	case 0x184:
 	    for(n=0;n<8;n++)
                {
                fs3s[0][n] = fs3s[0][n+4];
                fs3s[1][n] = fs3s[1][n+4];
                }
//              printf("wsbl1=%f,wsbl0=%f\n",wsbl1,wsbl0);
	    sret =  data.data[0]*256 + data.data[1];
            for(n=0;n<4;n++)
                {
//              printf("WS=%d\n",fsdata[n]);    
                if(sret == 0)
                    ret = 0;
                else
		    ret = (sret/10.0*0.1*100 + 5) / 10;
//                  ret = (sret/10*wsbl1*100 + wsbl0*100 + 5) / 10;
//                  ret = (sret*4.93 + 24 + 5) / 10;
                fs3s[0][n+8] = ret;
                fs3s[1][n+8] = qcsamp(ret, WS, TEN, 9999);
//              printf("fsdata[n]=%d,ret=%d,tag=%d\n",sret,fs3s[0][n+8],fs3s[1][n+8]);
                asciibyte4(sampws, ret, 4+secary*16+n*4);
//                if(secary<59)
//                    asciibyte4(sampws, ret, 4+(secary+1)*16+n*4);
                sampws[964] = '\r';
                sampws[965] = '\n';

                if(fs3s[1][n+8]==RI && ret>sampfs1mmax)
                    {
                    sampfs1mmax = ret;
                    sampfx1mmax = fx1m[0][secary];
                    }
                }
            fs1m[0][secary] = fs3s[0][11];
            fs1m[1][secary] = fs3s[1][11];
            fs2m120[0][secary+(minary%2)*60] = fs3s[0][11];
            fs2m120[1][secary+(minary%2)*60] = fs3s[1][11];
	break;
*/

        case 0x184:
            for(n=0;n<8;n++)
                {
                fs3s[0][n] = fs3s[0][n+4];
                fs3s[1][n] = fs3s[1][n+4];
                }
//              printf("wsbl1=%f,wsbl0=%f\n",wsbl1,wsbl0);
            sret =  data.data[0]*256 + data.data[1];
            for(n=0;n<4;n++)
                {
//              printf("WS=%d\n",fsdata[n]);    
                if(sret == 0)
                    ret = 0;
                else
                    ret = (sret/10.0*0.1*100 + 5) / 10;
//                  ret = (sret/10*wsbl1*100 + wsbl0*100 + 5) / 10;
//                  ret = (sret*4.93 + 24 + 5) / 10;
                fs3s[0][n+8] = ret;
                fs3s[1][n+8] = qcsamp(ret, WS, TEN, 9999);
//              printf("fsdata[n]=%d,ret=%d,tag=%d\n",sret,fs3s[0][n+8],fs3s[1][n+8]);
                asciibyte4(sampws, ret, 4+jd184*16+n*4);
//                if(secary<59)
//                    asciibyte4(sampws, ret, 4+(secary+1)*16+n*4);
                sampws[964] = '\r';
                sampws[965] = '\n';

                if(fs3s[1][n+8]==RI && ret>sampfs1mmax)
                    {
                    sampfs1mmax = ret;
                    sampfx1mmax = fx1m[0][jd284];
                    }
                }
            fs1m[0][jd184] = fs3s[0][11];
            fs1m[1][jd184] = fs3s[1][11];
            fs2m120[0][jd184+(minary%2)*60] = fs3s[0][11];
            fs2m120[1][jd184+(minary%2)*60] = fs3s[1][11];

            jd184++;
            if(jd184 >= 60)
                jd184 = 0;
        break;

/*
	case 0x284:
	    winddir = data.data[0]*256 + data.data[1];
	    fx1m[0][secary] = winddir;
            fx1m[1][secary] = qclimit(winddir, ceiling[WD], threshold[WD]);
//            printf("winddirection=%d,tag=%d,secary=%d\n",winddir,tag,secary);
//              printf("pthread:%d,%d,%d\n",secary,fx1m[0][secary],fx1m[1][secary]);
            asciibyte4(sampwd, winddir, 4+secary*4);
//            if(secary<59)
//                asciibyte4(sampwd, winddir, 4+(secary+1)*4);
            sampwd[244] = '\r';
            sampwd[245] = '\n';

#ifndef ZZ090310
        if(secary == 59)
            wdset = secary+(min%2)*60;
        else
            wdset = secary+(minary%2)*60;
//      printf("1m=%d,  min=%d, 2m=%d\n",secary,min,wdset);
        fx2m120[0][wdset] = winddir;
        fx2m120[1][wdset] = fx1m[1][secary];

        if(secary == 1)
            {
            fx1m[0][2] = fx1m[0][1];
            fx1m[1][2] = fx1m[1][1];
            fx2m120[0][wdset+1] = fx2m120[0][wdset];
            fx2m120[1][wdset+1] = fx2m120[1][wdset];
            }
#endif
	break;
*/
        case 0x284:
            winddir = data.data[0]*256 + data.data[1];
            fx1m[0][jd284] = winddir;
            fx1m[1][jd284] = qclimit(winddir, ceiling[WD], threshold[WD]);
//            printf("winddirection=%d,tag=%d,secary=%d\n",winddir,tag,secary);
//              printf("pthread:%d,%d,%d\n",secary,fx1m[0][secary],fx1m[1][secary]);
            asciibyte4(sampwd, winddir, 4+jd284*4);
//            if(secary<59)
//                asciibyte4(sampwd, winddir, 4+(secary+1)*4);
            sampwd[244] = '\r';
            sampwd[245] = '\n';

            jd284++;
            if(jd284 >= 60)
                jd284 = 0;
        break;

#ifdef TARH
	    case 0x183:
		temp45d = data.data[0]*256 + data.data[1];
                if(temp45d > 32767)
                    temp45d -= 65536;
//	        printf("wendu0=%2x%2x,%d,sec=%d\n",data.data[0],data.data[1],temp45d,sec);
	        temp45d += dzt0*10;                                                     // dzt0
//	        printf("res=%ftemp0=%dtemp1=%ddzt0=%dwpxx=%dtemp45D=%d\n",res,temp0,temp1,dzt0,wpxx,temp45d);
	        samp45d[0][jd28f] = temp45d;
    	        samp45d[1][jd28f] = qcsamp(temp45d, T0, HUNDRED, 9999);
	        if(temp45d>=9999 || temp45d<=-9999)
    	            tagnotRI(sampt, 4+jd28f*4);
	        else
    	            {
        	    if(temp45d >=0)
            	    	asciitemp(sampt, (temp45d+5)/10, 4+jd28f*4);
                    else
                    	asciitemp(sampt, (temp45d-5)/10, 4+jd28f*4);
                    }
            	sampt[124] = '\r';
            	sampt[125] = '\n';
//          	printf("samp45d = %d, qc=%d\n", samp45d[0][suffix], samp45d[1][suffix]);
//		jd18f++;
//		if(jd18f >= 30)
//  		    jd18f = 0;

		humi = ((data.data[2]*256 + data.data[3]) + 50) / 100;
		humi += dzu;
		if(humi > 99)
		    humi = 99;
		if(humi < 0)
		    humi = 0;
//		printf("humi=%2x%2x,%d\n",data.data[2],data.data[3],humi);
            	sampsd[0][jd28f] = humi;
            	sampsd[1][jd28f] = qcsamp(humi, U, 1, 9999);
            	asciibyte4(samprh, humi, 4+jd28f*4);
            	samprh[124] = '\r';
            	samprh[125] = '\n';
//	        printf("humi=%d, qc=%d, sec=%d\n",humi,sampsd[1][jd28f],sec);
		jd28f++;
		if(jd28f >= 30)
                    jd28f = 0;
            break;
#endif
	    default:
		usleep(1); 
	    }
        fflush(stdout);
        }
    return NULL;
    }

#define MAX_CANDATALEN	8
static void CanSendString(char *pstr, int id, int len)
	{
    CanData data;
//    int len=strlen(pstr);

    memset(&data,0,sizeof(CanData));
//  data.id=0x80;
    data.id=id;
    data.dlc=8;
    for(;len>MAX_CANDATALEN;len-=MAX_CANDATALEN)
		{
        memcpy(data.data, pstr, 8);
        write(fd_can, &data, sizeof(data));
        pstr+=8;
        }
    data.dlc=len;
    memcpy(data.data, pstr, len);
    //write(can_fd, pstr, len);
    write(fd_can, &data, sizeof(CanData));
	}


int main(int argc, void *argv[])
	{
    pthread_t th_tc0, th_tc1;
	pthread_t th_can, th_qy, th_njd;
	pthread_t th_raw;
//  at91_pio_arg arg;
    AD7792_CR_DATA argad;
	time_t timep;
	struct tm *p;
	int x ,y;
    int fd_tc1s = -1, fd_ad = -1, fd_fiqyl = -1;
	int fdfenzh = -1, fdxiaos = -1;
    int i;
    char dummy[32], data[32];
    int temp0, temp1;
    double res, zfevap;
	int radi, evap;
	int sampqy[2][30], sampnjd[2][4];
	int fs2m[2][2]={0,0,0,0}, fs10m[2][10], fx2m[2][2]={0,0,0,0}, fx10m[2][10];
	int qwmax=-9999, qwmin=9999, fjwdmax=-9999, fjwdmin=9999, sdmin=100, cmwdmax=-999, cmwdmin=999, dmwdmax=-999, dmwdmin=999, hwwdmax=-999, hwwdmin=999, qiyamax=0, qiyamin=20000, njdmin=64000;
	int id=0x00, slaveid;
	char str[256];

    int tempdec[2][3]={0};
    int tol, caseno, tempaver;
    int fci=0;	//30;
    int tempsort[3]={0};
	int fs1m3smax=-1, fx1m3smax=0, maxfs1msec=0, fs1h3smax=-1, fx1h3smax=0, maxfs1hhour=0, maxfs1hmin=0, fs1h10mmax=-1;

	int rrtime, rrhour, rrmin;
	int zfsbfl=0, zwxbfl=0, zjfsbfl=0, spmbfl=0, rzsj=0, xsrz=0, zfsmax=-1, zwxmax=-1, zjfsmax=-1;
	float floatzfsbfl=0, floatzwxbfl=0, floatzjfsbfl=0, floatspmbfl;;

	int td=0, localt=0, mintag=0, evap0=0, evap1=0;
	int fdparame=-1;
	char paramete[16], cc[38];

	int zbtemp;
	char pss[9] = {"DC,13.8\r\n"}, door[3]={"0\r\n"};
	int xdsd, suiqy, ludtemp;
	double ganqtemp, bhsqy, doublesuiqy;
	FILE *fdqcp;
	int qcpid, qcpidn;
	char qcps[BYTEQCPS], qcpm[BYTEQCPM];
	char *qcps1, *qcps2, *qcpm1, *qcpm2;
	char *delim=" \r";
	char *calib, *delimcc=",";

//	char insqy[6] = {"send\r\n"};	//PTB220
//    char insqy[4] = {".P\r\n"};		//PTB210
	char insqy[] = {"<P?>"};
	char insnjd[2] = {"FL"};
	char insnjdpw[9] = {"\r PW 1 0\r"};
	char insnjdbf[7] = {"#007V\r\n"};
	int rs485ret;
	int yuliang=0;
	int ylbl=5;
	int zfhstag=0, xszf0=0, xsyl0=0, zfaver0=0, zfcmp=0;

    float nianjr=0, jirxzz=0, tianswc=0, tiansq=0, shicha=0, chiwei=0, zhentys=0, taiysj=0, sinha=0;
	float locallong=0, locallat=0;
	int fzylzf=0, zfsw=0, zfsw0=1000;;
	
	int fdmaste=-1, mact=-1, wpxx=0;
	signed short int masttemp = 200;
	char statzfsw[4];

    char *string1 = {"                     "};                   // size=20;
	char senco[20];
//	float wsbl0=0, wsbl1=0.1, ws1bl0=0, ws1bl1=0.1, ratbl=0.1, rat1bl=0.5;
//	char rain[120];

	float floatradi;

//	int fd_485=-1;

	char insraw[6]={"RDPR\r\n"};

	int newtime=0, oldtime=0;

	int fdwrite;
	char readcr[7]={"readd\r\n"};
	char writecr[7]={"write\r\n"};

	int canjd=0;

//	New
	int wd_val_3s, wd_tag_3s, ws_val_3s, ws_tag_3s;
	char fzsj[512];
	int max0_w10min=1, max0_t0=1, min0_t0=1, min0_u=1, max0_p=1, min0_p=1, min0_vi=1;

    printf("weather 2016-03-25 15:00 V7.3\n");

    twoarrayinit(*sampqy, 2, 30);
	twoarrayinit(*samp45d, 2, 30);
	twoarrayinit(*sampdlwd, 2, 30);
	twoarrayinit(*sampdlsd, 2, 30);
    twoarrayinit(*sampsd, 2, 30);
    twoarrayinit(*sampnjd, 2, 4);

    twoarrayinit(*fx1m, 2, 60);
    twoarrayinit(*fx2m, 2, 2);
    twoarrayinit(*fx2m120, 2, 120);
    twoarrayinit(*fx10m, 2, 10);
    twoarrayinit(*fs3s, 2, 12);
    twoarrayinit(*fs1m, 2, 60);
    twoarrayinit(*fs2m, 2, 2);
	twoarrayinit(*fs2m120, 2, 120);
    twoarrayinit(*fs10m, 2, 10);
	twoarrayinit(*njd10min, 2, 10);

	memset(fzsj, ' ', CNT_FZSJ);

//	td
    if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
    	td = 0;
	else
		{
        read(fdparame,&paramete,4);
        close(fdparame);
		paramete[4]='\0';
//		printf("printf=%s\n",paramete);
        td = atof(paramete);
//      printf("td=%d\n",td);
		if(td<-180 || td>180)
			td = 0;
		}
	localt = (60-td%60) % 60;
//	printf("td=%d,localt=%d\n",td,localt);

    if((fdparame = open("/tmp/paramete/long.txt",O_RDONLY))< 1)
        {
        locallong = 120;
        }
    else
        {
        read(fdparame, paramete, 9);
        locallong = (paramete[0]-'0')*100 + (paramete[1]-'0')*10 + (paramete[2]-'0');
//      printf("id=%s\n",stidd);
        close(fdparame);
        }

    if((fdparame = open("/tmp/paramete/lat.txt",O_RDONLY))< 1)
        {
        locallat = 40;
        }
    else
        {
        read(fdparame, paramete, 8);
        locallat = (paramete[0]-'0')*10 + (paramete[1]-'0') + ((paramete[3]-'0')*10+(paramete[4]-'0'))/60.0;
//      printf("id=%s\n",stidd);
        close(fdparame);
        }

//	DZ
	if((fddz = open("/tmp/paramete/dztn.txt",O_RDONLY)) < 1)				// T0--T3
		usleep(1);
	else
		{
		read(fddz, dztn, sizeof(dztn));
        close(fddz);
		if(dztn[0] == '-')													// T0
			dzt0 = ((dztn[1]-'0')*10 + (dztn[2]-'0')) * (-1);
		else
            dzt0 = (dztn[1]-'0')*10 + (dztn[2]-'0');
        if(dztn[3] == '-')                                                  // T1
            dzt1 = ((dztn[4]-'0')*10 + (dztn[5]-'0')) * (-1);
        else
            dzt1 = (dztn[4]-'0')*10 + (dztn[5]-'0');
        if(dztn[6] == '-')                                                  // T2
            dzt2 = ((dztn[7]-'0')*10 + (dztn[8]-'0')) * (-1);
        else
            dzt2 = (dztn[7]-'0')*10 + (dztn[8]-'0');
        if(dztn[9] == '-')                                                  // T3
            dzt3 = ((dztn[10]-'0')*10 + (dztn[11]-'0')) * (-1);
        else
            dzt3 = (dztn[10]-'0')*10 + (dztn[11]-'0');
		}

    if((fddz = open("/tmp/paramete/dzstn.txt",O_RDONLY)) < 1)				// TG, ST0--ST8
        usleep(1);
    else
        {
        read(fddz, dzstn, sizeof(dzstn));
        close(fddz);
        if(dzstn[0] == '-')                                                 // TG
            dztg = ((dzstn[1]-'0')*10 + (dzstn[2]-'0')) * (-1);
        else
            dztg = (dzstn[1]-'0')*10 + (dzstn[2]-'0');
        if(dzstn[3] == '-')                                                 // T0
            dzst0 = ((dzstn[4]-'0')*10 + (dzstn[5]-'0')) * (-1);
        else
            dzst0 = (dzstn[4]-'0')*10 + (dzstn[5]-'0');
        if(dzstn[6] == '-')                                                 // T1
            dzst1 = ((dzstn[7]-'0')*10 + (dzstn[8]-'0')) * (-1);
        else
            dzst1 = (dzstn[7]-'0')*10 + (dzstn[8]-'0');
        if(dzstn[9] == '-')                                                 // T2
            dzst2 = ((dzstn[10]-'0')*10 + (dzstn[11]-'0')) * (-1);
        else
            dzst2 = (dzstn[10]-'0')*10 + (dzstn[11]-'0');
        if(dzstn[12] == '-')                                                // T3
            dzst3 = ((dzstn[13]-'0')*10 + (dzstn[14]-'0')) * (-1);
        else
            dzst3 = (dzstn[13]-'0')*10 + (dzstn[14]-'0');
        if(dzstn[15] == '-')                                                // T4
            dzst4 = ((dzstn[16]-'0')*10 + (dzstn[17]-'0')) * (-1);
        else
            dzst4 = (dzstn[16]-'0')*10 + (dzstn[17]-'0');
        if(dzstn[18] == '-')                                                // T5
            dzst5 = ((dzstn[19]-'0')*10 + (dzstn[20]-'0')) * (-1);
        else
            dzst5 = (dzstn[19]-'0')*10 + (dzstn[20]-'0');
        if(dzstn[21] == '-')                                                // T6
            dzst6 = ((dzstn[22]-'0')*10 + (dzstn[23]-'0')) * (-1);
        else
            dzst6 = (dzstn[22]-'0')*10 + (dzstn[23]-'0');
        if(dzstn[24] == '-')                                                // T7
            dzst7 = ((dzstn[25]-'0')*10 + (dzstn[26]-'0')) * (-1);
        else
            dzst7 = (dzstn[25]-'0')*10 + (dzstn[26]-'0');
        if(dzstn[27] == '-')                                                // T8
            dzst8 = ((dzstn[28]-'0')*10 + (dzstn[29]-'0')) * (-1);
        else
            dzst8 = (dzstn[28]-'0')*10 + (dzstn[29]-'0');
        }

    if((fddz = open("/tmp/paramete/dzran.txt",O_RDONLY)) < 1)				// RAT, RAW, U
        usleep(1);
    else
        {
        read(fddz, dzran, sizeof(dzran));
        close(fddz);
//		printf("dzran=%s",dzran);
        if(dzran[0] == '-')                                                 // RAT
            dzrat = ((dzran[1]-'0')*10 + (dzran[2]-'0')) * (-1);
        else
            dzrat = (dzran[1]-'0')*10 + (dzran[2]-'0');
        if(dzran[3] == '-')                                                 // RAW
            dzraw = ((dzran[4]-'0')*10 + (dzran[5]-'0')) * (-1);
        else
            dzraw = (dzran[4]-'0')*10 + (dzran[5]-'0');
        if(dzran[6] == '-')                                                 // U
            dzu = ((dzran[7]-'0')*10 + (dzran[8]-'0')) * (-1);
        else
            dzu = (dzran[7]-'0')*10 + (dzran[8]-'0');
        }
//		printf("dzu=%d\n",dzu);

    if((fddz = open("/tmp/paramete/dzsensi.txt",O_RDONLY)) < 1)				// GR, DR
        usleep(1);
    else
        {
        read(fddz, dzsensi, sizeof(dzsensi));
        close(fddz);
        if(dzsensi[0] == '-')                                               // GR
            dzgr = ((dzsensi[1]-'0')*10 + (dzsensi[2]-'0')) * (-1);
        else
            dzgr = (dzsensi[1]-'0')*10 + (dzsensi[2]-'0');
        if(dzsensi[3] == '-')                                               // DR
            dzdr = ((dzsensi[4]-'0')*10 + (dzsensi[5]-'0')) * (-1);
        else
            dzdr = (dzsensi[4]-'0')*10 + (dzsensi[5]-'0');
        }
	
//	printf("t0=%d,t1=%d,t2=%d,t3=%d\n",dzt0,dzt1,dzt2,dzt3);
//    printf("tg=%d,st0=%d,st1=%d,st2=%d,st3=%d,st4=%d,st5=%d,st6=%d,st7=%d,st8=%d\n",dztg,dzst0,dzst1,dzst2,dzst3,dzst4,dzst5,dzst6,dzst7,dzst8);
//	printf("rat=%d,raw=%d,u=%d\n",dzrat,dzraw,dzu);
//	printf("gr=%d,dr=%d\n",dzgr,dzdr);

//	QCPS
	if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
		usleep(1);
	else
		{
		for(qcpid=0,qcpidn=0; qcpid<9; qcpid++)
			{
			fseek(fdqcp, qcpidn, SEEK_SET);
			fread(qcps, BYTEQCPS, 1, fdqcp);
			qcps1 = strtok(qcps, delim);
			if(qcps1[0] == '/')
				threshold[qcpid] = -30000;
			else
				{
				threshold[qcpid] = atof(qcps1);
				}
            qcps1 = strtok(NULL, delim);
            if(qcps1[0] == '/')
                ceiling[qcpid] = 30000;
            else
                {
            	ceiling[qcpid] = atof(qcps1);
				}
            qcps1 = strtok(NULL, delim);
            if(qcps1[0] == '/')
                suspirate[qcpid] = 30000;
            else
                {
            	suspirate[qcpid] = atof(qcps1);
				}
			qcpidn += BYTEQCPS;
			}
		fclose(fdqcp);
		}

//  QCPM
    if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
        usleep(1);
    else
        {
        for(qcpid=0,qcpidn=0; qcpid<9; qcpid++)
            {
            fseek(fdqcp, qcpidn, SEEK_SET);
            fread(qcpm, BYTEQCPM, 1, fdqcp);
            qcpm1 = strtok(qcpm, delim);
            if(qcpm1[0] == '/')
                instthre[qcpid] = -30000;
            else
                {
                instthre[qcpid] = atof(qcpm1);
                }
            qcpm1 = strtok(NULL, delim);
            if(qcpm1[0] == '/')
                instceil[qcpid] = 30000;
            else
                {
                instceil[qcpid] = atof(qcpm1);
                }
            qcpm1 = strtok(NULL, delim);
            if(qcpm1[0] == '/')
                instsusp[qcpid] = 30000;
            else
                {
                instsusp[qcpid] = atof(qcpm1);
                }
            qcpm1 = strtok(NULL, delim);
            if(qcpm1[0] == '/')
                instwrong[qcpid] = 32000;
            else
                {
                instwrong[qcpid] = atof(qcpm1);
                }
            qcpidn += BYTEQCPM;
            }
        fclose(fdqcp);
		}
//	for(x=0;x<73;x++)
//		printf("wrong=%f	",instwrong[x]);	
//printf("\nceil=%f,thres=%f,sus=%f,wrong=%f,cost=%d\n",instceil[T0],instthre[T0],instsusp[T0],instwrong[T0],instcost[T0]);


//	time
    time(&timep);
    p = localtime(&timep);
    sec  = p->tm_sec;
    min  = p->tm_min;
    hour = p->tm_hour;
	mday = p->tm_mday;
	mon  = p->tm_mon + 1;
	year = p->tm_year % 100;
    if(sec == 0)
        secary = 59;
    else
        secary = sec - 1;
	suffix = secary / 2;

	beijintime[0]  = year / 10 + '0';
    beijintime[1]  = year % 10 + '0';
    beijintime[2]  = mon  / 10 + '0';
    beijintime[3]  = mon  % 10 + '0';
    beijintime[4]  = mday / 10 + '0';
    beijintime[5]  = mday % 10 + '0';
    beijintime[6]  = hour / 10 + '0';
    beijintime[7]  = hour % 10 + '0';
    beijintime[8]  = min  / 10 + '0';
    beijintime[9]  = min  % 10 + '0';
    beijintime[10] = sec  / 10 + '0';
    beijintime[11] = sec  % 10 + '0';


//	can-init	
    fd_can = open(CAN_DEV, O_RDWR);
	if(fd_can < 0)
		{
        printf("Error opening %s can device\n", CAN_DEV);
        return -1;
        }
    ioctl(fd_can, UPCAN_IOCTRL_PRINTRIGISTER, 1);
    ioctl(fd_can, UPCAN_IOCTRL_SETID, id);
//  ioctl(fd_can, UPCAN_IOCTRL_SETBAND, 1);
    /* Create the threads */
    pthread_create(&th_can, NULL, canRev, 0);

//	rs485
    fd_485 = open("/dev/zlg7289", O_RDWR);
    if(fd_485 == -1){
        printf("open /dev/zlg7289 failed\n");
        exit(1);
    }

//	pio-init
	fd_pio = open(DEV_PIO, O_RDWR);
    if (fd_pio < 0) 
		{
        printf("open PIO device error! %d\n", fd_pio);
        return -1;
        }

    for (i=0; i<8; i++)									// set winddir pio 
		{
        argwd.pin_idx = i;
        argwd.pin_dir = AT91PIO_DIR_INP;
        argwd.pin_sta = 0;
        ioctl(fd_pio, IOCTL_PIO_SETDIR, &argwd);
//		ioctl(fd_pio, IOCTL_PIO_SETSTA, &argwd);
        }

    arg.pin_idx = PIO_RS485_CTL2;						// 485-2 CTL
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 1;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
	ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_LVL_INP1;							// Door
    arg.pin_dir = AT91PIO_DIR_INP;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);


#ifdef AD7792 
//	ad-init
    fd_ad = open(DEV_AD, O_RDWR);
    if (fd_ad < 0) 
		{
        printf("open AD device error! %d\n", fd_ad);
        return -1;
        }

    argad.CRH_data=0x14;
    argad.CRL_data=0x10;
    ioctl(fd_ad, IOCTL_CR_ZERO_CONFIG, &argad);
    ioctl(fd_ad, IOCTL_CR_FULL_CONFIG, &argad);

    arg.pin_idx = PIO_ADX_EN;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_ADY_EN;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_ADX_SEL0;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_ADX_SEL1;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_ADX_SEL2;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

    arg.pin_idx = PIO_ADX_SEL3;
    arg.pin_dir = AT91PIO_DIR_OUT;
    arg.pin_sta = 0;
    ioctl(fd_pio, IOCTL_PIO_SETDIR, &arg);
    ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
#endif


//	tty-init
    fd_ttyS0 = open(SERIAL1, O_RDWR);
//    fd_ttyS0 = open(UART1, O_RDWR);
    if (fd_ttyS0 < 0)
        {
        printf("open SERIAL1 RAW error %d\n", fd_ttyS0);
        return -1;
        }
    set_speed(fd_ttyS0, 9600);
    set_other_attribute(fd_ttyS0, 8, 1, 0, 50);
    pthread_create(&th_raw, NULL, pthread_raw, 0);

	fd_ttyS1 = open(SERIAL0, O_RDWR);
//	fd_ttyS1 = open(UART1, O_RDWR);
    if (fd_ttyS1 < 0)
		{
        printf("open uart 1 VI error %d\n", fd_ttyS1);
		return -1;
		}
    set_speed(fd_ttyS1, 9600);
	set_other_attribute(fd_ttyS1, 8, 1, 0, 10);
	pthread_create(&th_njd, NULL, njdpthreadlandun, 0);
//	set_other_attribute(fd_ttyS1, 7, 1, 2, 50);
//    pthread_create(&th_njd, NULL, njdpthreadzhzh, 0);
//	rs485clt(fd_485, R485_STA_CTL1, RX);

	fd_ttyS2 = open(SERIAL2, O_RDWR);
//	fd_ttyS2 = open(UART2, O_RDWR);
    if (fd_ttyS2 < 0)
		{
        printf("open uart 2 P error %d\n", fd_ttyS2);
		return -1;
		}
    set_speed(fd_ttyS2, 9600);
    set_other_attribute(fd_ttyS2, 8, 1, 0, 100);
    pthread_create(&th_qy, NULL, qiyapthread, 0);

/*
//	tc-init
//	tc0, CNT
   	fd_tc0 = open(DEV_TC0, O_RDWR);
   	if (fd_tc0 < 0)
        {
        printf("open tc device error! %d\n", fd_tc0);
        return -1;
        }
    pthread_create(&th_tc0, NULL, tc0fs, 0);
//	ioctl(fd_tc0, IOCTL_TC_START, 4);
*/

//	test tc2, TIMER
    fd_tc1s = open(DEV_TC2, O_RDWR);
    if (fd_tc1s < 0) 
		{
        printf("open tc device error! %d\n", fd_tc1s);
        return -1;
      	}
    ioctl(fd_tc1s, IOCTL_TC_START, 4);

    fd_fiqyl = open(DEV_FIQ, O_RDWR);
    if (fd_fiqyl < 0) 
		{
        printf("open FIQ device error! %d\n", fd_fiqyl);
        return -1;
        }
	ioctl(fd_fiqyl, IOCTL_FIQ_SETIRQ, (unsigned long)fiq_handle);

    if((fdsenco = open("/tmp/paramete/sencows.txt", O_RDONLY)) < 1)
        usleep(1);
    else
        {
        read(fdsenco, senco, sizeof(senco));
        close(fdsenco);
        wsbl0 = atof(senco);
        string1 = strchr(senco, ' ');
        string1++;
        wsbl1 = atof(string1);
//        printf("wsbl0=%f, wsbl1=%f\n",wsbl0,wsbl1);
        }

    if((fdsenco = open("/tmp/paramete/sencorat.txt", O_RDONLY)) < 1)
        usleep(1);
    else
        {
        read(fdsenco, senco, sizeof(senco));
        close(fdsenco);
        string1 = strchr(senco, ' ');
        ratbl = atof(string1);
//        printf("ratbl=%f\n",ratbl);
        }

	while(1)
		{ 
        read(fd_tc1s, dummy, 0);
//		printf("timer exceed ...\n");

		time(&timep);
		p = gmtime(&timep);
		sec = p->tm_sec;
		min = p->tm_min;
		hour = p->tm_hour;
		mday = p->tm_mday;
        mon  = p->tm_mon + 1;
        year = p->tm_year % 100;
        if(min == 0)
            minary = 59;
        else
            minary = min - 1;
		if(sec == 0)
			secary = 59;
		else
			secary = sec - 1;
        suffix = secary / 2;
//		printf("sec=%d, min=%d ,suffux=%d\n",sec, min, suffix);
	    beijintime[0]  = year / 10 + '0';
        beijintime[1]  = year % 10 + '0';
	    beijintime[2]  = mon  / 10 + '0';
        beijintime[3]  = mon  % 10 + '0';
		beijintime[4]  = mday / 10 + '0';
	    beijintime[5]  = mday % 10 + '0';
        beijintime[6]  = hour / 10 + '0';
	    beijintime[7]  = hour % 10 + '0';
        beijintime[8]  = min  / 10 + '0';
	    beijintime[9]  = min  % 10 + '0';
        beijintime[10] = sec  / 10 + '0';
        beijintime[11] = sec  % 10 + '0';
//		printf("min:sec=%2d:%2d\n",min,sec);

		slaveid=0x80;
		str[0] = 0;
        CanSendString(str, slaveid, 0);
		if(sec == 0)
			{
	        slaveid=0x100;
			time(&timep);
			str[0] = timep % 256;
			str[1] = timep / 256 % 256;
			str[2] = timep / (256*256) % 256;
			str[3] = timep / (256*256*256);
			str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
        	CanSendString(str, slaveid, 8);
			usleep(100);

#ifdef CANSTAT
            slaveid=0x605;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 1;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x605;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 2;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x605;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 3;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x606;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 1;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x606;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 2;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x606;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 3;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x605;
            str[0] = 0x40;
            str[1] = 0x05;
            str[2] = 0x38;
            str[3] = 1;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x606;
            str[0] = 0x40;
            str[1] = 0x05;
            str[2] = 0x38;
            str[3] = 1;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);

            slaveid=0x60F;
            str[0] = 0x40;
            str[1] = 0x05;
            str[2] = 0x38;
            str[3] = 1;
            str[4] = 0;
            str[5] = 0;
            str[6] = 0;
            str[7] = 0;
            CanSendString(str, slaveid, 8);
            usleep(100);
#endif
			}

		yuliang = read(fd_fiqyl, 0, 0);
//		yuliang = yuliang * ratbl * 10;
		if(yuliang >= 2)
			{
			printf("rainover=%d\n",yuliang-1);
			yuliang = 1;
//			printf("rainover=%d\n",yuliang-1);
			}
		yuliang = yuliang * ratbl * 10;
		fzyl += yuliang;
//		xsyl += yuliang;
//		printf("yuliang=%d,fzyl=%d,xsyl=%d\n",yuliang,fzyl,xsyl);


//		tc0fs();
		averagefs(*fs3s, 12);
		ws_val_3s = aver;							// instantaneous value of ws3s
		ws_tag_3s = tag;							// instantaneous tag of ws3s
		wd_val_3s = fx1m[0][jd284];					// instantaneous value of wd3s
        wd_tag_3s = fx1m[1][jd284];					// instantaneous tag of wd3s
//		for(x=0;x<2;x++)
//			{
//			for(y=0;y<12;y++)
//				printf("%d;",fs3s[x][y]);
//			}
//		printf("fs3s=%d,tag=%d\n",aver,tag);
//      for(x=0;x<2;x++)
//          {
//          for(y=0;y<60;y++)
//              printf("%d;",fx1m[x][y]);
//          }
//		printf("\n");
		if(tag==RI && aver>fs1m3smax)
        	{
			fs1m3smax = aver;
			fx1m3smax = fx1m[0][jd284];
//        	maxfs1msec = sec;
        	}
        if(tag==RI && aver>fs1h3smax)
            {
            fs1h3smax = aver;
			fx1h3smax = fx1m[0][jd284];
//			printf("main:%d,%d\n",(secary+60-1)%60,fx1h3smax);
//			maxfs1hhour = hour;
//            maxfs1hmin = min;
			if(min==0 && sec<2)									// Changed by XGR at 2013-01-11 13:00(begin)
				{
				maxfs1hhour = hour;
                maxfs1hmin = min;
				}
			else if(min != 59)									// Changed by XGR at 2013-01-11 13:00(end)
				{
				maxfs1hhour = hour;
            	maxfs1hmin = min + 1;
				}
			else
				{
				maxfs1hmin = 0;
				if(hour != 23)
					maxfs1hhour = hour + 1;
				else
					maxfs1hhour = 0;
				}
            }

		if(sec%2 == 0)
			{
#ifndef TARH
            argad.CRH_data=0x14;
            argad.CRL_data=0x10;
            ioctl(fd_ad, IOCTL_CR_ZERO_CONFIG, &argad);
            ioctl(fd_ad, IOCTL_CR_FULL_CONFIG, &argad);
            arg.pin_idx = PIO_ADX_SEL0;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            arg.pin_idx = PIO_ADX_SEL1;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            arg.pin_idx = PIO_ADX_SEL2;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            read(fd_ad, data, 2);
            temp0 = data[1]*256 + data[0];
            arg.pin_idx = PIO_ADX_SEL0;
            arg.pin_sta = 1;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            read(fd_ad, data, 2);
            temp1 = data[1]*256 + data[0];
            res = temp1 * 100.0 / temp0;
            if(temp45d >= 0)
            	temp45d = (A*res*res + B*res + C + 0.005) * 100 + WDJJ;
//                temp45d = (E/10000000*res*res*res + F/10000*res*res + G*res - H + 0.005) * 100 + wpxx;
            else
            	temp45d = (A*res*res + B*res + C - 0.005) * 100 + WDJJ;
//                temp45d = (E/10000000*res*res*res + F/10000*res*res + G*res - H - 0.005) * 100 + wpxx;
			temp45d += dzt0*10;														// dzt0
//			printf("res=%ftemp0=%dtemp1=%ddzt0=%dwpxx=%dtemp45D=%d\n",res,temp0,temp1,dzt0,wpxx,temp45d);
			samp45d[0][suffix] = temp45d;
			samp45d[1][suffix] = qcsamp(temp45d, T0, HUNDRED, 9999);
			if(temp45d>=9999 || temp45d<=-9999)
				tagnotRI(sampt, 4+suffix*4);
			else
				{
				if(temp45d >=0)
            		asciitemp(sampt, (temp45d+5)/10, 4+suffix*4);
				else
                	asciitemp(sampt, (temp45d-5)/10, 4+suffix*4);
				}
            sampt[124] = '\r';
            sampt[125] = '\n';
//			printf("samp45d = %d, qc=%d\n", samp45d[0][suffix], samp45d[1][suffix]);

            argad.CRH_data=0x10;
            argad.CRL_data=0x00;
            ioctl(fd_ad, IOCTL_CR_ZERO_CONFIG, &argad);
            ioctl(fd_ad, IOCTL_CR_FULL_CONFIG, &argad);
            arg.pin_idx = PIO_ADX_SEL0;
            arg.pin_sta = 1;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            arg.pin_idx = PIO_ADX_SEL1;
            arg.pin_sta = 1;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            arg.pin_idx = PIO_ADX_SEL2;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
            arg.pin_idx = PIO_ADX_SEL3;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
//          ioctl(fd_ad, IOCTL_MR_START, &argad);
//            usleep(200000);
//          read(fd_ad, data, 2);
//            usleep(20000);
            read(fd_ad, data, 2);
            humi = ((data[1]*256 + data[0]) * 2500 / 65535 + 5) / 10;
//	        printf("humi=%d\n",humi);
            if(humi > 99)
                humi = 99;
            if(humi < 0)
                humi = 0;
			humi += dzu;                                                     					// dzu
			sampsd[0][suffix] = humi;
            sampsd[1][suffix] = qcsamp(humi, U, 1, 9999);
			asciibyte4(samprh, humi, 4+suffix*4);
            samprh[124] = '\r';
            samprh[125] = '\n';
//			printf("humi=%d, qc=%d\n",humi,sampsd[1][suffix]);
#endif

            write(fd_ttyS2, &insqy, sizeof(insqy));
//			printf("pressure = %d\n", pres);
            sampqy[0][suffix] = pres;
            sampqy[1][suffix] = qcsamp(pres, P, TEN, 65535);
			}	//if(sec%2 == i)

		if(sec==30 || sec==40 || sec==50)
			{
			mintag = 1;
			}

		if(sec==35)
			{
			write(fd_ttyS1, &insnjdbf, sizeof(insnjdbf));					// VI
			write(fd_ttyS0, &insraw, sizeof(insraw));                       // RAW
			}			 

		if(sec==0 || (sec==1 && mintag==1))
			{

		jd184 = 0;
		jd284 = 0;
		jd28f = 0;

//	DZ
	if((fddz = open("/tmp/paramete/dztn.txt",O_RDONLY)) < 1)				// T0--T3
		usleep(1);
	else
		{
		read(fddz, dztn, sizeof(dztn));
        close(fddz);
		if(dztn[0] == '-')													// T0
			dzt0 = ((dztn[1]-'0')*10 + (dztn[2]-'0')) * (-1);
		else
            dzt0 = (dztn[1]-'0')*10 + (dztn[2]-'0');
        if(dztn[3] == '-')                                                  // T1
            dzt1 = ((dztn[4]-'0')*10 + (dztn[5]-'0')) * (-1);
        else
            dzt1 = (dztn[4]-'0')*10 + (dztn[5]-'0');
        if(dztn[6] == '-')                                                  // T2
            dzt2 = ((dztn[7]-'0')*10 + (dztn[8]-'0')) * (-1);
        else
            dzt2 = (dztn[7]-'0')*10 + (dztn[8]-'0');
        if(dztn[9] == '-')                                                  // T3
            dzt3 = ((dztn[10]-'0')*10 + (dztn[11]-'0')) * (-1);
        else
            dzt3 = (dztn[10]-'0')*10 + (dztn[11]-'0');
		}

//          for(x=0;x<60;x++)
//              printf("%d;",fx1m[0][x]);
//			printf("\n");
			averagefx(*fx1m,60);
//			printf("averfx1m=%d,tag=%d\n",aver,tag);
			fx2m[0][minary%2] = aver;
        	fx2m[1][minary%2] = tag;
        	fx10m[0][minary%10] = aver;
        	fx10m[1][minary%10] = tag;

//          averagefx(*fx2m,2);
#ifdef ZZ090310
        for(x=0;x<60;x++)
            fx2m120[0][x+60] = fx2m120[0][x];
        for(x=0;x<60;x++)
            fx2m120[1][x+60] = fx2m120[1][x];
        for(x=0;x<60;x++)
            fx2m120[0][x] = fx1m[0][x];
        for(x=0;x<60;x++)
            fx2m120[1][x] = fx1m[1][x];
#endif

		if(wd_tag_3s != MI)								// shun shi feng xiang
			{
			fzsj[TAG+0] = wd_tag_3s + '0';
			asciibyte4(fzsj, wd_val_3s, 5);
			}
		else
			{
            fzsj[TAG+0] = wd_tag_3s + '0';
            tagnotRI(fzsj, 5);
            }

        if(ws_tag_3s != MI)								// shun shi feng su
            {
            fzsj[TAG+1] = ws_tag_3s + '0';
            asciibyte4(fzsj, ws_val_3s, 10);
            }
        else
            {
            fzsj[TAG+1] = ws_tag_3s + '0';
            tagnotRI(fzsj, 10);
            }

		averagefx(*fx2m120, 120);						// 2 min feng xiang
        if(tag != MI)
            {
            fzsj[TAG+2] = tag + '0';
            asciibyte4(fzsj, aver, 15);
            }
        else
            {
            fzsj[TAG+2] = tag + '0';
            tagnotRI(fzsj, 15);
            }

		averagefs(*fs2m120, 120);						// 2 min feng su
        if(tag != MI)
            {
            fzsj[TAG+3] = tag + '0';
            asciibyte4(fzsj, aver, 20);
            }
        else
            {
            fzsj[TAG+3] = tag + '0';
            tagnotRI(fzsj, 20);
            }

		averagefx(*fx10m,10);							// 10 min feng xiang
        if(tag != MI)
            {
            fzsj[TAG+4] = tag + '0';
            asciibyte4(fzsj, aver, 25);
            }
        else
            {
            fzsj[TAG+4] = tag + '0';
            tagnotRI(fzsj, 25);
            }

	    averagefs(*fs1m, 60);
		if(tag == RI)
			tag = qcinstance(aver, WS, TEN, 9999);
        fs2m[0][minary%2] = aver;
        fs2m[1][minary%2] = tag;
        fs10m[0][minary%10] = aver;
        fs10m[1][minary%10] = tag;

		averagefs(*fs10m, 10);							// 10 min feng su
		if(tag == RI)
			tag = qcinstance(aver, WS, TEN, 9999);
        if(tag != MI)
            {
            fzsj[TAG+5] = tag + '0';
            asciibyte4(fzsj, aver, 30);

			if(tag==RI && aver>fs1h10mmax)              // zui da feng su
            	{
                fs1h10mmax = aver;
                fzsj[TAG+6] = tag + '0';
                fzsj[TAG+7] = tag + '0';
                fzsj[TAG+8] = tag + '0';
                stringmove(fzsj, 25, fzsj, 35, 10);
                fzsj[45] = hour / 10 + '0';
                fzsj[46] = hour % 10 + '0';
                fzsj[47] = min / 10 + '0';
                fzsj[48] = min % 10 + '0';
				max0_w10min = 0;
                }
            }
        else
            {
            fzsj[TAG+5] = tag + '0';
            tagnotRI(fzsj, 30);
			if(max0_w10min == 1)
                {
                fzsj[TAG+6] = tag + '0';
                fzsj[TAG+7] = tag + '0';
                fzsj[TAG+8] = tag + '0';
				stringmove(fzsj, 25, fzsj, 35, 10);
				tagnotRI(fzsj, 45);
                }
            }

        fzsj[TAG+9] = RI + '0';							// fen zhong nei zui da feng su
        fzsj[TAG+10] = RI + '0';
        asciibyte4(fzsj, fx1m3smax, 50);
        asciibyte4(fzsj, fs1m3smax, 55);
        sampfs1mmax = -1;

        fzsj[TAG+11] = RI + '0';
        fzsj[TAG+12] = RI + '0';
        fzsj[TAG+13] = RI + '0';
        asciibyte4(fzsj, fx1h3smax, 60);
        asciibyte4(fzsj, fs1h3smax, 65);
		fzsj[70] = maxfs1hhour / 10 + '0';
        fzsj[71] = maxfs1hhour % 10 + '0';
        fzsj[72] = maxfs1hmin / 10 + '0';
        fzsj[73] = maxfs1hmin % 10 + '0';
        fs1m3smax = -1;

        fzyltag = qclimit(fzyl, instceil[RAT]*TEN, instthre[RAT]*TEN);			// RAT
	    if(fzyltag != MI)
			{
			fzsj[TAG+14] = fzyltag + '0';
			fzsj[TAG+15] = RI + '0';
	        asciibyte4(fzsj, fzyl, 75);
			if(fzyltag == RI)    
		    	xsyl += fzyl;
	        asciibyte4(fzsj, xsyl, 80);
			fzyl = 0;
			}
	    else
            {
            fzsj[TAG+14] = fzyltag + '0';
			tagnotRI(fzsj, 75);
			fzyl = 0;
			}

        if(fzcctag != MI)								// RAW
            {
            fzsj[TAG+16] = fzcctag + '0';
            fzsj[TAG+17] = RI + '0';
          	asciibyte4(fzsj, fzcc, 85);
			xscc += fzcc;
			fzcc = 0;
           	asciibyte4(fzsj, xscc, 90);
            }
        else
            {
            fzsj[TAG+16] = fzcctag + '0';
            tagnotRI(fzsj, 85);
            }

/*
	    for(canjd=jd28f,jd28f=0;canjd>0&&canjd<30;canjd++)
                {
                samp45d[0][canjd] = samp45d[0][canjd-1];
                samp45d[1][canjd] = samp45d[1][canjd-1];
		if(samp45d[0][canjd-1]>=0)
		    asciitemp(sampt, (samp45d[0][canjd-1]+5)/10, 4+canjd*4);
		else
		    asciitemp(sampt, (samp45d[0][canjd-1]-5)/10, 4+canjd*4);
                }
*/
	    average(*samp45d, 30);							// T0
//		printf("temp=%d\n",aver);
		if(aver >= 0)
			aver = (aver+5) / 10;
		else
			aver = (aver-5) / 10;
        if(tag == RI)
            tag = qcinstance(aver, T0, TEN, 9999);
//		printf("ceil=%f,thres=%f\ncost=%d,sus=%f,wrong=%f\n",instceil[T0],instthre[T0],instcost[T0],instsusp[T0],instwrong[T0]);
//		printf("aver45d=%d,tag=%d\n",aver,tag);
		if(tag != MI)
			{
			ganqtemp = aver / 10.0 + 273.15;
			fzsj[TAG+18] = tag + '0';
			asciitemp(fzsj, aver, 95);
			if(tag==RI && aver>qwmax)
				{
				fzsj[TAG+19] = tag + '0';
				fzsj[TAG+20] = tag + '0';
				qwmax = aver;
				asciitemp(fzsj, aver, 100);
				fzsj[105] = hour / 10 + '0';
                fzsj[106] = hour % 10 + '0';
                fzsj[107] = min / 10 + '0';
                fzsj[108] = min % 10 + '0';
				max0_t0 = 0;
				}
            if(tag==RI && aver<qwmin)
                {
                fzsj[TAG+21] = tag + '0';
                fzsj[TAG+22] = tag + '0';
                qwmin = aver;
                asciitemp(fzsj, aver, 110);
                fzsj[115] = hour / 10 + '0';
               	fzsj[116] = hour % 10 + '0';
                fzsj[117] = min / 10 + '0';
                fzsj[118] = min % 10 + '0';
				min0_t0 = 0;
                }
			}
		else
			{
			fzsj[TAG+18] = tag + '0';
			tagnotRI(fzsj, 95);
            if(max0_t0 == 1)
                {
                fzsj[TAG+19] = tag + '0';
                fzsj[TAG+20] = tag + '0';
                tagnotRI(fzsj, 100);
                tagnotRI(fzsj, 105);
                }
            if(min0_t0 == 1)
                {
                fzsj[TAG+21] = tag + '0';
                fzsj[TAG+22] = tag + '0';
                tagnotRI(fzsj, 110);
                tagnotRI(fzsj, 115);
                }
			}

/*
            for(canjd=jd28f,jd28f=0;canjd>0&&canjd<30;canjd++)
		{
                sampsd[0][canjd] = sampsd[0][canjd-1];
		sampsd[1][canjd] = sampsd[1][canjd-1];
		asciibyte4(samprh, sampsd[0][canjd-1], 4+canjd*4);
		}
*/
        average(*sampsd, 30);							// U
        if(tag == RI)
            tag = qcinstance(aver, U, 1, 9999);
        if(tag != MI)
			{
			xdsd = aver;
			fzsj[TAG+23] = tag + '0';
            asciibyte4(fzsj, aver, 120);
            if(tag==RI && aver<sdmin)
               	{
                fzsj[TAG+24] = '0';
                fzsj[TAG+25] = '0';
                sdmin = aver;
                asciibyte4(fzsj, aver, 125);
                fzsj[130] = hour / 10 + '0';
                fzsj[131] = hour % 10 + '0';
                fzsj[132] = min / 10 + '0';
                fzsj[133] = min % 10 + '0';
				min0_u = 0;
                }
			}
		else
			{
			fzsj[TAG+23] = tag + '0';
			tagnotRI(fzsj, 120);
            if(min0_u == 1)
                {
                fzsj[TAG+24] = tag + '0';
                fzsj[TAG+25] = tag + '0';
                tagnotRI(fzsj, 125);
                tagnotRI(fzsj, 130);
                }
            }

//			double bhsqy1, bhsqy2, bhsqy3, bhsqy4; 	
//			printf("%f\n", ganqtemp);
//			bhsqy1 = SQYA*(1-SXDWD/ganqtemp);
//			bhsqy2 = log10(ganqtemp/SXDWD);
//			bhsqy3 = SQYC*(1-pow(10,SQYD*(ganqtemp/SXDWD-1)));
//			bhsqy4 = SQYE*(pow(10,SQYF*(1-SXDWD/ganqtemp))-1);
//			printf("%f,	%f, %f, %f\n", bhsqy1, bhsqy2, bhsqy3, bhsqy4);
		if(fzsj[TAG+18] == RI+'0')					// shui qi ya
			{
			bhsqy = pow(10,SQYA*(1-SXDWD/ganqtemp) + SQYB*log10(ganqtemp/SXDWD) + SQYC*(1-pow(10,SQYD*(ganqtemp/SXDWD-1))) + SQYE*(pow(10,SQYF*(1-SXDWD/ganqtemp))-1) + SQYG);
            if(fzsj[TAG+23] == RI+'0')
				{
				doublesuiqy = xdsd * bhsqy / 10;
				suiqy = (xdsd * bhsqy + 5) / 10;
				fzsj[TAG+26] = RI + '0';
				asciibyte4(fzsj, suiqy, 135);
				}
            else
               	{
               	fzsj[TAG+26] = MI + '0';
               	tagnotRI(fzsj, 135);
               	}
			}
		else
			{
			bhsqy = 0;
            fzsj[TAG+26] = MI + '0';
            tagnotRI(fzsj, 135);
            }
//		printf("bhsqy = %f\n",bhsqy);

		if(fzsj[TAG+26] == RI+'0')						// lu dian wen du
			{
			if(suiqy > 0)
				{
//				ludtemp = (LDWDB*log10(suiqy/10.0/BHSQY0)) / (LDWDA-log10(suiqy/10.0/BHSQY0)) * 100;
				ludtemp = (LDWDB*log10(doublesuiqy/10.0/BHSQY0)) / (LDWDA-log10(doublesuiqy/10.0/BHSQY0)) * 100;
				if(ludtemp >= 0)
					ludtemp = (ludtemp+5) / 10;
				else
					ludtemp = (ludtemp-5) / 10;
//				printf("suiqy=%d,ludtemp=%d\n",suiqy,ludtemp);
            	tag = qcinstance(ludtemp, TD, TEN, 9999);
				}
			else
				{
				tag = 0;
				ludtemp = 0;
				}
       		fzsj[TAG+27] = tag + '0';
			if(tag != MI)
				asciitemp(fzsj, ludtemp, 140);
			else
            	tagnotRI(fzsj, 140);
			}
		else
			{
            fzsj[TAG+27] = MI + '0';
            tagnotRI(fzsj, 140);
            }

		average(*sampqy, 30);												// P
        if(tag == RI)
            tag = qcinstance(aver, P, TEN, 65535);
		if(tag != MI)
			{
			fzsj[TAG+28] = tag + '0';
			asciibyte5(fzsj, aver, 145);
			if(tag==RI && aver>qiyamax)
				{
				fzsj[TAG+29] = tag + '0';
               	fzsj[TAG+30] = tag + '0';
				qiyamax = aver;
				asciibyte5(fzsj, qiyamax, 151);
                fzsj[157] = hour / 10 + '0';
                fzsj[158] = hour % 10 + '0';
                fzsj[159] = min / 10 + '0';
                fzsj[160] = min % 10 + '0';
				max0_p = 0;
				}
            if(tag==RI && aver<qiyamin)
               	{
               	fzsj[TAG+31] = tag + '0';
               	fzsj[TAG+32] = tag + '0';
               	qiyamin = aver;
				asciibyte5(fzsj, qiyamin, 162);
                fzsj[168] = hour / 10 + '0';
                fzsj[169] = hour % 10 + '0';
                fzsj[170] = min / 10 + '0';
                fzsj[171] = min % 10 + '0';
				min0_p = 0;
				}
			}
		else
			{
           	fzsj[TAG+28] = tag + '0';
			tagMIbyte5(fzsj, 145);
            if(max0_p == 1)
                {
                fzsj[TAG+29] = tag + '0';
                fzsj[TAG+30] = tag + '0';
                tagMIbyte5(fzsj, 151);
                tagnotRI(fzsj, 157);
                }
            if(min0_p == 1)
                {
                fzsj[TAG+31] = tag + '0';
                fzsj[TAG+32] = tag + '0';
                tagMIbyte5(fzsj, 162);
                tagnotRI(fzsj, 168);
                }
           	}

		visisame = 0;
		if(visisame < 5)
			{
			fzsj[TAG+33] = RI + '0';
			asciibyte5(fzsj, visi, 173);
			if(visi10mintag != MI)
				{
				fzsj[TAG+34] = visi10mintag + '0';
           		asciibyte5(fzsj, visi10min, 179);
				if(visi10mintag==RI && visi10min<njdmin)
            		{
            		njdmin = visi10min;
		            fzsj[TAG+35] = RI + '0';
        		    fzsj[TAG+36] = RI + '0';
		            asciibyte5(fzsj, visi10min, 185);
		            fzsj[191] = hour / 10 + '0';
		            fzsj[192] = hour % 10 + '0';
		            fzsj[193] = min / 10 + '0';
		            fzsj[194] = min % 10 + '0';
		            min0_vi = 0;
		            }
				}
			else
				{
				fzsj[TAG+34] = MI + '0';
            	tagMIbyte5(fzsj, 179);
				if(min0_vi == 1)
                	{
                	fzsj[TAG+35] = tag + '0';
                	fzsj[TAG+36] = tag + '0';
                	tagMIbyte5(fzsj, 185);
                	tagnotRI(fzsj, 191);
                	}
				}
			}
        else
            {
			fzsj[TAG+33] = MI + '0';
            tagMIbyte5(fzsj, 173);
			fzsj[TAG+34] = MI + '0';
            tagMIbyte5(fzsj, 179);
            if(min0_vi == 1)
                {
                fzsj[TAG+35] = tag + '0';
                fzsj[TAG+36] = tag + '0';
                tagMIbyte5(fzsj, 185);
                tagnotRI(fzsj, 191);
                }
            }

		fzsj[TAG+37] = RI + '0';
		fzsj[196] = tqxx_js/10 + '0';
        fzsj[197] = tqxx_js%10 + '0';
        fzsj[TAG+38] = RI + '0';
        fzsj[199] = tqxx_sc/10 + '0';
        fzsj[200] = tqxx_sc%10 + '0';
        fzsj[TAG+39] = RI + '0';
        fzsj[202] = tqxx_nj/10 + '0';
        fzsj[203] = tqxx_nj%10 + '0';
        fzsj[TAG+40] = RI + '0';
        fzsj[205] = tqxx_ld/10 + '0';
        fzsj[206] = tqxx_ld%10 + '0';
        fzsj[TAG+41] = RI + '0';
        fzsj[208] = tqxx_qt/10 + '0';
        fzsj[209] = tqxx_qt%10 + '0';
		
		fzsj[TAG+42] = '\r';
		fzsj[TAG+43] = '\n';
		fzsj[TAG+44] = '\0';

/*
// added by XGR at 2012-03-07 for hytest
        	if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
            	usleep(1);
        	else
            	{
            	read(fdmaste, senstate, sizeof(senstate));
            	close(fdmaste);
            	}

            if(senstate[WD]=='0')
                {
                gg[208] = 'N';
                tagNbyte4(gg, 4);
                gg[210] = 'N';
                tagNbyte4(gg, 12);

                hzgg[351] = 'N';
                tagNbyte4(hzgg, 4);
                hzgg[353] = 'N';
                tagNbyte4(hzgg, 12);

                gg[212] = 'N';
                tagNbyte4(gg, 20);
				hzgg[355] = 'N';
				tagNbyte4(hzgg, 20);
                hzgg[358] = 'N';
                tagNbyte4(hzgg, 32);
				hzgg[360] = 'N';
				tagNbyte4(hzgg, 40);
                }

            if(senstate[WS]=='0')
                {
                gg[209] = 'N';
                tagNbyte4(gg, 8);
                gg[211] = 'N';
                tagNbyte4(gg, 16);
                gg[212] = 'N';
                tagNbyte4(gg, 20);
                gg[213] = 'N';
                tagNbyte4(gg, 24);

                hzgg[352] = 'N';
				tagNbyte4(hzgg, 8);
                hzgg[354] = 'N';
				tagNbyte4(hzgg, 16);
				hzgg[355] = 'N';
				hzgg[356] = 'N';
				hzgg[357] = 'N';
                tagNbyte4(hzgg, 20);
                tagNbyte4(hzgg, 24);
                tagNbyte4(hzgg, 28);
                hzgg[358] = 'N';
                tagNbyte4(hzgg, 32);
                hzgg[359] = 'N';
				tagNbyte4(hzgg, 36);
                hzgg[360] = 'N';
                hzgg[361] = 'N';
                hzgg[362] = 'N';
                tagNbyte4(hzgg, 40);
                tagNbyte4(hzgg, 44);
                tagNbyte4(hzgg, 48);
                }

            if(senstate[WS1]=='0')
                {
                gg[214] = 'N';
                tagNbyte4(gg, 28);
                gg[215] = 'N';
                tagNbyte4(gg, 32);
                gg[216] = 'N';
                tagNbyte4(gg, 36);

                hzgg[363] = 'N';
                tagNbyte4(hzgg, 52);
                hzgg[364] = 'N';
                tagNbyte4(hzgg, 56);
                hzgg[365] = 'N';
                hzgg[366] = 'N';
                tagNbyte4(hzgg, 60);
                tagNbyte4(hzgg, 64);
                hzgg[367] = 'N';
                tagNbyte4(hzgg, 68);
                hzgg[368] = 'N';
                hzgg[369] = 'N';
                tagNbyte4(hzgg, 72);
                tagNbyte4(hzgg, 76);
				}

            if(senstate[RAT]=='0')
                {
                gg[217] = 'N';
                tagNbyte4(gg, 40);
                gg[218] = 'N';
                tagNbyte4(gg, 44);

                hzgg[370] = 'N';
                tagNbyte4(hzgg, 80);
				}

            if(senstate[RAT1]=='0')
                {
                gg[219] = 'N';
                tagNbyte4(gg, 48);
                gg[220] = 'N';
                tagNbyte4(gg, 52);

                hzgg[371] = 'N';
                tagNbyte4(hzgg, 84);
                }

            if(senstate[RAW]=='0')
                {
                gg[221] = 'N';
                tagNbyte4(gg, 56);
                gg[222] = 'N';
                tagNbyte4(gg, 60);

                hzgg[372] = 'N';
                tagNbyte4(hzgg, 88);
                }

            if(senstate[T0]=='0')
                {
                gg[223] = 'N';
                tagNbyte4(gg, 64);

                hzgg[373] = 'N';
                hzgg[374] = 'N';
                hzgg[375] = 'N';
                hzgg[376] = 'N';
                hzgg[377] = 'N';
                tagNbyte4(hzgg, 92);
                tagNbyte4(hzgg, 96);
                tagNbyte4(hzgg, 100);
                tagNbyte4(hzgg, 104);
                tagNbyte4(hzgg, 108);

                gg[229] = 'N';
				gg[230] = 'N';
                tagNbyte4(gg, 88);
                tagNbyte4(gg, 92);
                hzgg[389] = 'N';
                hzgg[390] = 'N';
                tagNbyte4(hzgg, 156);
                tagNbyte4(hzgg, 160);
                }

            if(senstate[SV1]=='0' && senstate[SV2]=='0' && senstate[SV3]=='0')
                {
                gg[224] = 'N';
                tagNbyte4(gg, 68);

                hzgg[383] = 'N';
                tagNbyte4(hzgg, 132);
                }

            if(senstate[T1]=='0' && senstate[T2]=='0' && senstate[T3]=='0')
                {
                gg[225] = 'N';
                tagNbyte4(gg, 72);

                hzgg[378] = 'N';
                hzgg[379] = 'N';
                hzgg[380] = 'N';
                hzgg[381] = 'N';
                hzgg[382] = 'N';
                tagNbyte4(hzgg, 112);
                tagNbyte4(hzgg, 116);
                tagNbyte4(hzgg, 120);
                tagNbyte4(hzgg, 124);
                tagNbyte4(hzgg, 128);
                }

            if(senstate[TW]=='0')
                {
                gg[226] = 'N';
                tagNbyte4(gg, 76);

                hzgg[384] = 'N';
                tagNbyte4(hzgg, 136);
                }
			
            if(senstate[U]=='0')
                {
                gg[227] = 'N';
                tagNbyte4(gg, 80);
                gg[228] = 'N';
                tagNbyte4(gg, 84);

                hzgg[385] = 'N';
                hzgg[386] = 'N';
                hzgg[387] = 'N';
                hzgg[388] = 'N';
                tagNbyte4(hzgg, 140);
                tagNbyte4(hzgg, 144);
                tagNbyte4(hzgg, 148);
                tagNbyte4(hzgg, 152);

                gg[229] = 'N';
                gg[230] = 'N';
                tagNbyte4(gg, 88);
                tagNbyte4(gg, 92);
                hzgg[389] = 'N';
                hzgg[390] = 'N';
                tagNbyte4(hzgg, 156);
                tagNbyte4(hzgg, 160);
                }

            if(senstate[P]=='0')
                {
                gg[231] = 'N';
                tagNbyte5(gg, 96);

                hzgg[391] = 'N';
                hzgg[392] = 'N';
                hzgg[393] = 'N';
                hzgg[394] = 'N';
                hzgg[395] = 'N';
                tagNbyte5(hzgg, 164);
                tagNbyte5(hzgg, 169);
                tagNbyte4(hzgg, 174);
                tagNbyte5(hzgg, 178);
                tagNbyte4(hzgg, 183);
				}

			if(senstate[TG]=='0')
                {
                gg[232] = 'N';
                tagNbyte4(gg, 101);

                hzgg[396] = 'N';
                hzgg[397] = 'N';
                hzgg[398] = 'N';
                hzgg[399] = 'N';
                hzgg[400] = 'N';
                tagNbyte4(hzgg, 187);
                tagNbyte4(hzgg, 191);
                tagNbyte4(hzgg, 195);
                tagNbyte4(hzgg, 199);
                tagNbyte4(hzgg, 203);
                }

            if(senstate[ST0]=='0')
                {
                gg[233] = 'N';
                tagNbyte4(gg, 105);

                hzgg[401] = 'N';
                hzgg[402] = 'N';
                hzgg[403] = 'N';
                hzgg[404] = 'N';
                hzgg[405] = 'N';
                tagNbyte4(hzgg, 207);
                tagNbyte4(hzgg, 211);
                tagNbyte4(hzgg, 215);
                tagNbyte4(hzgg, 219);
                tagNbyte4(hzgg, 223);
                }

            if(senstate[IR]=='0')
                {
                gg[234] = 'N';
                tagNbyte4(gg, 109);

                hzgg[406] = 'N';
                hzgg[407] = 'N';
                hzgg[408] = 'N';
                hzgg[409] = 'N';
                hzgg[410] = 'N';
                tagNbyte4(hzgg, 227);
                tagNbyte4(hzgg, 231);
                tagNbyte4(hzgg, 235);
                tagNbyte4(hzgg, 239);
                tagNbyte4(hzgg, 243);
                }

            if(senstate[ST1]=='0')
                {
                gg[235] = 'N';
                tagNbyte4(gg, 113);

                hzgg[411] = 'N';
                tagNbyte4(hzgg, 247);
                }

            if(senstate[ST2]=='0')
                {
                gg[236] = 'N';
                tagNbyte4(gg, 117);

                hzgg[412] = 'N';
                tagNbyte4(hzgg, 251);
                }

            if(senstate[ST3]=='0')
                {
                gg[237] = 'N';
                tagNbyte4(gg, 121);

                hzgg[413] = 'N';
                tagNbyte4(hzgg, 255);
                }

            if(senstate[ST4]=='0')
                {
                gg[238] = 'N';
                tagNbyte4(gg, 125);

                hzgg[414] = 'N';
                tagNbyte4(hzgg, 259);
                }

            if(senstate[ST5]=='0')
                {
                gg[239] = 'N';
                tagNbyte4(gg, 129);

                hzgg[415] = 'N';
                tagNbyte4(hzgg, 263);
                }

            if(senstate[ST6]=='0')
                {
                gg[240] = 'N';
                tagNbyte4(gg, 133);

                hzgg[416] = 'N';
                tagNbyte4(hzgg, 267);
                }

            if(senstate[ST7]=='0')
                {
                gg[241] = 'N';
                tagNbyte4(gg, 137);

                hzgg[417] = 'N';
                tagNbyte4(hzgg, 271);
                }

            if(senstate[ST8]=='0')
                {
                gg[242] = 'N';
                tagNbyte4(gg, 141);

                hzgg[418] = 'N';
                tagNbyte4(hzgg, 275);
                }

            if(senstate[LE]=='0')
                {
                gg[243] = 'N';
                tagNbyte4(gg, 145);
                gg[244] = 'N';
                tagNbyte4(gg, 149);

                hzgg[419] = 'N';
                tagNbyte4(hzgg, 279);
				hzgg[420] = 'N';
                tagNbyte4(hzgg, 283);
                }

            if(senstate[VI]=='0')
                {
                gg[245] = 'N';
                tagNbyte5(gg, 153);
				gg[246] = 'N';
                tagNbyte5(gg, 158);

                hzgg[421] = 'N';
                hzgg[422] = 'N';
                hzgg[423] = 'N';
                hzgg[424] = 'N';
                tagNbyte5(hzgg, 287);
                tagNbyte5(hzgg, 292);
                tagNbyte5(hzgg, 297);
                tagNbyte4(hzgg, 302);
                }

            if(senstate[CH]=='0')
                {
                gg[247] = 'N';
                tagNbyte5(gg, 163);

                hzgg[425] = 'N';
                tagNbyte5(hzgg, 306);
                }

            if(senstate[TCA]=='0')
                {
                gg[248] = 'N';
                tagNbyte4(gg, 168);

                hzgg[426] = 'N';
                tagNbyte4(hzgg, 311);
                }

            if(senstate[LCA]=='0')
                {
                gg[249] = 'N';
                tagNbyte4(gg, 172);

                hzgg[427] = 'N';
                tagNbyte4(hzgg, 315);
                }

            if(senstate[WW]=='0')
                {
                gg[250] = 'N';
                tagNbyte4(gg, 176);
                tagNbyte4(gg, 180);
                tagNbyte4(gg, 184);

                hzgg[428] = 'N';
                tagNbyte4(hzgg, 319);
                tagNbyte4(hzgg, 323);
                tagNbyte4(hzgg, 327);
                }

            if(senstate[SD]=='0')
                {
                gg[251] = 'N';
                tagNbyte4(gg, 188);

                hzgg[429] = 'N';
                tagNbyte4(hzgg, 331);
                }

            if(senstate[FR]=='0')
                {
                gg[252] = 'N';
                tagNbyte4(gg, 192);

                hzgg[430] = 'N';
                tagNbyte4(hzgg, 335);
                }

            if(senstate[WI]=='0')
                {
                gg[253] = 'N';
                tagNbyte4(gg, 196);

                hzgg[431] = 'N';
                tagNbyte4(hzgg, 339);
                }

            if(senstate[FSD]=='0')
                {
                gg[254] = 'N';
                tagNbyte4(gg, 200);

                hzgg[432] = 'N';
                tagNbyte4(hzgg, 343);
                }

            if(senstate[LNF]=='0')
                {
                gg[255] = 'N';
                tagNbyte4(gg, 204);

                hzgg[433] = 'N';
                tagNbyte4(hzgg, 347);
                }
// end
*/
			newtime = hour*60 + min;									// Added by XGR at 2014-03-19
			if(newtime != oldtime)
				{
            	fzsj[0] = hour / 10 + '0';
            	fzsj[1] = hour % 10 + '0';
            	fzsj[2] = min / 10 + '0';
            	fzsj[3] = min % 10 + '0';
            	if((fdfenzh=open("/usr/fzsj.tmp",O_WRONLY|O_CREAT)) < 1)
            		{
                	goto whileend;
                	}
            	write(fdfenzh, fzsj, CNT_FZSJ);
            	close(fdfenzh);
//				memset(fzsj, ' ', CNT_FZSJ);
				oldtime = newtime;
				}

// samp
            sampt[0] = hour / 10 + '0';
            sampt[1] = hour % 10 + '0';
            sampt[2] = min / 10 + '0';
            sampt[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampt0.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampt, sizeof(sampt));
            close(fdfenzh);
	    memset(sampt, '-', sizeof(sampt));

            samprh[0] = hour / 10 + '0';
            samprh[1] = hour % 10 + '0';
            samprh[2] = min / 10 + '0';
            samprh[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampuu.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, samprh, sizeof(samprh));
            close(fdfenzh);
            memset(samprh, '-', sizeof(samprh));

            samppp[0] = hour / 10 + '0';
            samppp[1] = hour % 10 + '0';
            samppp[2] = min / 10 + '0';
            samppp[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/samppp.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, samppp, sizeof(samppp));
            close(fdfenzh);

            sampwd[0] = hour / 10 + '0';
            sampwd[1] = hour % 10 + '0';
            sampwd[2] = min / 10 + '0';
            sampwd[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampwd.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampwd, sizeof(sampwd));
            close(fdfenzh);
            memset(sampwd, '-', sizeof(sampwd));

            sampws[0] = hour / 10 + '0';
            sampws[1] = hour % 10 + '0';
            sampws[2] = min / 10 + '0';
            sampws[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampws.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampws, sizeof(sampws));
            close(fdfenzh);
            memset(sampws, '-', sizeof(sampws));

			if((fdfenzh=open("/tmp/paramete/senstate.txt",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, senstate, sizeof(senstate));
            close(fdfenzh);
//			memset(senstate, '1', sizeof(senstate));


            argad.CRH_data=0x10;											// supply voltage
            argad.CRL_data=0x01;
            ioctl(fd_ad, IOCTL_CR_ZERO_CONFIG, &argad);
            ioctl(fd_ad, IOCTL_CR_FULL_CONFIG, &argad);

            arg.pin_idx = PIO_ADX_SEL0;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

            arg.pin_idx = PIO_ADX_SEL1;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

            arg.pin_idx = PIO_ADX_SEL2;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

            arg.pin_idx = PIO_ADX_SEL3;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

//          ioctl(fd_ad, IOCTL_MR_START, &argad);
//          usleep(200000);
//          read(fd_ad, data, 2);
	        usleep(20000);
            read(fd_ad, data, 2);
            zbtemp = ((data[1]*256 + data[0]) * 250 * 6 / 65535 +5) / 10 ;
//			printf("data=%x%x,zbtemp=%d\n",data[1],data[0],zbtemp);
			if(zbtemp > 130)				//Changed pss 13.5->13.0 bu XGR
				pss[0] = 'A';
			else
				pss[0] = 'D';
			pss[3] = zbtemp / 100 + '0';
			pss[4] = zbtemp / 10 % 10 + '0';
			pss[6] = zbtemp % 10 + '0';
            if((fdfenzh=open("/usr/pss.tmp",O_WRONLY|O_CREAT)) < 1)
            	{
               	goto whileend;
                }
            write(fdfenzh, pss, sizeof(pss));
            close(fdfenzh);

            arg.pin_idx = PIO_LVL_INP1;
            ioctl(fd_pio, IOCTL_PIO_GETSTA, &arg);
            door[0] = arg.pin_sta + '0';
//          printf("PIN %d, VAL = %d\n", arg.pin_idx, arg.pin_sta);
            if((fdfenzh=open("/usr/door.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, door, sizeof(door));
            close(fdfenzh);

/*
		    twoarrayinit(*sampqy, 2, 30);						//增加每分钟气压、温度、湿度、地温初始化
		    twoarrayinit(*samp45d, 2, 30);
		    twoarrayinit(*sampsd, 2, 30);
		    twoarrayinit(*samp061A001, 2, 30);
		    twoarrayinit(*samp061A002, 2, 30);
		    twoarrayinit(*samp061A011, 2, 30);
		    twoarrayinit(*samp061A012, 2, 30);
		    twoarrayinit(*samp061A013, 2, 30);
		    twoarrayinit(*samp061A014, 2, 30);
		    twoarrayinit(*samp061A021, 2, 30);
		    twoarrayinit(*samp061A022, 2, 30);
		    twoarrayinit(*samp061A023, 2, 30);
		    twoarrayinit(*samp061A024, 2, 30);
			pres = 20000;
			temp45d = 9999;
			humi = 9999;	
			visi = 0;
			visi10min = 0;

			memset(sampt, '/', sizeof(sampt));
			memset(samptg, '/', sizeof(samptg));
            memset(samps0, '/', sizeof(samps0));
            memset(samps1, '/', sizeof(samps1));
            memset(samps2, '/', sizeof(samps2));
            memset(samps3, '/', sizeof(samps3));
            memset(samps4, '/', sizeof(samps4));
            memset(samps5, '/', sizeof(samps5));
            memset(samps6, '/', sizeof(samps6));
            memset(samps7, '/', sizeof(samps7));
            memset(samps8, '/', sizeof(samps8));
            memset(samppp, '/', sizeof(samppp));
            memset(samprh, '/', sizeof(samprh));
            memset(sample, '/', sizeof(sample));
            memset(sampgr, '/', sizeof(sampgr));
            memset(sampdr, '/', sizeof(sampdr));
            memset(sampwd, '/', sizeof(sampwd));
            memset(sampws, '/', sizeof(sampws));
            memset(sampws1, '/', sizeof(sampws1));
*/
			mintag = 0;
			}	// if(sec == 0)

		if(min==0 && sec==1)
            {
//          twoarrayinit(*sampqy, 2, 30);                       //增加每分钟气压、温度、湿度、地温初始化
	        twoarrayinit(*samp45d, 2, 30);
	        twoarrayinit(*sampsd, 2, 30);
            pres = 20000;
            temp45d = 9999;
            humi = 9999;
            visi = 0;
            visi10min = 0;

			memset(fzsj, ' ', CNT_FZSJ);
			max0_w10min = 1;
			max0_t0 = 1;
			min0_t0 = 1;
			min0_u = 1;
			max0_p = 1;
			min0_p = 1;
			min0_vi = 1;
			}

//		if(min==0 && sec==0)
		if(min==0 && (sec==4 || sec==5))
			{
/*
			stringmove(hzgg, 4, hz, 4, BYTENUMHZ-4);
            hz[0] = mday / 10 + '0';
            hz[1] = mday % 10 + '0';
			hz[2] = hour / 10 + '0';
            hz[3] = hour % 10 + '0';
            if((fdxiaos=open("/usr/hourzz.tmp",O_WRONLY|O_CREAT)) < 1)
            	{
                goto whileend;
                }
            write(fdxiaos, hz, sizeof(hz));
            close(fdxiaos);
*/
zzpause:
			xsyl = 0;
			xscc = 0;

			fs1m3smax = -1;									// Added by XGR at 2013-01-15 13:00
        	fs1h3smax = -1;
			maxfs1hmin = 0;
			fs1h10mmax = -1;

			qwmax   = -9999;
			qwmin   =  9999;
			sdmin   =  100;
			qiyamax =  0;
			qiyamin =  20000;
			njdmin  =  64000;
			}	// if(min==0 && sec==0)

whileend:
//		printf("min:sec=%2d:%2d\n",min,sec);
		usleep(1);
//		printf("whileend\n");
		}	//while(1)

    ioctl(fd_tc1s, IOCTL_TC_STOP, 0);
    close(fd_can);
	close(fd_pio);
	close(fd_ad);
	close(fd_tc0);
	close(fd_fiqyl);
    close(fd_tc1s);
	close(fd_ttyS1);
    close(fd_ttyS2);
	return 1;
	}

