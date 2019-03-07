//weather 2012-07-03 16:00 V6.2 for 安徽八要素
//PTB210、PT100、EE180	//智能温湿度

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
//#define TRSFCJQ 1

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

#define A	0.00097392						//PT100
#define B	2.363857
#define C	-246.122453
#define PI	3.141592654
#define WDJJ	40
//#define A 0.001098                        //EE180
//#define B 2.333921
//#define C -244.691437
//#define WDJJ  0

#define E 8.49235
#define F 7.31276
#define G 2.38673
#define H 246.704		// 246.834(标准)	

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
/*
#define CCFo1   1050.9		//41506
#define CCA1    1.68095e-2
#define CCB1    9.20268e-6
#define CCFo2   1044.2		//45706
#define CCA2    1.65595e-2
#define CCB2    9.18278e-6
#define CCFo3   1050.9		//41606
#define CCA3    1.67272e-2
#define CCB3    9.16390e-6
*/
#define	P	0	
#define	T0	1
#define	T1	2
#define	T2	3
#define T3 	4
#define TW  5
#define	U	6
#define	TD	7
#define SV1 8
#define SV2 9
#define SV3 10
#define WD	11
#define WS	12
#define WS1	13
#define	RAT 14
#define RAT1 15
#define	RAW	16
#define TG	17
#define IR	18
#define ST0 19
#define ST1 20
#define ST2 21
#define ST3 22
#define ST4 23
#define ST5 24
#define ST6 25
#define ST7 26
#define ST8 27
#define LE	28
#define VI 	29
#define CH  30
#define TCA 31
#define LCA 32
#define WW  33
#define SD  34
#define FR  35
#define WI  36
#define FSD 37
#define LNF 38
#define GR	39
#define NR  40
#define DR	41
#define SR  42
#define RR  43
#define UR  44
#define UVA 45
#define UVB 46
#define AR  47
#define ART 48
#define TR  49
#define TRT 50
#define PR  51
#define SSD 52
#define SM1	53
#define SM2 54
#define SM3 55
#define SM4	56
#define SM5 57
#define SM6 58
#define SM7 59
#define SM8 60
#define WT  61
#define BA  62
#define OT  63
#define OS  64
#define OC  65
#define OH  66
#define OP  67
#define OD  68
#define OV  69
#define TL  70
#define OTU 71
#define OCC 72
#define WS2 73
#define WS3 74

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

#define ZFABEND	3
//#define ZZ090316 1

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

static float threshold[73]={400,-75,-75,-75,-75,-9999,0,-80,0,0,0,0,0,0,0,0,0,-90,-90,-90,-80,-70,-60,-50,-45,-40,-40,-40,0,0,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,0,-9999,0,0,0,0,0,0,-9999,-9999,-9999,-9999,-9999,0,0,0,0,0,0,0,0,0,0,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999};
static float ceiling[73]={1100,80,80,80,80,9999,100,50,9999,9999,9999,360,150,150,40,40,40,90,90,90,80,70,60,50,45,40,40,40,100,64000,9999,9999,9999,9999,9999,9999,9999,9999,9999,2000,9999,1400,1200,1200,250,50,200,9999,9999,9999,9999,9999,1,100,100,100,100,100,100,100,100,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};
static float suspirate[73]={0.3,2,2,2,2,9999,5,2,9999,9999,9999,360,20,20,9999,9999,9999,2,2,2,2,2,2,2,2,2,2,2,0.3,1000,9999,9999,9999,9999,9999,9999,9999,9999,9999,800,800,800,800,800,800,800,800,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};
static int cost[73] = {65535,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,65535,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999}; 

static float instthre[75]={400,-75,-75,-75,-75,-9999,0,-80,0,0,0,0,0,0,0,0,0,-90,-90,-90,-80,-70,-60,-50,-45,-40,-40,-40,0,0,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,0,-9999,0,0,0,0,0,0,-9999,-9999,-9999,-9999,-9999,0,0,0,0,0,0,0,0,0,0,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,-9999,0,0};
static float instceil[75]={1100,80,80,80,80,9999,100,50,9999,9999,9999,360,150,150,40,40,40,90,90,90,80,70,60,50,45,40,40,40,100,64000,9999,9999,9999,9999,9999,9999,9999,9999,9999,2000,9999,1400,1200,1200,250,50,200,9999,9999,9999,9999,9999,1,100,100,100,100,100,100,100,100,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,75,75};
static float instsusp[75]={0.5,3,3,3,3,9999,10,4,9999,9999,9999,360,10,10,9999,9999,9999,5,5,5,2, 1,1,0.5,0.5,0.3,0.3,0.3,0.3,1000,9999,9999,9999,9999,9999,9999,9999,9999,9999,800,800,800,800,800,800,30,90,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,10,10};
static float instwrong[75]={2,5,5,5,5,9999,15,5,9999,9999,9999,360,20,20,9999,9999,9999,10,10,10,5,5,3,2,1,0.5,0.5,0.5,1,10000,9999,9999,9999,9999,9999,9999,9999,9999,9999,1000,1000,1000,1000,1000,800,30,90,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,20,20};
static int instcost[75] = {65535,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,65535,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999,9999};

static int aver=0, tag=0;
static int fd_tc0=-1, fd_tc1=-1, fd_can=-1, fd_pio=-1, fd_ttyS0=-1, fd_ttyS1=-1, fd_ttyS2=-1;
static int fs3s[2][12], fs1m[2][60], fx1m[2][60], qhfs3s[2][3], fs2m120[2][120], fx2m120[2][120];
static int sec, min, hour, mday, mon, year, suffix, secary, minary;
static char beijintime[12], loctime[12];
static int samp031A004[2][30],samp031A011[2][30], samp031A012[2][30], samp031A013[2][30], samp031A014[2][30], samp031A021[2][60], samp031A034[2][30];
static int qhfs2m120[2][120];
static int samp051A001[2][30], samp051A024[2][30];
static int samp061A001[2][30], samp061A002[2][30], samp061A011[2][30], samp061A012[2][30], samp061A013[2][30], samp061A014[2][30], samp061A021[2][30], samp061A022[2][30], samp061A023[2][30], samp061A024[2][30];
static int samp091A001[2][6], samp091A002[2][6], samp091A003[2][6], samp091A004[2][6], samp091A011[2][6], samp091A012[2][6], samp091A013[2][6], samp091A014[2][6];
static int temp[2][3]={{0,0,0},{1,1,1}};
static int f0, f1, f2, gtjs;
static int fzyl=0, fzyltag=0, xsyl=0, xszf=0; 
static int fzdfdyl=0, fzdfdyltag=0, xsdfdyl=0;
static int fzcc=0, fzcctag=0, xscc=0, cclj;
static int sampfs1mmax=-1, sampfx1mmax=0;
static char qiya[14], njd[100];
static int visi, tjhsl;
static long pres;

static int sampslave;
static int dfdylmc=0, temp45d=0, humi=0;
static char sampt[126], sampt1[126], sampt2[126], sampt3[126], samprh[126], sampwd[246], sampws[966], sampws1[246], sampd[126], sampgr[126], sampdr[126];
static char sampsm1[126];
static float sensigr, sensidr;
static double cca1, cca2, cca3, ccb1, ccb2, ccb3, ccfo1, ccfo2, ccfo3;
static char senstate[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};
static int senst = 0;

static int fddz;
static char dztn[BYTEDZTN], dzstn[BYTEDZSTN], dzran[BYTEDZRAN], dzsensi[BYTEDZSENSI];
static int dzt0=0, dzt1=0, dzt2=0, dzt3=0;
static int dztg=0, dzst0=0, dzst1=0, dzst2=0, dzst3=0, dzst4=0, dzst5=0, dzst6=0, dzst7=0, dzst8=0;
static int dzrat=0, dzraw=0, dzu=0;
static int dzgr=0, dzdr=0;

static int fdstat=-1, fdsenco=-1;

static char cssj[22]={"CSSJ +0000+0000+0000\r\n"};

static int t1,t2,t3,t4,t5,t6,t7,t8,t9,t10;
static int wpxx1=0;

static int sampdlwd[2][30], sampdlsd[2][30];
static float wsbl0=0.2315, wsbl1=0.0495, ws1bl0=0.2315, ws1bl1=0.0495, ratbl=0.1, rat1bl=0.5;


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
    long int sampsum=0, sampcnt=0, sampaver=0;
    int j;
    for(j=0;j<n;j++)
        {
//  	if(samp[1][j] == RI)
		if(*(samp+n+j) == RI)
        	{
	    	sampsum += *(samp+j);
//      	sampsum += samp[0][j];
        	sampcnt++;
        	}
        }
//	printf("sum=%ld,cnt=%d\n", sampsum, sampcnt);
    if(sampcnt >= n*2/3)
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

static void midnum(int *p, int sort[])
    {
    if(*(p) >= *(p+1))
    	{
        if(*(p+1) >= *(p+2))
            {
            sort[0] = *(p);
            sort[1] = *(p+1);
            sort[2] = *(p+2);
            }
        else
            {
            if(*(p) >= *(p+2))
                {
                sort[0] = *(p);
                sort[1] = *(p+2);
                sort[2] = *(p+1);
                }
            else
                {
                sort[0] = *(p+2);
                sort[1] = *(p);
                sort[2] = *(p+1);
                }
            }
        }
    else
        {
        if(*(p) >= *(p+2))
            {
            sort[0] = *(p+1);
            sort[1] = *(p);
            sort[2] = *(p+2);
            }
        else
            {
            if(*(p+1) >= *(p+2))
                {
                sort[0] = *(p+1);
                sort[1] = *(p+2);
                sort[2] = *(p);
                }
            else
                {
                sort[0] = *(p+2);
                sort[1] = *(p+1);
                sort[2] = *(p);
                }
            }
        }
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
	    	*(string+i*n+j) = MI;
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

static int yearahargana()
    {
    int ahargana, year2000=0, leapyear=0;
    year2000 = year + 2000;
    leapyear = (year2000%4==0 && year2000%100!=0 || year2000%400==0);
    if(year2000)
        {
        switch(mon)
            {
            case 1:  ahargana = mday - 1;       break;
            case 2:  ahargana = 31 + mday - 1;  break;
            case 3:  ahargana = 60 + mday - 1;  break;
            case 4:  ahargana = 91 + mday - 1;  break;
            case 5:  ahargana = 121 + mday - 1; break;
            case 6:  ahargana = 152 + mday - 1; break;
            case 7:  ahargana = 182 + mday - 1; break;
            case 8:  ahargana = 213 + mday - 1; break;
            case 9:  ahargana = 244 + mday - 1; break;
            case 10: ahargana = 274 + mday - 1; break;
            case 11: ahargana = 305 + mday - 1; break;
            case 12: ahargana = 335 + mday - 1; break;
            default: ahargana = 0;
            }
        }
    else
        {
        switch(mon)
            {
            case 1:  ahargana = mday - 1;       break;
            case 2:  ahargana = 31 + mday - 1;  break;
            case 3:  ahargana = 59 + mday - 1;  break;
            case 4:  ahargana = 90 + mday - 1;  break;
            case 5:  ahargana = 120 + mday - 1; break;
            case 6:  ahargana = 151 + mday - 1; break;
            case 7:  ahargana = 181 + mday - 1; break;
            case 8:  ahargana = 212 + mday - 1; break;
            case 9:  ahargana = 243 + mday - 1; break;
            case 10: ahargana = 273 + mday - 1; break;
            case 11: ahargana = 304 + mday - 1; break;
            case 12: ahargana = 334 + mday - 1; break;
            default: ahargana = 0;
            }
        }
//  printf("ahargana=%d\n",ahargana);
    return(ahargana);
    }

static int mvtotjhsl(int mv)
	{
	int tjhsl;
	if(mv < 120)		tjhsl = WRONG;
	else if(mv < 210)	tjhsl = (mv - 120) / 18 + 0;
    else if(mv < 310)   tjhsl = (mv - 210) / 20 + 5;
    else if(mv < 415)   tjhsl = (mv - 310) / 21 + 10;
    else if(mv < 510)   tjhsl = (mv - 415) / 19 + 15;
    else if(mv < 610)   tjhsl = (mv - 510) / 20 + 20;
    else if(mv < 720)   tjhsl = (mv - 610) / 18 + 25;
    else if(mv < 825)   tjhsl = (mv - 720) / 21 + 30;
    else if(mv < 895)   tjhsl = (mv - 825) / 14 + 35;
    else if(mv < 955)   tjhsl = (mv - 895) / 12 + 40;
    else if(mv < 1005)  tjhsl = (mv - 955) / 10 + 45;
    else if(mv < 1015)  tjhsl = (mv - 1005) / 3 + 50;
    else if(mv < 1025)  tjhsl = (mv - 1015) / 2 + 55;
    else if(mv < 1035)  tjhsl = (mv - 1025) / 2 + 60;
    else if(mv < 1045)  tjhsl = (mv - 1035) / 2 + 65;
    else if(mv < 1055)  tjhsl = (mv - 1045) / 2 + 70;
    else if(mv < 1065)  tjhsl = (mv - 1055) / 2 + 75;
    else if(mv < 1070)  tjhsl = (mv - 1065) / 1 + 80;
    else if(mv < 1080)  tjhsl = (mv - 1070) / 2 + 85;
    else if(mv < 1095)  tjhsl = (mv - 1080) / 3 + 90;
    else if(mv < 1120)  tjhsl = (mv - 1095) / 5 + 95;
	else				tjhsl = WRONG;
	return(tjhsl);
	}

/*
static void extremumtime_hm(char *string, int n, int hour, int min)
	{
	int hmtotal, lasthour, lastmin;
	hmtotal = hour * 60 + min;
	if(hmtotal == 0)
		hmtotal = 1440;
	lasthour = (hmtotal-1) / 60;
	lastmin  = (hmtotal-1) % 60;
	string[n]   = lasthour / 10 + '0';
    string[n+1] = lasthour % 10 + '0';
    string[n+2] = lastmin  / 10 + '0';
    string[n+3] = lastmin  % 10 + '0';
	}
*/

/*
static void arrayinit()
    {
    twoarrayinit(samp45d, 2, 30);
    twoarrayinit(sampsd, 2, 30);
    twoarrayinit(sampzfs, 2, 30);
    twoarrayinit(sampzf, 2, 6);
    twoarrayinit(sampfjwd, 2, 30);

    twoarrayinit(fx1m, 2, 60);
    twoarrayinit(fx2m, 2, 2);
    twoarrayinit(fx10m, 2, 10);
    twoarrayinit(fs2m, 2, 2);
    twoarrayinit(fs10m, 2, 10);
    }
*/

static void* njdpthreadxlht(void *t)
    {
    int a, i;
    char c;

    while(1)
        {
        read(fd_ttyS1, &c, 1);
//		getchar(c);
//        if(c == 'P')
	    	{
	    	for(i=0;i<100 && c!='m';i++)
	    		{
	    		read(fd_ttyS1, &c, 1);
	    		njd[i] = c;
	    		}
	    	njd[i] = '\0';
//			printf("njd=%s\n",njd);
            for(;i>0 && njd[i]!='.';i--)
	        	usleep(1);
	    	i--; 	
            for(a=1,visi=0; i>0 && njd[i]!=' ' && njd[i]!=','; i--)
	    		{
	    		visi =  visi + (njd[i]-'0') * a;
	    		a = 10 * a;
	    		}
//	    	printf("visi=%d\n",visi);
	    	}
//        sleep(1);
        }
    }

static void* njdpthreadzhzh(void *t)
    {
    int a, i;
    char c, njdvalue[5];

    while(1)
        {
        read(fd_ttyS1, &c, 1);
        if(c == 'P')
            {
            for(i=0;i<50 && c!=0x0A;i++)
                {
                read(fd_ttyS1, &c, 1);
                njd[i] = c;
                }
            njd[i] = '\0';
//			printf("njd=%s\n",njd);
			stringmove(njd, 9, njdvalue, 0, sizeof(njdvalue));
			visi = atof(njdvalue);
//			printf("visi=%d\n",visi);
            }
//        sleep(1);
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
		usleep(800000);
//		fflush(stdout);
        }
    }

static void* tc0fs(void *t)
//static void tc0fs()
    {
    int ret, fsdata[4]={0};
    int fs1=0, fs2=0, fs3=0, intfs3s=0;
    int tc0tag=MI;
//    int fs1m3smax=0, maxfs1msec=0;
//    int hd=0;		//hua dong ping jun zhi xia biao
    int gray, winddir;
    int graycode[7]={0};
    int i, n=0;
	int wdset;

    ioctl(fd_tc0, IOCTL_TC_START, 4);

    while(1)
	{
//	n++;
//	if(n >= 4)
//		n = 0;
//	if(n == 0)
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
        case 127: winddir = 238; break;
        default: winddir = -1;
        }

        fx1m[0][secary] = winddir;
        fx1m[1][secary] = qclimit(winddir, ceiling[WD], threshold[WD]);
//		printf("winddirection=%d,tag=%d\n",winddir,tag);
//		printf("pthread:%d,%d,%d\n",secary,fx1m[0][secary],fx1m[1][secary]);
        asciibyte4(sampwd, winddir, 4+secary*4);
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
//			printf("ret=%d,tag=%d\n",fs3s[0][n+8],fs3s[1][n+8]);
			asciibyte4(sampws, ret, 4+secary*16+n*4);
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
	char statclim[37]={"STATCLIM 0 120 0 +000 0 0 - - - - -\r\n"};
    char statradi[37]={"STATRADI 0 120 0 +000 0 0 - - - - -\r\n"};
    char stateath[37]={"STATEATH 0 120 0 +000 0 0 - - - - -\r\n"};
	char statintl[39]={"STATINTL_1 0 120 0 +000 0 0 - - - - -\r\n"};
	char statccsw[4];

    for(;;)
		{
        read(fd_can, &data, sizeof(CanData));

//      printf("Receive CAN package:\n");
//      printf("ID  = %x\n", data.id);
//      printf("FF  = %d\n", data.IsExt);
//     printf("RTR = %d\n", data.rxRTR);
//      printf("DAT = ");
//      for(i=0;i<data.dlc;i++)
//          printf("%x ", data.data[i]);
//      printf("\n\n");

		switch(data.id)
			{
			case 0x183:
            	candata = data.data[1]*256 + data.data[0];
                if(candata > 32767)
                	candata = candata - 65536;
				candata += dzt1; 
//				printf("wendu1=%x%x,%d\n",data.data[1],data.data[0],candata);

				if(candata >=0)														// CSSJ
					{
					cssj[5] = '+';
                	cssj[6] = candata / 1000 + '0';
                	cssj[7] = (candata - candata/1000*1000) / 100 + '0';
                	cssj[8] = (candata - candata/100*100) / 10 + '0';
                	cssj[9] = candata % 10 + '0';
					}
				else
					{
					cssj[5] = '-';
					cssj[6] = (0-candata) / 1000 + '0';
					cssj[7] = ((0-candata) - (0-candata)/1000*1000) / 100 + '0';
					cssj[8] = ((0-candata) - (0-candata)/100*100) / 10 + '0';
					cssj[9] = (0-candata) % 10 + '0';
					}																// CSSJ

                temp[0][0] = candata;
                temp[1][0] = qcsamp(candata, T1, HUNDRED, 9999);
				if(candata >= 0)
					sampslave = (candata + 5) / 10;
				else
					sampslave = (candata - 5) / 10;
				asciitemp(sampt1, sampslave, 4+suffix*4);

                candata = data.data[3]*256 + data.data[2] + dzt2;
                if(candata > 32767)
                	candata = candata - 65536;
				candata += dzt2;
//				printf("wendu2=%x%x,%d\n",data.data[3],data.data[2],candata);
 
               if(candata >=0)														// CSSJ
                    {
                    cssj[10] = '+';
                    cssj[11] = candata / 1000 + '0';
                    cssj[12] = (candata - candata/1000*1000) / 100 + '0';
                    cssj[13] = (candata - candata/100*100) / 10 + '0';
                    cssj[14] = candata % 10 + '0';
                    }
                else
                    {
                    cssj[10] = '-';
                    cssj[11] = (0-candata) / 1000 + '0';
                    cssj[12] = ((0-candata) - (0-candata)/1000*1000) / 100 + '0';
                    cssj[13] = ((0-candata) - (0-candata)/100*100) / 10 + '0';
                    cssj[14] = (0-candata) % 10 + '0';
                    }																// CSSJ

                temp[0][1] = candata;
                temp[1][1] = qcsamp(candata, T2, HUNDRED, 9999);
                if(candata >= 0)
                    sampslave = (candata + 5) / 10;
                else
                    sampslave = (candata - 5) / 10;
                asciitemp(sampt2, sampslave, 4+suffix*4);

                candata = data.data[5]*256 + data.data[4] + dzt3;	
                if(candata > 32767)
                    candata = candata - 65536;
				candata += dzt3;
//				printf("wendu3=%x%x,%d\n",data.data[5],data.data[4],candata);

                if(candata >=0)														// CSSJ
                    {
                    cssj[15] = '+';
                    cssj[16] = candata / 1000 + '0';
                    cssj[17] = (candata - candata/1000*1000) / 100 + '0';
                    cssj[18] = (candata - candata/100*100) / 10 + '0';
                    cssj[19] = candata % 10 + '0';
                    }
                else
                    {
                    cssj[15] = '-';
                    cssj[16] = (0-candata) / 1000 + '0';
                    cssj[17] = ((0-candata) - (0-candata)/1000*1000) / 100 + '0';
                    cssj[18] = ((0-candata) - (0-candata)/100*100) / 10 + '0';
                    cssj[19] = (0-candata) % 10 + '0';
                    }																// CSSJ

                temp[0][2] = candata;
                temp[1][2] = qcsamp(candata, T3, HUNDRED, 9999);
                if(candata >= 0)
                    sampslave = (candata + 5) / 10;
                else
                    sampslave = (candata - 5) / 10;
                asciitemp(sampt3, sampslave, 4+suffix*4);

                candata = data.data[7]*256 + data.data[6];
                if(candata > 32767)
                    candata = candata - 65536;
//				printf("IR=%d\n",candata);
                samp031A004[0][suffix] = candata;
                samp031A004[1][suffix] = qcsamp(candata, IR, TEN, 9999);
			break;

			case 0x283:
				fzcc = 0;
				if(ccshijian == min)
					break;
//				printf("gt01=%f,gt02=%f,gt03=%f\n",gt01,gt02,gt03);
				ccxuan = data.data[1]*256 + data.data[0];
				cc1 = ccxuan;
				if(ccxuan == 0)
					gt1 = 0;
				else
//					gtjs1 = CCA1*(ccxuan-CCFo1) + CCB1*(ccxuan-CCFo1)*(ccxuan-CCFo1);
            		gt1 = cca1*(ccxuan/10.0-ccfo1) + ccb1*(ccxuan/10.0-ccfo1)*(ccxuan/10.0-ccfo1);
				if(gt01 == 0)
					{
					gt01 = gt1;
					cclj = 0;
					}
				gtjs1 = gt1 - gt01;
				if(gtjs1 < 0)
					gtjs1 = 0;
//          	printf("ccxuan1=%d, gt01=%f, gt1=%f\n, gtjs1=%f\n",ccxuan,gt01,gt1,gtjs1);

            	ccxuan = data.data[3]*256 + data.data[2];
				cc2 = ccxuan;
                if(ccxuan == 0)
                    gt2 = 0;
                else
//					gtjs2 = CCA2*(ccxuan-CCFo2) + CCB2*(ccxuan-CCFo2)*(ccxuan-CCFo2);
            		gt2 = cca2*(ccxuan/10.0-ccfo2) + ccb2*(ccxuan/10.0-ccfo2)*(ccxuan/10.0-ccfo2);
				if(gt02 == 0)
					gt02 = gt2;
            	gtjs2 = gt2 - gt02;
            	if(gtjs2 < 0)
                	gtjs2 = 0;
//          	printf("ccxuan2=%d, gt02=%f, gt2=%f\n, gtjs2=%f\n",ccxuan,gt02,gt2,gtjs2);

            	ccxuan = data.data[5]*256 + data.data[4];
				cc3 = ccxuan;
                if(ccxuan == 0)
                    gt3 = 0;
                else
//					gtjs3 = CCA3*(ccxuan-CCFo3) + CCB3*(ccxuan-CCFo3)*(ccxuan-CCFo3);
					gt3 = cca3*(ccxuan/10.0-ccfo3) + ccb3*(ccxuan/10.0-ccfo3)*(ccxuan/10.0-ccfo3);
            	if(gt03 == 0)
                	gt03 = gt3;
            	gtjs3 = gt3 - gt03;
            	if(gtjs3 < 0)
                	gtjs3 = 0;
//          	printf("ccxuan3=%d, gt03=%f, gt3=%f\n, gtjs3=%f\n",ccxuan,gt03,gt3,gtjs3);

				if(abs(gt1-(gt2+gt3)/2) > 3)
					fzcc = ((gtjs2 + gtjs3) * 100 + 1) / 2 - cclj;
				else if(abs(gt2-(gt3+gt1)/2) > 3)
                    fzcc = ((gtjs3 + gtjs1) * 100 + 1) / 2 - cclj;
                else if(abs(gt3-(gt1+gt2)/2) > 3)
                    fzcc = ((gtjs1 + gtjs2) * 100 + 1) / 2 - cclj;
				else
	        		fzcc = ((gtjs1 + gtjs2 + gtjs3) * 200 + 3) / 6 - cclj;
				if(fzcc<0 || fzcc>1000)
					fzcc = 0;
       			fzcctag = qclimit(fzcc, ceiling[RAW]*TEN, threshold[RAW]*TEN);
				if(fzcctag == RI)
					{
        			xscc += fzcc;
					cclj += fzcc;
					}
/*
				if(cclj >= cclj0)
					{
					tempfzcc = fzcc;
					fzcc = fzcc0;
					fzcc0 = tempfzcc;
                    tempxscc = xscc;
                    xscc = xscc0;
                    xscc0 = tempxscc;
					cclj0 = cclj;
					}
				else
					{
					fzcc = 0;
					fzcc0 = fzcc;
					xscc0 = xscc;
					cclj0 = cclj;
					}
*/
//				printf("fzcc=%d,fzcctag=%d,xscc=%d\n",fzcc,fzcctag,xscc);
				if(cc1==0 && cc2==0 && cc3==0)										// CC Restart
					{
                    gt01 = 0;
                    gt02 = 0;
					gt03 = 0;
					cclj = 0;
					}
//				printf("cclj=%d\n",cclj);
				ccshijian = min;
				statccsw[0] = cclj/1000 + '0';
                statccsw[1] = cclj/100%10 + '0';
                statccsw[2] = cclj/10%10 + '0';
                statccsw[3] = cclj%10 + '0'; 
	            if((fdstat = open("/usr/statccsw.tmp",O_WRONLY|O_CREAT)) < 1)
    	            usleep(1);
        	    else
            	    {
                	write(fdstat, statccsw, sizeof(statccsw));
                	close(fdstat);
                	}


				dfdylmc = data.data[7]*256 + data.data[6];							// 大翻斗雨量脉冲数
			break;

			case 0x383:
				candata = data.data[1]*256 + data.data[0];
//				printf("data1=%d,data0=%d,", data.data[1],data.data[0]);
//				printf("qhfs=%d\n",candata);
				if(candata <= 0)
					candata = 0;
				else
					candata = (candata*ws1bl1*100 + ws1bl0*100 + 5) / 10;
	            samp031A021[0][secary] = candata;
                samp031A021[1][secary] = qcsamp(candata, WS1, TEN, 9999);
				qhfs2m120[0][secary+(minary%2)*60] = candata;
                qhfs2m120[1][secary+(minary%2)*60] = samp031A021[1][secary];
				qhfs3s[0][0] = qhfs3s[0][1];
                qhfs3s[0][1] = qhfs3s[0][2];
                qhfs3s[0][2] = candata;
                qhfs3s[1][0] = qhfs3s[1][1];
                qhfs3s[1][1] = qhfs3s[1][2];
                qhfs3s[1][2] = samp031A021[1][secary];

                asciibyte4(sampws1, candata, 4+secary*4);
                sampws1[244] = '\r';
                sampws1[245] = '\n';
//				printf("candata=%d\n",candata);
//    			qhfs1m[0][secary] = candata;
//    			qhfs1m[1][secary] = samp031A021[1][secary];
//				printf("qhfs1=%d, qhfs2=%d, qhfs3=%d, \n", qhfs3s[0][0], qhfs3s[0][1], qhfs3s[0][2]);
//              printf("qhfs1=%d, qhfs2=%d, qhfs3=%d, \n", qhfs3s[1][0], qhfs3s[1][1], qhfs3s[1][2]);
				break;

			case 0x483:
				candata = data.data[1]*256 + data.data[0];
				f0 = candata;
                candata = data.data[3]*256 + data.data[2];
                f1 = candata;
                candata = data.data[5]*256 + data.data[4];
                f2 = candata;

/*
                candata = data.data[7]*256 + data.data[6] + dzst0;							// dzst0
//				printf("sec=%d,st0=%d\n",sec,candata);
                if(candata > 32767)
                	candata = candata - 65536;
                samp031A034[0][suffix] = candata;
                samp031A034[1][suffix] = qcsamp(candata, ST0, TEN, 9999);
                asciitemp(sampd, candata, 4+suffix*4);
                sampd[124] = '\r';
                sampd[125] = '\n';
*/
			break;

			case 0x185:
				if(data.data[1] > 128)
					candata = 0;	//65536 - (data.data[1]*256 + data.data[0]);
				else
//					candata = (data.data[1]*256 + data.data[0]) / sensigr + dzgr;			// dzgr
					candata = (data.data[1]*256 + data.data[0]) / sensigr;
//					candata = data.data[1]*256 + data.data[0];
				printf("gr=%d\n",candata);
				printf("data1=%2x,data0=%2x\n", data.data[1],data.data[0]);
//				printf("zfsgr=%d, sensigr=%f\n",candata,sensigr);
                samp051A001[0][suffix] = candata;
                samp051A001[1][suffix] = qcsamp(candata, GR, 1, 9999);
				asciibyte4(sampgr, candata, 4+suffix*4);
                sampgr[124] = '\r';
                sampgr[125] = '\n';
			break;

        	case 0x385:
            	if(data.data[7] > 128)
                	candata = 0;	//65536 - (data.data[7]*256 + data.data[6]);
            	else
//                	candata = (data.data[7]*256 + data.data[6]) / sensidr + dzdr;			// dzdr
					candata = (data.data[7]*256 + data.data[6]) / sensidr;
//                    candata = data.data[7]*256 + data.data[6];
//                printf("dr=%d\n",candata);
//              printf("data7=%2x,data6=%2x,", data.data[7],data.data[6]);
//              printf("zfsdr=%d, sensidr=%f\n",candata,sensidr);
                samp051A024[0][suffix] = candata;
                samp051A024[1][suffix] = qcsamp(candata, DR, 1, 9999);
				asciibyte4(sampdr, candata, 4+suffix*4);
                sampdr[124] = '\r';
                sampdr[125] = '\n';
			break;

			case 0x186:
				candata = (data.data[1]*256 + data.data[0]) + dztg;							// dztg
				if(candata > 32767)
					candata = candata - 65536;
		        samp061A001[0][suffix] = candata;
                samp061A001[1][suffix] = qcsamp(candata, TG, TEN, 9999);
//				printf("tg=%d,tag=%d\n",candata,samp061A001[1][suffix]);
//                asciitemp(sampsm1, candata, 4+suffix*4);
//                sampsm1[124] = '\r';
//                sampsm1[125] = '\n';

				candata = (data.data[3]*256 + data.data[2]) + dzst0;							// dzst0
//				printf("ST0=%2x%2x,candata=%d\n",data.data[3],data.data[2],candata);
                if(candata > 32767)
                	candata = candata - 65536;
                samp061A002[0][suffix] = candata;
                samp061A002[1][suffix] = qcsamp(candata, ST0, TEN, 9999);
                asciitemp(sampd, candata, 4+suffix*4);
                sampd[124] = '\r';
                sampd[125] = '\n';
//				if(suffix==29)
//					{
//					sampd[126] = '\0';
//					printf("sampd=%s",sampd);
//					}
			break;

            case 0x286:
                candata = (data.data[1]*256 + data.data[0]) + dzst1;                          // dzst1
//				printf("ST1=%2x%2x,candata=%d\n",data.data[1],data.data[0],candata);
            	if(candata > 32767)
                	candata = candata - 65536;
                samp061A011[0][suffix] = candata;
                samp061A011[1][suffix] = qcsamp(candata, ST1, TEN, 9999);
//                asciitemp(sampsm1, candata, 4+suffix*4);
//                sampsm1[124] = '\r';
//                sampsm1[125] = '\n';

                candata = (data.data[3]*256 + data.data[2]) + dzst2;                          // dzst2
//                printf("0x=%2x%2x,10cm=%d,dzst2=%d\n",data.data[3],data.data[2],candata,dzst2);
                if(candata > 32767)
                	candata = candata - 65536;
                samp061A012[0][suffix] = candata;
                samp061A012[1][suffix] = qcsamp(candata, ST2, TEN, 9999);

                candata = (data.data[5]*256 + data.data[4]) + dzst3;                          // dzst3
//				printf("0x=%2x%2x,15cm=%d,dzst3=%d\n",data.data[5],data.data[4],candata,dzst3);
                if(candata > 32767)
                    candata = candata - 65536;
                samp061A013[0][suffix] = candata;
                samp061A013[1][suffix] = qcsamp(candata, ST3, TEN, 9999);

                candata = (data.data[7]*256 + data.data[6]) + dzst4;                          // dzst4
//                printf("0x=%2x%2x,20cm=%d,dzst4=%d\n",data.data[7],data.data[6],candata,dzst4);
                if(candata > 32767)
                    candata = candata - 65536;
                samp061A014[0][suffix] = candata;
                samp061A014[1][suffix] = qcsamp(candata, ST4, TEN, 9999);
            break;                                                                                                                                               
            case 0x386:
                candata = (data.data[1]*256 + data.data[0]) + dzst5;                          // dzst5
                if(candata > 32767)
                    candata = candata - 65536;
                samp061A021[0][suffix] = candata;
                samp061A021[1][suffix] = qcsamp(candata, ST5, TEN, 9999);
//				printf("dataH=%2x,dataL=%2x,st5=%d\n", data.data[1],data.data[0],candata);

                candata = (data.data[3]*256 + data.data[2]) + dzst6;                          // dzst6
                if(candata > 32767)
                    candata = candata - 65536;
                samp061A022[0][suffix] = candata;
                samp061A022[1][suffix] = qcsamp(candata, ST6, TEN, 9999);
//				printf("dataH=%2x,dataL=%2x,st6=%d\n", data.data[3],data.data[2],candata);

                candata = (data.data[5]*256 + data.data[4]) + dzst7;                          // dzst7
                if(candata > 32767)
                    candata = candata - 65536;
                samp061A023[0][suffix] = candata;
                samp061A023[1][suffix] = qcsamp(candata, ST7, TEN, 9999);
//				printf("dataH=%2x,dataL=%2x,st7=%d\n", data.data[5],data.data[4],candata);
                        
				candata = (data.data[7]*256 + data.data[6]) + dzst8;                          // dzst8
                if(candata > 32767)
                    candata = candata - 65536;
//	            printf("dataH=%2x,dataL=%2x,st8=%d\n", data.data[7],data.data[6],candata);
                samp061A024[0][suffix] = candata;
                samp061A024[1][suffix] = qcsamp(candata, ST8, TEN, 9999);
//				printf("ST8=%d,tag=%d\n",candata,samp061A024[1][suffix]);
//                asciitemp(sampsm1, candata, 4+suffix*4);
//                sampsm1[124] = '\r';
//                sampsm1[125] = '\n';
			break;
/*
			case 0x18f:
				temp45d = data.data[1]*256 + data.data[0];
                if(temp45d > 32767)
                    temp45d -= 65536;
                temp45d += dzt0;
//	            printf("wendu0=%2x%2x,%d\n",data.data[1],data.data[0],temp45d);
			break;

            case 0x28f:
				humi = data.data[5]*256 + data.data[4];
				humi += dzu;
				if(humi > 99)
					humi = 99;
				if(humi < 0)
					humi = 0;
//				printf("humi=%2x%2x,%d\n",data.data[5],data.data[4],humi);
            break;
*/
#ifdef TRSFCJQ
			case 0x189:
				candata = data.data[1]*256 + data.data[0];
				tjhsl = mvtotjhsl(candata);
//				printf("candata1=%x%x,tjhsl1=%d\n", data.data[1],data.data[0],tjhsl);
				if(tjhsl == WRONG)
					{
					samp091A001[0][secary/10] = 0;
					samp091A001[1][secary/10] = MI;
					}
				else
					{
                    samp091A001[0][secary/10] = tjhsl;
                    samp091A001[1][secary/10] = qclimit(tjhsl, ceiling[SM1], threshold[SM1]);;
                    }

                candata = data.data[3]*256 + data.data[2];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A002[0][secary/10] = 0;
                    samp091A002[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A002[0][secary/10] = tjhsl;
                    samp091A002[1][secary/10] = qclimit(tjhsl, ceiling[SM2], threshold[SM2]);;
                    }

                candata = data.data[5]*256 + data.data[4];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A003[0][secary/10] = 0;
                    samp091A003[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A003[0][secary/10] = tjhsl;
                    samp091A003[1][secary/10] = qclimit(tjhsl, ceiling[SM3], threshold[SM3]);;
                    }

                candata = data.data[7]*256 + data.data[6];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A004[0][secary/10] = 0;
                    samp091A004[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A004[0][secary/10] = tjhsl;
                    samp091A004[1][secary/10] = qclimit(tjhsl, ceiling[SM4], threshold[SM4]);;
                    }
				break;

			case 0x289:
                candata = data.data[1]*256 + data.data[0];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A011[0][secary/10] = 0;
                    samp091A011[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A011[0][secary/10] = tjhsl;
                    samp091A011[1][secary/10] = qclimit(tjhsl, ceiling[SM5], threshold[SM5]);;
                    }

                candata = data.data[3]*256 + data.data[2];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A012[0][secary/10] = 0;
                    samp091A012[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A012[0][secary/10] = tjhsl;
                    samp091A012[1][secary/10] = qclimit(tjhsl, ceiling[SM6], threshold[SM6]);;
                    }

                candata = data.data[5]*256 + data.data[4];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A013[0][secary/10] = 0;
                    samp091A013[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A013[0][secary/10] = tjhsl;
                    samp091A013[1][secary/10] = qclimit(tjhsl, ceiling[SM7], threshold[SM7]);;
                    }

                candata = data.data[7]*256 + data.data[6];
                tjhsl = mvtotjhsl(candata);
                if(tjhsl == WRONG)
                    {
                    samp091A004[0][secary/10] = 0;
                    samp091A004[1][secary/10] = MI;
                    }
                else
                    {
                    samp091A004[0][secary/10] = tjhsl;
                    samp091A004[1][secary/10] = qclimit(tjhsl, ceiling[SM8], threshold[SM8]);;
                    }
				break;
#endif

			case 0x583:
				switch(data.data[1])
					{
					case 5:
						voltage = data.data[4];
                        if((fbcandata=open("/usr/statclim.tmp",O_RDWR)) < 1)
                            {
                            if((fbcandata=open("/usr/statclim.tmp",O_WRONLY|O_CREAT)) < 1)
                                break;
                            }
                        else
                            read(fbcandata, statclim, sizeof(statclim));
                        statclim[11] = voltage / 100 + '0';
                        statclim[12] = voltage / 10 % 10 + '0';
                        statclim[13] = voltage % 10 + '0';
                        write(fbcandata, statclim, sizeof(statclim));
                        close(fbcandata);
					case 0:
						if(data.data[3] == 1)
							{
							senstate[2] = data.data[4] + '0';
							senstate[3] = data.data[5] + '0';
							senstate[4] = data.data[6] + '0';
							senstate[17] = data.data[7] + '0';
							}
						else if(data.data[3] == 2)
                    		{
                    		senstate[15] = data.data[4] + '0';
                    		}
                		else if(data.data[3] == 3)
                    		{
                    		senstate[13] = data.data[4] + '0';
                    		}
                		else if(data.data[3] == 4)
                    		{
                    		senstate[8] = data.data[4] + '0';
                    		senstate[9] = data.data[5] + '0';
                    		senstate[10] = data.data[6] + '0';
                    		senstate[18] = data.data[7] + '0';
                    		}
						else
							usleep(1);
					default:
						usleep(1);
					}
				break;

            case 0x585:
                switch(data.data[1])
                    {
                    case 5:
                        voltage = data.data[4];
//                        printf("voltage=%d\n",data.data[4]);
            			if((fbcandata=open("/usr/statradi.tmp",O_RDWR)) < 1)
                			{
            				if((fbcandata=open("/usr/statradi.tmp",O_WRONLY|O_CREAT)) < 1)
								break;
							}
						else
							read(fbcandata, statradi, sizeof(statradi));
						statradi[11] = voltage / 100 + '0';
                        statradi[12] = voltage / 10 % 10 + '0';
                        statradi[13] = voltage % 10 + '0';
                        write(fbcandata, statradi, sizeof(statradi));
                        close(fbcandata);
                    case 0:
                		if(data.data[3] == 1)
                    		{
                    		senstate[38] = data.data[4] + '0';
                    		senstate[39] = data.data[5] + '0';
                    		senstate[40] = data.data[6] + '0';
                    		senstate[42] = data.data[7] + '0';
                    		}
                		else if(data.data[3] == 2)
                    		{
                    		senstate[44] = data.data[4] + '0';
                    		senstate[45] = data.data[5] + '0';
                    		senstate[46] = data.data[6] + '0';
                  			senstate[48] = data.data[7] + '0';
                    		}
                		else if(data.data[3] == 3)
                    		{
                    		senstate[50] = data.data[4] + '0';
                    		senstate[47] = data.data[5] + '0';
                    		senstate[49] = data.data[6] + '0';
                    		senstate[40] = data.data[7] + '0';
                    		}
                		else
                    		usleep(1);
					default:
						usleep(1);
					}
                break;

            case 0x586:
                switch(data.data[1])
                    {
                    case 5:
                        voltage = data.data[4];
                        if((fbcandata=open("/usr/stateath.tmp",O_RDWR)) < 1)
                            {
                            if((fbcandata=open("/usr/stateath.tmp",O_WRONLY|O_CREAT)) < 1)
                                break;
                            }
                        else
                            read(fbcandata, stateath, sizeof(stateath));
                        stateath[11] = voltage / 100 + '0';
                        stateath[12] = voltage / 10 % 10 + '0';
                        stateath[13] = voltage % 10 + '0';
                        write(fbcandata, stateath, sizeof(stateath));
                        close(fbcandata);
						// STATINTL_1
						if((fbcandata=open("/usr/statintl1.tmp",O_RDWR)) < 1)
                            {
                            if((fbcandata=open("/usr/statintl1.tmp",O_WRONLY|O_CREAT)) < 1)
                                break;
                            }
                        else
                            read(fbcandata, statintl, sizeof(statintl));
                        statintl[13] = voltage / 100 + '0';
                        statintl[14] = voltage / 10 % 10 + '0';
                        statintl[15] = voltage % 10 + '0';
                        write(fbcandata, statintl, sizeof(statintl));
                        close(fbcandata);
					case 0:
                		if(data.data[3] == 1)
                    		{
                    		senstate[16] = data.data[4] + '0';
                    		senstate[18] = data.data[5] + '0';
                    		}
                		else if(data.data[3] == 2)
                    		{
                    		senstate[52] = data.data[4] + '0';
                    		senstate[53] = data.data[5] + '0';
                    		senstate[54] = data.data[6] + '0';
                    		senstate[55] = data.data[7] + '0';
                    		}
                		else if(data.data[3] == 3)
                    		{
                    		senstate[56] = data.data[4] + '0';
                    		senstate[57] = data.data[5] + '0';
                    		senstate[58] = data.data[6] + '0';
                    		senstate[59] = data.data[7] + '0';
                    		}
                		else
                    		usleep(1);
					default:
						usleep(1);
					}
                break;
/*
            case 0x58f:
                switch(data.data[1])
                    {
                    case 5:
                        voltage = data.data[4];
                        if((fbcandata=open("/usr/statintl.tmp",O_RDWR)) < 1)
                            {
                            if((fbcandata=open("/usr/statintl.tmp",O_WRONLY|O_CREAT)) < 1)
                                break;
                            }
                        else
                            read(fbcandata, statintl, sizeof(statintl));
                        statintl[13] = voltage / 100 + '0';
                        statintl[14] = voltage / 10 % 10 + '0';
                        statintl[15] = voltage % 10 + '0';
                        write(fbcandata, statintl, sizeof(statintl));
                        close(fbcandata);
					default:
                        usleep(1);
                    }
                break;
*/
#ifdef TRSFCJQ
            case 0x589:
                if(data.data[3] == 1)
                    {
                    senstate[52] = data.data[4] + '0';
                    senstate[53] = data.data[5] + '0';
                    senstate[54] = data.data[6] + '0';
                    senstate[55] = data.data[7] + '0';
                    }
                else if(data.data[3] == 2)
                    {
                    senstate[56] = data.data[4] + '0';
                    senstate[57] = data.data[5] + '0';
                    senstate[58] = data.data[6] + '0';
                    senstate[59] = data.data[7] + '0';
                    }
#endif
			default:	usleep(1); 
			}
//                printf("Receive CAN package:\n");
//                printf("ID  = %x\n", data.id);
//                printf("FF  = %d\n", data.IsExt);
//                printf("RTR = %d\n", data.rxRTR);
//                printf("DAT = ");
//		int i;
//                for(i=0;i<data.dlc;i++)
//                        printf("%x ", data.data[i]);
//                printf("\n\n");
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
	int sampqy[2][30], samp45d[2][30], sampsd[2][30], sampzfs[2][30], sampzf[2][12], sampfjwd[2][30], sampnjd[2][4];
	int fs2m[2][2]={0,0,0,0}, fs10m[2][10], fx2m[2][2]={0,0,0,0}, fx10m[2][10];
    int qhfs2m[2][2], qhfs10m[2][10];
	char gg[252], hz[430], hzgg[430], rr[136], hr[256], hrrr[256];
	int qwmax=-9999, qwmin=9999, fjwdmax=-9999, fjwdmin=9999, sdmin=100, cmwdmax=-999, cmwdmin=999, dmwdmax=-999, dmwdmin=999, hwwdmax=-999, hwwdmin=999, qiyamax=0, qiyamin=20000, njdmin=64000;
	int id=0x00, slaveid;
	char str[256];

    int tempdec[2][3]={0};
    int tol, caseno, tempaver;
    int fci=0;	//30;
    int tempsort[3]={0};
	int fs1m3smax=-1, fx1m3smax=0, maxfs1msec=0, fs1h3smax=-1, fx1h3smax=0, maxfs1hhour=0, maxfs1hmin=0, fs1h10mmax=-1;
	int qhfs1m3smax=-1, maxqhfs1msec=0, qhfs1h3smax=-1, maxqhfs1hhour=0, maxqhfs1hmin=0, maxqhfs1hsec=0, qhfs1h10mmax=-1;
	int fjsd, fjsd1h[2][60];

	int rrtime, rrhour, rrmin;
	int zfsbfl=0, zjfsbfl=0, spmbfl=0, rzsj=0, xsrz=0, zfsmax=-1, zjfsmax=-1;
	float floatzfsbfl=0, floatzjfsbfl=0, floatspmbfl;;

	int td=0, localt=0, mintag=0, evap0=0;
	int fdparame=-1;
	char paramete[16], cc[38];

	int zbtemp;
	char pss[9] = {"DC,13.8\r\n"}, door[3]={"0\r\n"};
	int xdsd, suiqy, ludtemp;
	double ganqtemp, bhsqy;
	FILE *fdqcp;
	int qcpid, qcpidn;
	char qcps[BYTEQCPS], qcpm[BYTEQCPM];
	char *qcps1, *qcps2, *qcpm1, *qcpm2;
	char *delim=" \r";
	char *calib, *delimcc=",";

//	char insqy[6] = {"send\r\n"};	//PTB220
    char insqy[4] = {".P\r\n"};		//PTB210
	char insnjd[2] = {"FL"};
	char insnjdpw[9] = {"\r PW 1 0\r"};
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

//	int sec, min, hour, mday, mon, year;
	int yltemp;

	int fludtemp;
	double floatsuiqy;

#ifdef TRSFCJQ
	int sm1[3] = {28,34,12};
    int sm2[3] = {28,34,12};
    int sm3[3] = {28,34,12};
    int sm4[3] = {28,34,12};
    int sm5[3] = {28,34,12};
    int sm6[3] = {28,34,12};
    int sm7[3] = {28,34,12};
    int sm8[3] = {28,34,12};
	int hsm1[2][60], hsm2[2][60], hsm3[2][60], hsm4[2][60], hsm5[2][60], hsm6[2][60], hsm7[2][60], hsm8[2][60];
	char ss[51], hs[251];
	int zlhsl, trxdsd, sfzcl;
#endif


    printf("weather 2012-07-03 16:00 V6.2 for AnHui\n");
	printf("xiuzheng ludianwendu\n");

    twoarrayinit(*sampqy, 2, 30);
	twoarrayinit(*samp45d, 2, 30);
	twoarrayinit(*sampdlwd, 2, 30);
	twoarrayinit(*sampdlsd, 2, 30);
    twoarrayinit(*sampsd, 2, 30);
    twoarrayinit(*sampzfs, 2, 30);
    twoarrayinit(*sampzf, 2, 12);
    twoarrayinit(*sampfjwd, 2, 30);
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
    twoarrayinit(*qhfs3s, 2, 3);
//  twoarrayinit(*qhfs1m, 2, 60);
    twoarrayinit(*qhfs2m, 2, 2);
    twoarrayinit(*qhfs2m120, 2, 120);
    twoarrayinit(*qhfs10m, 2, 10);

    twoarrayinit(*fjsd1h, 2, 60);

    twoarrayinit(*samp031A004, 2, 30);
    twoarrayinit(*samp031A011, 2, 30);
    twoarrayinit(*samp031A012, 2, 30);
    twoarrayinit(*samp031A013, 2, 30);
    twoarrayinit(*samp031A014, 2, 30);
    twoarrayinit(*samp031A021, 2, 60);
    twoarrayinit(*samp031A034, 2, 30);
    twoarrayinit(*samp051A001, 2, 30);
    twoarrayinit(*samp051A024, 2, 30);
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

#ifdef TRSFCJQ
    twoarrayinit(*samp091A001, 2, 6);
    twoarrayinit(*samp091A002, 2, 6);
    twoarrayinit(*samp091A003, 2, 6);
    twoarrayinit(*samp091A004, 2, 6);
    twoarrayinit(*samp091A011, 2, 6);
    twoarrayinit(*samp091A012, 2, 6);
    twoarrayinit(*samp091A013, 2, 6);
    twoarrayinit(*samp091A014, 2, 6);

    memset(&ss, '/', sizeof(rr));
	memset(ss+40, '8', 9);
    ss[49] = '\r';
    ss[50] = '\n';
    ss[51] = '\0';
    memset(&hs, '/', sizeof(hr));
	memset(hs+200, '8', 49);
    hs[249] = '\r';
    hs[250] = '\n';
    hs[251] = '\0';
#endif

	memset(&gg, '/', sizeof(gg));
	memset(gg+203, '8', 47);
	gg[250] = '\r';
	gg[251] = '\n';
	gg[252] = '\0';
    memset(&hz, '/', sizeof(hz));
	memset(hz+346, '8', 82);
    hz[428] = '\r';
    hz[429] = '\n';
    hz[430] = '\0';
    memset(&hzgg, '/', sizeof(hzgg));
    memset(hzgg+346, '8', 82);
    hzgg[428] = '\r';
    hzgg[429] = '\n';
    hzgg[430] = '\0';
    memset(&rr, '/', sizeof(rr));
	memset(rr+108, '8', 26);
    rr[134] = '\r';
    rr[135] = '\n';
    rr[136] = '\0';
    memset(&hr, '/', sizeof(hr));
	memset(hr+204, '8', 50);
    hr[254] = '\r';
    hr[255] = '\n';
    hr[256] = '\0';
    memset(&hrrr, '/', sizeof(hrrr));
    memset(hrrr+204, '8', 50);
    hrrr[254] = '\r';
    hrrr[255] = '\n';
    hrrr[256] = '\0';
//	memset(senstate, '1', sizeof(senstate));
//	senstate[73] = '\r';
//  senstate[74] = '\n';
//  senstate[75] = '\0';
//	printf("senstate=%s\n",senstate);
//	memset(&rain, '0', sizeof(rain));

//	td
    if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
    	td = 0;
	else
		{
        read(fdparame,&paramete,4);
        close(fdparame);
		paramete[4]='\0';
//		printf("printf=%s\n",paramete);
//		if(paramete[0] == '-')
//			td = 0 - ((paramete[1]-'0')*100 + (paramete[2]-'0')*10 + (paramete[3]-'0'));
//		else
//            td = (paramete[1]-'0')*100 + (paramete[2]-'0')*10 + (paramete[3]-'0');		
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

//	sensi
    if((fdparame=open("/tmp/paramete/sensigr.txt",O_RDONLY)) < 1)
    	sensigr = 8.65;
	else
		{
    	read(fdparame,&paramete,5);
		close(fdparame);
		sensigr = atof(paramete);
//		sensigr = ((paramete[0]-'0')*1000.0 + (paramete[1]-'0')*100.0 + (paramete[3]-'0')*10.0 +(paramete[4]-'0')) / 100.0;
		}

    if((fdparame=open("/tmp/paramete/sensidr.txt",O_RDONLY)) < 1)
    	sensidr = 10.30;
	else
		{
        read(fdparame,&paramete,5);
        close(fdparame);
		sensidr = atof(paramete);
//    	sensidr = ((paramete[0]-'0')*1000.0 + (paramete[1]-'0')*100.0 + (paramete[3]-'0')*10.0 + (paramete[4]-'0')) /100.0;
		}
//	printf("gr=%f,dr=%f\n",sensigr,sensidr);

//	calibration certificate
    if((fdparame=open("/tmp/paramete/calib1.txt",O_RDONLY)) < 1)
        {
		cca1 = 0.016559493414;
		ccb1 = 0.00000918277921;
		ccfo1 = 1044.2;
        }
	else
		{
        read(fdparame,&cc,sizeof(cc));
        close(fdparame);
		cc[38] = '\0';
		calib = strtok(cc, delimcc);
		cca1 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccb1 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccfo1 = atof(calib);
		}
//	printf("cca1=%f\nccab1=%f\nccfo1=%f\n",cca1,ccb1,ccfo1);

    if((fdparame=open("/tmp/paramete/calib2.txt",O_RDONLY)) < 1)
        {
        cca2 = 0.016914835808;
        ccb2 = 0.00000914412168;
        ccfo2 = 1051.1;
        }
	else
		{
        read(fdparame,&cc,sizeof(cc));
        close(fdparame);
        cc[38] = '\0';
        calib = strtok(cc, delimcc);
        cca2 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccb2 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccfo2 = atof(calib);
		}
//	printf("cca2=%f\nccab2=%f\nccfo2=%f\n",cca2,ccb2,ccfo2);

    if((fdparame=open("/tmp/paramete/calib3.txt",O_RDONLY)) < 1)
        {
        cca3 = 0.016855066463;
        ccb3 = 0.00000915054467;
        ccfo3 = 1051.5;
        }
	else
		{
        read(fdparame,&cc,sizeof(cc));
        close(fdparame);
        cc[38] = '\0';
        calib = strtok(cc, delimcc);
        cca3 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccb3 = atof(calib);
        calib = strtok(NULL, delimcc);
        ccfo3 = atof(calib);
		}
//	printf("cca3=%f\nccab3=%f\nccfo3=%f\n",cca3,ccb3,ccfo3);


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
		for(qcpid=0,qcpidn=0; qcpid<71; qcpid++)
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
        for(qcpid=0,qcpidn=0; qcpid<73; qcpid++)
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
//    if(min == 0)
//        minary = 59;
//    else
//        minary = min - 1;
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
	stringmove(beijintime, 0, loctime, 0, 12);

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
	fd_ttyS1 = open(SERIAL1, O_RDWR);
    if (fd_ttyS1 < 0)
		{
        printf("open uart 1 VI error %d\n", fd_ttyS1);
		return -1;
		}
    set_speed(fd_ttyS1, 9600);
	set_other_attribute(fd_ttyS1, 8, 1, 0, 50);
    pthread_create(&th_njd, NULL, njdpthreadxlht, 0);
//	set_other_attribute(fd_ttyS1, 7, 1, 2, 50);
//    pthread_create(&th_njd, NULL, njdpthreadzhzh, 0);

    fd_ttyS2 = open(SERIAL2, O_RDWR);
    if (fd_ttyS2 < 0)
		{
        printf("open uart 2 P error %d\n", fd_ttyS2);
		return -1;
		}
    set_speed(fd_ttyS2, 9600);
    set_other_attribute(fd_ttyS2, 8, 1, 0, 100);
    pthread_create(&th_qy, NULL, qiyapthread, 0);


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

    if((fdsenco = open("/tmp/paramete/sencows1.txt", O_RDONLY)) < 1)
        usleep(1);
    else
        {
        read(fdsenco, senco, sizeof(senco));
        close(fdsenco);
        ws1bl0 = atof(senco);
        string1 = strchr(senco, ' ');
        string1++;
        ws1bl1 = atof(string1);
//        printf("ws1bl0=%f, ws1bl1=%f\n",ws1bl0,ws1bl1);
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

    if((fdsenco = open("/tmp/paramete/sencorat1.txt", O_RDONLY)) < 1)
        usleep(1);
    else
        {
        read(fdsenco, senco, sizeof(senco));
        close(fdsenco);
        string1 = strchr(senco, ' ');
        rat1bl = atof(string1);
//        printf("rat1bl=%f\n",rat1bl);
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
        stringmove(beijintime, 0, loctime, 0, 12);
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

            slaveid=0x603;
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

            slaveid=0x603;
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

            slaveid=0x603;
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

            slaveid=0x603;
            str[0] = 0x40;
            str[1] = 0;
            str[2] = 0x38;
            str[3] = 4;
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

            slaveid=0x609;
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

            slaveid=0x609;
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

            slaveid=0x603;
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
			}

//		printf("sec=%d,",sec);
		yuliang = 0;
		yuliang = read(fd_fiqyl, 0, 0);
//		yltemp = read(fd_fiqyl, 0, 0);
//		if(!(yltemp==1||yltemp==0))
//			printf("Read Error!yltemp=%d\n",yltemp);
//		yuliang = yltemp;
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
			fx1m3smax = fx1m[0][(secary+60-1)%60];
//        	maxfs1msec = sec;
        	}
        if(tag==RI && aver>fs1h3smax)
            {
            fs1h3smax = aver;
			fx1h3smax = fx1m[0][(secary+60-1)%60];
//			printf("main:%d,%d\n",(secary+60-1)%60,fx1h3smax);
//			maxfs1hhour = hour;
//            maxfs1hmin = min;	
			if(min != 59)
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

        averagefs(*qhfs3s, 3);
//      for(x=0;x<2;x++)
//         	{
//			for(y=0;y<3;y++)
//			printf("%d;",qhfs3s[x][y]);
//          }
//		printf("qhfs3s=%d,tag=%d\n",aver,tag);
		if(tag==RI && aver>qhfs1m3smax)
            {
            qhfs1m3smax = aver;
//            maxqhfs1msec = sec;
            }
        if(tag==RI && aver>qhfs1h3smax)
            {
            qhfs1h3smax = aver;
//			maxqhfs1hhour = hour;
//            maxqhfs1hmin = min;
            if(min != 59)
                {
                maxqhfs1hhour = hour;
                maxqhfs1hmin = min + 1;
                }
            else
                {
                maxqhfs1hmin = 0;
                if(hour != 23)
                    maxqhfs1hhour = hour + 1;
                else
                    maxqhfs1hhour = 0;
                }

//			maxqhfs1hsec = sec;
            }


		if(sec%2 == 0)
			{

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
            	temp45d = (A*res*res + B*res + C + 0.005) * 100 + WDJJ + dzt0*10;                       //dzt0
//                temp45d = (E/10000000*res*res*res + F/10000*res*res + G*res - H + 0.005) * 100 + wpxx + dzt0;
            else
            	temp45d = (A*res*res + B*res + C - 0.005) * 100 + WDJJ + dzt0*10;
//                temp45d = (E/10000000*res*res*res + F/10000*res*res + G*res - H - 0.005) * 100 + wpxx + dzt0;
//            printf("res=%ftemp0=%dtemp1=%ddzt0=%dwpxx=%dtemp45D=%d\n",res,temp0,temp1,dzt0,wpxx,temp45d);

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
            humi = ((data[1]*256 + data[0]) * 2500 / 65535 + 5) / 10 + dzu;             // dzu
//	        printf("humi=%d\n",humi);
            if(humi > 99)
                humi = 99;
            if(humi < 0)
                humi = 0;
			
			sampsd[0][suffix] = humi;
            sampsd[1][suffix] = qcsamp(humi, U, 1, 9999);
			asciibyte4(samprh, humi, 4+suffix*4);
            samprh[124] = '\r';
            samprh[125] = '\n';
//			printf("humi=%d, qc=%d\n",humi,sampsd[1][suffix]);
//			printf("temp1=%d,%d;\ntemp2=%d,%d;\ntemp3=%d,%d;\n",temp[0][0],temp[1][0],temp[0][1],temp[1][1],temp[0][2],temp[1][2]);
	        if(temp[1][0]==RI && temp[1][1]==RI)
        	    {
              	tempdec[0][0] = abs(temp[0][0]-temp[0][1]);
               	if(abs(temp[0][0])<=500 && abs(temp[0][1])<=500)
                   	tol = 30;
               	else
                   	tol = 60;
              	if(tempdec[0][0]<=tol)
                   	tempdec[1][0] = RI;
               	else
                   	tempdec[1][0] = WR;
               	}
        	else
               	tempdec[1][0] = WR;
                                                                                                                                               
    		if(temp[1][1]==RI && temp[1][2]==RI)
        		{
        		tempdec[0][1] = abs(temp[0][1]-temp[0][2]);
        		if(abs(temp[0][1])<=500 && abs(temp[0][2])<=500)
            		tol = 30;
        		else
            		tol = 60;
        		if(tempdec[0][1]<=tol)
            		tempdec[1][1] = RI;
        		else
            		tempdec[1][1] = WR;
        		}
    		else
        		tempdec[1][1] = WR;

    		if(temp[1][2]==RI && temp[1][0]==RI)
        		{
        		tempdec[0][2] = abs(temp[0][2]-temp[0][0]);
        		if(abs(temp[0][2])<=500 && abs(temp[0][0])<=500)
            		tol = 30;
        		else
            		tol = 60;
        		if(tempdec[0][2]<=tol)
            		tempdec[1][2] = RI;
        		else
            		tempdec[1][2] = WR;
        		}
    		else
        		tempdec[1][2] = WR;
                                                                                                                                              
        	if((f0>=fci && f1>=fci && f2>=fci) && ((tempdec[1][0]==RI&&tempdec[1][1]==RI) || (tempdec[1][1]==RI&&tempdec[1][2]==RI) || (tempdec[1][2]==RI&&tempdec[1][0]==RI)))
               	caseno = 1;
        	else if((f0>=fci && f1>=fci && f2>=fci) && (tempdec[1][0]==RI && tempdec[1][1]!=RI && tempdec[1][2]!=RI))
               	caseno = 2;
    		else if((f0>=fci && f1>=fci && f2>=fci) && (tempdec[1][0]!=RI && tempdec[1][1]==RI && tempdec[1][2]!=RI))
        		caseno = 3;
    		else if((f0>=fci && f1>=fci && f2>=fci) && (tempdec[1][0]!=RI && tempdec[1][1]!=RI && tempdec[1][2]==RI))
        		caseno = 4;
        	else if((f0>=fci && f1>=fci && f2>=fci) && (tempdec[1][0]!=RI && tempdec[1][1]!=RI && tempdec[1][2]!=RI))
        		caseno = 8;
        	else if((f0>=fci && f1>=fci && f2<fci) && tempdec[1][0]==RI)
               	caseno = 2;
    		else if((f0>=fci && f1>=fci && f2<fci) && tempdec[1][0]!=RI)
        		caseno = 8;
    		else if((f0<fci && f1>=fci && f2>=fci) && tempdec[1][1]==RI)
        		caseno = 3;
    		else if((f0<fci && f1>=fci && f2>=fci) && tempdec[1][1]!=RI)
        		caseno = 8;
    		else if((f0>=fci && f1<fci && f2>=fci) && tempdec[1][2]==RI)
        		caseno = 4;
    		else if((f0>=fci && f1<fci && f2>=fci) && tempdec[1][2]!=RI)
        		caseno = 8;
        	else if(f0>=fci && f1<fci && f2<fci)
               	caseno = 5;
    		else if(f0<fci && f1>=fci && f2<fci)
        		caseno = 6;
    		else if(f0<fci && f1<fci && f2>=fci)
        		caseno = 7;
        	else
               	caseno = 8;
//        	printf("caseno=%d\n",caseno);
                                                                                                                                              
			switch(caseno)
                {
                case 1: midnum(*temp,tempsort); tempaver = tempsort[1]; break;
                case 2: tempaver = (temp[0][0]+temp[0][1])/2+(temp[0][0]+temp[0][1])%2; break;
        		case 3: tempaver = (temp[0][1]+temp[0][2])/2+(temp[0][1]+temp[0][2])%2; break;
        		case 4: tempaver = (temp[0][2]+temp[0][0])/2+(temp[0][2]+temp[0][0])%2; break;
        		case 5: tempaver = temp[0][0]; break;
        		case 6: tempaver = temp[0][1]; break;
        		case 7: tempaver = temp[0][2]; break;
                case 8: tempaver = WRONG; break;
                default: tempaver = WRONG;
                }

//			printf("tempaver=%d\n",tempaver);
			sampfjwd[0][suffix] = tempaver;
			if(tempaver == WRONG)
				sampfjwd[1][suffix] = MI;
			else
				sampfjwd[1][suffix] = RI;
//             	printf("tempaver=%d\n",tempaver);

            write(fd_ttyS2, &insqy, sizeof(insqy));
//			printf("pressure = %d\n", pres);
            sampqy[0][suffix] = pres;
            sampqy[1][suffix] = qcsamp(pres, P, TEN, 65535);
			}	//if(sec%2 == i)


		if(sec%10 == 0)
			{
#ifdef AD7792
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
            arg.pin_sta = 1;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
			arg.pin_idx = PIO_ADX_SEL3;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);

//            read(fd_ad, data, 2);
//            usleep(20000);
            read(fd_ad, data, 2);
			zfevap = ((data[1]*256+data[0]) / 65536.0 * 2.5 * 1000.0 - 400) / 1600.0 * 1000.0;
			evap = zfevap + 0;
//			printf("data=%2x%2x,zfevap=%f\n",data[1],data[0],zfevap);
			if(evap < 0)
				{
				evap = 0;
				sampzf[0][minary%2*6+secary/10] = 0;
            	sampzf[1][minary%2*6+secary/10] = MI;
				}
			else
				{
            	sampzf[0][minary%2*6+secary/10] = evap;
            	sampzf[1][minary%2*6+secary/10] = qcsamp(evap, LE, TEN, 9999);
				}

			sampzf[0][minary%2*6+secary/10] = evap;
            sampzf[1][minary%2*6+secary/10] = qcsamp(evap, LE, TEN, 9999);
//            printf("sampzf = %d, qc=%d\n", sampzf[0][minary%5*6+secary/10], sampzf[1][minary%5*6+secary/10]);
#endif
			}	//if(sec%10 == 0)


		if(sec%15 == 0)
			{
            arg.pin_idx = PIO_RS485_CTL2;
            arg.pin_sta = 1;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
	        rs485ret = write(fd_ttyS1, insnjd, sizeof(insnjd));
			usleep(10);
            usleep(1000);
            arg.pin_idx = PIO_RS485_CTL2;
            arg.pin_sta = 0;
            ioctl(fd_pio, IOCTL_PIO_SETSTA, &arg);
			usleep(100);
//			printf("visa=%d\n",visi);
			sampnjd[0][secary/15] = visi;
            sampnjd[1][secary/15] = qcsamp(visi, VI, 1, 65535);
			}	

		if(sec==30 || sec==40 || sec==50)
			{
			mintag = 1;
			}

		if(sec==0 || (sec==1 && mintag==1))
			{
//  WPDZ
/*
            if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
				usleep(1);
			else
            	{
				mact = read(fdmaste, &masttemp, 2);
            	usleep(10000);
            	mact = read(fdmaste, &masttemp, 2);
//				printf("mact=%d\n",masttemp);
                close(fdmaste);
				}
			if(masttemp<800 && masttemp>=500)		
				{
				wpxx = 4 + (masttemp*10-6000) * (2.0/2000.0);;
				wpxx1 = 2 + (masttemp*10-6000) * (1.0/2000.0);;
				}
			else if(masttemp<500 && masttemp>=300)
				{
				wpxx = 2 + (masttemp*10-4000) * (2.0/2000.0);
				wpxx1 = 2 + (masttemp*10-4000) * (2.0/2000.0);
				}
			else if(masttemp<300 && masttemp>=100)
				{
				wpxx = 0 + (masttemp*10-2000) * (2.0/2000.0);
				wpxx1 = 0 + (masttemp*10-2000) * (2.0/2000.0);
				}
			else if(masttemp<100 && masttemp>=-100)
				{
				wpxx = -3 + (masttemp*10-0) * (2.0/2000.0);
				wpxx1 = -2 + (masttemp*10-0) * (2.0/2000.0);
				}
			else if(masttemp<-100 && masttemp>=-300)
				{
				wpxx = -6 + (masttemp*10+2000) * (2.0/2000.0);
				wpxx1 = -3 + (masttemp*10+2000) * (1.0/2000.0);
				}
			else if(masttemp<-300)
				{
				wpxx = -9 + (masttemp*10+4000) * (2.0/2000.0);
				wpxx1 = -5 + (masttemp*10+4000) * (2.0/2000.0);
				}
			else
				{
				wpxx = 0;
				wpxx1 = 0;
				}
//			printf("mact=%d,	wpxx=%d,	wpxx1=%d,	dzt0=%d,	dzt1=%d\n",masttemp,wpxx,wpxx1,dzt0,dzt1);
*/

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
//          for(x=0;x<120;x++)
//              printf("%d;",fx2m120[1][x]);
//            printf("\n");
			averagefx(*fx2m120, 120);										// 2分风向
//			printf("averfx2m=%d,tag=%d\n",aver,tag);
			if(tag != MI)
				{
				gg[203] = tag + '0';
				asciibyte4(gg, aver, 4);
				}
			else
				{
				gg[203] = tag + '0';
				tagnotRI(gg, 4);
				}
			hzgg[346] = gg[203];
			stringmove(gg, 4, hzgg, 4, 4);
	
            averagefx(*fx10m,10);											// 10分风向
            if(tag != MI)
                {
                gg[205] = tag + '0';
                asciibyte4(gg, aver, 12);
                }
            else
                {
                gg[205] = tag + '0';
				tagnotRI(gg, 12);
                }
            hzgg[348] = gg[205];
            stringmove(gg, 12, hzgg, 12, 4);

			averagefs(*fs1m, 60);											 
            if(tag == RI)
				tag = qcinstance(aver, WS, TEN, 9999);
            fs2m[0][minary%2] = aver;
            fs2m[1][minary%2] = tag;
            fs10m[0][minary%10] = aver;
            fs10m[1][minary%10] = tag;

			averagefs(*fs2m120, 120);										// 2分风速
			if(tag == RI)
                tag = qcinstance(aver, WS2, TEN, 9999);
        	if(tag != MI)
                {
                gg[204] = tag + '0';
                asciibyte4(gg, aver, 8);
                }
            else
                {
                gg[204] = tag + '0';
				tagnotRI(gg, 8);
                }
            hzgg[347] = gg[204];
            stringmove(gg, 8, hzgg, 8, 4);

            averagefs(*fs10m, 10);											// 10分风速
            if(tag == RI)
                tag = qcinstance(aver, WS3, TEN, 9999);
            if(tag != MI)
                {
                gg[206] = tag + '0';
                asciibyte4(gg, aver, 16);
				
				if(tag==RI && aver>fs1h10mmax)								// 最大风速
					{
					fs1h10mmax = aver;
					hzgg[350] = tag + '0';
                    hzgg[351] = tag + '0';
                    hzgg[352] = tag + '0';
					stringmove(gg, 12, hzgg, 20, 8);
					hzgg[28] = hour / 10 + '0';
                    hzgg[29] = hour % 10 + '0';
                    hzgg[30] = min / 10 + '0';
                    hzgg[31] = min % 10 + '0';
					}
                }
            else
                {
                gg[206] = tag + '0';
				tagnotRI(gg, 16);
                }
            hzgg[349] = gg[206];
            stringmove(gg, 16, hzgg, 16, 4);

            gg[207] = RI + '0';												// 最大瞬时风速
            gg[208] = RI + '0';
            asciibyte4(gg, sampfx1mmax, 20);
            asciibyte4(gg, sampfs1mmax, 24);
            sampfs1mmax = -1;
            hzgg[353] = gg[207];
            hzgg[354] = gg[208];
            stringmove(gg, 20, hzgg, 32, 8);

            hzgg[355] = RI + '0';												// 极大风速
            hzgg[356] = RI + '0';
            hzgg[357] = RI + '0';
            asciibyte4(hzgg, fx1h3smax, 40);
            asciibyte4(hzgg, fs1h3smax, 44);
			hzgg[48] = maxfs1hhour / 10 + '0';
            hzgg[49] = maxfs1hhour % 10 + '0';
            hzgg[50] = maxfs1hmin / 10 + '0';
            hzgg[51] = maxfs1hmin % 10 + '0';
            fs1m3smax = -1;

            averagefs(*samp031A021, 60);
            qhfs2m[0][minary%2] = aver;
            qhfs2m[1][minary%2] = tag;
            qhfs10m[0][minary%10] = aver;
            qhfs10m[1][minary%10] = tag;

//            averagefs(*qhfs2m, 2);
			averagefs(*qhfs2m120, 120);										// 2分风速（气候）
            if(tag != MI)
            	{
               	gg[209] = tag + '0';
				asciibyte4(gg, aver, 28);
                }
            else
                {
                gg[209] = tag + '0';
				tagnotRI(gg, 28);
                }
			hzgg[358] = gg[209];
            stringmove(gg, 28, hzgg, 52, 4);

           	averagefs(*qhfs10m, 10);										// 10分风速（气候)
            if(tag != MI)
                {
                gg[210] = tag + '0';
                asciibyte4(gg, aver, 32);
                if(tag==RI && aver>qhfs1h10mmax)
                	{
                    qhfs1h10mmax = aver;
                    hzgg[360] = tag + '0';
                    hzgg[361] = tag + '0';
                    stringmove(gg, 36, hzgg, 60, 4);
                    hzgg[64] = hour / 10 + '0';
                    hzgg[65] = hour % 10 + '0';
                    hzgg[66] = min / 10 + '0';
                    hzgg[67] = min % 10 + '0';
                    }
                }
            else
                {
                gg[210] = tag + '0';
				tagnotRI(gg, 32);
                }
			hzgg[359] = gg[210];
            stringmove(gg, 32, hzgg, 56, 4);

			if(qhfs1m3smax == -1)											// 分钟极大风速
				{
            	gg[211] = MI + '0';
				tagnotRI(gg, 36);
				}
			else
				{
                gg[211] = RI + '0';
				asciibyte4(gg, qhfs1m3smax, 36);
				qhfs1m3smax = -1;
				}
			hzgg[362] = gg[211];
			stringmove(gg, 36, hzgg, 68, 4);

			if(qhfs1h3smax == -1)											// 极大风速
				{
            	hzgg[363] = MI + '0';
            	hzgg[364] = MI + '0';
				tagnotRI(hzgg, 72);
				tagnotRI(hzgg, 76);
				}
			else
				{
                hzgg[363] = RI + '0';
                hzgg[364] = RI + '0';
            	asciibyte4(hzgg, qhfs1h3smax, 72);
            	hzgg[76] = maxqhfs1hhour / 10 + '0';
            	hzgg[77] = maxqhfs1hhour % 10 + '0';
            	hzgg[78] = maxqhfs1hmin / 10 + '0';
            	hzgg[79] = maxqhfs1hmin % 10 + '0';
				}

//	        fzyl = read(fd_fiqyl, 0, 0);									// RAT
        	fzyltag = qclimit(fzyl, ceiling[RAT], threshold[RAT]);
//			printf("fzyl=%d,fzyltag=%d\n",fzyl,fzyltag);
			if(fzyltag != MI)
				{
				gg[212] = fzyltag + '0';
				gg[213] = RI + '0';
	            asciibyte4(gg, fzyl, 40);
				xsyl += fzyl;
	            asciibyte4(gg, xsyl, 44);
				fzylzf = fzyl;
				fzyl = 0;
//				printf("xsyl=%d\n",xsyl);
				}
			else
                {
                gg[212] = fzyltag + '0';
				tagnotRI(gg, 40);
				fzylzf = 0;
				fzyl = 0;
				}
			hzgg[365] = gg[213];
			stringmove(gg, 44, hzgg, 80, 4);
//			rain[minary*2] = fzyl / 10 + '0';
//			rain[minary*2+1] = fzyl * 10 + '0';

//            if((fdsenco = open("/tmp/paramete/sencorat1.txt", O_RDONLY)) < 1)
//				usleep(1);
//			else
//				{
//				read(fdsenco, senco, sizeof(senco));
//				close(fdsenco);
//				string1 = strchr(senco, ' ');
//				string2 = strchr(string1, ' ');
//                string1 = strchr(string2, ' ');
//				rat1bl = atof(string1);
//				printf("rat1bl=%f\n",rat1bl);
//				}
			fzdfdyl = dfdylmc * rat1bl * 10;
            fzdfdyltag = qclimit(fzdfdyl, ceiling[RAT1], threshold[RAT1]);	// RAT1
//          printf("fzdfdyl=%d,fzdfdyltag=%d\n",fzdfdyl,fzdfdyltag);
            if(fzdfdyltag != MI)
                {
                gg[214] = fzdfdyltag + '0';
                gg[215] = RI + '0';
                asciibyte4(gg, fzdfdyl, 48);
                xsdfdyl += fzdfdyl;
                asciibyte4(gg, xsdfdyl, 52);
                fzdfdyl = 0;
//              printf("xsdfdyl=%d\n",xsdfdyl);
                }
            else
                {
                gg[214] = fzdfdyltag + '0';
                tagnotRI(gg, 48);
                fzdfdyl = 0;
                }
            hzgg[366] = gg[215];
            stringmove(gg, 52, hzgg, 84, 4);

            if(fzcctag != MI)												// RAW
                {
                gg[216] = fzcctag + '0';
                gg[217] = RI + '0';
                asciibyte4(gg, fzcc, 56);
				fzcc = 0;
                asciibyte4(gg, xscc, 60);
                }
            else
                {
                gg[216] = fzcctag + '0';
                tagnotRI(gg, 56);
                }
            hzgg[367] = gg[217];
            stringmove(gg, 60, hzgg, 88, 4);

			average(*samp45d, 30);											// T0			
//			printf("temp=%d\n",aver);
			if(aver >= 0)
				aver = (aver+5) / 10;
			else
				aver = (aver-5) / 10;
            if(tag == RI)
                tag = qcinstance(aver, T0, TEN, 9999);
//printf("ceil=%f,thres=%f\ncost=%d,sus=%f,wrong=%f\n",instceil[T0],instthre[T0],instcost[T0],instsusp[T0],instwrong[T0]);
//			printf("aver45d=%d,tag=%d\n",aver,tag);
			if(tag != MI)
				{
				ganqtemp = aver / 10.0 + 273.15;
				gg[218] = tag + '0';
				asciitemp(gg, aver, 64);
				if(tag==RI && aver>qwmax)
					{
					hzgg[369] = tag + '0';
					hzgg[370] = tag + '0';
					qwmax = aver;
					asciitemp(hzgg, aver, 96);
					hzgg[100] = hour / 10 + '0';
                    hzgg[101] = hour % 10 + '0';
                    hzgg[102] = min / 10 + '0';
                    hzgg[103] = min % 10 + '0';
					}
                if(tag==RI && aver<qwmin)
                    {
                    hzgg[371] = tag + '0';
                    hzgg[372] = tag + '0';
                    qwmin = aver;
                    asciitemp(hzgg, aver, 104);
                    hzgg[108] = hour / 10 + '0';
                    hzgg[109] = hour % 10 + '0';
                    hzgg[110] = min / 10 + '0';
                    hzgg[111] = min % 10 + '0';
                    }
				}
			else
				{
				gg[218] = tag + '0';
				tagnotRI(gg, 64);
				}
			hzgg[368] = gg[218];
			stringmove(gg, 64, hzgg, 92, 4);

			if(f0>=fci && f1>=fci && f2>=fci)							// 风机速度
				fjsd = (f0+f1+f2) / 3;
			else if(f0>=fci && f1>=fci)
				fjsd = (f0+f1) / 2;
            else if(f1>=fci && f2>=fci)
                fjsd = (f1+f2) / 2;
            else if(f2>=fci && f0>=fci)
                fjsd = (f2+f0) / 2;
			else if(f0>=fci)
				fjsd = f0;
            else if(f1>=fci)
                fjsd = f1;
            else if(f2>=fci)
                fjsd = f2;
			else
				fjsd = -1;
			if(fjsd >=0)
				{
				fjsd1h[0][minary] = fjsd;
				fjsd1h[1][minary] = RI;
				gg[219] = RI + '0';
            	asciibyte4(gg, fjsd, 68);
				}
			else
				{
                fjsd1h[0][minary] = 0;
                fjsd1h[1][minary] = MI;
                gg[219] = MI + '0';
                tagnotRI(gg,68);
				}
            hzgg[378] = gg[219];
            stringmove(gg, 68, hzgg, 132, 4);

//			for(x=0;x<30;x++)
//				printf("%d;",sampfjwd[0][30]);
            average(*sampfjwd, 30);											// 气温（气候）
            if(aver >= 0)
                aver = (aver+5) / 10;
            else
                aver = (aver-5) / 10;
            if(tag == RI)
                tag = qcinstance(aver, T1, TEN, 9999);
//          printf("aver45d=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag != MI)
                {
                gg[220] = tag + '0';
                asciitemp(gg, aver, 72);
                if(tag==RI && aver>fjwdmax)
            	    {
                    hzgg[374] = tag + '0';
                    hzgg[375] = tag + '0';
                    fjwdmax = aver;
                    asciitemp(hzgg, aver, 116);
                    hzgg[120] = hour / 10 + '0';
                    hzgg[121] = hour % 10 + '0';
                    hzgg[122] = min / 10 + '0';
                    hzgg[123] = min % 10 + '0';
                    }
                if(tag==RI && aver<fjwdmin)
                    {
                    hzgg[376] = tag + '0';
                    hzgg[377] = tag + '0';
                    fjwdmin = aver;
                    asciitemp(hzgg, aver, 124);
                    hzgg[128] = hour / 10 + '0';
                    hzgg[129] = hour % 10 + '0';
                    hzgg[130] = min / 10 + '0';
                    hzgg[131] = min % 10 + '0';
                    }
                }
            else
                {
                gg[220] = tag + '0';
				tagnotRI(gg, 72);
				}
            hzgg[373] = gg[220];
            stringmove(gg, 72, hzgg, 112, 4);

			gg[221] = MI + '0';
			gg[76] = ' ';
            gg[77] = ' ';
            gg[78] = ' ';
            gg[79] = '*';

            average(*sampsd, 30);											// 湿度
            if(tag == RI)
                tag = qcinstance(aver, U, 1, 9999);
            if(tag != MI)
				{
				xdsd = aver;
				gg[223] = tag + '0';
                asciibyte4(gg, aver, 84);
                if(tag==RI && aver<sdmin)
                	{
                    hzgg[382] = '0';
                    hzgg[383] = '0';
                    sdmin = aver;
                    asciibyte4(hzgg, aver, 148);
                    hzgg[152] = hour / 10 + '0';
                    hzgg[153] = hour % 10 + '0';
                    hzgg[154] = min / 10 + '0';
                    hzgg[155] = min % 10 + '0';
                    }
				}
			else
				{
				gg[223] = tag + '0';
				tagnotRI(gg, 84);
                }
			gg[222] = gg[223];
			stringmove(gg, 84, gg, 80, 4);
			stringmove(gg, 221, hzgg, 379, 3);
			stringmove(gg, 76, hzgg, 136, 12);

//			double bhsqy1, bhsqy2, bhsqy3, bhsqy4; 	
//			printf("%f\n", ganqtemp);
//			bhsqy1 = SQYA*(1-SXDWD/ganqtemp);
//			bhsqy2 = log10(ganqtemp/SXDWD);
//			bhsqy3 = SQYC*(1-pow(10,SQYD*(ganqtemp/SXDWD-1)));
//			bhsqy4 = SQYE*(pow(10,SQYF*(1-SXDWD/ganqtemp))-1);
//			printf("%f,	%f, %f, %f\n", bhsqy1, bhsqy2, bhsqy3, bhsqy4);
			if(gg[218] == RI+'0')
				{
				bhsqy = pow(10,SQYA*(1-SXDWD/ganqtemp) + SQYB*log10(ganqtemp/SXDWD) + SQYC*(1-pow(10,SQYD*(ganqtemp/SXDWD-1))) + SQYE*(pow(10,SQYF*(1-SXDWD/ganqtemp))-1) + SQYG);
            	if(gg[223] == RI+'0')
					{
					floatsuiqy = xdsd * bhsqy / 10;
					suiqy = (xdsd * bhsqy + 5) / 10;
					gg[224] = RI + '0';
					asciibyte4(gg, suiqy, 88);
					}
            	else
                	{
                	gg[224] = MI + '0';
                	tagnotRI(gg, 88);
                	}
				}
			else
				{
				bhsqy = 0;
                gg[224] = MI + '0';
                tagnotRI(gg, 88);
                }
//			printf("bhsqy = %f\n",bhsqy);

			if(gg[224] == RI+'0')
				{
				if(suiqy > 0)
					{
					fludtemp = ((LDWDB*log10(floatsuiqy/10.0/BHSQY0)) / (LDWDA-log10(floatsuiqy/10.0/BHSQY0)) * 100 + 5) / 10;
					ludtemp = (LDWDB*log10(suiqy/10.0/BHSQY0)) / (LDWDA-log10(suiqy/10.0/BHSQY0)) * 100;
					if(ludtemp >= 0)
						ludtemp = (ludtemp+5) / 10;
					else
						ludtemp = (ludtemp-5) / 10;
//					printf("suiqy=%d,ludtemp=%d\n",suiqy,ludtemp);
            		tag = qcinstance(ludtemp, TD, TEN, 9999);
					}
				else
					{
					tag = 0;
					ludtemp = 0;
					}
        		gg[225] = tag + '0';
				if(tag != MI)
					asciitemp(gg, ludtemp, 92);
				else
            		tagnotRI(gg, 92);
				}
			else
				{
                gg[225] = MI + '0';
                tagnotRI(gg, 92);
                }
			stringmove(gg, 224, hzgg, 384, 2);
			stringmove(gg, 88, hzgg, 156, 8);
			printf("suiqy=%d,ludtemp=%d\nfloatsuiqy=%f,fludtemp=%d\n",suiqy,ludtemp,floatsuiqy,fludtemp);

			average(*sampqy, 30);												// P
            if(tag == RI)
                tag = qcinstance(aver, P, TEN, 65535);
			if(tag != MI)
				{
				gg[226] = tag + '0';
				asciibyte5(gg, aver, 96);
				if(tag==RI && aver>qiyamax)
					{
					hzgg[387] = tag + '0';
                	hzgg[388] = tag + '0';
					qiyamax = aver;
					stringmove(gg, 96, hzgg, 169, 5);
            		hzgg[174] = hour / 10 + '0';
                	hzgg[175] = hour % 10 + '0';
                	hzgg[176] = min / 10 + '0';
                	hzgg[177] = min % 10 + '0';
					}
            	if(tag==RI && aver<qiyamin)
                	{
                	hzgg[389] = tag + '0';
                	hzgg[390] = tag + '0';
                	qiyamin = aver;
					stringmove(gg, 96, hzgg, 178, 5);
                	hzgg[183] = hour / 10 + '0';
                	hzgg[184] = hour % 10 + '0';
                	hzgg[185] = min / 10 + '0';
                	hzgg[186] = min % 10 + '0';
					}
				}
			else
				{
            	gg[226] = tag + '0';
				tagMIbyte5(gg, 96);
            	}
			hzgg[386] = gg[226];
			stringmove(gg, 96, hzgg, 164, 5);

        	average(*samp061A001, 30);
//	     	fz061A001[0][min] = aver;
//     		fz061A001[1][min] = tag;
//	      	printf("aver=%d,tag=%d\n",aver,tag);
            if(tag == RI)
                tag = qcinstance(aver, TG, TEN, 9999);
        	if(tag != MI)
                {
                gg[227] = tag + '0';
                asciitemp(gg, aver, 101);
                if(tag==RI && aver>cmwdmax)
     	          	{
                    hzgg[392] = '0';
                    hzgg[393] = '0';
                    cmwdmax = aver;
                    asciitemp(hzgg, aver, 191);
                    hzgg[195] = hour / 10 + '0';
                    hzgg[196] = hour % 10 + '0';
                    hzgg[197] = min / 10 + '0';
                    hzgg[198] = min % 10 + '0';
                    }
                if(tag==RI && aver<cmwdmin)
                    {
                    hzgg[394] = '0';
                    hzgg[395] = '0';
                    cmwdmin = aver;
                    asciitemp(hzgg, aver, 199);
                    hzgg[203] = hour / 10 + '0';
                    hzgg[204] = hour % 10 + '0';
                    hzgg[205] = min / 10 + '0';
                    hzgg[206] = min % 10 + '0';
                    }
                }
            else
                {
                gg[227] = tag + '0';
				tagnotRI(gg, 101);
                }
			hzgg[391] = gg[227];
			stringmove(gg, 101, hzgg, 187, 4);

            average(*samp061A002, 30);
//          fz061A002[0][min] = aver;
//          fz061A002[1][min] = tag;
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST0, TEN, 9999);
            if(tag != MI)
                {
                gg[228] = tag + '0';
                asciitemp(gg, aver, 105);
                if(tag==RI && aver>dmwdmax)
                	{
                    hzgg[397] = '0';
                    hzgg[398] = '0';
                    dmwdmax = aver;
                    asciitemp(hzgg, aver, 211);
                    hzgg[215] = hour / 10 + '0';
                    hzgg[216] = hour % 10 + '0';
                    hzgg[217] = min / 10 + '0';
                    hzgg[218] = min % 10 + '0';
                    }
                if(tag==RI && aver<dmwdmin)
                    {
                    hzgg[399] = '0';
                    hzgg[400] = '0';
                    dmwdmin = aver;
                    asciitemp(hzgg, aver, 219);
                    hzgg[223] = hour / 10 + '0';
                    hzgg[224] = hour % 10 + '0';
                    hzgg[225] = min / 10 + '0';
                    hzgg[226] = min % 10 + '0';
                    }
                }
            else
                {
                gg[228] = tag + '0';
				tagnotRI(gg, 105);
                }
            hzgg[396] = gg[228];
            stringmove(gg, 105, hzgg, 207, 4);

            average(*samp031A004, 30);
//          fz061A002[0][min] = aver;
//          fz061A002[1][min] = tag;
//			printf("aver=%d,tag=%d\n",aver,tag);
            if(tag == RI)
                tag = qcinstance(aver, IR, TEN, 9999);
            if(tag != MI)
                {
                gg[229] = tag + '0';
                asciitemp(gg, aver, 109);
                if(tag==RI && aver>hwwdmax)
                	{
                    hzgg[402] = '0';
                    hzgg[403] = '0';
                    hwwdmax = aver;
                    asciitemp(hzgg, aver, 231);
                    hzgg[235] = hour / 10 + '0';
                    hzgg[236] = hour % 10 + '0';
                    hzgg[237] = min / 10 + '0';
                    hzgg[238] = min % 10 + '0';
                    }
                if(tag==RI && aver<hwwdmin)
                    {
                    hzgg[404] = '0';
                    hzgg[405] = '0';
                    hwwdmin = aver;
                    asciitemp(hzgg, aver, 239);
                    hzgg[243] = hour / 10 + '0';
                    hzgg[244] = hour % 10 + '0';
                    hzgg[245] = min / 10 + '0';
                    hzgg[246] = min % 10 + '0';
                    }
                }
            else
                {
                gg[229] = tag + '0';
                tagnotRI(gg, 109);
                }
            hzgg[401] = gg[229];
            stringmove(gg, 109, hzgg, 227, 4);

            average(*samp061A011, 30);
//          fz061A011[0][min] = aver;
//          fz061A011[1][min] = tag;
//			for(x=0;x<2;x++)
//	        	{
//				for(y=0;y<30;y++)
//				printf("%d;",samp061A011[x][y]);
//				}
//			printf("\naver1=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST1, TEN, 9999);
            if(tag != MI)
                {
                gg[230] = tag + '0';
                asciitemp(gg, aver, 113);
				}
#ifndef ZZ090317
			else
                {
                gg[230] = tag + '0';
				tagnotRI(gg, 113);
                }
#endif

            average(*samp061A012, 30);
//          fz061A012[0][min] = aver;
//          fz061A012[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A012[x][y]);
//                }
//            printf("\naver2=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST2, TEN, 9999);
            if(tag != MI)
                {
                gg[231] = tag + '0';
                asciitemp(gg, aver, 117);
                }
#ifndef ZZ090317
            else
                {
                gg[231] = tag + '0';
                tagnotRI(gg, 117);
                }
#endif

            average(*samp061A013, 30);
//          fz061A013[0][min] = aver;
//          fz061A013[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A013[x][y]);
//                }
//            printf("\naver3=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST3, TEN, 9999);
            if(tag != MI)
                {
                gg[232] = tag + '0';
                asciitemp(gg, aver, 121);
                }
#ifndef ZZ090317
            else
                {
                gg[232] = tag + '0';
                tagnotRI(gg, 121);
                }
#endif

            average(*samp061A014, 30);
//          fz061A014[0][min] = aver;
//          fz061A014[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A014[x][y]);
//                }
//            printf("\naver4=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST4, TEN, 9999);
            if(tag != MI)
                {
                gg[233] = tag + '0';
                asciitemp(gg, aver, 125);
                }
#ifndef ZZ090317
            else
                {
                gg[233] = tag + '0';
                tagnotRI(gg, 125);
                }
#endif

            average(*samp061A021, 30);
//          fz061A021[0][min] = aver;
//          fz061A021[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A021[x][y]);
//                }
//            printf("\naver5=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST5, TEN, 9999);
            if(tag != MI)
                {
                gg[234] = tag + '0';
                asciitemp(gg, aver, 129);
                }
#ifndef ZZ090317
            else
                {
                gg[234] = tag + '0';
                tagnotRI(gg, 129);
                }
#endif

            average(*samp061A022, 30);
//          fz061A022[0][min] = aver;
//          fz061A022[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A022[x][y]);
//                }
//            printf("\naver6=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST6, TEN, 9999);
            if(tag != MI)
                {
                gg[235] = tag + '0';
                asciitemp(gg, aver, 133);
                }
#ifndef ZZ090317
            else
                {
                gg[235] = tag + '0';
                tagnotRI(gg, 133);
                }
#endif

            average(*samp061A023, 30);
//          fz061A023[0][min] = aver;
//          fz061A023[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A023[x][y]);
//                }
//            printf("\naver7=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST7, TEN, 9999);
            if(tag != MI)
                {
                gg[236] = tag + '0';
                asciitemp(gg, aver, 137);
                }
#ifndef ZZ090317
            else
                {
                gg[236] = tag + '0';
                tagnotRI(gg, 137);
                }
#endif

            average(*samp061A024, 30);
//          fz061A024[0][min] = aver;
//          fz061A024[1][min] = tag;
//            for(x=0;x<2;x++)
//                {
//                for(y=0;y<30;y++)
//                printf("%d;",samp061A024[x][y]);
//                }
//            printf("\naver8=%d,tag=%d\n",aver,tag);
//          printf("\naver=%d,tag=%d\n",qw1h[0][min],qw1h[1][min]);
            if(tag == RI)
                tag = qcinstance(aver, ST8, TEN, 9999);
            if(tag != MI)
                {
                gg[237] = tag + '0';
                asciitemp(gg, aver, 141);
                }
#ifndef ZZ090317
            else
                {
                gg[237] = tag + '0';
                tagnotRI(gg, 141);
                }
#endif

			stringmove(gg, 230, hzgg, 406, 8);
			stringmove(gg, 113, hzgg, 247, 32);

            average(*sampzf, 12);
            if(aver > 1000)
                aver = 1000;
            zfsw = 1000 - aver;
            if(tag == MI)
                {
                if(evap0==2000 || evap0==9999)
                    {
                    gg[238] = MI + '0';
                    tagnotRI(gg, 145);
                    gg[239] = MI + '0';
                    tagnotRI(gg, 149);
                    }
                else
                    {
                    gg[238] = MI + '0';
//                    tagnotRI(gg, 283);
//                  asciibyte4(gg, aver, 283);
                    asciibyte4(gg, zfsw0, 145);
                    gg[239] = RI + '0';
                    asciibyte4(gg, xszf, 149);
                    }
                }
            else
                {
                zfsw0 = zfsw;
                if(evap0==2000 || evap0==9999)
                    {
                    evap0 = aver;
                    xszf = 0;
                    xszf0 = 0;
                    }
                else
                    {
                    if(fzylzf == 0)
                        xszf = aver - evap0 + xsyl + zfcmp - xsyl0;
                    else
                        xszf = aver - evap0 + xsyl + zfcmp - xsyl0 - 1;
//                  zfhstag = abs(xszf - xszf0);                    // zf huan shui
                    zfhstag = abs(aver - zfaver0);
                    if(zfhstag > ZFABEND)
                        {
                        xszf = xszf0;
                        xsyl0 = xsyl;
                        evap0 = aver;
//                      zfcmp = zfcmp + xszf;
                        zfcmp = xszf0;
                        gg[238] = MI + '0';
//                      asciibyte4(gg, aver, 283);
                        asciibyte4(gg, zfsw, 145);
                        gg[239] = RI + '0';
                        asciibyte4(gg, xszf, 149);
                        }
                    else
                        {
                        if(xszf < 0)
                            xszf = 0;
                        if(xszf < xszf0)
                            xszf = xszf0;
                        else
                            xszf0 = xszf;
                        gg[238] = RI + '0';
//                      asciibyte4(gg, aver, 283);
                        asciibyte4(gg, zfsw, 145);
                        gg[239] = RI + '0';
                       asciibyte4(gg, xszf, 149);
                        }
                    }
                zfaver0 = aver;
                }
			stringmove(gg, 238, hzgg, 414, 2);
			stringmove(gg, 145, hzgg, 279, 8);
//			printf("aver=%d,tag=%d,evap0=%d,xszf=%d,xszf0=%d,zfcmp=0\n",aver,tag,evap0,xszf,xszf0,zfcmp);
//			printf("xsyl=%d,xsyl0=%d,fzylzf=%d\n",xsyl,xsyl0,fzylzf);
			asciibyte4(statzfsw, zfsw0, 0);
            if((fdstat = open("/usr/statzfsw.tmp",O_WRONLY|O_CREAT)) < 1)
				usleep(1);
			else
				{
            	write(fdstat, statzfsw, sizeof(statzfsw));
            	close(fdstat);
				}

//			printf("sampnjd[0][0]=%d,sampnjd[0][1]=%d,sampnjd[0][2]=%d,sampnjd[0][3]=%d\n",sampnjd[0][0],sampnjd[0][1],sampnjd[0][2],sampnjd[0][3]);
			average(*sampnjd, 4);
//			printf("aver=%d,tag=%d\n",aver,tag);
            if(tag == RI)
                tag = qcinstance(aver, VI, 1, 65535);
            if(tag != MI)
                {
                gg[240] = tag + '0';
				asciibyte5(gg, aver, 153);
				if(tag==RI && aver<njdmin)
					{
					njdmin = aver;
					hzgg[417] = tag + '0';
					hzgg[418] = tag + '0';
					stringmove(gg, 153, hzgg, 292, 5);
					hzgg[297] = hour / 10 + '0';
                    hzgg[298] = hour % 10 + '0';
                    hzgg[299] = min / 10 + '0';
                    hzgg[300] = min % 10 + '0';
					}
                }
            else
                {
                gg[240] = tag + '0';
				tagMIbyte5(gg, 153);
                }
			hzgg[416] = gg[240];
			stringmove(gg, 153, hzgg, 287, 5);

			gg[241] = MI + '0';
			tagMIbyte5(gg, 158);
            gg[242] = MI + '0';
			tagnotRI(gg,163);
            gg[243] = MI + '0';
            tagnotRI(gg,167);
            gg[244] = MI + '0';
            tagnotRI(gg,171);
            tagnotRI(gg,175);
            tagnotRI(gg,179);
            gg[245] = MI + '0';
            tagnotRI(gg,183);
            gg[246] = MI + '0';
            tagnotRI(gg,187);
            gg[247] = MI + '0';
            tagnotRI(gg,191);
            gg[248] = MI + '0';
            tagnotRI(gg,195);
            gg[249] = MI + '0';
            tagnotRI(gg,199);
			stringmove(gg, 241, hzgg, 419, 9);
			stringmove(gg, 158, hzgg, 301, 45);

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
                gg[203] = 'N';
                tagNbyte4(gg, 4);
                gg[205] = 'N';
                tagNbyte4(gg, 12);

                hzgg[346] = 'N';
                tagNbyte4(hzgg, 4);
                hzgg[348] = 'N';
                tagNbyte4(hzgg, 12);

                gg[207] = 'N';
                tagNbyte4(gg, 20);
				hzgg[350] = 'N';
				tagNbyte4(hzgg, 20);
                hzgg[353] = 'N';
                tagNbyte4(hzgg, 32);
				hzgg[355] = 'N';
				tagNbyte4(hzgg, 40);
                }

            if(senstate[WS]=='0')
                {
                gg[204] = 'N';
                tagNbyte4(gg, 8);
                gg[206] = 'N';
                tagNbyte4(gg, 16);
                gg[207] = 'N';
                tagNbyte4(gg, 20);
                gg[208] = 'N';
                tagNbyte4(gg, 24);

                hzgg[347] = 'N';
				tagNbyte4(hzgg, 8);
                hzgg[349] = 'N';
				tagNbyte4(hzgg, 16);
				hzgg[350] = 'N';
				hzgg[351] = 'N';
				hzgg[352] = 'N';
                tagNbyte4(hzgg, 20);
                tagNbyte4(hzgg, 24);
                tagNbyte4(hzgg, 28);
                hzgg[353] = 'N';
                tagNbyte4(hzgg, 32);
                hzgg[354] = 'N';
				tagNbyte4(hzgg, 36);
                hzgg[355] = 'N';
                hzgg[356] = 'N';
                hzgg[357] = 'N';
                tagNbyte4(hzgg, 40);
                tagNbyte4(hzgg, 44);
                tagNbyte4(hzgg, 48);
                }

            if(senstate[WS1]=='0')
                {
                gg[209] = 'N';
                tagNbyte4(gg, 28);
                gg[210] = 'N';
                tagNbyte4(gg, 32);
                gg[211] = 'N';
                tagNbyte4(gg, 36);

                hzgg[358] = 'N';
                tagNbyte4(hzgg, 52);
                hzgg[359] = 'N';
                tagNbyte4(hzgg, 56);
                hzgg[360] = 'N';
                hzgg[361] = 'N';
                tagNbyte4(hzgg, 60);
                tagNbyte4(hzgg, 64);
                hzgg[362] = 'N';
                tagNbyte4(hzgg, 68);
                hzgg[363] = 'N';
                hzgg[364] = 'N';
                tagNbyte4(hzgg, 72);
                tagNbyte4(hzgg, 76);
				}

            if(senstate[RAT]=='0')
                {
                gg[212] = 'N';
                tagNbyte4(gg, 40);
                gg[213] = 'N';
                tagNbyte4(gg, 44);

                hzgg[365] = 'N';
                tagNbyte4(hzgg, 80);
				}

            if(senstate[RAT1]=='0')
                {
                gg[214] = 'N';
                tagNbyte4(gg, 48);
                gg[215] = 'N';
                tagNbyte4(gg, 52);

                hzgg[366] = 'N';
                tagNbyte4(hzgg, 84);
                }

            if(senstate[RAW]=='0')
                {
                gg[216] = 'N';
                tagNbyte4(gg, 56);
                gg[217] = 'N';
                tagNbyte4(gg, 60);

                hzgg[367] = 'N';
                tagNbyte4(hzgg, 88);
                }

            if(senstate[T0]=='0')
                {
                gg[218] = 'N';
                tagNbyte4(gg, 64);

                hzgg[368] = 'N';
                hzgg[369] = 'N';
                hzgg[370] = 'N';
                hzgg[371] = 'N';
                hzgg[372] = 'N';
                tagNbyte4(hzgg, 92);
                tagNbyte4(hzgg, 96);
                tagNbyte4(hzgg, 100);
                tagNbyte4(hzgg, 104);
                tagNbyte4(hzgg, 108);

                gg[224] = 'N';
				gg[225] = 'N';
                tagNbyte4(gg, 88);
                tagNbyte4(gg, 92);
                hzgg[384] = 'N';
                hzgg[385] = 'N';
                tagNbyte4(hzgg, 156);
                tagNbyte4(hzgg, 160);
                }

            if(senstate[SV1]=='0' && senstate[SV2]=='0' && senstate[SV3]=='0')
                {
                gg[219] = 'N';
                tagNbyte4(gg, 68);

                hzgg[378] = 'N';
                tagNbyte4(hzgg, 132);
                }

            if(senstate[T1]=='0' && senstate[T2]=='0' && senstate[T3]=='0')
                {
                gg[220] = 'N';
                tagNbyte4(gg, 72);

                hzgg[373] = 'N';
                hzgg[374] = 'N';
                hzgg[375] = 'N';
                hzgg[376] = 'N';
                hzgg[377] = 'N';
                tagNbyte4(hzgg, 112);
                tagNbyte4(hzgg, 116);
                tagNbyte4(hzgg, 120);
                tagNbyte4(hzgg, 124);
                tagNbyte4(hzgg, 128);
                }

            if(senstate[TW]=='0')
                {
                gg[221] = 'N';
                tagNbyte4(gg, 76);

                hzgg[379] = 'N';
                tagNbyte4(hzgg, 136);
                }
			
            if(senstate[U]=='0')
                {
                gg[222] = 'N';
                tagNbyte4(gg, 80);
                gg[223] = 'N';
                tagNbyte4(gg, 84);

                hzgg[380] = 'N';
                hzgg[381] = 'N';
                hzgg[382] = 'N';
                hzgg[383] = 'N';
                tagNbyte4(hzgg, 140);
                tagNbyte4(hzgg, 144);
                tagNbyte4(hzgg, 148);
                tagNbyte4(hzgg, 152);

                gg[224] = 'N';
                gg[225] = 'N';
                tagNbyte4(gg, 88);
                tagNbyte4(gg, 92);
                hzgg[384] = 'N';
                hzgg[385] = 'N';
                tagNbyte4(hzgg, 156);
                tagNbyte4(hzgg, 160);
                }

            if(senstate[P]=='0')
                {
                gg[226] = 'N';
                tagNbyte5(gg, 96);

                hzgg[386] = 'N';
                hzgg[387] = 'N';
                hzgg[388] = 'N';
                hzgg[389] = 'N';
                hzgg[390] = 'N';
                tagNbyte5(hzgg, 164);
                tagNbyte5(hzgg, 169);
                tagNbyte4(hzgg, 174);
                tagNbyte5(hzgg, 178);
                tagNbyte4(hzgg, 183);
				}

			if(senstate[TG]=='0')
                {
                gg[227] = 'N';
                tagNbyte4(gg, 101);

                hzgg[391] = 'N';
                hzgg[392] = 'N';
                hzgg[393] = 'N';
                hzgg[394] = 'N';
                hzgg[395] = 'N';
                tagNbyte4(hzgg, 187);
                tagNbyte4(hzgg, 191);
                tagNbyte4(hzgg, 195);
                tagNbyte4(hzgg, 199);
                tagNbyte4(hzgg, 203);
                }

            if(senstate[ST0]=='0')
                {
                gg[228] = 'N';
                tagNbyte4(gg, 105);

                hzgg[396] = 'N';
                hzgg[397] = 'N';
                hzgg[398] = 'N';
                hzgg[399] = 'N';
                hzgg[400] = 'N';
                tagNbyte4(hzgg, 207);
                tagNbyte4(hzgg, 211);
                tagNbyte4(hzgg, 215);
                tagNbyte4(hzgg, 219);
                tagNbyte4(hzgg, 223);
                }

            if(senstate[IR]=='0')
                {
                gg[229] = 'N';
                tagNbyte4(gg, 109);

                hzgg[401] = 'N';
                hzgg[402] = 'N';
                hzgg[403] = 'N';
                hzgg[404] = 'N';
                hzgg[405] = 'N';
                tagNbyte4(hzgg, 227);
                tagNbyte4(hzgg, 231);
                tagNbyte4(hzgg, 235);
                tagNbyte4(hzgg, 239);
                tagNbyte4(hzgg, 243);
                }

            if(senstate[ST1]=='0')
                {
                gg[230] = 'N';
                tagNbyte4(gg, 113);

                hzgg[406] = 'N';
                tagNbyte4(hzgg, 247);
                }

            if(senstate[ST2]=='0')
                {
                gg[231] = 'N';
                tagNbyte4(gg, 117);

                hzgg[407] = 'N';
                tagNbyte4(hzgg, 251);
                }

            if(senstate[ST3]=='0')
                {
                gg[232] = 'N';
                tagNbyte4(gg, 121);

                hzgg[408] = 'N';
                tagNbyte4(hzgg, 255);
                }

            if(senstate[ST4]=='0')
                {
                gg[233] = 'N';
                tagNbyte4(gg, 125);

                hzgg[409] = 'N';
                tagNbyte4(hzgg, 259);
                }

            if(senstate[ST5]=='0')
                {
                gg[234] = 'N';
                tagNbyte4(gg, 129);

                hzgg[410] = 'N';
                tagNbyte4(hzgg, 263);
                }

            if(senstate[ST6]=='0')
                {
                gg[235] = 'N';
                tagNbyte4(gg, 133);

                hzgg[411] = 'N';
                tagNbyte4(hzgg, 267);
                }

            if(senstate[ST7]=='0')
                {
                gg[236] = 'N';
                tagNbyte4(gg, 137);

                hzgg[412] = 'N';
                tagNbyte4(hzgg, 271);
                }

            if(senstate[ST8]=='0')
                {
                gg[237] = 'N';
                tagNbyte4(gg, 141);

                hzgg[413] = 'N';
                tagNbyte4(hzgg, 275);
                }

            if(senstate[LE]=='0')
                {
                gg[238] = 'N';
                tagNbyte4(gg, 145);
                gg[239] = 'N';
                tagNbyte4(gg, 149);

                hzgg[414] = 'N';
                tagNbyte4(hzgg, 279);
				hzgg[415] = 'N';
                tagNbyte4(hzgg, 283);
                }

            if(senstate[VI]=='0')
                {
                gg[240] = 'N';
                tagNbyte5(gg, 153);

                hzgg[416] = 'N';
                hzgg[417] = 'N';
                hzgg[418] = 'N';
                tagNbyte5(hzgg, 287);
                tagNbyte5(hzgg, 292);
                tagNbyte4(hzgg, 297);
                }

            if(senstate[CH]=='0')
                {
                gg[241] = 'N';
                tagNbyte5(gg, 158);

                hzgg[419] = 'N';
                tagNbyte5(hzgg, 301);
                }

            if(senstate[TCA]=='0')
                {
                gg[242] = 'N';
                tagNbyte4(gg, 163);

                hzgg[420] = 'N';
                tagNbyte4(hzgg, 306);
                }

            if(senstate[LCA]=='0')
                {
                gg[243] = 'N';
                tagNbyte4(gg, 167);

                hzgg[421] = 'N';
                tagNbyte4(hzgg, 310);
                }

            if(senstate[WW]=='0')
                {
                gg[244] = 'N';
                tagNbyte4(gg, 171);
                tagNbyte4(gg, 175);
                tagNbyte4(gg, 179);

                hzgg[422] = 'N';
                tagNbyte4(hzgg, 314);
                tagNbyte4(hzgg, 318);
                tagNbyte4(hzgg, 322);
                }

            if(senstate[SD]=='0')
                {
                gg[245] = 'N';
                tagNbyte4(gg, 183);

                hzgg[423] = 'N';
                tagNbyte4(hzgg, 326);
                }

            if(senstate[FR]=='0')
                {
                gg[246] = 'N';
                tagNbyte4(gg, 187);

                hzgg[424] = 'N';
                tagNbyte4(hzgg, 330);
                }

            if(senstate[WI]=='0')
                {
                gg[247] = 'N';
                tagNbyte4(gg, 191);

                hzgg[425] = 'N';
                tagNbyte4(hzgg, 334);
                }

            if(senstate[FSD]=='0')
                {
                gg[248] = 'N';
                tagNbyte4(gg, 195);

                hzgg[426] = 'N';
                tagNbyte4(hzgg, 338);
                }

            if(senstate[LNF]=='0')
                {
                gg[249] = 'N';
                tagNbyte4(gg, 199);

                hzgg[427] = 'N';
                tagNbyte4(hzgg, 342);
                }
// end

            gg[0] = hour / 10 + '0';
            gg[1] = hour % 10 + '0';
            gg[2] = min / 10 + '0';
            gg[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/mingg.tmp",O_WRONLY|O_CREAT)) < 1)
            	{
                goto whileend;
                }
            write(fdfenzh, gg, sizeof(gg));
            close(fdfenzh);


        	rrtime = hour*60 + min + td;
        	if(rrtime < 0)
            	rrtime = rrtime + 1440;
            if(rrtime >= 1440)
                rrtime = rrtime - 1440;
        	rrhour = rrtime / 60;
        	rrmin = rrtime % 60;

			average(*samp051A001, 30);
            if(tag == RI)
                tag = qcinstance(aver, GR, 1, 9999);
			if(tag != MI)
				{
				hrrr[204] = tag + '0';
                asciibyte4(hrrr, aver, 4);
//				zfsbfl = (zfsbfl*10 + aver*60*10/100 + 5) / 10;
                floatzfsbfl = (floatzfsbfl + aver*60.0/1000000.0);
                zfsbfl = (floatzfsbfl + 0.005) * 100;
                hrrr[205] = tag + '0';
                asciibyte4(hrrr, zfsbfl, 8);
				if(tag==RI && aver>zfsmax)
					{
					zfsmax = aver;
					hrrr[206] = tag + '0';
                    hrrr[207] = tag + '0';
					stringmove(hrrr, 4, hrrr, 12, 4);
					hrrr[16] = rrhour / 10 + '0';
                    hrrr[17] = rrhour % 10 + '0';
                    hrrr[18] = rrmin / 10 + '0';
                    hrrr[19] = rrmin % 10 + '0';
					}
				}
			else
				{
				hrrr[204] = tag + '0';
				tagnotRI(hrrr, 4);
				}

            average(*samp051A024, 30);
            if(tag == RI)
                tag = qcinstance(aver, DR, 1, 9999);
            if(tag != MI)
                {
                hrrr[214] = tag + '0';
                asciibyte4(hrrr, aver, 44);
                floatzjfsbfl = (floatzjfsbfl + aver*60.0/1000000.0);
				zjfsbfl = (floatzjfsbfl + 0.005) * 100;
                hrrr[215] = tag + '0';
                asciibyte4(hrrr, zjfsbfl, 48);
                if(tag==RI && aver>zjfsmax)
                	{
                    zjfsmax = aver;
                    hrrr[216] = tag + '0';
                    hrrr[217] = tag + '0';
                    stringmove(hrrr, 44, hrrr, 52, 4);
                    hrrr[56] = rrhour / 10 + '0';
                    hrrr[57] = rrhour % 10 + '0';
                    hrrr[58] = rrmin / 10 + '0';
                    hrrr[59] = rrmin % 10 + '0';
                    }
                if(tag==RI && aver>=120)
                    {
                    xsrz++;
                    hrrr[227] = tag + '0';
                   	asciibyte4(hrrr, 1, 96);
                    }
                else
                    {
                    hrrr[227] = tag + '0';
                  	asciibyte4(hrrr, 0, 96);
                    }

                hrrr[218] = tag + '0';
                nianjr = yearahargana();
                jirxzz = (hour+min/60.0)/24.0 - locallong/360.0;
                tianswc = 79.6764 + 0.2422*(year+15) - (int)(0.25*(year+15));
//              printf("%f,%f,%f\n",nianjr,jirxzz,tianswc);
                tiansq = 2*PI*57.3*(nianjr+jirxzz-tianswc) / 365.2422 * PI / 180.0;
                chiwei = 0.3723 + 23.2567*sin(tiansq) + 0.1149*sin(2*tiansq) - 0.1712*sin(3*tiansq) - 0.7580*cos(tiansq) +
0.3656*cos(2*tiansq) + 0.0201*cos(3*tiansq);
                shicha = 0.0028 - 1.9857*sin(tiansq) + 9.9059*sin(2*tiansq) - 7.0924*cos(tiansq) - 0.6882*cos(2*tiansq);
//              printf("%f,%f,%f\n",tiansq,chiwei,shicha);
                zhentys = hour + min/60.0 + (locallong-120)*4/60.0 + shicha/60.0;
                taiysj = (zhentys-12) * 15;
//              printf("%f,%f,%f\n",zhentys, taiysj, locallat);
     			sinha = sin(locallat*PI/180.0)*sin(chiwei*PI/180.0) + cos(locallat*PI/180.0)*cos(chiwei*PI/180.0)*cos(taiysj*PI/180.0);
//              printf("%f\n",sinha);
                floatspmbfl = (floatspmbfl + aver*sinha*60.0/1000000.0);
				if(floatspmbfl < 0)
					floatspmbfl = 0;
                spmbfl = (floatspmbfl + 0.005) * 100;
                asciibyte4(hrrr, spmbfl, 60);
                }
            else
                {
                hrrr[214] = tag + '0';
               	tagnotRI(hrrr, 44);
				hrrr[227] = tag + '0';
                tagnotRI(hrrr, 96);
                }
			
			rr[108] = hrrr[204];
			rr[109] = hrrr[205];
			rr[112] = hrrr[214];
            rr[113] = hrrr[215];
            rr[114] = hrrr[218];
            rr[119] = hrrr[227];
			stringmove(hrrr, 4, rr, 4, 8);
            stringmove(hrrr, 44, rr, 20, 8);
            stringmove(hrrr, 60, rr, 28, 4);
            stringmove(hrrr, 96, rr, 48, 4);

// added by XGR at 2012-03-07 for hytest
            if(senstate[GR]=='0')
				{
				rr[108] = 'N';
                rr[109] = 'N';
				tagNbyte4(rr, 4);
                tagNbyte4(rr, 8);

                hrrr[204] = 'N';
                hrrr[205] = 'N';
                tagNbyte4(hrrr, 4);
                tagNbyte4(hrrr, 8);
				}

            if(senstate[DR]=='0')
                {
                rr[112] = 'N';
                rr[113] = 'N';
                rr[114] = 'N';
                tagNbyte4(rr, 20);
                tagNbyte4(rr, 24);
                tagNbyte4(rr, 28);

                hrrr[214] = 'N';
                hrrr[215] = 'N';
                hrrr[218] = 'N';
                tagNbyte4(hrrr, 44);
                tagNbyte4(hrrr, 48);
                tagNbyte4(hrrr, 60);

                rr[119] = 'N';
                tagNbyte4(rr, 48);
                hrrr[227] = 'N';
                tagNbyte4(hrrr, 96);
                }

            if(senstate[SSD]=='0')
                {
				rr[119] = 'N';
                tagNbyte4(rr, 48);
                hrrr[227] = 'N';
                tagNbyte4(hrrr, 96);
				}
// end

			if(rrhour==0 && rrmin==0)
				rrhour = 24;
            rr[0] = rrhour / 10 + '0';
            rr[1] = rrhour % 10 + '0';
            rr[2] = rrmin / 10 + '0';
            rr[3] = rrmin % 10 + '0';
            if((fdfenzh=open("/usr/minrr.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, rr, sizeof(rr));
            close(fdfenzh);

#ifdef TRSFCJQ
			average(*samp091A001, 6);
			hsm1[0][minary] = aver;
			hsm1[1][minary] = tag;
			if(tag != MI)
				{
				ss[40] = tag + '0';
				asciibyte4(ss, aver, 4);
				}
			else
				{
				ss[40] = tag + '0';
				tagnotRI(ss, 4);
                }

            average(*samp091A002, 6);
        	hsm2[0][minary] = aver;
        	hsm2[1][minary] = tag;
            if(tag != MI)
                {
                ss[41] = tag + '0';
                asciibyte4(ss, aver, 8);
                }
            else
                {
                ss[41] = tag + '0';
                tagnotRI(ss, 8);
                }

            average(*samp091A003, 6);
        	hsm3[0][minary] = aver;
        	hsm3[1][minary] = tag;
            if(tag != MI)
                {
                ss[42] = tag + '0';
                asciibyte4(ss, aver, 12);
                }
            else
                {
                ss[42] = tag + '0';
                tagnotRI(ss, 12);
                }

            average(*samp091A004, 6);
        	hsm4[0][minary] = aver;
        	hsm4[1][minary] = tag;
            if(tag != MI)
                {
                ss[43] = tag + '0';
                asciibyte4(ss, aver, 16);
                }
            else
                {
                ss[43] = tag + '0';
                tagnotRI(ss, 16);
                }

            average(*samp091A011, 6);
        	hsm5[0][minary] = aver;
        	hsm5[1][minary] = tag;
            if(tag != MI)
                {
                ss[44] = tag + '0';
                asciibyte4(ss, aver, 20);
                }
            else
                {
                ss[44] = tag + '0';
                tagnotRI(ss, 20);
                }

            average(*samp091A012, 6);
        	hsm6[0][minary] = aver;
        	hsm6[1][minary] = tag;
            if(tag != MI)
                {
                ss[45] = tag + '0';
                asciibyte4(ss, aver, 24);
                }
            else
                {
                ss[45] = tag + '0';
                tagnotRI(ss, 24);
                }

            average(*samp091A013, 6);
	        hsm7[0][minary] = aver;
    	    hsm7[1][minary] = tag;
            if(tag != MI)
                {
                ss[46] = tag + '0';
                asciibyte4(ss, aver, 28);
                }
            else
                {
                ss[46] = tag + '0';
                tagnotRI(ss, 28);
                }

            average(*samp091A014, 6);
        	hsm8[0][minary] = aver;
        	hsm8[1][minary] = tag;
            if(tag != MI)
                {
                ss[47] = tag + '0';
                asciibyte4(ss, aver, 32);
                }
            else
                {
                ss[47] = tag + '0';
                tagnotRI(ss, 32);
                }

			ss[48] = MI + '0';
			tagnotRI(ss, 36);

            ss[0] = hour / 10 + '0';
            ss[1] = hour % 10 + '0';
            ss[2] = min / 10 + '0';
            ss[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/minss.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, ss, sizeof(ss));
            close(fdfenzh);
#endif

// samp
            sampt[0] = hour / 10 + '0';
            sampt[1] = hour % 10 + '0';
            sampt[2] = min / 10 + '0';
            sampt[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampt.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampt, sizeof(sampt));
            close(fdfenzh);

            sampt1[0] = hour / 10 + '0';
            sampt1[1] = hour % 10 + '0';
            sampt1[2] = min / 10 + '0';
            sampt1[3] = min % 10 + '0';
            sampt1[124] = '\r';
            sampt1[125] = '\n';
            if((fdfenzh=open("/usr/sampt1.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampt1, sizeof(sampt1));
            close(fdfenzh);

            sampt2[0] = hour / 10 + '0';
            sampt2[1] = hour % 10 + '0';
            sampt2[2] = min / 10 + '0';
            sampt2[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampt2.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampt2, sizeof(sampt2));
            close(fdfenzh);

            sampt3[0] = hour / 10 + '0';
            sampt3[1] = hour % 10 + '0';
            sampt3[2] = min / 10 + '0';
            sampt3[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampt3.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampt3, sizeof(sampt3));
            close(fdfenzh);

            samprh[0] = hour / 10 + '0';
            samprh[1] = hour % 10 + '0';
            samprh[2] = min / 10 + '0';
            samprh[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/samprh.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, samprh, sizeof(samprh));
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

            sampws1[0] = hour / 10 + '0';
            sampws1[1] = hour % 10 + '0';
            sampws1[2] = min / 10 + '0';
            sampws1[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampws1.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampws1, sizeof(sampws1));
            close(fdfenzh);

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

            sampd[0] = hour / 10 + '0';
            sampd[1] = hour % 10 + '0';
            sampd[2] = min / 10 + '0';
            sampd[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampd.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampd, sizeof(sampd));
            close(fdfenzh);

            sampgr[0] = hour / 10 + '0';
            sampgr[1] = hour % 10 + '0';
            sampgr[2] = min / 10 + '0';
            sampgr[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampgr.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampgr, sizeof(sampgr));
            close(fdfenzh);

            sampdr[0] = hour / 10 + '0';
            sampdr[1] = hour % 10 + '0';
            sampdr[2] = min / 10 + '0';
            sampdr[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampdr.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampdr, sizeof(sampdr));
            close(fdfenzh);

            sampsm1[0] = hour / 10 + '0';
            sampsm1[1] = hour % 10 + '0';
            sampsm1[2] = min / 10 + '0';
            sampsm1[3] = min % 10 + '0';
            if((fdfenzh=open("/usr/sampsm1.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, sampsm1, sizeof(sampsm1));
            close(fdfenzh);


            if((fdfenzh=open("/usr/mintn.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdfenzh, cssj, sizeof(cssj));
            close(fdfenzh);


//			printf("senstate=%s\n",senstate);
//			senstate[1] = '0';
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
			if(zbtemp > 135)
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
			memset(sampt, '/', sizeof(sampt));
            memset(sampt1, '/', sizeof(sampt1));
            memset(sampt2, '/', sizeof(sampt2));
            memset(sampt3, '/', sizeof(sampt3));
			memset(sampd, '/', sizeof(sampd));
            memset(samprh, '/', sizeof(samprh));
            memset(sampgr, '/', sizeof(sampgr));
            memset(sampdr, '/', sizeof(sampdr));
            memset(sampwd, '/', sizeof(sampwd));
            memset(sampws, '/', sizeof(sampws));
            memset(sampws1, '/', sizeof(sampws1));
            memset(sampsm1, '/', sizeof(sampsm1));
*/
			mintag = 0;
			}	// if(sec == 0)


//		if(min==0 && sec==0)
		if(min==0 && sec==5)
			{
			stringmove(hzgg, 4, hz, 4, 426);
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

zzpause:
			xsyl = 0;
			xscc = 0;
//			xszf = 0;
			average(*sampzf, 12);
			evap0 = aver;
			xszf0 = 0;
			xsyl0 = 0;
			zfcmp = 0;
            zfsw0 = 1000;

        	fs1h3smax = -1;
			maxfs1hmin = 0;
			fs1h10mmax = -1;
			qhfs1h3smax = -1;
			maxqhfs1hmin = 0;
			qhfs1h10mmax = -1;

			qwmax   = -9999;
			qwmin   =  9999;
			fjwdmax = -9999;
			fjwdmin =  9999;
			sdmin   =  100;
			cmwdmax = -999;
			cmwdmin =  999;
			dmwdmax = -999;
			dmwdmin =  999;
			hwwdmax = -999;
			hwwdmin =  999;
			qiyamax =  0;
			qiyamin =  20000;
			njdmin  =  64000;

#ifdef TRSFCJQ
			hs[200] = ss[40];								// ti ji han shui lv
			stringmove(ss, 4, hs, 4, 4);
			average(*hsm1, 60);								// xiao shi ping jun tjhsl
			hs[201] = tag + '0';
			if(tag != MI)
				asciibyte4(hs, aver, 8);
			else
				tagnotRI(hs, 8);
            if(hs[200] == RI+'0')                           // xiang dui shi du
                {
                hs[202] = RI + '0';
                trxdsd = hsm1[0][59] / (sm1[1]*sm1[0]) * 100;
                asciibyte4(hs, trxdsd, 12);
                }
            else
                {
                hs[202] = MI + '0';
                tagnotRI(hs, 12);
                }
            if(hs[201] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[203] = RI + '0';
                trxdsd = aver / (sm1[1]*sm1[0]) * 100;
                asciibyte4(hs, trxdsd, 16);
                }
            else
                {
                hs[203] = MI + '0';
                tagnotRI(hs, 16);
                }
			if(hs[201] == RI+'0')							// zhong liang han shui lv
				{
				hs[204] = RI + '0';
				zlhsl = aver / sm1[1];
				asciibyte4(hs, zlhsl, 20);
				}
			else
				{
				hs[204] = MI + '0';
				tagnotRI(hs, 20);
				}
            if(hs[204] == RI+'0')							// you xiao shui fen zu cun liang
                {
                hs[205] = RI + '0';
                sfzcl = 10 * TCHD * sm1[1] * (zlhsl - sm1[2]) / 100;
				if(sfzcl < 0)
					sfzcl = 0;
                asciibyte4(hs, sfzcl, 24);
                }
            else
                {
                hs[205] = MI + '0';
                tagnotRI(hs, 24);
                }

			hs[206] = ss[41];								// ti ji han shui lv
			stringmove(ss, 8, hs, 28, 4);
			average(*hsm2, 60);								// xiao shi ping jun tjhsl
			hs[207] = tag + '0';
			if(tag != MI)
				asciibyte4(hs, aver, 32);
			else
				tagnotRI(hs, 32);
            if(hs[206] == RI+'0')                           // xiang dui shi du
                {
                hs[208] = RI + '0';
                trxdsd = hsm2[0][59] / (sm2[1]*sm2[0]) * 100;
                asciibyte4(hs, trxdsd, 36);
                }
            else
                {
                hs[208] = MI + '0';
                tagnotRI(hs, 36);
                }
            if(hs[207] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[209] = RI + '0';
                trxdsd = aver / (sm2[1]*sm2[0]) * 100;
                asciibyte4(hs, trxdsd, 40);
                }
            else
                {
                hs[209] = MI + '0';
                tagnotRI(hs, 40);
                }
			if(hs[207] == RI+'0')							// zhong liang han shui lv
				{
				hs[210] = RI + '0';
				zlhsl = aver / sm2[1];
				asciibyte4(hs, zlhsl, 44);
				}
			else
				{
				hs[210] = MI + '0';
				tagnotRI(hs, 44);
				}
            if(hs[210] == RI+'0')							// you xiao shui fen zu cun liang
                {
                hs[211] = RI + '0';
                sfzcl = 10 * TCHD * sm2[1] * (zlhsl - sm2[2]) / 100;
				if(sfzcl < 0)
					sfzcl = 0;
                asciibyte4(hs, sfzcl, 48);
                }
            else
                {
                hs[211] = MI + '0';
                tagnotRI(hs, 48);
                }

            hs[212] = ss[42];                           	// ti ji han shui lv
            stringmove(ss, 12, hs, 52, 4);
            average(*hsm3, 60);                         	// xiao shi ping jun tjhsl
            hs[213] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 56);
            else
                tagnotRI(hs, 56);
            if(hs[212] == RI+'0')                           // xiang dui shi du
                {
                hs[214] = RI + '0';
                trxdsd = hsm3[0][59] / (sm3[1]*sm3[0]) * 100;
                asciibyte4(hs, trxdsd, 60);
                }
            else
                {
                hs[214] = MI + '0';
                tagnotRI(hs, 60);
                }
            if(hs[213] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[215] = RI + '0';
                trxdsd = aver / (sm3[1]*sm3[0]) * 100;
                asciibyte4(hs, trxdsd, 64);
                }
            else
                {
                hs[215] = MI + '0';
                tagnotRI(hs, 64);
                }
            if(hs[213] == RI+'0')                           // zhong liang han shui lv
                {
                hs[216] = RI + '0';
                zlhsl = aver / sm3[1];
                asciibyte4(hs, zlhsl, 68);
                }
            else
                {
                hs[216] = MI + '0';
                tagnotRI(hs, 68);
                }
            if(hs[216] == RI+'0')                           // you xiao shui fen zu cun liang
                {
                hs[217] = RI + '0';
                sfzcl = 10 * TCHD * sm3[1] * (zlhsl - sm3[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 72);
                }
            else
                {
                hs[217] = MI + '0';
                tagnotRI(hs, 72);
                }
			
            hs[218] = ss[43];                           	// ti ji han shui lv
            stringmove(ss, 16, hs, 76, 4);
            average(*hsm4, 60);                         	// xiao shi ping jun tjhsl
            hs[219] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 80);
            else
                tagnotRI(hs, 80);
            if(hs[218] == RI+'0')                           // xiang dui shi du
                {
                hs[220] = RI + '0';
                trxdsd = hsm4[0][59] / (sm4[1]*sm4[0]) * 100;
                asciibyte4(hs, trxdsd, 84);
                }
            else
                {
                hs[220] = MI + '0';
                tagnotRI(hs, 84);
                }
            if(hs[219] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[221] = RI + '0';
                trxdsd = aver / (sm4[1]*sm4[0]) * 100;
                asciibyte4(hs, trxdsd, 88);
                }
            else
                {
                hs[221] = MI + '0';
                tagnotRI(hs, 88);
                }
            if(hs[219] == RI+'0')                           // zhong liang han shui lv
                {
                hs[222] = RI + '0';
                zlhsl = aver / sm4[1];
                asciibyte4(hs, zlhsl, 92);
                }
            else
                {
                hs[222] = MI + '0';
                tagnotRI(hs, 92);
                }
            if(hs[222] == RI+'0')                           // you xiao shui fen zu cun liang
                {
                hs[223] = RI + '0';
                sfzcl = 10 * TCHD * sm4[1] * (zlhsl - sm4[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 96);
                }
            else
                {
                hs[223] = MI + '0';
                tagnotRI(hs, 96);
                }

            hs[224] = ss[44];                           	// ti ji han shui lv
            stringmove(ss, 20, hs, 100, 4);
            average(*hsm5, 60);                         	// xiao shi ping jun tjhsl
            hs[225] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 104);
            else
                tagnotRI(hs, 104);
            if(hs[224] == RI+'0')                           // xiang dui shi du
                {
                hs[226] = RI + '0';
                trxdsd = hsm5[0][59] / (sm5[1]*sm5[0]) * 100;
                asciibyte4(hs, trxdsd, 108);
                }
            else
                {
                hs[226] = MI + '0';
                tagnotRI(hs, 108);
                }
            if(hs[225] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[227] = RI + '0';
                trxdsd = aver / (sm5[1]*sm5[0]) * 100;
                asciibyte4(hs, trxdsd, 112);
                }
            else
                {
                hs[227] = MI + '0';
                tagnotRI(hs, 112);
                }
            if(hs[225] == RI+'0')                           // zhong liang han shui lv
                {
                hs[228] = RI + '0';
                zlhsl = aver / sm5[1];
                asciibyte4(hs, zlhsl, 116);
                }
            else
                {
                hs[228] = MI + '0';
                tagnotRI(hs, 116);
                }
            if(hs[228] == RI+'0')                           // you xiao shui fen zu cun liang
                {
                hs[229] = RI + '0';
                sfzcl = 10 * TCHD * sm5[1] * (zlhsl - sm5[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 120);
                }
            else
                {
                hs[229] = MI + '0';
                tagnotRI(hs, 120);
                }

            hs[230] = ss[45];                           	// ti ji han shui lv
            stringmove(ss, 24, hs, 124, 4);
            average(*hsm6, 60);                         	// xiao shi ping jun tjhsl
            hs[231] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 128);
            else
                tagnotRI(hs, 128);
            if(hs[230] == RI+'0')                           // xiang dui shi du
                {
                hs[232] = RI + '0';
                trxdsd = hsm6[0][59] / (sm6[1]*sm6[0]) * 100;
                asciibyte4(hs, trxdsd, 132);
                }
            else
                {
                hs[232] = MI + '0';
                tagnotRI(hs, 132);
                }
            if(hs[231] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[233] = RI + '0';
                trxdsd = aver / (sm6[1]*sm6[0]) * 100;
                asciibyte4(hs, trxdsd, 136);
                }
            else
                {
                hs[233] = MI + '0';
                tagnotRI(hs, 136);
                }
            if(hs[231] == RI+'0')                           // zhong liang han shui lv
                {
                hs[234] = RI + '0';
                zlhsl = aver / sm6[1];
                asciibyte4(hs, zlhsl, 140);
                }
            else
                {
                hs[234] = MI + '0';
                tagnotRI(hs, 140);
                }
            if(hs[234] == RI+'0')                           // you xiao shui fen zu cun liang
                {
                hs[235] = RI + '0';
                sfzcl = 10 * TCHD * sm6[1] * (zlhsl - sm6[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 144);
                }
            else
                {
                hs[235] = MI + '0';
                tagnotRI(hs, 144);
                }

            hs[236] = ss[46];                           	// ti ji han shui lv
            stringmove(ss, 28, hs, 148, 4);
            average(*hsm7, 60);                         	// xiao shi ping jun tjhsl
            hs[237] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 152);
            else
                tagnotRI(hs, 152);
            if(hs[236] == RI+'0')                           // xiang dui shi du
                {
                hs[238] = RI + '0';
                trxdsd = hsm7[0][59] / (sm7[1]*sm7[0]) * 100;
                asciibyte4(hs, trxdsd, 156);
                }
            else
                {
                hs[238] = MI + '0';
                tagnotRI(hs, 156);
                }
            if(hs[237] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[239] = RI + '0';
                trxdsd = aver / (sm7[1]*sm7[0]) * 100;
                asciibyte4(hs, trxdsd, 160);
                }
            else
                {
                hs[239] = MI + '0';
                tagnotRI(hs, 160);
                }
            if(hs[237] == RI+'0')                           // zhong liang han shui lv
                {
                hs[240] = RI + '0';
                zlhsl = aver / sm7[1];
                asciibyte4(hs, zlhsl, 164);
                }
            else
                {
                hs[240] = MI + '0';
                tagnotRI(hs, 164);
                }
            if(hs[240] == RI+'0')                           // you xiao shui fen zu cun liang
                {
                hs[241] = RI + '0';
                sfzcl = 10 * TCHD * sm7[1] * (zlhsl - sm7[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 168);
                }
            else
                {
                hs[241] = MI + '0';
                tagnotRI(hs, 168);
                }

            hs[242] = ss[47];                           	// ti ji han shui lv
            stringmove(ss, 32, hs, 172, 4);
            average(*hsm8, 60);                         	// xiao shi ping jun tjhsl
            hs[243] = tag + '0';
            if(tag != MI)
                asciibyte4(hs, aver, 176);
            else
                tagnotRI(hs, 176);
            if(hs[242] == RI+'0')                           // xiang dui shi du
                {
                hs[244] = RI + '0';
                trxdsd = hsm8[0][59] / (sm8[1]*sm8[0]) * 100;
                asciibyte4(hs, trxdsd, 180);
                }
            else
                {
                hs[244] = MI + '0';
                tagnotRI(hs, 180);
                }
            if(hs[243] == RI+'0')                           // xiao shi pin jun trxdsd
                {
                hs[245] = RI + '0';
                trxdsd = aver / (sm8[1]*sm8[0]) * 100;
                asciibyte4(hs, trxdsd, 184);
                }
            else
                {
                hs[245] = MI + '0';
                tagnotRI(hs, 184);
                }
            if(hs[243] == RI+'0')                           // zhong liang han shui lv
                {
                hs[246] = RI + '0';
                zlhsl = aver / sm8[1];
                asciibyte4(hs, zlhsl, 188);
                }
            else
                {
                hs[246] = MI + '0';
                tagnotRI(hs, 188);
                }
           if(hs[246] == RI+'0')							// you xiao shui fen zu cun liang
                {
                hs[247] = RI + '0';
                sfzcl = 10 * TCHD * sm8[1] * (zlhsl - sm8[2]) / 100;
                if(sfzcl < 0)
                    sfzcl = 0;
                asciibyte4(hs, sfzcl, 192);
                }
            else
                {
                hs[247] = MI + '0';
                tagnotRI(hs, 192);
                }

            hs[248] = ss[48];                           	// di xia shui wei
            stringmove(ss, 36, hs, 196, 4);

            hs[0] = mday / 10 + '0';
            hs[1] = mday % 10 + '0';
	        hs[2] = hour / 10 + '0';
            hs[3] = hour % 10 + '0';
            if((fdxiaos=open("/usr/hourss.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
                }
            write(fdxiaos, hs, sizeof(hs));
            close(fdxiaos);
#endif
			}	// if(min==0 && sec==0)

        if(min==localt && sec==5)
            {
			timeconvert(loctime, (td*60));
			stringmove(loctime, 4, hr, 0, 4);
			stringmove(hrrr, 4, hr, 4, 252);
            if((fdxiaos=open("/usr/hourrr.tmp",O_WRONLY|O_CREAT)) < 1)
                {
                goto whileend;
            	}
            write(fdxiaos, hr, sizeof(hr));
            close(fdxiaos);

rrpause:
			zfsbfl = 0;
			zjfsbfl = 0;
			spmbfl = 0;
			floatzfsbfl = 0;
			floatzjfsbfl = 0;
			floatspmbfl = 0;
			xsrz = 0;
			zfsmax = -1;
			zjfsmax = -1;
//			memset(&rain, '0', sizeof(rain));		
			}

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

