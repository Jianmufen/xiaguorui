/*
 * pc485.c 2012-08-01 17:00 V6.5 for 青岛 RS232输出
 * source from pc485.c 2012-06-27 16:00 V6.4 for 青岛 RS232输出
 * 忽略大小写
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

#include <sys/ioctl.h>
#include <linux/rtc.h>
#include <string.h>


#define R485_STA_CTL1   1
#define WDT_STA_CTL     2
#define TX	1
#define RX  0
#define GTIME   900000              // 360-540
#define LTIME	600000				// 180-360
#define MTIME   300000              // 60-180
#define STIME   100000              // 0-60

#define DEV_RTC "/dev/rtc"

#define UART0   "/dev/ttyAT1"
#define UART1   "/dev/ttyAT2"
#define UART2   "/dev/ttyAT3"
#define SERIAL0 "/dev/ttyS0"
#define SERIAL1 "/dev/ttyS1"
#define SERIAL2 "/dev/ttyS2"
#define SERIAL3 "/dev/ttyS3"

#define P   0
#define T0  1
#define T1  2
#define T2  3
#define T3  4
#define TW	5
#define U   6
#define TD  7
#define SV1	8
#define SV2	9
#define SV3	10
#define WD  11
#define WS  12
#define WS1 13
#define RAT 14
#define RAT1 15
#define RAW 16
#define TG  17
#define IR  18
#define ST0 19
#define ST1 20
#define ST2 21
#define ST3 22
#define ST4 23
#define ST5 24
#define ST6 25
#define ST7 26
#define ST8 27
#define LE  28
#define VI  29
#define CH	30
#define TCA 31
#define LCA 32
#define WW	33
#define SD	34
#define FR	35
#define WI	36
#define FSD 37
#define LNF 38
#define GR  39
#define NR  40
#define DR  41
#define SR	42
#define RR  43
#define UR  44
#define UVA 45
#define UVB 46
#define AR	47
#define ART 48
#define TR  49
#define TRT 50
#define PR  51
#define SSD	52
#define SM1 53
#define SM2 54
#define SM3 55
#define SM4 56
#define SM5 57
#define SM6 58
#define SM7 59
#define SM8 60
#define WT  61
#define BA  62
#define OT  63
#define OS  64
#define OC	65
#define OH	66
#define OP  67
#define OD	68
#define OV  69
#define TL  70
#define OTU 71
#define OCC 72
#define WS2 73
#define WS3 74

#define RECNUMHOUR      744
#define RECNUMMIN       1440
#define BYTENUMMZ       252
#define BYTENUMMS       51
#define BYTENUMMR       136
#define BYTENUMMO		138
#define BYTENUMHZ       430
#define BYTENUMHS       251
#define BYTENUMHR       256

#define BYTEDMGD		357
#define BYTEDMCD		108
#define BYTEDMSD		94
#define BYTEDMRD		213
#define BYTEDHGD        518
#define BYTEDHCD        203
#define BYTEDHSD		371
#define BYTEDHRD        378

#define BYTEDZTN    	14
#define BYTEDZSTN   	32
#define BYTEDZRAN   	11
#define BYTEDZSENSI		8

#define SAMPLE120       126
#define SAMPLE240       246
#define SAMPLE960       966
#define SAMPLEX30  		183
#define SAMPLEXX30		184
#define SAMPLEXX60      334
#define SAMPLEXX240     1234
#define SAMPLEXXX30     185
#define BYTEQCPS		20
#define BYTEQCPM        30
#define BYTESCV			100

#define SDELAY		1
#define MDELAY		2
#define LDELAY		3

#define RAIN0000	1


static int fd_485=-1, fd_rtc=-1;
struct rtc_time rtc_tm;
static char instime[]={"date 123120002008.00"};

static char stid[7];
static int portfd;
static int autotime;

static struct termios newtios, oldtios;		/* terminal settings */
static int saved_portfd = -1;           /* serial port fd */

/* cleanup atexit handler */
static void reset_tty_atexit(void)
{
	if(saved_portfd != -1){
		tcsetattr(saved_portfd, TCSANOW, &oldtios);
	}
}

/* cleanup signal handler */
static void reset_tty_handler(int signal)
{
	if(saved_portfd != -1){
		tcsetattr(saved_portfd, TCSANOW, &oldtios);
	}
	_exit(EXIT_FAILURE);
}

static int open_port(const char *portname, int baud)
{
	struct sigaction sa;
	int portfd;

	printf("opening serial port: %s\n", portname);

	/* open serial port */
	if((portfd = open(portname, O_RDWR|O_NOCTTY)) < 0){
		printf("open serial port %s fail\n", portname);
		return portfd;
	}

	/* get serial port params, save away */
	tcgetattr(portfd, &newtios);
	memcpy(&oldtios, &newtios, sizeof newtios);

	/* configure new values */
	cfmakeraw(&newtios);            /* see man page */
	newtios.c_iflag |= IGNPAR;      /* ignore parity on input */
	newtios.c_oflag &= ~(OPOST|ONLCR|OLCUC|OCRNL|ONOCR|ONLRET|OFILL);
	newtios.c_cflag = CS8|CLOCAL|CREAD;
	newtios.c_cc[VMIN] = 1;         /* block until 1 char received */
	newtios.c_cc[VTIME] = 0;        /* no inter-character timer */
	
	/* 115200 bps */
	switch(baud)
	{
	case 1:	cfsetospeed(&newtios, B300);	   cfsetispeed(&newtios, B300);      break;
        case 2: cfsetospeed(&newtios, B1200);      cfsetispeed(&newtios, B1200);     break;
        case 3: cfsetospeed(&newtios, B2400);      cfsetispeed(&newtios, B2400);     break;
        case 4: cfsetospeed(&newtios, B4800);      cfsetispeed(&newtios, B4800);     break;
        case 5: cfsetospeed(&newtios, B9600);      cfsetispeed(&newtios, B9600);     break;
        case 6: cfsetospeed(&newtios, B19200);     cfsetispeed(&newtios, B19200);    break;
        case 7: cfsetospeed(&newtios, B38400);     cfsetispeed(&newtios, B38400);    break;
        case 8: cfsetospeed(&newtios, B57600);     cfsetispeed(&newtios, B57600);    break;
        case 9: cfsetospeed(&newtios, B115200);    cfsetispeed(&newtios, B115200);   break;
	default: usleep(1);
	}

	/* register cleanup stuff  */
	atexit(reset_tty_atexit);
	memset(&sa, 0, sizeof sa);
	sa.sa_handler = reset_tty_handler;
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	/* apply modified termios */
	saved_portfd = portfd;
	tcflush(portfd, TCIFLUSH);
	tcsetattr(portfd, TCSADRAIN, &newtios);

	return portfd;
}

static void close_port(int portfd)
{
	tcsetattr(portfd, TCSANOW, &oldtios);
	close(portfd);
	saved_portfd = -1;
}


static void stringmove(char *from, int i, char *to, int j, int k)
{       //int i, j, k, m, n;
        //char from[m], to[n];
        for( ;k>0;k--){
                to[j]=from[i];
                i++;
                j++;
        }
}

static void string_add_space(char *from, int i, char *to, long j, int k, int n)
{
        int m=k;
        for( ;n>0;n--){
                for( ;k>0;k--){
                        to[j]=from[i];
                        j++;
                        i++;
                }
                to[j]=' ';
                j++;
                k=m;
        }
}

static void string_change_1(char *from, int m, int n)
{
    int i=0, j=0, l=0, k;
//	int count=0;
//	printf("m=%d,n=%d\n",m,n);
    for(;i<n;)
        {
        if(from[m+i] == '~')
            {
//            count++;
            k = n-i-1;
            j = 0;
            l = m+i;
            for( ;j<k+1;j++)
                {
                from[l] = from[l+1];
                l++;
                }
//			printf("from=%s\n",from);
            }
        else
            {
            i++;
            }
        }
//  printf("from=%s",from);
//    if(count != 0)
//        memset(from+m+n-count, ' ', count);
//    return(count);
}

static int string_change_2(char *from, int n)
{
	int i=0, j=0, m=0, k, count=0;
//	n = n-1;
	for( ;i<n; )
		{
		if((from[i] == ' ') && (from[i+1]==' ' || from[i+1]=='\r'))
			{
			count++;
			k = n-i-1;
			j = 0;
			m = i;
			for( ;j<k;j++)
				{
				from[m] = from[m+1];
				m++;
				}
			from[m+1] = '\0';
			}
		else
			{
			i++;
			}
		}
//	printf("from=%s",from);
	return(count);
}

static void datamove(char *datatime, char *insdata, int n)
{
	datatime[0] = insdata[n];
        datatime[1] = insdata[n+1];
        datatime[2] = insdata[n+3];
        datatime[3] = insdata[n+4];
        datatime[4] = insdata[n+6];
        datatime[5] = insdata[n+7];
        datatime[6] = insdata[n+9];
        datatime[7] = insdata[n+10];
        datatime[8] = insdata[n+12];
        datatime[9] = insdata[n+13];
        datatime[10] = '0';
        datatime[11] = '0';
}

static void datamovexs(char *datatime, char *insdata, int n)
{
        datatime[0] = insdata[n];
        datatime[1] = insdata[n+1];
        datatime[2] = insdata[n+3];
        datatime[3] = insdata[n+4];
        datatime[4] = insdata[n+6];
        datatime[5] = insdata[n+7];
        datatime[6] = insdata[n+9];
        datatime[7] = insdata[n+10];
        datatime[8] = '0';
        datatime[9] = '0';
        datatime[10] = '0';
        datatime[11] = '0';
}

static long fenzhong_bianhao(char *string)
{
        int temp1, temp2, id;
        temp1 = (string[6] - '0') * 10 + (string[7] - '0');
        temp2 = (string[8] - '0') * 10 + (string[9] - '0');
        id = temp1 * 60 + temp2;
        return(id);
}

static long xiaoshi_bianhao(char *string)
{
        int temp1, temp2, id;
        temp1 = (string[4] - '0') * 10 + (string[5] - '0') - 1;
        temp2 = (string[6] - '0') * 10 + (string[7] - '0');
        id = temp1 * 24 + temp2;
        return(id);
}

static long sample_bianhao(char *string, int n)
{
        int temp1, temp2, id;
        temp1 = (string[n-5] - '0') * 10 + (string[n-4] - '0');
        temp2 = (string[n-2] - '0') * 10 + (string[n-1] - '0');
        id = temp1 * 60 + temp2;
		if(id == 0)
			id = 1440;
        return(id);
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

static long timedistance(char *string)
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
	return(timep);
}

static long datetosecond()
{
        time_t timep;
        struct tm *p;
        time(&timep);
        p = localtime(&timep);
        timep = mktime(p);
    return(timep);
}

static void stationid(char *string)
    {	
    int fdstbd;

    if((fdstbd = open("/tmp/paramete/stid.txt",O_RDONLY))< 1)
	{
	string[0] = '0';
        string[1] = '0';
        string[2] = '0';
        string[3] = '0';
        string[4] = '0';
	}
    else
	{
	read(fdstbd, string, 5);
//	printf("id=%s\n",stidd);
    	close(fdstbd);
	}
    }

static void filetime(char *string, int td)
    {
    time_t timep;
    struct tm *p;
    int sec, min, hour, mday, mon, year;

    time(&timep);
    p = localtime(&timep);
    timep = mktime(p);
    timep = timep + td*60;
    p = localtime(&timep);

    if(p->tm_year >= 100)
		{
		string[0] = '2';
		string[1] = '0';
		}
    else
        {
        string[0] = '1';
        string[1] = '9';
        }
    string[2] = (p->tm_year % 100) / 10 + '0';
    string[3] = (p->tm_year) % 10 + '0';
    string[4] = '-';
    string[5] = (p->tm_mon + 1) / 10 + '0';
    string[6] = (p->tm_mon + 1) % 10 + '0';
    string[7] = '-';
    string[8] = (p->tm_mday) / 10 + '0';
    string[9] = (p->tm_mday) % 10 + '0';
    string[10] = ' ';
    string[11] = (p->tm_hour) / 10 + '0';
    string[12] = (p->tm_hour) % 10 + '0';
    string[13] = ':';
    string[14] = (p->tm_min) / 10 + '0';
    string[15] = (p->tm_min) % 10 + '0';
    string[16] = ':';
    string[17] = (p->tm_sec) / 10 + '0';
    string[18] = (p->tm_sec) % 10 + '0';
    }

static void rs485clt(int fd, int pin, int sta)
    {
    unsigned char databuf[50];
    databuf[0] = pin;
    databuf[1] = sta;
    write(fd, databuf, 2);
    }

static void returnTorF(int portfd, char *returntf, int ret)
	{
    if(ret > 0)
    	returntf[0] = 'T';
    else
        returntf[0] = 'F';
    returntf[1] = '\r';
    returntf[2] = '\n';
    rs485clt(fd_485, R485_STA_CTL1, TX);
	write(portfd, returntf, 3);
    usleep(STIME);
    rs485clt(fd_485, R485_STA_CTL1, RX);
	}

static void returncommand(int portfd, char *command, int size, int delay)
    {
    rs485clt(fd_485, R485_STA_CTL1, TX);
    write(portfd, command, size);
	if(delay == SDELAY)
    	usleep(STIME);
	else if(delay == MDELAY)
		usleep(MTIME);
	else
		usleep(LTIME);
    rs485clt(fd_485, R485_STA_CTL1, RX);
    }

/*static char comreturn(int portfd, char t)
{
        FD_CLR(portfd, &fds);
        FD_SET(portfd, &fds);
        rtimeout.tv_sec = 1;
        rtimeout.tv_usec = 0;
        retval = select(portfd + 1, &fds, NULL, NULL, rstimeout);

	return(t);	
}
*/

/*
static int fd_ttyS0=-1;
static void* gpspthread(void *t)
//static void gpsfunction()
    {
    int i;
    char c;
    char gps[13];
    int ret;
    struct rtc_time rtc_tm;

    while(1)
        {
        read(fd_ttyS0, &c, 1);
//      getchar(c);
        if(c == 0x62)
            {
            for(i=0;i<13;i++)
                {
                read(fd_ttyS0, &c, 1);
                gps[i] = c;
//                printf("%2x;",c);
                }
            gps[i] = '\0';

			fd_rtc = open(DEV_RTC, O_RDWR);
        	if(fd_rtc == -1)
				printf("open RTC error!\n");
			else
				{
//      		ret = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
	    		rtc_tm.tm_year = gps[2]*256 + gps[3] - 1900;
	        	rtc_tm.tm_mon  = gps[0] - 1;   
				rtc_tm.tm_mday = gps[1];
				rtc_tm.tm_hour = gps[4];
				rtc_tm.tm_min  = gps[5];
				rtc_tm.tm_sec  = gps[6];
//				fprintf(stderr, "\n\nCurrent RTC date/time is %d-%d-%d, %02d:%02d:%02d.\n",rtc_tm.tm_mday, rtc_tm.tm_mon + 1, rtc_tm.tm_year + 1900,rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);

				ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
//				printf("retset=%d\n",ret);
//				ret = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
//            	printf("retrd=%d\n",ret);
//    			fprintf(stderr, "\n\nCurrent RTC date/time is %d-%d-%d, %02d:%02d:%02d.\n",rtc_tm.tm_mday, rtc_tm.tm_mon + 1, rtc_tm.tm_year + 1900,rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
				close(fd_rtc);

    	    	instime[5]  = (rtc_tm.tm_mon + 1) / 10 + '0';
				instime[6]  = (rtc_tm.tm_mon + 1) % 10 + '0';
				instime[7]  = (rtc_tm.tm_mday) / 10 + '0';
				instime[8]  = (rtc_tm.tm_mday) % 10 + '0';
				instime[9]  = (rtc_tm.tm_hour) / 10 + '0';
				instime[10] = (rtc_tm.tm_hour) % 10 + '0';
				instime[11] = (rtc_tm.tm_min) / 10 + '0';
				instime[12] = (rtc_tm.tm_min) % 10 + '0';
				instime[13] = (rtc_tm.tm_year + 1900) / 1000 + '0';
				instime[14] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
				instime[15] = (rtc_tm.tm_year - 100) / 10 + '0';
				instime[16] = (rtc_tm.tm_year - 100) % 10 + '0';
				instime[17] = '.';
				instime[18] = (rtc_tm.tm_sec) / 10 + '0';
				instime[19] = (rtc_tm.tm_sec) % 10 + '0';
				instime[20] = '\0';
//				printf("instime=%s\n",instime);
				system(instime);
				}
			fflush(stdout);
			}
		}
    }
*/

static int fd_ttyS0=-1;
static void* gpspthread(void *t)
    {
    int i;
    char c;
    char gps[18];
    int ret;
    struct rtc_time rtc_tm;

    while(1)
        {
        read(fd_ttyS0, &c, 1);
		printf("%c",c);
        if(c == '<')
            {
			memset(&gps, '9', sizeof(gps));
            for(i=0;i<18 && c!='>';i++)
                {
                read(fd_ttyS0, &c, 1);
                gps[i] = c;
                printf("%c",c);
                }
            gps[i] = '\0';

			if(gps[16] == 'A')
			{
            fd_rtc = open(DEV_RTC, O_RDWR);
            if(fd_rtc == -1)
                printf("open RTC error!\n");
            else
                {
                rtc_tm.tm_year = (gps[4]-'0')*10 + (gps[5]-'0') + 100;
                rtc_tm.tm_mon  = (gps[6]-'0')*10 + (gps[7]-'0') - 1;
                rtc_tm.tm_mday = (gps[8]-'0')*10 + (gps[9]-'0');
                rtc_tm.tm_hour = (gps[10]-'0')*10 + (gps[11]-'0');
                rtc_tm.tm_min  = (gps[12]-'0')*10 + (gps[13]-'0');
                rtc_tm.tm_sec  = (gps[14]-'0')*10 + (gps[15]-'0');
				if((rtc_tm.tm_year>111&&rtc_tm.tm_year<133) && (rtc_tm.tm_mon>=0&&rtc_tm.tm_mon<12) && (rtc_tm.tm_mday<32) && (rtc_tm.tm_hour<24) && (rtc_tm.tm_min<60) && (rtc_tm.tm_sec<60))
                	{
					ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
                	close(fd_rtc);

                	instime[5]  = (rtc_tm.tm_mon + 1) / 10 + '0';
                	instime[6]  = (rtc_tm.tm_mon + 1) % 10 + '0';
                	instime[7]  = (rtc_tm.tm_mday) / 10 + '0';
                	instime[8]  = (rtc_tm.tm_mday) % 10 + '0';
                	instime[9]  = (rtc_tm.tm_hour) / 10 + '0';
                	instime[10] = (rtc_tm.tm_hour) % 10 + '0';
                	instime[11] = (rtc_tm.tm_min) / 10 + '0';
                	instime[12] = (rtc_tm.tm_min) % 10 + '0';
                	instime[13] = (rtc_tm.tm_year + 1900) / 1000 + '0';
                	instime[14] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
                	instime[15] = (rtc_tm.tm_year - 100) / 10 + '0';
                	instime[16] = (rtc_tm.tm_year - 100) % 10 + '0';
                	instime[17] = '.';
                	instime[18] = (rtc_tm.tm_sec) / 10 + '0';
                	instime[19] = (rtc_tm.tm_sec) % 10 + '0';
                	instime[20] = '\0';
//              	printf("instime=%s\n",instime);
                	system(instime);
					}
				else
					close(fd_rtc);
                }
			}
            fflush(stdout);
            }
        }
    }

static void* autosendpthread(void *t)
	{
    time_t timep;
    struct tm *p;
	int sec, min, hour, mday, mon, year;
//	int autotime=0;
	char data[1000], csdata[1000];;
	char presenttime[19];
	FILE *fbcs;
	char sstime[5], wjtime[5];
    int sametime=0;
	int fdmaste;
	int x, recount;
	int autotag=0;
	char senstate[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};
	char pss[9];
	char insdmgd[4]={"DMGD"};
    char insdhgd[4]={"DHGD"};

	while(1)
		{
        time(&timep);
        p = gmtime(&timep);
        sec  = p->tm_sec;
        min  = p->tm_min;
        hour = p->tm_hour;
        mday = p->tm_mday;
        mon  = p->tm_mon + 1;
        year = p->tm_year % 100;

		if((sec==5 || (sec==6 && autotag==1)) && autotime!=0 && (min%autotime)==0)
			{
            stringmove(insdmgd, 0, data, 0, 4);                     // Instruction
            data[4] = ' ';
            stationid(stid);                            // Station Id
            stringmove(stid, 0, data, 5, 5);
            data[10] = ' ';
            filetime(presenttime, 0);                   // Time
            stringmove(presenttime, 0, data, 11, 16);
            data[27] = ' ';

            for(x=0;x<40;x++)                           // Data Index
	            {
                data[28+x] = '1';
                }
            data[68] = '0';
            data[69] = '0';
            data[70] = '0';
            data[71] = '0';
            data[72] = '0';
            data[73] = ' ';

            if((fbcs = fopen("/usr/mingg.tmp","r")) == NULL)
	            goto autoend;
            fread(&csdata, BYTENUMMZ, 1, fbcs);
            fclose(fbcs);

#ifdef RAIN0000
            if(csdata[40]=='0' && csdata[41]=='0' && csdata[42]=='0' && csdata[43]=='0')
                {
                csdata[40] = ' ';
                csdata[41] = ' ';
                csdata[42] = ' ';
                }
            if(csdata[44]=='0' && csdata[45]=='0' && csdata[46]=='0' && csdata[47]=='0')
                {
                csdata[44] = ' ';
                csdata[45] = ' ';
                csdata[46] = ' ';
                }
            if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                {
                csdata[48] = ' ';
                csdata[49] = ' ';
                csdata[50] = ' ';
                }
            if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                {
                csdata[52] = ' ';
                csdata[53] = ' ';
                csdata[54] = ' ';
                }
            if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                {
                csdata[56] = ' ';
                csdata[57] = ' ';
                csdata[58] = ' ';
                }
            if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                {
                csdata[61] = ' ';
                csdata[62] = ' ';
                csdata[63] = ' ';
                }
            if(csdata[79]=='*')
                {
                csdata[76] = ' ';
                csdata[77] = ' ';
                csdata[78] = ' ';
                }
#endif

//			printf("CSDATA:\n%s\n",csdata);
            stringmove(csdata,0,sstime,0,4);
            sstime[4]='\0';
//			printf("sstime=%s,  wjtime=%s,  sametime=%d\n",sstime,wjtime,sametime);
            if((strcmp(sstime, wjtime)) == 0)
	            {
                sametime++;
                printf("sametime=%d\n",sametime);
                if(sametime > 5)
	                {
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
//                    write(portfd, restart1, sizeof(restart1));
//                    usleep(MTIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
//                    printf("Sametime > 5! Now restart Linux! Please wait ...\n");
//                    close_port(portfd);
//                    sleep(10);
                    rs485clt(fd_485, WDT_STA_CTL, 1);
                    }
                }
            else
                {
                stringmove(sstime,0,wjtime,0,4);
                wjtime[4]='\0';
                sametime = 0;
//                printf("sstime=%s   wjtime=%s   sametime=%d\n",sstime,wjtime,sametime);
                }

            stringmove(csdata, 203, data, 74, 6);       // Quality Control
            stringmove(csdata, 212, data, 80, 7);
            stringmove(csdata, 221, data, 87, 1);
            stringmove(csdata, 223, data, 88, 6);
            stringmove(csdata, 230, data, 94, 20);
            data[114] = '~';
            data[115] = '~';
            data[116] = '~';
            data[117] = '~';
            data[118] = '~';
            data[119] = ' ';

            string_add_space(csdata, 4, data, 120, 4, 6);
            string_add_space(csdata, 40, data, 150, 4, 7);
            string_add_space(csdata, 76, data, 185, 4, 1);
            string_add_space(csdata, 84, data, 190, 4, 3);
            string_add_space(csdata, 96, data, 205, 5, 1);
            string_add_space(csdata, 101, data, 211, 4, 2);
            string_add_space(csdata, 113, data, 221, 4, 10);
            string_add_space(csdata, 153, data, 271, 5, 2);
            string_add_space(csdata, 163, data, 283, 4, 2);
            string_add_space(csdata, 171, data, 293, 12, 1);
            string_add_space(csdata, 183, data, 306, 4, 5);

            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
	            usleep(1);
            else
                {
                read(fdmaste, senstate, sizeof(senstate));
                close(fdmaste);
                }
            if(senstate[WD]=='1')
                {
                memset(data+28, '1', 1);
                memset(data+30, '1', 1);
                memset(data+32, '1', 1);
                }
            else
                {
                memset(data+28, '0', 1);
                memset(data+74, '~', 1);
                memset(data+120, ' ', 5);
                memset(data+30, '0', 1);
                memset(data+76, '~', 1);
                memset(data+130, ' ', 5);
                memset(data+32, '0', 1);
                memset(data+78, '~', 1);
                memset(data+140, ' ', 5);
                }
            if(senstate[WS]=='1')
                {
                memset(data+29, '1', 1);
                memset(data+31, '1', 1);
                memset(data+33, '1', 1);
                }
            else
                {
                memset(data+29, '0', 1);
                memset(data+75, '~', 1);
                memset(data+125, ' ', 5);
                memset(data+31, '0', 3);
                memset(data+77, '~', 3);
                memset(data+135, ' ', 15);
                }
            if(senstate[RAT]=='1')
                {
                memset(data+34, '1', 2);
                }
            else
                {
                memset(data+34, '0', 2);
                memset(data+80, '~', 2);
                memset(data+150, ' ', 10);
                }
            if(senstate[RAT1]=='1')
                {
                memset(data+36, '1', 2);
                }
            else
                {
                memset(data+36, '0', 2);
                memset(data+82, '~', 2);
                memset(data+160, ' ', 10);
                }
            if(senstate[RAW]=='1')
                {
                memset(data+38, '1', 2);
                }
            else
                {
                memset(data+38, '0', 2);
                memset(data+84, '~', 2);
                memset(data+170, ' ', 10);
                }
            if(senstate[T0]=='1')
                {
                memset(data+40, '1', 1);
                }
            else
                {
                memset(data+40, '0', 1);
                memset(data+86, '~', 1);
                memset(data+180, ' ', 5);
                }
            if(senstate[TW]=='1')
                {
                memset(data+41, '1', 1);
                }
            else
                {
                memset(data+41, '0', 1);
                memset(data+87, '~', 1);
                memset(data+185, ' ', 5);
                }
            if(senstate[U]=='1')
                {
                memset(data+42, '1', 1);
                }
            else
                {
                memset(data+42, '0', 1);
                memset(data+88, '~', 1);
                memset(data+190, ' ', 5);
                }
            if(senstate[T0]=='1' && senstate[U]=='1')
                {
                memset(data+43, '1', 2);
                }
            else
                {
                memset(data+43, '0', 2);
                memset(data+89, '~', 2);
                memset(data+195, ' ', 10);
                }
            if(senstate[P]=='1')
                {
                memset(data+45, '1', 1);
                }
            else
                {
                memset(data+45, '0', 1);
                memset(data+91, '~', 1);
                memset(data+205, ' ', 6);
                }
            if(senstate[TG]=='1')
                {
                memset(data+46, '1', 1);
                }
            else
                {
                memset(data+46, '0', 1);
                memset(data+92, '~', 1);
                memset(data+211, ' ', 5);
                }
            if(senstate[ST0]=='1')
                {
                memset(data+47, '1', 1);
                }
            else
                {
                memset(data+47, '0', 1);
                memset(data+93, '~', 1);
                memset(data+216, ' ', 5);
                }
            if(senstate[ST1]=='1')
                {
                memset(data+48, '1', 1);
                }
            else
                {
                memset(data+48, '0', 1);
                memset(data+94, '~', 1);
                memset(data+221, ' ', 5);
                }
            if(senstate[ST2]=='1')
                {
                memset(data+49, '1', 1);
                }
            else
                {
                memset(data+49, '0', 1);
                memset(data+95, '~', 1);
                memset(data+226, ' ', 5);
                }
            if(senstate[ST3]=='1')
                {
                memset(data+50, '1', 1);
                }
            else
                {
                memset(data+50, '0', 1);
                memset(data+96, '~', 1);
                memset(data+231, ' ', 5);
                }
            if(senstate[ST4]=='1')
                {
                memset(data+51, '1', 1);
                }
            else
                {
                memset(data+51, '0', 1);
                memset(data+97, '~', 1);
                memset(data+236, ' ', 5);
                }
            if(senstate[ST5]=='1')
                {
                memset(data+52, '1', 1);
                }
            else
                {
                memset(data+52, '0', 1);
                memset(data+98, '~', 1);
                memset(data+241, ' ', 5);
                }
            if(senstate[ST6]=='1')
                {
                memset(data+53, '1', 1);
                }
            else
                {
                memset(data+53, '0', 1);
                memset(data+99, '~', 1);
                memset(data+246, ' ', 5);
                }
            if(senstate[ST7]=='1')
                {
                memset(data+54, '1', 1);
                }
            else
                {
                memset(data+54, '0', 1);
                memset(data+100, '~', 1);
                memset(data+251, ' ', 5);
                }
            if(senstate[ST8]=='1')
                {
                memset(data+55, '1', 1);
                }
            else
                {
                memset(data+55, '0', 1);
                memset(data+101, '~', 1);
                memset(data+256, ' ', 5);
                }
            if(senstate[LE]=='1')
                {
                memset(data+56, '1', 2);
                }
            else
                {
                memset(data+56, '0', 2);
                memset(data+102, '~', 2);
                memset(data+261, ' ', 10);
                }
            if(senstate[VI]=='1')
                {
                memset(data+58, '1', 1);
                }
            else
                {
                memset(data+58, '0', 1);
                memset(data+104, '~', 1);
                memset(data+271, ' ', 6);
                }
            if(senstate[CH]=='1')
                {
                memset(data+59, '1', 1);
                }
            else
                {
                memset(data+59, '0', 1);
                memset(data+105, '~', 1);
                memset(data+277, ' ', 6);
                }
            if(senstate[TCA]=='1')
                {
                memset(data+60, '1', 1);
                }
            else
                {
                memset(data+60, '0', 1);
                memset(data+106, '~', 1);
                memset(data+283, ' ', 5);
                }
            if(senstate[LCA]=='1')
                {
                memset(data+61, '1', 1);
                }
            else
                {
                memset(data+61, '0', 1);
                memset(data+107, '~', 1);
                memset(data+288, ' ', 5);
                }
            if(senstate[WW]=='1')
                {
                memset(data+62, '1', 1);
                }
            else
                {
                memset(data+62, '0', 1);
                memset(data+108, '~', 1);
                memset(data+293, ' ', 13);
                }
            if(senstate[SD]=='1')
                {
                memset(data+63, '1', 1);
                }
            else
                {
                memset(data+63, '0', 1);
                memset(data+109, '~', 1);
                memset(data+306, ' ', 5);
                }
            if(senstate[FR]=='1')
                {
                memset(data+64, '1', 1);
                }
            else
                {
                memset(data+64, '0', 1);
                memset(data+110, '~', 1);
                memset(data+311, ' ', 5);
                }
            if(senstate[WI]=='1')
                {
                memset(data+65, '1', 1);
                }
            else
                {
                memset(data+65, '0', 1);
                memset(data+111, '~', 1);
                memset(data+316, ' ', 5);
                }
            if(senstate[FSD]=='1')
                {
                memset(data+66, '1', 1);
                }
            else
                {
                memset(data+66, '0', 1);
                memset(data+112, '~', 1);
                memset(data+321, ' ', 5);
                }
            if(senstate[LNF]=='1')
                {
                memset(data+67, '1', 1);
                }
            else
                {
                memset(data+67, '0', 1);
                memset(data+113, '~', 1);
                memset(data+326, ' ', 5);
                }
            memset(data+68, '0', 5);
            memset(data+114, '~', 5);
            memset(data+331, ' ', 25);
            data[BYTEDMGD-2] = '\r';
            data[BYTEDMGD-1] = '\n';
            data[BYTEDMGD] = '\0';

            string_change_1(data, 74, 45);
            recount = string_change_2(data, BYTEDMGD);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd ,&data, BYTEDMGD-recount);
            usleep(GTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);

			if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                goto autoend;
            read(fdmaste, pss, sizeof(pss));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, pss, sizeof(pss));
            usleep(STIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);

			if(min == 0)
				{
                stringmove(insdhgd, 0, data, 0, 4);                     // Instruction
                data[4] = ' ';
                stationid(stid);                                        // Station Id
                stringmove(stid, 0, data, 5, 5);
                data[10] = ' ';
                filetime(presenttime, 0);                               // Time
                stringmove(presenttime, 0, data, 11, 13);
                data[24] = ' ';
                for(x=0;x<63;x++)                                       // Data Index
                    {
                    data[25+x] = '1';
                    }
                data[88] = '-';
                data[89] = '-';
                data[90] = '-';
                data[91] = '-';
                data[92] = '-';
                data[93] = ' ';

                if((fbcs = fopen("/usr/hourzz.tmp","r")) == NULL)
                    goto autoend;
                fread(&csdata, BYTENUMHZ, 1, fbcs);
                fclose(fbcs);

#ifdef RAIN0000
                if(csdata[80]=='0' && csdata[81]=='0' && csdata[82]=='0' && csdata[83]=='0')
                    {
                    csdata[80] = ' ';
                    csdata[81] = ' ';
                    csdata[82] = ' ';
                    }
                if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                    {
                    csdata[84] = ' ';
                    csdata[85] = ' ';
                    csdata[86] = ' ';
                    }
                if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                    {
                    csdata[88] = ' ';
                    csdata[89] = ' ';
                    csdata[90] = ' ';
                    }
                if(csdata[139]=='*')
                    {
                    csdata[136] = ' ';
                    csdata[137] = ' ';
                    csdata[138] = ' ';
                    }
#endif

//              printf("csdata=%s",csdata);
                stringmove(csdata, 346, data, 94, 12);                  // Quality Control
                stringmove(csdata, 365, data, 106, 8);
                stringmove(csdata, 379, data, 114, 1);
                stringmove(csdata, 381, data, 115, 20);
                stringmove(csdata, 406, data, 135, 22);
                data[157] = '-';
                data[158] = '-';
                data[159] = '-';
                data[160] = '-';
                data[161] = '-';
                data[162] = ' ';

                string_add_space(csdata,   4, data, 163, 4, 12);
                string_add_space(csdata,  80, data, 223, 4, 3);
                string_add_space(csdata,  92, data, 238, 4, 5);
                string_add_space(csdata, 136, data, 263, 4, 1);
                string_add_space(csdata, 144, data, 268, 4, 5);
                string_add_space(csdata, 164, data, 293, 5, 2);
                string_add_space(csdata, 174, data, 305, 4, 1);
                string_add_space(csdata, 178, data, 310, 5, 1);
                string_add_space(csdata, 183, data, 316, 4, 1);
                string_add_space(csdata, 187, data, 321, 4, 10);
                string_add_space(csdata, 247, data, 371, 4, 10);
                string_add_space(csdata, 287, data, 421, 5, 2);
                string_add_space(csdata, 297, data, 433, 4, 1);
                string_add_space(csdata, 301, data, 438, 5, 1);
                string_add_space(csdata, 306, data, 444, 4, 2);
                string_add_space(csdata, 314, data, 454, 12, 1);
                string_add_space(csdata, 326, data, 467, 4, 5);
                if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                    usleep(1);
                else
                    {
                    read(fdmaste, senstate, sizeof(senstate));
                    close(fdmaste);
                    }
                if(senstate[WD]=='1')
                    {
                    memset(data+25, '1', 1);
                    memset(data+27, '1', 1);
                    memset(data+29, '1', 1);
                    memset(data+32, '1', 1);
                    memset(data+34, '1', 1);
                    }
                else
                    {
                    memset(data+25, '0', 1);
                    memset(data+94, '~', 1);
                    memset(data+163, ' ', 5);
                    memset(data+27, '0', 1);
                    memset(data+96, '~', 1);
                    memset(data+173, ' ', 5);
                    memset(data+29, '0', 1);
                    memset(data+98, '~', 1);
                    memset(data+183, ' ', 5);
                    memset(data+32, '0', 1);
                    memset(data+101, '~', 1);
                    memset(data+198, ' ', 5);
                    memset(data+34, '0', 1);
                    memset(data+103, '~', 1);
                    memset(data+208, ' ', 5);
                    }
                if(senstate[WS]=='1')
                    {
                    memset(data+26, '1', 1);
                    memset(data+28, '1', 9);
                    }
                else
                    {
                    memset(data+26, '0', 1);
                    memset(data+95, '~', 1);
                    memset(data+168, ' ', 5);
                    memset(data+28, '0', 9);
                    memset(data+97, '~', 9);
                    memset(data+178, ' ', 45);
                    }
                if(senstate[RAT]=='1')
                    {
                    memset(data+37, '1', 1);
                    }
                else
                    {
                    memset(data+37, '0', 1);
                    memset(data+106, '~', 1);
                    memset(data+223, ' ', 5);
                    }
                if(senstate[RAT1]=='1')
                    {
                    memset(data+38, '1', 1);
                    }
                else
                    {
                    memset(data+38, '0', 1);
                    memset(data+107, '~', 1);
                    memset(data+228, ' ', 5);
                    }
                if(senstate[RAW]=='1')
                    {
                    memset(data+39, '1', 1);
                    }
                else
                    {
                    memset(data+39, '0', 1);
                    memset(data+108, '~', 1);
                    memset(data+233, ' ', 5);
                    }
                if(senstate[T0]=='1')
                    {
                    memset(data+40, '1', 5);
                    }
                else
                    {
                    memset(data+40, '0', 5);
                    memset(data+109, '~', 5);
                    memset(data+238, ' ', 25);
                    }
                if(senstate[TW]=='1')
                    {
                    memset(data+45, '1', 1);
                    }
                else
                    {
                    memset(data+45, '0', 1);
                    memset(data+114, '~', 1);
                    memset(data+263, ' ', 5);
                    }
                if(senstate[U]=='1')
                    {
                    memset(data+46, '1', 3);
                    }
                else
                    {
                    memset(data+46, '0', 3);
                    memset(data+115, '~', 3);
                    memset(data+268, ' ', 15);
                    }
                if(senstate[T0]=='1' && senstate[U]=='1')
                    {
                    memset(data+49, '1', 2);
                    }
                else
                    {
                    memset(data+49, '0', 2);
                    memset(data+118, '~', 2);
                    memset(data+283, ' ', 10);
                    }
                if(senstate[P]=='1')
                    {
                    memset(data+51, '1', 5);
                    }
                else
                    {
                    memset(data+51, '0', 5);
                    memset(data+120, '~', 5);
                    memset(data+293, ' ', 28);
                    }
                if(senstate[TG]=='1')
                    {
                    memset(data+56, '1', 5);
                    }
                else
                    {
                    memset(data+56, '0', 5);
                    memset(data+125, '~', 5);
                    memset(data+321, ' ', 25);
                    }
                if(senstate[ST0]=='1')
                    {
                    memset(data+61, '1', 5);
                    }
                else
                    {
                    memset(data+61, '0', 5);
                    memset(data+130, '~', 5);
                    memset(data+346, ' ', 25);
                    }
                if(senstate[ST1]=='1')
                    {
                    memset(data+66, '1', 1);
                    }
                else
                    {
                    memset(data+66, '0', 1);
                    memset(data+135, '~', 1);
                    memset(data+371, ' ', 5);
                    }
                if(senstate[ST2]=='1')
                    {
                    memset(data+67, '1', 1);
                    }
                else
                    {
                    memset(data+67, '0', 1);
                    memset(data+136, '~', 1);
                    memset(data+376, ' ', 5);
                    }
                if(senstate[ST3]=='1')
                    {
                    memset(data+68, '1', 1);
                    }
                else
                    {
                    memset(data+68, '0', 1);
                    memset(data+137, '~', 1);
                    memset(data+381, ' ', 5);
                    }
                if(senstate[ST4]=='1')
                    {
                    memset(data+69, '1', 1);
                    }
                else
                    {
                    memset(data+69, '0', 1);
                    memset(data+138, '~', 1);
                    memset(data+386, ' ', 5);
                    }
                if(senstate[ST5]=='1')
                    {
                    memset(data+70, '1', 1);
                    }
                else
                    {
                    memset(data+70, '0', 1);
                    memset(data+139, '~', 1);
                    memset(data+391, ' ', 5);
                    }
                if(senstate[ST6]=='1')
                    {
                    memset(data+71, '1', 1);
                    }
                else
                    {
                    memset(data+71, '0', 1);
                    memset(data+140, '~', 1);
                    memset(data+396, ' ', 5);
                    }
                if(senstate[ST7]=='1')
                    {
                    memset(data+72, '1', 1);
                    }
                else
                    {
                    memset(data+72, '0', 1);
                    memset(data+141, '~', 1);
                    memset(data+401, ' ', 5);
                    }
                if(senstate[ST8]=='1')
                    {
                    memset(data+73, '1', 1);
                    }
                else
                    {
                    memset(data+73, '0', 1);
                    memset(data+142, '~', 1);
                    memset(data+406, ' ', 5);
                    }
                if(senstate[LE]=='1')
                    {
                    memset(data+74, '1', 2);
                    }
                else
                    {
                    memset(data+74, '0', 2);
                    memset(data+143, '~', 2);
                    memset(data+411, ' ', 10);
                    }
                if(senstate[VI]=='1')
                    {
                    memset(data+76, '1', 3);
                    }
                else
                    {
                    memset(data+76, '0', 3);
                    memset(data+145, '~', 3);
                    memset(data+421, ' ', 17);
                    }
                if(senstate[CH]=='1')
                    {
                    memset(data+79, '1', 1);
                    }
                else
                    {
                    memset(data+79, '0', 1);
                    memset(data+148, '~', 1);
                    memset(data+438, ' ', 6);
                    }
                if(senstate[TCA]=='1')
                    {
                    memset(data+80, '1', 1);
                    }
                else
                    {
                    memset(data+80, '0', 1);
                    memset(data+149, '~', 1);
                    memset(data+444, ' ', 5);
                    }
                if(senstate[LCA]=='1')
                    {
                    memset(data+81, '1', 1);
                    }
                else
                    {
                    memset(data+81, '0', 1);
                    memset(data+150, '~', 1);
                    memset(data+449, ' ', 5);
                    }
                if(senstate[WW]=='1')
                    {
                    memset(data+82, '1', 1);
                    }
                else
                    {
                    memset(data+82, '0', 1);
                    memset(data+151, '~', 1);
                    memset(data+454, ' ', 13);
                    }
                if(senstate[SD]=='1')
                    {
                    memset(data+83, '1', 1);
                    }
                else
                    {
                    memset(data+83, '0', 1);
                    memset(data+152, '~', 1);
                    memset(data+467, ' ', 5);
                    }
                if(senstate[FR]=='1')
                    {
                    memset(data+84, '1', 1);
                    }
                else
                    {
                    memset(data+84, '0', 1);
                    memset(data+153, '~', 1);
                    memset(data+472, ' ', 5);
                    }
                if(senstate[WI]=='1')
                    {
                    memset(data+85, '1', 1);
                    }
                else
                    {
                    memset(data+85, '0', 1);
                    memset(data+154, '~', 1);
                    memset(data+477, ' ', 5);
                    }
                if(senstate[FSD]=='1')
                    {
                    memset(data+86, '1', 1);
                    }
                else
                    {
                    memset(data+86, '0', 1);
                    memset(data+155, '~', 1);
                    memset(data+482, ' ', 5);
                    }
                if(senstate[LNF]=='1')
                    {
                    memset(data+87, '1', 1);
                    }
                else
                    {
                    memset(data+87, '0', 1);
                    memset(data+156, '~', 1);
                    memset(data+487, ' ', 5);
                    }
                memset(data+88, '0', 5);
                memset(data+157, '~', 5);
                memset(data+492, ' ', 25);
                data[BYTEDHGD-2] = '\r';
                data[BYTEDHGD-1] = '\n';
                data[BYTEDHGD] = '\0';

                string_change_1(data, 94, 68);
                recount = string_change_2(data, BYTEDHGD);
                rs485clt(fd_485, R485_STA_CTL1, TX);
                write(portfd, &data, BYTEDHGD-recount);
                usleep(LTIME);
                rs485clt(fd_485, R485_STA_CTL1, RX);
				}
autoend:
			autotag = 0;
			sleep(55);
			autotag = 1;
			}
		else
			sleep(1);
		}
	}


int main(int argc, char *argv[])
{
	fd_set fds;
//	int portfd;
	int retval;

	char ins[100], insdata[100];
	char csdata[1000];

    unsigned char c;

    int ii=0, nn;
    long time;
	int recount;

    char namemz[]={"/tmp/data/GGYYMMDD.DAT"};
    char namems[]={"/tmp/data/SSYYMMDD.DAT"};
    char namemr[]={"/tmp/data/RRYYMMDD.DAT"};
    char namehz[]={"/tmp/data/hzYYMM.TXT"};
    char namehs[]={"/tmp/data/hsYYMM.TXT"};
    char namehr[]={"/tmp/data/hrYYMM.TXT"};

	char datatime1[12], datatime2[12], sourcetime[12];
    long id, idn, count1, timedp1, timedp2;

 	struct rtc_time rtc, rtc_tm;
	int fdtime, fdstbd, fdmaste, fdset,ret;
	char date[12], timed[10], returntf[3]={"F\r\n"};
	char baseinfo[21];
	char stidd[5], stlat[10], stlong[11], sttd[6], stalt[8], staltp[8], stip[17];
    char senstate[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};
	char dauset[7]={"11100\r\n"};
	char sst[3]={"1\r\n"};
	char mact[7], pss[9], door[3], rsta[91];
	char alarm4[4], alarm5[5];
	signed short int masttemp;
	char sensigr[7], sensidr[7], sensinr[15];
	char smcsm[10];

    int samplex, fdsensit, fddcbd, fddcrd1, fdtongxun, fdsetc, fdcf;
	char samplexx[2], samplexxx[3], samplesy[33], setc[42], hwxs[56];
	char sensit[8], sensitnr[15],  dcbd[19], dcrd[50], dcrd1[7], dcrd2[40], sensor[3];
	char tongxun[7], tongxunn[14], baudd[1], baud3[3], baud4[4], baud5[5], baud6[6];
	char cfmem[26], cfmemc[18];
	int baud, baudc, datepd, acheck;
	char voltage[8]={"Voltage "}, channel[8]={"Channel "}, memory[7]={"Memory "};
	char allpass[11]={"all pass!\r\n"};
	char pass[5]={"pass,"}, fault[6]={"fault,"}, rturn[2]={"\r\n"}, openerr[16]={"open file err!\r\n"};
	char normal[8]={"  normal"}, abnormal[8]={"abnormal"};

	int sametime=0;
	char wjtime[5];	//={"2000-01-01 00:00"};
	char sstime[5];	//={"2000-01-01 01:00"};
	char restart1[]={"Sametime > 5! Now restart Linux! Please wait ...\r\n"};
    char restart2[]={"Now restart Linux! Please wait ...\r\n"};

    int fdalarm;
    char alarmyz[22], alarmsy[7], alarmxx[28];

	int fbps;

    struct timeval stimeout, rtimeout;

    FILE *fbcs, *fbsamp;
    int fdparame=-1;
    char presenttime[19], paramete[8];
    int x;
    char data[1500];
    int td=0;
    time_t timep;
    struct tm *p;
	char senco[20], df[300];
	int senconum;
	char statzfsw[4], statccsw[4];
	long dmtime=0, dmraw=0, dmle=0;
//	int devmodetime=0;
	char devmoderaw[6]={"2 30\r\n"};
	char devmodele[6]={"2 30\r\n"};
	char zero[3]={"0\r\n"};

	FILE *fdqcp;
	char qcps[BYTEQCPS], qcpm[BYTEQCPM], scv[BYTESCV];
	char *qcps1 = {"                     "};				// size=21
	char *qcps2 = {"                     "};
	char *qcpm1 = {"                               "};		// size=31
	char *qcpm2 = {"                               "};
	char *scv1 = {"                                                                                                    "};
	char *scv2 = {"                                                                                                    "};
															// size=101
	char *string1 = {"                "};					// size=16;
    char *string2 = {"                "};                   // size=16;

	char dztn[BYTEDZTN], dzstn[BYTEDZSTN], dzran[BYTEDZRAN], dzsensi[BYTEDZSENSI];
	char autocheck[200]={"2009-01-01 00:00:00;GPS abnormal;115200 8 N 1;+00.0;DC,00.0;01# abnormal;02# abnormal;03# abnormal;04# abnormal;05# abnormal;1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};
	char collect[64]={"CLIM  normal;EATH  normal;RADI  normal;SEA abnormal;INTL  normal"};

	char statmain[64]={"STATMAIN 0 120 1 +200 0 0 0 0512 0 0 0 0 0 1000 1000 - - - - -\r\n"};
	char statclim[37]={"STATCLIM 0 120 1 +200 0 0 - - - - -\r\n"};
    char statradi[37]={"STATRADI 0 120 1 +200 0 0 - - - - -\r\n"};
    char stateath[37]={"STATEATH 0 120 1 +200 0 0 - - - - -\r\n"};
    char statsoil[37]={"STATSOIL N 120 1 +200 0 0 - - - - -\r\n"};
    char statseaa[37]={"STATSEAA N 120 1 +200 0 0 - - - - -\r\n"};
    char statintl[39]={"STATINTL_1 0 136 0  231 0 0 - - - - -\r\n"};
	char statsensor[75]={"00000N1N0000000000000000000000NNNNNNNNN0N0NNNNNNNNNN000000000NNNNNNNNNNNN\r\n"};
	char senstonoff[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};
	char stat[259]={"STAT 2010-10-01 00:00 0012000200000010000000000000000000000000000000000000000000N-----------N-----------000000000000N-----------N-----------N-----------N-----------N-----------111110101111111111111111111111000000000101000000000011111111100000000000010001000\r\n"};
	int onoff;

	pthread_t th_gps;
//	char insgps[4] = {"send"};
	char insgps[5] = {"<G21>"};
	char welcome[28] = {"Welcome To SHCWQX New-AWS!\r\n"};

	char dmod[]={"DMOD 2008-12-01 11:02 0000000000000000000000 8888888888888888888888 / / / / / / / / / / / / / / / / / / / / / /\r\n"};
	char dhod[]={"DHOD 2008-12-01 10 0000000000000000000000000000 8888888888888888888888888888 / / / / / / / / / / / / / / / / / / / / / / / / / / / / /\r\n"};
	char help[]={"SETCOM,IP,BASEINFO,AUTOCHECK,DATE,TIME,ID,LAT,LONG,TD,ALTP,ALT,SENSI,SMC,DOOR,MACT,PSS,SENST,RSTA,SENCO,DEVMODE,DAUSET,GPSSET,CFSET,STATMAIN,STATCLIM,STATRADI,STATEATH,STATSOIL,STATSEAA,STATINTL,STATSENSOR,STAT,QCPS,QCPM,SCV,DMGD,DMCD,DMRD,DMSD,DMOD,DHGD,DHCD,DHRD,DHSD,DHOD,SAMPLE,GALE,TMAX,TMIN,RMAX,DTLT,DTLV\r\n"};

//	char badcommand[]={"BAD COMMAND\r\n"};
	char badcommand[]={"F\r\n"};
	char helpdmgd[]={"DMGD\nDMGD YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nDMGD YYYY-MM-DD HH:MM n\r\n"};
    char helpdmcd[]={"DMCD\nDMCD YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nDMCD YYYY-MM-DD HH:MM n\r\n"};
    char helpdmsd[]={"DMSD\nDMSD YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nDMSD YYYY-MM-DD HH:MM n\r\n"};
    char helpdmrd[]={"DMRD\nDMRD YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nDMRD YYYY-MM-DD HH:MM n\r\n"};
    char helpdmod[]={"DMOD\nDMOD YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nDMOD YYYY-MM-DD HH:MM n\r\n"};
    char helpdhgd[]={"DHGD\nDHGD YYYY-MM-DD HH YYYY-MM-DD HH\nDHGD YYYY-MM-DD HH n\r\n"};
    char helpdhcd[]={"DHCD\nDHCD YYYY-MM-DD HH YYYY-MM-DD HH\nDHCD YYYY-MM-DD HH n\r\n"};
    char helpdhsd[]={"DHSD\nDHSD YYYY-MM-DD HH YYYY-MM-DD HH\nDHSD YYYY-MM-DD HH n\r\n"};
    char helpdhrd[]={"DHRD\nDHRD YYYY-MM-DD HH YYYY-MM-DD HH\nDHRD YYYY-MM-DD HH n\r\n"};
    char helpdhod[]={"DHOD\nDHOD YYYY-MM-DD HH YYYY-MM-DD HH\nDHOD YYYY-MM-DD HH n\r\n"};
	char helpsample[]={"SAMPLE X/XX/XXX YYYY-MM-DD HH:MM\r\n"};
	char helpsetcom[]={"SETCOM [Baudrate Databit Parity Stopbit]\r\n"};
	char helpip[]={"IP [*.*.*.*] (0<=*<256)\r\n"};
	char helpbaseinfo[]={"BASEINFO\r\n"};
	char helpautocheck[]={"AUTOCHECK\r\n"};
	char helpdate[]={"DATE [YYYY-MM-DD]\r\n"};
	char helptime[]={"TIME [HH:MM:SS]\r\n"};
	char helpid[]={"ID [XXXXX]\r\n"};
	char helplat[]={"LAT [DD.MM.SS]\r\n"};
	char helplong[]={"LONG [DDD.MM.SS]\r\n"};
	char helptd[]={"TD [*] (-240<*<240)\r\n"};
	char helpalt[]={"ALT [*] (-9999.9<*<9999.9)\r\n"};
	char helpaltp[]={"ALTP [*] (-9999.9<*<9999.9)\r\n"};
	char helpscv[]={"SCV X/XX/XXX [*,*/*.*/---/99.9,*]\r\n"};
	char helpsensi[]={"SENSI XX [XX.XX]\r\n"};
	char helpsmc[]={"SMC XXX [* * *]\r\n"};
	char helpdoor[]={"DOOR\r\n"};
	char helpmact[]={"MACT\r\n"};
	char helppss[]={"PSS\r\n"};
	char helpsenst[]={"SENST\nSENST *\nSENST X/XX/XXX\nSENST X/XX/XXX Y(Y=0 or Y=1)\r\n"};
	char helprsta[]={"RSTA\r\n"};
	char helphelp[]={"HELP\r\n"};
	char helpqcps[]={"QCPS X/XX/XXX * * *\r\n"};
	char helpqcpm[]={"QCPM X/XX/XXX * * * * *\r\n"};
	char helpgale[]={"GALE [*]\r\n"};
    char helptmax[]={"TMAX [*]\r\n"};
    char helptmin[]={"TMIN [*]\r\n"};
    char helprmax[]={"RMAX [*]\r\n"};
    char helpdtlt[]={"DTLT [*]\r\n"};
    char helpdtlv[]={"DTLV [*]\r\n"};
	char helpsenco[]={"SENCO XXX [* * * *]\r\n"};
	char helpdauset[]={"DAUSET XXXX [x]\r\n"};
	char helpgpsset[]={"GPSSET [x]\r\n"};
    char helpcfset[]={"CFSET [x]\r\n"};
	char helpstatmain[]={"STATMAIN\r\n"};
    char helpstatclim[]={"STATCLIM\r\n"};
    char helpstatradi[]={"STATRADI\r\n"};
    char helpstateath[]={"STATEATH\r\n"};
    char helpstatsoil[]={"STATSOIL\r\n"};
    char helpstatseaa[]={"STATSEAA\r\n"};
    char helpstatintl[]={"STATINTL\r\n"};
    char helpstatsensor[]={"STATSENSOR\r\n"};
    char helpstat[]={"STAT\r\n"};

	int hyyear, hymonth, hyday, hyhour, hymin, hysec;

	pthread_t th_autosend;
	char autosend[2];
	char stautosend[]={"AUTOSEND 00\r\n"};

	fd_ttyS0 = open_port(SERIAL0, 5);
    pthread_create(&th_gps, NULL, gpspthread, 0);


tongxun:
	if((fdtongxun=open("/tmp/paramete/tongxun.txt",O_RDONLY)) < 1)
		{
		baud = 5;
		}
	else
		{
		read(fdtongxun,&tongxun,sizeof(tongxun));
		stringmove(tongxun,0,baudd,0,1);
		close(fdtongxun);
		baud = baudd[0] - '0';
		}
//	portfd = open_port(UART1, baud);		// RS485
	portfd = open_port(UART0, baud);		// RS232

    fd_485 = open("/dev/zlg7289", O_RDWR);
    if(fd_485 == -1){
        printf("open /dev/zlg7289 failed\n");
        exit(1);
    }

    if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
	    {
        returnTorF(portfd, returntf, 0);
//        goto txend;
        }
    ret = read(fdmaste, &masttemp, 2);
    usleep(10000);
    ret = read(fdmaste, &masttemp, 2);
//  printf("temp=%d\n",masttemp);
    close(fdmaste);

	printf("pc485 2012-08-01 17:00 V6.5 RS232 for QingDao\n");

    rs485clt(fd_485, R485_STA_CTL1, TX);
    write(portfd ,welcome, sizeof(welcome));
    usleep(STIME);
    rs485clt(fd_485, R485_STA_CTL1, RX);

    if((fdstbd = open("/tmp/paramete/autosend.txt", O_RDONLY)) < 1)
		autotime = 0;
	else
		{                    
		read(fdstbd, autosend, sizeof(autosend));
        close(fdstbd);
		autotime = (autosend[0]-'0')*10 + (autosend[1]-'0');
        }

	pthread_create(&th_autosend, NULL, autosendpthread, 0);

    while(1)
		{
txbegin:
	rs485clt(fd_485, R485_STA_CTL1, RX);

	FD_ZERO(&fds);
	FD_SET(portfd, &fds);
        stimeout.tv_sec = 59;
        stimeout.tv_usec = 0;
        retval = select(portfd + 1, &fds, NULL, NULL, &stimeout);
//	retval = select(portfd + 1, &fds, NULL, NULL, NULL);
//	printf("select=%d\n",retval);
	if(FD_ISSET(portfd, &fds))
	    {
	    time = 0;
	    do	{
                if(read(portfd,&c,1)==1)
	   	    {
//		    putchar(c);
			if(c>='a' && c<='z')
				c=c-32;
		    ins[ii]=c;
                    ii++;
		    }
		time++;
                if(ii>100)
		    {
//                  break;
		    goto txend;
		    }
                if(time>2000)
		    {
//                  break;
		    goto txend;
		    }
                }while(c!='\n');
		ins[ii] = '\0';
//        printf("%s  %d\n",ins,ii);			
	    ii=ii-2;
	    stringmove(ins,0,insdata,0,ii);
	    insdata[ii]='\0';
//	    printf("%s	%d\n",insdata,ii);

/*
        if(strstr(insdata, "CSSJ") != 0)
         	{
            if((fbcs = fopen("/usr/mintn.tmp","r")) == NULL)
	            goto txend;
            fread(&csdata,22,1,fbcs);
            fclose(fbcs);
			rs485clt(fd_485, R485_STA_CTL1, TX);
			write(portfd,&csdata,22);
			usleep(MTIME);
			rs485clt(fd_485, R485_STA_CTL1, RX);
			}
*/
		if(strstr(insdata, "AUTOSEND") != 0)
            {
//            memset(&stautosend, ' ', sizeof(stautosend));
            switch(ii)
                {
                case 8:
                    if((fdstbd = open("/tmp/paramete/autosend.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, autosend, sizeof(autosend));
                    close(fdstbd);
					stringmove(autosend, 0, stautosend, 9, 2);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stautosend, sizeof(stautosend));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 11:
                    if((fdstbd = open("/tmp/paramete/autosend.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 9, autosend, 0, 2);
					autotime = (autosend[0]-'0')*10 + (autosend[1]-'0');
//					printf("autotime=%d\n",autotime);
                    ret = write(fdstbd, autosend, sizeof(autosend));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
               	default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
            }

        else if(strstr(insdata, "DMGD") != 0)
			{
			if(strstr(insdata, "/?") != 0)
				returncommand(portfd, helpdmgd, sizeof(helpdmgd), MDELAY);
			else
				{
			switch(ii)
		    	{
		    	case 4:
					stringmove(insdata, 0, data, 0, 4);			// Instruction
					data[4] = ' ';
                    stationid(stid);							// Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';			
	                filetime(presenttime, 0);					// Time
//					for(x=0;x<20;x++)
//					printf("%c",presenttime[x]);
					stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';

                    for(x=0;x<40;x++)							// Data Index
                        {
                        data[28+x] = '1';
                        }
					data[68] = '0';
                    data[69] = '0';
                    data[70] = '0';
                    data[71] = '0';
                    data[72] = '0';
                    data[73] = ' ';

                    if((fbcs = fopen("/usr/mingg.tmp","r")) == NULL)
                        goto txend;
                    fread(&csdata, BYTENUMMZ, 1, fbcs);
                    fclose(fbcs);

#ifdef RAIN0000
					if(csdata[40]=='0' && csdata[41]=='0' && csdata[42]=='0' && csdata[43]=='0')
						{
						csdata[40] = ' ';
                        csdata[41] = ' ';
                        csdata[42] = ' ';
						}
                    if(csdata[44]=='0' && csdata[45]=='0' && csdata[46]=='0' && csdata[47]=='0')
                        {
                        csdata[44] = ' ';
                        csdata[45] = ' ';
                        csdata[46] = ' ';
                        }
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }
                    if(csdata[79]=='*')
                        {
                        csdata[76] = ' ';
                        csdata[77] = ' ';
                        csdata[78] = ' ';
                        }
#endif

//					printf("CSDATA:\n%s\n",csdata);
                    stringmove(csdata,0,sstime,0,4);
                    sstime[4]='\0';
//                    printf("sstime=%s,  wjtime=%s,  sametime=%d\n",sstime,wjtime,sametime);
                    if((strcmp(sstime, wjtime)) == 0)
                        {
                        sametime++;
                        printf("sametime=%d\n",sametime);
                        if(sametime > 5)
                            {
		                    rs485clt(fd_485, R485_STA_CTL1, TX);
		                    write(portfd, restart1, sizeof(restart1));
		                    usleep(MTIME);
		                    rs485clt(fd_485, R485_STA_CTL1, RX);
							printf("Sametime > 5! Now restart Linux! Please wait ...\n");
							close_port(portfd);
							sleep(10);
            				rs485clt(fd_485, WDT_STA_CTL, 1);
                            }
                        }
                    else
                        {
                        stringmove(sstime,0,wjtime,0,4);
                        wjtime[4]='\0';
                        sametime = 0;
//                      printf("sstime=%s   wjtime=%s   sametime=%d\n",sstime,wjtime,sametime);
                        }

					stringmove(csdata, 203, data, 74, 6);		// Quality Control
                    stringmove(csdata, 212, data, 80, 7);
                    stringmove(csdata, 221, data, 87, 1);
                    stringmove(csdata, 223, data, 88, 6);
                    stringmove(csdata, 230, data, 94, 20);
                    data[114] = '~';
                    data[115] = '~';
                    data[116] = '~';
                    data[117] = '~';
                    data[118] = '~';
                    data[119] = ' ';

					string_add_space(csdata, 4, data, 120, 4, 6);
                    string_add_space(csdata, 40, data, 150, 4, 7);
                    string_add_space(csdata, 76, data, 185, 4, 1);
                    string_add_space(csdata, 84, data, 190, 4, 3);
                    string_add_space(csdata, 96, data, 205, 5, 1);
                    string_add_space(csdata, 101, data, 211, 4, 2);
                    string_add_space(csdata, 113, data, 221, 4, 10);
                    string_add_space(csdata, 153, data, 271, 5, 2);
                    string_add_space(csdata, 163, data, 283, 4, 2);
                    string_add_space(csdata, 171, data, 293, 12, 1);
                    string_add_space(csdata, 183, data, 306, 4, 5);

                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
                    if(senstate[WD]=='1')
                        {
                        memset(data+28, '1', 1);
						memset(data+30, '1', 1);
						memset(data+32, '1', 1);
                        }
                    else
                        {
                        memset(data+28, '0', 1);
                        memset(data+74, '~', 1);
                        memset(data+120, ' ', 5);
                        memset(data+30, '0', 1);
                        memset(data+76, '~', 1);
                        memset(data+130, ' ', 5);
                        memset(data+32, '0', 1);
                        memset(data+78, '~', 1);
                        memset(data+140, ' ', 5);
                        }
                    if(senstate[WS]=='1')
                        {
                        memset(data+29, '1', 1);
                        memset(data+31, '1', 1);
						memset(data+33, '1', 1);
                        }
                    else
                        {
                        memset(data+29, '0', 1);
                        memset(data+75, '~', 1);
                        memset(data+125, ' ', 5);
                        memset(data+31, '0', 3);
                        memset(data+77, '~', 3);
                        memset(data+135, ' ', 15);
                        }
                    if(senstate[RAT]=='1')
                        {
                        memset(data+34, '1', 2);
                        }
                    else
                        {
                        memset(data+34, '0', 2);
                        memset(data+80, '~', 2);
                        memset(data+150, ' ', 10);
                        }
                    if(senstate[RAT1]=='1')
                        {
                        memset(data+36, '1', 2);
                        }
                    else
                        {
                        memset(data+36, '0', 2);
                        memset(data+82, '~', 2);
                        memset(data+160, ' ', 10);
                        }
                    if(senstate[RAW]=='1')
                        {
                        memset(data+38, '1', 2);
                        }
                    else
                        {
                        memset(data+38, '0', 2);
                        memset(data+84, '~', 2);
                        memset(data+170, ' ', 10);
                        }
                    if(senstate[T0]=='1')
                        {
                        memset(data+40, '1', 1);
                        }
                    else
                        {
                        memset(data+40, '0', 1);
                        memset(data+86, '~', 1);
                        memset(data+180, ' ', 5);
                        }
                    if(senstate[TW]=='1')
                        {
                        memset(data+41, '1', 1);
                        }
                    else
                        {
                        memset(data+41, '0', 1);
                        memset(data+87, '~', 1);
                        memset(data+185, ' ', 5);
                        }
                    if(senstate[U]=='1')
                        {
                        memset(data+42, '1', 1);
                        }
                    else
                        {
                        memset(data+42, '0', 1);
                        memset(data+88, '~', 1);
                        memset(data+190, ' ', 5);
                        }
                    if(senstate[T0]=='1' && senstate[U]=='1')
                        {
                        memset(data+43, '1', 2);
                        }
                    else
                        {
                        memset(data+43, '0', 2);
                        memset(data+89, '~', 2);
                        memset(data+195, ' ', 10);
                        }
                    if(senstate[P]=='1')
                        {
                        memset(data+45, '1', 1);
                        }
                    else
                        {
                        memset(data+45, '0', 1);
                        memset(data+91, '~', 1);
                        memset(data+205, ' ', 6);
                        }
                    if(senstate[TG]=='1')
                        {
                        memset(data+46, '1', 1);
                        }
                    else
                        {
                        memset(data+46, '0', 1);
                        memset(data+92, '~', 1);
                        memset(data+211, ' ', 5);
                        }
                    if(senstate[ST0]=='1')
                        {
                        memset(data+47, '1', 1);
                        }
                    else
                        {
                        memset(data+47, '0', 1);
                        memset(data+93, '~', 1);
                        memset(data+216, ' ', 5);
                        }
                    if(senstate[ST1]=='1')
                        {
                        memset(data+48, '1', 1);
                        }
                    else
                        {
                        memset(data+48, '0', 1);
                        memset(data+94, '~', 1);
                        memset(data+221, ' ', 5);
                        }
                    if(senstate[ST2]=='1')
                        {
                        memset(data+49, '1', 1);
                        }
                    else
                        {
                        memset(data+49, '0', 1);
                        memset(data+95, '~', 1);
                        memset(data+226, ' ', 5);
                        }
                    if(senstate[ST3]=='1')
                        {
                        memset(data+50, '1', 1);
                        }
                    else
                        {
                        memset(data+50, '0', 1);
                        memset(data+96, '~', 1);
                        memset(data+231, ' ', 5);
                        }
                    if(senstate[ST4]=='1')
                        {
                        memset(data+51, '1', 1);
                        }
                    else
                        {
                        memset(data+51, '0', 1);
                        memset(data+97, '~', 1);
                        memset(data+236, ' ', 5);
                        }
                    if(senstate[ST5]=='1')
                        {
                        memset(data+52, '1', 1);
                        }
                    else
                        {
                        memset(data+52, '0', 1);
                        memset(data+98, '~', 1);
                        memset(data+241, ' ', 5);
                        }
                    if(senstate[ST6]=='1')
                        {
                        memset(data+53, '1', 1);
                        }
                    else
                        {
                        memset(data+53, '0', 1);
                        memset(data+99, '~', 1);
                        memset(data+246, ' ', 5);
                        }
                    if(senstate[ST7]=='1')
                        {
                        memset(data+54, '1', 1);
                        }
                    else
                        {
                        memset(data+54, '0', 1);
                        memset(data+100, '~', 1);
                        memset(data+251, ' ', 5);
                        }
                    if(senstate[ST8]=='1')
                        {
                        memset(data+55, '1', 1);
                        }
                    else
                        {
                        memset(data+55, '0', 1);
                        memset(data+101, '~', 1);
                        memset(data+256, ' ', 5);
                        }
                    if(senstate[LE]=='1')
                        {
                        memset(data+56, '1', 2);
                        }
                    else
                        {
                        memset(data+56, '0', 2);
                        memset(data+102, '~', 2);
                        memset(data+261, ' ', 10);
                        }
                    if(senstate[VI]=='1')
                        {
                        memset(data+58, '1', 1);
                        }
                    else
                        {
                        memset(data+58, '0', 1);
                        memset(data+104, '~', 1);
                        memset(data+271, ' ', 6);
                        }
                    if(senstate[CH]=='1')
                        {
                        memset(data+59, '1', 1);
                        }
                    else
                        {
                        memset(data+59, '0', 1);
                        memset(data+105, '~', 1);
                        memset(data+277, ' ', 6);
                        }
                    if(senstate[TCA]=='1')
                        {
                        memset(data+60, '1', 1);
                        }
                    else
                        {
                        memset(data+60, '0', 1);
                        memset(data+106, '~', 1);
                        memset(data+283, ' ', 5);
                        }
                    if(senstate[LCA]=='1')
                        {
                        memset(data+61, '1', 1);
                        }
                    else
                        {
                        memset(data+61, '0', 1);
                        memset(data+107, '~', 1);
                        memset(data+288, ' ', 5);
                        }
                    if(senstate[WW]=='1')
                        {
                        memset(data+62, '1', 1);
                        }
                    else
                        {
                        memset(data+62, '0', 1);
                        memset(data+108, '~', 1);
                        memset(data+293, ' ', 13);
                        }
                    if(senstate[SD]=='1')
                        {
                        memset(data+63, '1', 1);
                        }
                    else
                        {
                        memset(data+63, '0', 1);
                        memset(data+109, '~', 1);
                        memset(data+306, ' ', 5);
                        }
                    if(senstate[FR]=='1')
                        {
                        memset(data+64, '1', 1);
                        }
                    else
                        {
                        memset(data+64, '0', 1);
                        memset(data+110, '~', 1);
                        memset(data+311, ' ', 5);
                        }
                    if(senstate[WI]=='1')
                        {
                        memset(data+65, '1', 1);
                        }
                    else
                        {
                        memset(data+65, '0', 1);
                        memset(data+111, '~', 1);
                        memset(data+316, ' ', 5);
                        }
                    if(senstate[FSD]=='1')
                        {
                        memset(data+66, '1', 1);
                        }
                    else
                        {
                        memset(data+66, '0', 1);
                        memset(data+112, '~', 1);
                        memset(data+321, ' ', 5);
                        }
                    if(senstate[LNF]=='1')
                        {
                        memset(data+67, '1', 1);
                        }
                    else
                        {
                        memset(data+67, '0', 1);
                        memset(data+113, '~', 1);
                        memset(data+326, ' ', 5);
                        }
                    memset(data+68, '0', 5);
                    memset(data+114, '~', 5);
                    memset(data+331, ' ', 25);
					data[BYTEDMGD-2] = '\r';
					data[BYTEDMGD-1] = '\n';
					data[BYTEDMGD] = '\0';

//					printf("S1:\n%s\n",data);
					string_change_1(data, 74, 45);
                    recount = string_change_2(data, BYTEDMGD);
					rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd ,&data, BYTEDMGD-recount);
					usleep(GTIME);
					rs485clt(fd_485, R485_STA_CTL1, RX);
				
//					write(fd_ttyS0, insgps, sizeof(insgps));                //GPS
					break;

                case 38:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    datepd = (insdata[22]-'0')*1000 + (insdata[23]-'0')*100 + (insdata[24]-'0')*10 + (insdata[25]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[33] - '0') * 10 + (insdata[34] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[36] - '0') * 10 + (insdata[37] - '0');
                    if(datepd > 60)
                        goto txend;

                    datamove(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamove(datatime2,insdata,24);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 60 + 1;

                    stringmove(insdata, 0, data, 0, 4);         // Instruction
                    data[4] = ' ';
                    stationid(stid);                            // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                   // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<40;x++)                           // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[68] = '0';
                    data[69] = '0';
                    data[70] = '0';
                    data[71] = '0';
                    data[72] = '0';
                    data[73] = ' ';

                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namemz,12,6);
                    	if((fbcs = fopen(namemz,"r")) == NULL)
							{
							returnTorF(portfd, returntf, 0);
                        	goto txend;
							}
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMZ;
                        fseek(fbcs, idn, SEEK_SET);
                    	fread(&csdata, BYTENUMMZ, 1, fbcs);
                    	fclose(fbcs);
#ifdef RAIN0000
                    if(csdata[40]=='0' && csdata[41]=='0' && csdata[42]=='0' && csdata[43]=='0')
                       	{
                       	csdata[40] = ' ';
                        csdata[41] = ' ';
                        csdata[42] = ' ';
                        }
                    if(csdata[44]=='0' && csdata[45]=='0' && csdata[46]=='0' && csdata[47]=='0')
                        {
                        csdata[44] = ' ';
                        csdata[45] = ' ';
                        csdata[46] = ' ';
                        }
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }
                    if(csdata[79]=='*')
                        {
                        csdata[76] = ' ';
                        csdata[77] = ' ';
                        csdata[78] = ' ';
                        }
#endif

	                    stringmove(csdata, 203, data, 74, 6);       // Quality Control
    	                stringmove(csdata, 212, data, 80, 7);
        	            stringmove(csdata, 221, data, 87, 1);
            	        stringmove(csdata, 223, data, 88, 6);
                	    stringmove(csdata, 230, data, 94, 20);
						memset(data+114, ' ', 5);
	                    data[119] = ' ';

						stringmove(sourcetime,0,data,13,2);
        	            stringmove(sourcetime,2,data,16,2);
            	        stringmove(sourcetime,4,data,19,2);
                	    stringmove(sourcetime,6,data,22,2);
                    	stringmove(sourcetime,8,data,25,2);
                    
						string_add_space(csdata, 4, data, 120, 4, 6);
    	                string_add_space(csdata, 40, data, 150, 4, 7);
        	            string_add_space(csdata, 76, data, 185, 4, 1);
            	        string_add_space(csdata, 84, data, 190, 4, 3);
                	    string_add_space(csdata, 96, data, 205, 5, 1);
                    	string_add_space(csdata, 101, data, 211, 4, 2);
	                    string_add_space(csdata, 113, data, 221, 4, 10);
    	                string_add_space(csdata, 153, data, 271, 5, 2);
        	            string_add_space(csdata, 163, data, 283, 4, 2);
            	        string_add_space(csdata, 171, data, 293, 12, 1);
                	    string_add_space(csdata, 183, data, 306, 4, 5);

						memset(data+331, ' ', 25);
                    	data[BYTEDMGD-2] = '\r';
	                    data[BYTEDMGD-1] = '\n';
    	                data[BYTEDMGD] = '\0';

        	            recount = string_change_2(data, BYTEDMGD);
						rs485clt(fd_485, R485_STA_CTL1, TX);
            	        write(portfd ,&data, BYTEDMGD-recount);
						usleep(LTIME);
						rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
					break;

                case 23:
                case 24:
                case 25:
                case 26:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    switch(ii)
                    	{
                    	case 23: count1 = insdata[22] - '0';break;
                    	case 24: count1 = (insdata[22] - '0') * 10 + (insdata[23] - '0');break;
                    	case 25: count1 = (insdata[22] - '0') * 100 + (insdata[23] - '0') * 10 + (insdata[24] - '0');break;
                    	case 26: count1 = (insdata[22] - '0') * 1000 + (insdata[23] - '0') * 100 + (insdata[24] - '0') * 10 + (insdata[25] - '0');break;
                    	default: returnTorF(portfd, returntf, 0);
                    	}

                    stringmove(insdata, 0, data, 0, 4);         // Instruction
                    data[4] = ' ';
                    stationid(stid);                            // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                   // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<40;x++)                           // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[68] = '0';
                    data[69] = '0';
                    data[70] = '0';
                    data[71] = '0';
                    data[72] = '0';
                    data[73] = ' ';

					datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namemz,12,6);
                        if((fbcs = fopen(namemz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[40]=='0' && csdata[41]=='0' && csdata[42]=='0' && csdata[43]=='0')
                        {
                        csdata[40] = ' ';
                        csdata[41] = ' ';
                        csdata[42] = ' ';
                        }
                    if(csdata[44]=='0' && csdata[45]=='0' && csdata[46]=='0' && csdata[47]=='0')
                        {
                        csdata[44] = ' ';
                        csdata[45] = ' ';
                        csdata[46] = ' ';
                        }
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }
                    if(csdata[79]=='*')
                        {
                        csdata[76] = ' ';
                        csdata[77] = ' ';
                        csdata[78] = ' ';
                        }
#endif

                        stringmove(csdata, 203, data, 74, 6);       // Quality Control
                        stringmove(csdata, 212, data, 80, 7);
                        stringmove(csdata, 221, data, 87, 1);
                        stringmove(csdata, 223, data, 88, 6);
                        stringmove(csdata, 230, data, 94, 20);
                        memset(data+114, ' ', 5);
                        data[119] = ' ';

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

                        string_add_space(csdata, 4, data, 120, 4, 6);
                        string_add_space(csdata, 40, data, 150, 4, 7);
                        string_add_space(csdata, 76, data, 185, 4, 1);
                        string_add_space(csdata, 84, data, 190, 4, 3);
                        string_add_space(csdata, 96, data, 205, 5, 1);
                        string_add_space(csdata, 101, data, 211, 4, 2);
                        string_add_space(csdata, 113, data, 221, 4, 10);
                        string_add_space(csdata, 153, data, 271, 5, 2);
                        string_add_space(csdata, 163, data, 283, 4, 2);
                        string_add_space(csdata, 171, data, 293, 12, 1);
                        string_add_space(csdata, 183, data, 306, 4, 5);

                        memset(data+331, ' ', 25);
                        data[BYTEDMGD-2] = '\r';
                        data[BYTEDMGD-1] = '\n';
                        data[BYTEDMGD] = '\0';

                        recount = string_change_2(data, BYTEDMGD);
						rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd ,&data, BYTEDMGD-recount);
						usleep(LTIME);
						rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
					break;

//		    	default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
		    	}
				}
			}	// if(strstr(insdata, "DMGD") != 0)


        else if(strstr(insdata, "DMCD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdmcd, sizeof(helpdmcd), MDELAY);
            else
                {
            switch(ii)
                {
                case 4:
                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<11;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
					data[39] = ' ';

                    if((fbcs = fopen("/usr/mingg.tmp","r")) == NULL)
                        goto txend;
                    fread(&csdata, BYTENUMMZ, 1, fbcs);
                    fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }
#endif

					stringmove(csdata, 220, data, 40, 1);					// Quality Control
                    stringmove(csdata, 219, data, 41, 1);
                    stringmove(csdata, 214, data, 42, 4);
                    stringmove(csdata, 209, data, 46, 3);
                    stringmove(csdata, 228, data, 49, 2);
                    data[51] = ' ';

                    string_add_space(csdata, 72, data, 52, 4, 1);
                    string_add_space(csdata, 68, data, 57, 4, 1);
                    string_add_space(csdata, 48, data, 62, 4, 4);
                    string_add_space(csdata, 28, data, 82, 4, 3);
                    string_add_space(csdata, 105, data, 97, 4, 2);

                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
					if(senstate[T1]=='1' || senstate[T2]=='1' ||senstate[T3]=='1')
						{
						memset(data+28, '1', 1);
						}
					else
						{
                        memset(data+28, '0', 1);
                        memset(data+40, '~', 1);
                        memset(data+52, ' ', 5);
                        }
                    if(senstate[SV1]=='1' || senstate[SV2]=='1' ||senstate[SV3]=='1')
                        {
                        memset(data+29, '1', 1);
                        }
                    else
                        {
                        memset(data+29, '0', 1);
                        memset(data+41, '~', 1);
                        memset(data+57, ' ', 5);
                        }
                    if(senstate[RAT1]=='1')
                        {
                        memset(data+30, '1', 2);
                        }
                    else
                        {
                        memset(data+30, '0', 2);
                        memset(data+42, '~', 2);
                        memset(data+62, ' ', 10);
                        }
                    if(senstate[RAW]=='1')
                        {
                        memset(data+32, '1', 2);
                        }
                    else
                        {
                        memset(data+32, '0', 2);
                        memset(data+44, '~', 2);
                        memset(data+72, ' ', 10);
                        }
                    if(senstate[WS1]=='1')
                        {
                        memset(data+34, '1', 3);
                        }
                    else
                        {
                        memset(data+34, '0', 3);
                        memset(data+46, '~', 3);
                        memset(data+82, ' ', 15);
                        }
                    if(senstate[ST0]=='1')
                        {
                        memset(data+37, '1', 1);
                        }
                    else
                        {
                        memset(data+37, '0', 1);
                        memset(data+49, '~', 1);
                        memset(data+97, ' ', 5);
                        }
                    if(senstate[IR]=='1')
                        {
                        memset(data+38, '1', 1);
                        }
                    else
                        {
                        memset(data+38, '0', 1);
                        memset(data+50, '~', 1);
                        memset(data+102, ' ', 5);
                        }
                    data[BYTEDMCD-2] = '\r';
                    data[BYTEDMCD-1] = '\n';
                    data[BYTEDMCD] = '\0';

					string_change_1(data, 40, 11);
                    recount = string_change_2(data, BYTEDMCD);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDMCD-recount);
					usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					break;

                case 38:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    datepd = (insdata[22]-'0')*1000 + (insdata[23]-'0')*100 + (insdata[24]-'0')*10 + (insdata[25]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[33] - '0') * 10 + (insdata[34] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[36] - '0') * 10 + (insdata[37] - '0');
                    if(datepd > 60)
                        goto txend;

                    datamove(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamove(datatime2,insdata,24);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 60 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<11;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[39] = ' ';

                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namemz,12,6);
                        if((fbcs = fopen(namemz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }
#endif

	                    stringmove(csdata, 220, data, 40, 1);                   // Quality Control
    	                stringmove(csdata, 219, data, 41, 1);
        	            stringmove(csdata, 214, data, 42, 4);
            	        stringmove(csdata, 209, data, 46, 3);
                	    stringmove(csdata, 228, data, 49, 2);
                    	data[51] = ' ';

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

	                    string_add_space(csdata, 72, data, 52, 4, 1);
    	                string_add_space(csdata, 68, data, 57, 4, 1);
        	            string_add_space(csdata, 48, data, 62, 4, 4);
            	        string_add_space(csdata, 28, data, 82, 4, 3);
                	    string_add_space(csdata, 105, data, 97, 4, 2);
                    	data[BYTEDMCD-2] = '\r';
                    	data[BYTEDMCD-1] = '\n';
                    	data[BYTEDMCD] = '\0';

                    	recount = string_change_2(data, BYTEDMCD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                    	write(portfd, &data, BYTEDMCD-recount);
						usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
                    break;

                case 23:
                case 24:
                case 25:
                case 26:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    switch(ii)
                    {
                    case 23: count1 = insdata[22] - '0';break;
                    case 24: count1 = (insdata[22] - '0') * 10 + (insdata[23] - '0');break;
                    case 25: count1 = (insdata[22] - '0') * 100 + (insdata[23] - '0') * 10 + (insdata[24] - '0');break;
                    case 26: count1 = (insdata[22] - '0') * 1000 + (insdata[23] - '0') * 100 + (insdata[24] - '0') * 10 + (
insdata[25] - '0');break;
                    default: returnTorF(portfd, returntf, 0);
                    }

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<11;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[39] = ' ';

					datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namemz,12,6);
                        if((fbcs = fopen(namemz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[48]=='0' && csdata[49]=='0' && csdata[50]=='0' && csdata[51]=='0')
                        {
                        csdata[48] = ' ';
                        csdata[49] = ' ';
                        csdata[50] = ' ';
                        }
                    if(csdata[52]=='0' && csdata[53]=='0' && csdata[54]=='0' && csdata[55]=='0')
                        {
                        csdata[52] = ' ';
                        csdata[53] = ' ';
                        csdata[54] = ' ';
                        }
                    if(csdata[56]=='0' && csdata[57]=='0' && csdata[58]=='0' && csdata[59]=='0')
                        {
                        csdata[56] = ' ';
                        csdata[57] = ' ';
                        csdata[58] = ' ';
                        }
                    if(csdata[60]=='0' && csdata[61]=='0' && csdata[62]=='0' && csdata[63]=='0')
                        {
                        csdata[61] = ' ';
                        csdata[62] = ' ';
                        csdata[63] = ' ';
                        }

#endif

                        stringmove(csdata, 220, data, 40, 1);                   // Quality Control
                        stringmove(csdata, 219, data, 41, 1);
                        stringmove(csdata, 214, data, 42, 4);
                        stringmove(csdata, 209, data, 46, 3);
                        stringmove(csdata, 228, data, 49, 2);
                        data[51] = ' ';

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

                        string_add_space(csdata, 72, data, 52, 4, 1);
                        string_add_space(csdata, 68, data, 57, 4, 1);
                        string_add_space(csdata, 48, data, 62, 4, 4);
                        string_add_space(csdata, 28, data, 82, 4, 3);
                        string_add_space(csdata, 105, data, 97, 4, 2);
                        data[BYTEDMCD-2] = '\r';
                        data[BYTEDMCD-1] = '\n';
                        data[BYTEDMCD] = '\0';

                        recount = string_change_2(data, BYTEDMCD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDMCD-recount);
						usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
                    break;

//		    	default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
		    	}
			}

        else if(strstr(insdata, "DMSD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdmsd, sizeof(helpdmsd), MDELAY);
            else
                {
            switch(ii)
                {
                case 4:
                    stringmove(insdata, 0, data, 0, 4);         // Instruction
                    data[4] = ' ';
                    stationid(stid);                            // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                   // Time
//          for(x=0;x<20;x++)
//          printf("%c",presenttime[x]);
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<9;x++)                           // Data Index
                        {
                        data[28+x] = '1';
                        }
					data[37] = ' ';

                    if((fbcs = fopen("/usr/minss.tmp","r")) == NULL)
                        goto txend;
                    fread(&csdata, BYTENUMMS, 1, fbcs);
                    fclose(fbcs);

                    stringmove(csdata, 40, data, 38, 9);      // Quality Control
					data[47] = ' ';
				
                    string_add_space(csdata, 4, data, 48, 4, 9);
                    data[BYTEDMSD-2] = '\r';
                    data[BYTEDMSD-1] = '\n';
                    data[BYTEDMSD] = '\0';
                    recount = string_change_2(data, BYTEDMSD);
					rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDMSD-recount);
					usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;

               case 38:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    datepd = (insdata[22]-'0')*1000 + (insdata[23]-'0')*100 + (insdata[24]-'0')*10 + (insdata[25]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[33] - '0') * 10 + (insdata[34] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[36] - '0') * 10 + (insdata[37] - '0');
                    if(datepd > 60)
                        goto txend;

                    datamove(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamove(datatime2,insdata,24);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 60 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<9;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[37] = ' ';

                    datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namems,12,6);
                        if((fbcs = fopen(namems,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMS;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMS, 1, fbcs);
                        fclose(fbcs);

                        stringmove(csdata, 40, data, 38, 9);           // Quality Control
                        data[47] = ' ';

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

                        string_add_space(csdata, 4, data,  48, 4, 9);
                        data[BYTEDMSD-2] = '\r';
                        data[BYTEDMSD-1] = '\n';
                        data[BYTEDMSD] = '\0';

                        recount = string_change_2(data, BYTEDMSD);
                    	rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDMSD-recount);
						usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
                    break;

                case 23:
                case 24:
                case 25:
                case 26:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    switch(ii)
                    {
                    case 23: count1 = insdata[22] - '0';break;
                    case 24: count1 = (insdata[22] - '0') * 10 + (insdata[23] - '0');break;
                    case 25: count1 = (insdata[22] - '0') * 100 + (insdata[23] - '0') * 10 + (insdata[24] - '0');break;
                    case 26: count1 = (insdata[22] - '0') * 1000 + (insdata[23] - '0') * 100 + (insdata[24] - '0') * 10 + (
insdata[25] - '0');break;
                    default: returnTorF(portfd, returntf, 0);
                    }

                   stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<9;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[37] = ' ';

                    datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,14340);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namems,12,6);
                        if((fbcs = fopen(namems,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMS;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMS, 1, fbcs);
                        fclose(fbcs);

                        stringmove(csdata, 40, data, 38, 9);           // Quality Control
                        data[47] = ' ';

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

                        string_add_space(csdata, 4, data,  48, 4, 9);
                        data[BYTEDMSD-2] = '\r';
                        data[BYTEDMSD-1] = '\n';
                        data[BYTEDMSD] = '\0';

                        recount = string_change_2(data, BYTEDMSD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDMSD-recount);
						usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "DMRD") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdmrd, sizeof(helpdmrd), MDELAY);
            else
                {
		    if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
        		td = 0;
    		else
        		{
        		read(fdparame,&paramete,4);
        		close(fdparame);
        		paramete[4]='\0';
        		td = atof(paramete);
//      		printf("td=%d\n",td);
        		if(td<-180 || td>180)
            		td = 0;
        		}

			switch(ii)
		    	{
		    	case 4: 
                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<26;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[54] = ' ';

					if((fbcs = fopen("/usr/minrr.tmp","r")) == NULL)
			    		break;
                	fread(&csdata, BYTENUMMR, 1, fbcs);
                	fclose(fbcs);

                	stringmove(csdata, 108, data, 55, 26);                  // Quality Control
                	data[81] = ' ';

//					printf("S1:\n%s\n",csdata);
                	string_add_space(csdata, 4, data, 82, 4, 26);

                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
                    if(senstate[GR]=='1')
                        {
                        memset(data+28, '1', 2);
                        }
                    else
                        {
                        memset(data+28, '0', 2);
                        memset(data+55, '~', 2);
                        memset(data+82, ' ', 10);
                        }
                    if(senstate[NR]=='1')
                        {
                        memset(data+30, '1', 2);
                        }
                    else
                        {
                        memset(data+30, '0', 2);
                        memset(data+57, '~', 2);
                        memset(data+92, ' ', 10);
                        }
                    if(senstate[DR]=='1')
                        {
                        memset(data+32, '1', 3);
						memset(data+39, '1', 1);
                        }
                    else
                        {
                        memset(data+32, '0', 3);
                        memset(data+59, '~', 3);
                        memset(data+102, ' ', 15);
                        memset(data+39, '0', 1);
                        memset(data+66, '~', 1);
                        memset(data+137, ' ', 5);
                        }
                    if(senstate[SR]=='1')
                        {
                        memset(data+35, '1', 2);
                        }
                    else
                        {
                        memset(data+35, '0', 2);
                        memset(data+62, '~', 2);
                        memset(data+117, ' ', 10);
                        }
                    if(senstate[RR]=='1')
                        {
                        memset(data+37, '1', 2);
                        }
                    else
                        {
                        memset(data+37, '0', 2);
                        memset(data+64, '~', 2);
                        memset(data+127, ' ', 10);
                        }
                    memset(data+40, '0', 2);
                    memset(data+67, '~', 2);
                    memset(data+142, ' ', 10);
                    if(senstate[UR]=='1')
                        {
                        memset(data+42, '1', 2);
                        }
                    else
                        {
                        memset(data+42, '0', 2);
                        memset(data+69, '~', 2);
                        memset(data+152, ' ', 10);
                        }
                    if(senstate[UVA]=='1')
                        {
                        memset(data+44, '1', 2);
                        }
                    else
                        {
                        memset(data+44, '0', 2);
                        memset(data+71, '~', 2);
                        memset(data+162, ' ', 10);
                        }
                    if(senstate[UVB]=='1')
                        {
                        memset(data+46, '1', 2);
                        }
                    else
                        {
                        memset(data+46, '0', 2);
                        memset(data+73, '~', 2);
                        memset(data+172, ' ', 10);
                        }
                    if(senstate[AR]=='1')
                        {
                        memset(data+48, '1', 2);
                        }
                    else
                        {
                        memset(data+48, '0', 2);
                        memset(data+75, '~', 2);
                        memset(data+182, ' ', 10);
                        }
                    if(senstate[TR]=='1')
                        {
                        memset(data+50, '1', 2);
                        }
                    else
                        {
                        memset(data+50, '0', 2);
                        memset(data+77, '~', 2);
                        memset(data+192, ' ', 10);
                        }
                    if(senstate[PR]=='1')
                        {
                        memset(data+52, '1', 2);
                        }
                    else
                        {
                        memset(data+52, '0', 2);
                        memset(data+79, '~', 2);
                        memset(data+202, ' ',10);
                        }
                	data[BYTEDMRD-2] = '\r';
                	data[BYTEDMRD-1] = '\n';
                	data[BYTEDMRD] = '\0';
//					printf("data=%s",data);

					string_change_1(data, 55, 26);
					recount = string_change_2(data, BYTEDMRD);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                	write(portfd, &data, BYTEDMRD-recount);
					usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                	break;

				case 38:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                	    goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

                    datepd = (insdata[22]-'0')*1000 + (insdata[23]-'0')*100 + (insdata[24]-'0')*10 + (insdata[25]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[33] - '0') * 10 + (insdata[34] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[36] - '0') * 10 + (insdata[37] - '0');
                    if(datepd > 60)
                        goto txend;

					datamove(datatime1,insdata,7);
					timedp1 = timedistance(datatime1);	
					datamove(datatime2,insdata,24);
					timedp2 = timedistance(datatime2);
					count1 = (timedp2 - timedp1) / 60 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<26;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[54] = ' ';

                    datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,-60);

                    for( ;count1>0;count1--)
	                    {
                        stringmove(datatime1,0,namemr,12,6);
//						printf("namemr=%s\n",namemr);
                        if((fbcs = fopen(namemr,"r")) == NULL)
							{
							returnTorF(portfd, returntf, 0);
							goto txend;
							}
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMR;
//						printf("id=%d,idn=%d\n",id,idn);
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMR, 1, fbcs);
//						printf("csdata=%s\n",csdata);
						fclose(fbcs);

	                    stringmove(sourcetime,0,data,13,2);
         	            stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

                    	stringmove(csdata, 108, data, 55, 26);                  // Quality Control
                    	data[81] = ' ';
    	                string_add_space(csdata, 4, data, 82, 4, 26);
        	            data[BYTEDMRD-2] = '\r';
            	        data[BYTEDMRD-1] = '\n';
                	    data[BYTEDMRD] = '\0';

                    	recount = string_change_2(data, BYTEDMRD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                    	write(portfd, &data, BYTEDMRD-recount);
						usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
						timeconvert(sourcetime,60);
                        }
//					fclose(fbxz4);
					break;

				case 23:
				case 24:
				case 25:
				case 26:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                    	goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
                    datepd = (insdata[19] - '0') * 10 + (insdata[20] - '0');
                    if(datepd > 60)
                        goto txend;

//					datamove(datatime1,insdata,7);                                        
					switch(ii)
						{
						case 23: count1 = insdata[22] - '0';break;
						case 24: count1 = (insdata[22] - '0') * 10 + (insdata[23] - '0');break;
						case 25: count1 = (insdata[22] - '0') * 100 + (insdata[23] - '0') * 10 + (insdata[24] - '0');break;
						case 26: count1 = (insdata[22] - '0') * 1000 + (insdata[23] - '0') * 100 + (insdata[24] - '0') * 10 + (insdata[25] - '0');break;
						default: returnTorF(portfd, returntf, 0);
						}

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 16);
                    data[27] = ' ';
                    for(x=0;x<26;x++)                                       // Data Index
                        {
                        data[28+x] = '1';
                        }
                    data[54] = ' ';

                    datamove(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,-60);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namemr,12,6);
                        if((fbcs = fopen(namemr,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = fenzhong_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMMR;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMMR, 1, fbcs);
                        fclose(fbcs);

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
                        stringmove(sourcetime,8,data,25,2);

						stringmove(csdata, 108, data, 55, 26);                  // Quality Control
                        data[81] = ' ';
                        string_add_space(csdata, 4, data, 82, 4, 26);
                        data[BYTEDMRD-2] = '\r';
                        data[BYTEDMRD-1] = '\n';
                        data[BYTEDMRD] = '\0';

                        recount = string_change_2(data, BYTEDMRD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDMRD-recount);
						usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,60);
						timeconvert(sourcetime,60);
						} 
					break;
//				default: returnTorF(portfd, returntf, 0); 
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

		else if(strstr(insdata, "DMOD") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdmod, sizeof(helpdmod), MDELAY);
            else
                {
				returnTorF(portfd, returntf, 0);
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
//            	write(portfd, dmod, sizeof(dmod));
//            	usleep(MTIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
			}

        else if(strstr(insdata, "DHGD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdhgd, sizeof(helpdhgd), MDELAY);
            else
                {
            switch(ii)
                {
                case 4:
                	stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<63;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[88] = '-';
                    data[89] = '-';
                    data[90] = '-';
                    data[91] = '-';
                    data[92] = '-';
                    data[93] = ' ';

                    if((fbcs = fopen("/usr/hourzz.tmp","r")) == NULL)
                            goto txend;
                    fread(&csdata, BYTENUMHZ, 1, fbcs);
                    fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[80]=='0' && csdata[81]=='0' && csdata[82]=='0' && csdata[83]=='0')
                        {
                        csdata[80] = ' ';
                        csdata[81] = ' ';
                        csdata[82] = ' ';
                        }
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                    if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
                    if(csdata[139]=='*')
                        {
                        csdata[136] = ' ';
                        csdata[137] = ' ';
                        csdata[138] = ' ';
                        }
#endif

//					printf("csdata=%s",csdata);
                    stringmove(csdata, 346, data, 94, 12);                  // Quality Control
                    stringmove(csdata, 365, data, 106, 8);
                    stringmove(csdata, 379, data, 114, 1);
                    stringmove(csdata, 381, data, 115, 20);
                    stringmove(csdata, 406, data, 135, 22);
                    data[157] = '-';
                    data[158] = '-';
                    data[159] = '-';
                    data[160] = '-';
                    data[161] = '-';
                    data[162] = ' ';

                    string_add_space(csdata,   4, data, 163, 4, 12);
                    string_add_space(csdata,  80, data, 223, 4, 3);
                    string_add_space(csdata,  92, data, 238, 4, 5);
                    string_add_space(csdata, 136, data, 263, 4, 1);
                    string_add_space(csdata, 144, data, 268, 4, 5);
                    string_add_space(csdata, 164, data, 293, 5, 2);
                    string_add_space(csdata, 174, data, 305, 4, 1);
                    string_add_space(csdata, 178, data, 310, 5, 1);
                    string_add_space(csdata, 183, data, 316, 4, 1);
                    string_add_space(csdata, 187, data, 321, 4, 10);
                    string_add_space(csdata, 247, data, 371, 4, 10);
                    string_add_space(csdata, 287, data, 421, 5, 2);
                    string_add_space(csdata, 297, data, 433, 4, 1);
                    string_add_space(csdata, 301, data, 438, 5, 1);
                    string_add_space(csdata, 306, data, 444, 4, 2);
                    string_add_space(csdata, 314, data, 454, 12, 1);
                    string_add_space(csdata, 326, data, 467, 4, 5);
                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
                    if(senstate[WD]=='1')
                        {
                        memset(data+25, '1', 1);
						memset(data+27, '1', 1);
                        memset(data+29, '1', 1);
                        memset(data+32, '1', 1);
                        memset(data+34, '1', 1);
                        }
                    else
                        {
                        memset(data+25, '0', 1);
                        memset(data+94, '~', 1);
                        memset(data+163, ' ', 5);
                        memset(data+27, '0', 1);
                        memset(data+96, '~', 1);
                        memset(data+173, ' ', 5);
                        memset(data+29, '0', 1);
                        memset(data+98, '~', 1);
                        memset(data+183, ' ', 5);
                        memset(data+32, '0', 1);
                        memset(data+101, '~', 1);
                        memset(data+198, ' ', 5);
                        memset(data+34, '0', 1);
                        memset(data+103, '~', 1);
                        memset(data+208, ' ', 5);
                        }
                    if(senstate[WS]=='1')
                        {
                        memset(data+26, '1', 1);
                        memset(data+28, '1', 9);
                        }
                    else
                        {
                        memset(data+26, '0', 1);
                        memset(data+95, '~', 1);
                        memset(data+168, ' ', 5);
                        memset(data+28, '0', 9);
                        memset(data+97, '~', 9);
                        memset(data+178, ' ', 45);
                        }
                    if(senstate[RAT]=='1')
                        {
                        memset(data+37, '1', 1);
                        }
                    else
                        {
                        memset(data+37, '0', 1);
                        memset(data+106, '~', 1);
                        memset(data+223, ' ', 5);
                        }
                    if(senstate[RAT1]=='1')
                        {
                        memset(data+38, '1', 1);
                        }
                    else
                        {
                        memset(data+38, '0', 1);
                        memset(data+107, '~', 1);
                        memset(data+228, ' ', 5);
                        }
                    if(senstate[RAW]=='1')
                        {
                        memset(data+39, '1', 1);
                        }
                    else
                        {
                        memset(data+39, '0', 1);
                        memset(data+108, '~', 1);
                        memset(data+233, ' ', 5);
                        }
                    if(senstate[T0]=='1')
                        {
                        memset(data+40, '1', 5);
                        }
                    else
                        {
                        memset(data+40, '0', 5);
                        memset(data+109, '~', 5);
                        memset(data+238, ' ', 25);
                        }
                    if(senstate[TW]=='1')
                        {
                        memset(data+45, '1', 1);
                        }
                    else
                        {
                        memset(data+45, '0', 1);
                        memset(data+114, '~', 1);
                        memset(data+263, ' ', 5);
                        }
                    if(senstate[U]=='1')
                        {
                        memset(data+46, '1', 3);
                        }
                    else
                        {
                        memset(data+46, '0', 3);
                        memset(data+115, '~', 3);
                        memset(data+268, ' ', 15);
                        }
                    if(senstate[T0]=='1' && senstate[U]=='1')
                        {
                        memset(data+49, '1', 2);
                        }
                    else
                        {
                        memset(data+49, '0', 2);
                        memset(data+118, '~', 2);
                        memset(data+283, ' ', 10);
                        }
                    if(senstate[P]=='1')
                        {
                        memset(data+51, '1', 5);
                        }
                    else
                        {
                        memset(data+51, '0', 5);
                        memset(data+120, '~', 5);
                        memset(data+293, ' ', 28);
                        }
                    if(senstate[TG]=='1')
                        {
                        memset(data+56, '1', 5);
                        }
                    else
                        {
                        memset(data+56, '0', 5);
                        memset(data+125, '~', 5);
                        memset(data+321, ' ', 25);
                        }
                    if(senstate[ST0]=='1')
                        {
                        memset(data+61, '1', 5);
                        }
                    else
                        {
                        memset(data+61, '0', 5);
                        memset(data+130, '~', 5);
                        memset(data+346, ' ', 25);
                        }
                    if(senstate[ST1]=='1')
                        {
                        memset(data+66, '1', 1);
                        }
                    else
                        {
                        memset(data+66, '0', 1);
                        memset(data+135, '~', 1);
                        memset(data+371, ' ', 5);
                        }
                    if(senstate[ST2]=='1')
                        {
                        memset(data+67, '1', 1);
                        }
                    else
                        {
                        memset(data+67, '0', 1);
                        memset(data+136, '~', 1);
                        memset(data+376, ' ', 5);
                        }
                    if(senstate[ST3]=='1')
                        {
                        memset(data+68, '1', 1);
                        }
                    else
                        {
                        memset(data+68, '0', 1);
                        memset(data+137, '~', 1);
                        memset(data+381, ' ', 5);
                        }
                    if(senstate[ST4]=='1')
                        {
                        memset(data+69, '1', 1);
                        }
                    else
                        {
                        memset(data+69, '0', 1);
                        memset(data+138, '~', 1);
                        memset(data+386, ' ', 5);
                        }
                    if(senstate[ST5]=='1')
                        {
                        memset(data+70, '1', 1);
                        }
                    else
                        {
                        memset(data+70, '0', 1);
                        memset(data+139, '~', 1);
                        memset(data+391, ' ', 5);
                        }
                    if(senstate[ST6]=='1')
                        {
                        memset(data+71, '1', 1);
                        }
                    else
                        {
                        memset(data+71, '0', 1);
                        memset(data+140, '~', 1);
                        memset(data+396, ' ', 5);
                        }
                    if(senstate[ST7]=='1')
                        {
                        memset(data+72, '1', 1);
                        }
                    else
                        {
                        memset(data+72, '0', 1);
                        memset(data+141, '~', 1);
                        memset(data+401, ' ', 5);
                        }
                    if(senstate[ST8]=='1')
                        {
                        memset(data+73, '1', 1);
                        }
                    else
                        {
                        memset(data+73, '0', 1);
                        memset(data+142, '~', 1);
                        memset(data+406, ' ', 5);
                        }
                    if(senstate[LE]=='1')
                        {
                        memset(data+74, '1', 2);
                        }
                    else
                        {
                        memset(data+74, '0', 2);
                        memset(data+143, '~', 2);
                        memset(data+411, ' ', 10);
                        }
                    if(senstate[VI]=='1')
                        {
                        memset(data+76, '1', 3);
                        }
                    else
                        {
                        memset(data+76, '0', 3);
                        memset(data+145, '~', 3);
                        memset(data+421, ' ', 17);
                        }
                    if(senstate[CH]=='1')
                        {
                        memset(data+79, '1', 1);
                        }
                    else
                        {
                        memset(data+79, '0', 1);
                        memset(data+148, '~', 1);
                        memset(data+438, ' ', 6);
                        }
                    if(senstate[TCA]=='1')
                        {
                        memset(data+80, '1', 1);
                        }
                    else
                        {
                        memset(data+80, '0', 1);
                        memset(data+149, '~', 1);
                        memset(data+444, ' ', 5);
                        }
                    if(senstate[LCA]=='1')
                        {
                        memset(data+81, '1', 1);
                        }
                    else
                        {
                        memset(data+81, '0', 1);
                        memset(data+150, '~', 1);
                        memset(data+449, ' ', 5);
                        }
                    if(senstate[WW]=='1')
                        {
                        memset(data+82, '1', 1);
                        }
                    else
                        {
                        memset(data+82, '0', 1);
                        memset(data+151, '~', 1);
                        memset(data+454, ' ', 13);
                        }
                    if(senstate[SD]=='1')
                        {
                        memset(data+83, '1', 1);
                        }
                    else
                        {
                        memset(data+83, '0', 1);
                        memset(data+152, '~', 1);
                        memset(data+467, ' ', 5);
                        }
                    if(senstate[FR]=='1')
                        {
                        memset(data+84, '1', 1);
                        }
                    else
                        {
                        memset(data+84, '0', 1);
                        memset(data+153, '~', 1);
                        memset(data+472, ' ', 5);
                        }
                    if(senstate[WI]=='1')
                        {
                        memset(data+85, '1', 1);
                        }
                    else
                        {
                        memset(data+85, '0', 1);
                        memset(data+154, '~', 1);
                        memset(data+477, ' ', 5);
                        }
                    if(senstate[FSD]=='1')
                        {
                        memset(data+86, '1', 1);
                        }
                    else
                        {
                        memset(data+86, '0', 1);
                        memset(data+155, '~', 1);
                        memset(data+482, ' ', 5);
                        }
                    if(senstate[LNF]=='1')
                        {
                        memset(data+87, '1', 1);
                        }
                    else
                        {
                        memset(data+87, '0', 1);
                        memset(data+156, '~', 1);
                        memset(data+487, ' ', 5);
                        }
                    memset(data+88, '0', 5);
                    memset(data+157, '~', 5);
                    memset(data+492, ' ', 25);
                    data[BYTEDHGD-2] = '\r';
                    data[BYTEDHGD-1] = '\n';
					data[BYTEDHGD] = '\0';

					string_change_1(data, 94, 68);
                    recount = string_change_2(data, BYTEDHGD);
//                    data[BYTEDHGD-recount-3] = '\r';
//                    data[BYTEDHGD-recount-2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDHGD-recount);
                    usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);

					write(fd_ttyS0, insgps, sizeof(insgps));				//GPS
                    break;

                case 32:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
	                    goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;

	                datepd = (insdata[19]-'0')*1000 + (insdata[20]-'0')*100 + (insdata[21]-'0') * 10 +(insdata[22]-'0');
                    if(datepd < 2000)
	                    goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[24] - '0') * 10 + (insdata[25] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 24)
                        goto txend;

                    datamovexs(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamovexs(datatime2,insdata,21);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 3600 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<63;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[88] = '0';
                    data[89] = '0';
                    data[90] = '0';
                    data[91] = '0';
                    data[92] = '0';
                    data[93] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                    	{
                        stringmove(datatime1,0,namehz,12,4);
                        if((fbcs = fopen(namehz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[80]=='0' && csdata[81]=='0' && csdata[82]=='0' && csdata[83]=='0')
                        {
                        csdata[80] = ' ';
                        csdata[81] = ' ';
                        csdata[82] = ' ';
                        }
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                    if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
                    if(csdata[139]=='*')
                        {
                        csdata[136] = ' ';
                        csdata[137] = ' ';
                        csdata[138] = ' ';
                        }
#endif

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

	                    stringmove(csdata, 346, data, 94, 12);                  // Quality Control
    	                stringmove(csdata, 365, data, 106, 8);
        	            stringmove(csdata, 379, data, 114, 1);
            	        stringmove(csdata, 381, data, 115, 20);
                	    stringmove(csdata, 406, data, 135, 22);
						memset(data+157, ' ', 5);
	                    data[162] = ' ';

    	                string_add_space(csdata,   4, data, 163, 4, 12);
        	            string_add_space(csdata,  80, data, 223, 4, 3);
            	        string_add_space(csdata,  92, data, 238, 4, 5);
                	    string_add_space(csdata, 136, data, 263, 4, 1);
	                    string_add_space(csdata, 144, data, 268, 4, 5);
    	                string_add_space(csdata, 164, data, 293, 5, 2);
        	            string_add_space(csdata, 174, data, 305, 4, 1);
            	        string_add_space(csdata, 178, data, 310, 5, 1);
                	    string_add_space(csdata, 183, data, 316, 4, 1);
	                    string_add_space(csdata, 187, data, 321, 4, 10);
    	                string_add_space(csdata, 247, data, 371, 4, 10);
        	            string_add_space(csdata, 287, data, 421, 5, 2);
            	        string_add_space(csdata, 297, data, 433, 4, 1);
                	    string_add_space(csdata, 301, data, 438, 5, 1);
	                    string_add_space(csdata, 306, data, 444, 4, 2);
    	                string_add_space(csdata, 314, data, 454, 12, 1);
        	            string_add_space(csdata, 326, data, 467, 4, 5);

                        memset(data+492, ' ', 25);
	                    data[BYTEDHGD-2] = '\r';
    	                data[BYTEDHGD-1] = '\n';
        	            data[BYTEDHGD] = '\0';

            	        recount = string_change_2(data, BYTEDHGD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                	    write(portfd, &data, BYTEDHGD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
						}
					break;						

                case 20:
                case 21:
                case 22:
                case 23:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
	                    goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
//                  datamovexs(datatime1,insdata,7);
                    switch(ii)
                    	{
                    	case 20: count1 = insdata[19] - '0';break;
                    	case 21: count1 = (insdata[19] - '0') * 10 + (insdata[20] - '0');break;
                    	case 22: count1 = (insdata[19] - '0') * 100 + (insdata[20] - '0') * 10 + (insdata[21] - '0');break;
                    	case 23: count1 = (insdata[19] - '0') * 1000 + (insdata[20] - '0') * 100 + (insdata[21] - '0') * 10 + (insdata[22] - '0');break;
                    	default:  returnTorF(portfd, returntf, 0);
                    	}

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<63;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[88] = '0';
                    data[89] = '0';
                    data[90] = '0';
                    data[91] = '0';
                    data[92] = '0';
                    data[93] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehz,12,4);
                        if((fbcs = fopen(namehz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[80]=='0' && csdata[81]=='0' && csdata[82]=='0' && csdata[83]=='0')
                        {
                        csdata[80] = ' ';
                        csdata[81] = ' ';
                        csdata[82] = ' ';
                        }
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                    if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
                    if(csdata[135]=='*')
                        {
                        csdata[132] = ' ';
                        csdata[133] = ' ';
                        csdata[134] = ' ';
                        }
#endif

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

                        stringmove(csdata, 346, data, 94, 12);                  // Quality Control
                        stringmove(csdata, 365, data, 106, 8);
                        stringmove(csdata, 379, data, 114, 1);
                        stringmove(csdata, 381, data, 115, 20);
                        stringmove(csdata, 406, data, 135, 22);
                        memset(data+157, ' ', 5);
                        data[162] = ' ';

                        string_add_space(csdata,   4, data, 163, 4, 12);
                        string_add_space(csdata,  80, data, 223, 4, 3);
                        string_add_space(csdata,  92, data, 238, 4, 5);
                        string_add_space(csdata, 136, data, 263, 4, 1);
                        string_add_space(csdata, 144, data, 268, 4, 5);
                        string_add_space(csdata, 164, data, 293, 5, 2);
                        string_add_space(csdata, 174, data, 305, 4, 1);
                        string_add_space(csdata, 178, data, 310, 5, 1);
                        string_add_space(csdata, 183, data, 316, 4, 1);
                        string_add_space(csdata, 187, data, 321, 4, 10);
                        string_add_space(csdata, 247, data, 371, 4, 10);
                        string_add_space(csdata, 287, data, 421, 5, 2);
                        string_add_space(csdata, 297, data, 433, 4, 1);
                        string_add_space(csdata, 301, data, 438, 5, 1);
                        string_add_space(csdata, 306, data, 444, 4, 2);
                        string_add_space(csdata, 314, data, 454, 12, 1);
                        string_add_space(csdata, 326, data, 467, 4, 5);

                        memset(data+492, ' ', 25);
                        data[BYTEDHGD-2] = '\r';
                        data[BYTEDHGD-1] = '\n';
                        data[BYTEDHGD] = '\0';

                        recount = string_change_2(data, BYTEDHGD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDHGD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }       // if(strstr(insdata, "DHGD") != 0))


        else if(strstr(insdata, "DHCD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdhcd, sizeof(helpdhcd), MDELAY);
            else
                {
            switch(ii)
                {
                case 4:
                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<25;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[50] = ' ';

                    if((fbcs = fopen("/usr/hourzz.tmp","r")) == NULL)
                        goto txend;
                    fread(&csdata, BYTENUMHZ, 1, fbcs);
                    fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                   if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
#endif

                    stringmove(csdata, 373, data, 51, 5);                   // Quality Control
                    stringmove(csdata, 378, data, 56, 1);
                    stringmove(csdata, 366, data, 57, 2);
                    stringmove(csdata, 358, data, 59, 7);
                    stringmove(csdata, 396, data, 66, 10);
                    data[76] = ' ';

                    string_add_space(csdata, 112, data,  77, 4, 5);
                    string_add_space(csdata, 132, data, 102, 4, 1);
                    string_add_space(csdata,  84, data, 107, 4, 2);
                    string_add_space(csdata,  52, data, 117, 4, 7);
                    string_add_space(csdata, 207, data, 152, 4, 10);

                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
					if(senstate[T1]=='1' || senstate[T2]=='1' ||senstate[T3]=='1')
						{
						memset(data+25, '1', 5);
						}
					else
						{
                        memset(data+25, '0', 5);
                        memset(data+51, '~', 5);
                        memset(data+77, ' ', 25);
                        }
                    if(senstate[SV1]=='1' || senstate[SV2]=='1' ||senstate[SV3]=='1')
                        {
                        memset(data+30, '1', 1);
                        }
                    else
                        {
                        memset(data+30, '0', 1);
                        memset(data+56, '~', 1);
                        memset(data+102, ' ', 5);
                        }
                    if(senstate[RAT1]=='1')
                        {
                        memset(data+31, '1', 1);
                        }
                    else
                        {
                        memset(data+31, '0', 1);
                        memset(data+57, '~', 1);
                        memset(data+107, ' ', 5);
                        }
                    if(senstate[RAW]=='1')
                        {
                        memset(data+32, '1', 1);
                        }
                    else
                        {
                        memset(data+32, '0', 1);
                        memset(data+58, '~', 1);
                        memset(data+112, ' ', 5);
                        }
                    if(senstate[WS1]=='1')
                        {
                        memset(data+33, '1', 7);
                        }
                    else
                        {
                        memset(data+33, '0', 7);
                        memset(data+59, '~', 7);
                        memset(data+117, ' ', 35);
                        }
                    if(senstate[ST0]=='1')
                        {
                        memset(data+40, '1', 5);
                        }
                    else
                        {
                        memset(data+40, '0', 5);
                        memset(data+66, '~', 5);
                        memset(data+152, ' ', 25);
                        }
                    if(senstate[IR]=='1')
                        {
                        memset(data+45, '1', 5);
                        }
                    else
                        {
                        memset(data+45, '0', 5);
                        memset(data+71, '~', 5);
                        memset(data+177, ' ', 25);
                        }
                    data[BYTEDHCD-2] = '\r';
                    data[BYTEDHCD-1] = '\n';
                    data[BYTEDHCD] = '\0';

					string_change_1(data, 25, 25);
                    recount = string_change_2(data, BYTEDHCD);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDHCD-recount);
                    usleep(MTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;

                case 32:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;

                    datepd = (insdata[19]-'0')*1000 + (insdata[20]-'0')*100 + (insdata[21]-'0') * 10 +(insdata[22]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[24] - '0') * 10 + (insdata[25] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 24)
                        goto txend;

                    datamovexs(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamovexs(datatime2,insdata,21);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 3600 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<25;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[50] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehz,12,4);
                        if((fbcs = fopen(namehz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                   if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
#endif

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

	                    stringmove(csdata, 373, data, 51, 5);                   // Quality Control
    	                stringmove(csdata, 378, data, 56, 1);
        	            stringmove(csdata, 366, data, 57, 2);
            	        stringmove(csdata, 358, data, 59, 7);
                	    stringmove(csdata, 396, data, 66, 10);
                    	data[76] = ' ';

	                    string_add_space(csdata, 112, data,  77, 4, 5);
    	                string_add_space(csdata, 132, data, 102, 4, 1);
        	            string_add_space(csdata,  84, data, 107, 4, 2);
            	        string_add_space(csdata,  52, data, 117, 4, 7);
                	    string_add_space(csdata, 207, data, 152, 4, 10);
        	            data[BYTEDHCD-2] = '\r';
            	        data[BYTEDHCD-1] = '\n';
                	    data[BYTEDHCD] = '\0';

                    	recount = string_change_2(data, BYTEDHCD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                    	write(portfd, &data, BYTEDHCD-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;

                case 20:
                case 21:
                case 22:
                case 23:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
//                  datamovexs(datatime1,insdata,7);
                    switch(ii)
                        {
                        case 20: count1 = insdata[19] - '0';break;
                        case 21: count1 = (insdata[19] - '0') * 10 + (insdata[20] - '0');break;
                        case 22: count1 = (insdata[19] - '0') * 100 + (insdata[20] - '0') * 10 + (insdata[21] - '0');break;
                        case 23: count1 = (insdata[19] - '0') * 1000 + (insdata[20] - '0') * 100 + (insdata[21] - '0') * 10 + (insdata[22] - '0');break;
                        default:  returnTorF(portfd, returntf, 0);
                        }

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<25;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[50] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehz,12,4);
                        if((fbcs = fopen(namehz,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHZ;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHZ, 1, fbcs);
                        fclose(fbcs);

#ifdef RAIN0000
                    if(csdata[84]=='0' && csdata[85]=='0' && csdata[86]=='0' && csdata[87]=='0')
                        {
                        csdata[84] = ' ';
                        csdata[85] = ' ';
                        csdata[86] = ' ';
                        }
                   	if(csdata[88]=='0' && csdata[89]=='0' && csdata[90]=='0' && csdata[91]=='0')
                        {
                        csdata[88] = ' ';
                        csdata[89] = ' ';
                        csdata[90] = ' ';
                        }
#endif

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

                        stringmove(csdata, 373, data, 51, 5);                   // Quality Control
                        stringmove(csdata, 378, data, 56, 1);
                        stringmove(csdata, 366, data, 57, 2);
                        stringmove(csdata, 358, data, 59, 7);
                        stringmove(csdata, 396, data, 66, 10);
                        data[76] = ' ';

                        string_add_space(csdata, 112, data,  77, 4, 5);
                        string_add_space(csdata, 132, data, 102, 4, 1);
                        string_add_space(csdata,  84, data, 107, 4, 2);
                        string_add_space(csdata,  52, data, 117, 4, 7);
                        string_add_space(csdata, 207, data, 152, 4, 10);
                        data[BYTEDHCD-2] = '\r';
                        data[BYTEDHCD-1] = '\n';
                        data[BYTEDHCD] = '\0';

                        recount = string_change_2(data, BYTEDHCD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDHCD-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;
//		  		default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
		   		}
			}

        else if(strstr(insdata, "DHSD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdhsd, sizeof(helpdhsd), MDELAY);
            else
                {
            switch(ii)
                {
                case 4:
                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<49;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[74] = ' ';

                    if((fbcs = fopen("/usr/hourss.tmp","r")) == NULL)
                        goto txend;
                    fread(&csdata, BYTENUMHS, 1, fbcs);
                    fclose(fbcs);

                    stringmove(csdata, 200, data, 75, 49);                   // Quality Control
                    data[124] = ' ';

                    string_add_space(csdata, 4, data, 125, 4, 49);
                    data[BYTEDHSD-2] = '\r';
                    data[BYTEDHSD-1] = '\n';
                    data[BYTEDHSD] = '\0';
                    recount = string_change_2(data, BYTEDHSD);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDHSD-recount);
                    usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;

                case 32:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;

                    datepd = (insdata[19]-'0')*1000 + (insdata[20]-'0')*100 + (insdata[21]-'0') * 10 +(insdata[22]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[24] - '0') * 10 + (insdata[25] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 24)
                        goto txend;

                    datamovexs(datatime1,insdata,7);
                    timedp1 = timedistance(datatime1);
                    datamovexs(datatime2,insdata,21);
                    timedp2 = timedistance(datatime2);
                    count1 = (timedp2 - timedp1) / 3600 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<49;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[74] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehs,12,4);
                        if((fbcs = fopen(namehs,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHS;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHS, 1, fbcs);
                        fclose(fbcs);

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

	                    stringmove(csdata, 200, data, 75, 49);                   // Quality Control
                    	data[124] = ' ';

        	            string_add_space(csdata, 4, data, 125, 4, 49);
        	            data[BYTEDHSD-2] = '\r';
            	        data[BYTEDHSD-1] = '\n';
                	    data[BYTEDHSD] = '\0';

                    	recount = string_change_2(data, BYTEDHSD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                    	write(portfd, &data, BYTEDHSD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;

                case 20:
                case 21:
                case 22:
                case 23:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;
//                  datamovexs(datatime1,insdata,7);
                    switch(ii)
                        {
                        case 20: count1 = insdata[19] - '0';break;
                        case 21: count1 = (insdata[19] - '0') * 10 + (insdata[20] - '0');break;
                        case 22: count1 = (insdata[19] - '0') * 100 + (insdata[20] - '0') * 10 + (insdata[21] - '0');break;
                        case 23: count1 = (insdata[19] - '0') * 1000 + (insdata[20] - '0') * 100 + (insdata[21] - '0') * 10 + (insdata[22] - '0');break;
                        default:  returnTorF(portfd, returntf, 0);
                        }

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, 0);                               // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<49;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[74] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,10800);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehs,12,4);
                        if((fbcs = fopen(namehs,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHS;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata, BYTENUMHS, 1, fbcs);
                        fclose(fbcs);

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

                        stringmove(csdata, 200, data, 75, 49);                   // Quality Control
                        data[124] = ' ';

                        string_add_space(csdata, 4, data, 125, 4, 49);
                        data[BYTEDHSD-2] = '\r';
                        data[BYTEDHSD-1] = '\n';
                        data[BYTEDHSD] = '\0';

                        recount = string_change_2(data, BYTEDHSD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDHSD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;
//		  		default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
		   		}
			}

        else if(strstr(insdata, "DHRD") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdhrd, sizeof(helpdhrd), MDELAY);
            else
                {
            if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
                td = 0;
            else
                {
                read(fdparame,&paramete,4);
                close(fdparame);
                paramete[4]='\0';
                td = atof(paramete);
//              printf("td=%d\n",td);
                if(td<-180 || td>180)
                    td = 0;
                }

			switch(ii)
		    	{
		    	case 4:
                	stringmove(insdata, 0, data, 0, 4);                     // Instruction
                  	data[4] = ' ';
                   	stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<50;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[75] = ' ';

                    if((fbcs = fopen("/usr/hourrr.tmp","r")) == NULL)
                        break;
                    fread(&csdata, BYTENUMHR, 1, fbcs);
                    fclose(fbcs);

                    stringmove(csdata, 204, data, 76, 50);                  // Quality Control
					data[126] = ' ';

                    string_add_space(csdata, 4, data, 127, 4, 50);
                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)       // Data Index
                        usleep(1);
                    else
                        {
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        }
                    if(senstate[GR]=='1')
                        {
                        memset(data+25, '1', 4);
                        }
                    else
                        {
                        memset(data+25, '0', 4);
                        memset(data+76, '~', 4);
                        memset(data+127, ' ', 20);
                        }
                    if(senstate[NR]=='1')
                        {
                        memset(data+29, '1', 6);
                        }
                    else
                        {
                        memset(data+29, '0', 6);
                        memset(data+80, '~', 6);
                        memset(data+147, ' ', 30);
                        }
                    if(senstate[DR]=='1')
                        {
                        memset(data+35, '1', 5);
						memset(data+48, '1', 1);
                        }
                    else
                        {
                        memset(data+35, '0', 5);
                        memset(data+86, '~', 5);
                        memset(data+177, ' ', 25);
                        memset(data+48, '0', 1);
                        memset(data+99, '~', 1);
                        memset(data+242, ' ', 5);
                        }
                    if(senstate[SR]=='1')
                        {
                        memset(data+40, '1', 4);
                        }
                    else
                        {
                        memset(data+40, '0', 4);
                        memset(data+91, '~', 4);
                        memset(data+202, ' ', 20);
                        }
                    if(senstate[RR]=='1')
                        {
                        memset(data+44, '1', 4);
                        }
                    else
                        {
                        memset(data+44, '0', 4);
                        memset(data+95, '~', 4);
                        memset(data+222, ' ', 20);
                        }
                    memset(data+49, '0', 2);
                    memset(data+100, '~', 2);
                    memset(data+247, ' ', 10);
                    if(senstate[UR]=='1')
                        {
                        memset(data+51, '1', 4);
                        }
                    else
                        {
                        memset(data+51, '0', 4);
                        memset(data+102, '~', 4);
                        memset(data+257, ' ', 20);
                        }
                    if(senstate[UVA]=='1')
                        {
                        memset(data+55, '1', 4);
                        }
                    else
                        {
                        memset(data+55, '0', 4);
                        memset(data+106, '~', 4);
                        memset(data+277, ' ', 20);
                        }
                    if(senstate[UVB]=='1')
                        {
                        memset(data+59, '1', 4);
                        }
                    else
                        {
                        memset(data+59, '0', 4);
                        memset(data+110, '~', 4);
                        memset(data+297, ' ', 20);
                        }
                    if(senstate[AR]=='1')
                        {
                        memset(data+63, '1', 4);
                        }
                    else
                        {
                        memset(data+63, '0', 4);
                        memset(data+114, '~', 4);
                        memset(data+317, ' ', 20);
                        }
                    if(senstate[TR]=='1')
                        {
                        memset(data+67, '1', 4);
                        }
                    else
                        {
                        memset(data+67, '0', 4);
                        memset(data+118, '~', 4);
                        memset(data+337, ' ', 20);
                        }
                    if(senstate[PR]=='1')
                        {
                        memset(data+71, '1', 4);
                        }
                    else
                        {
                        memset(data+71, '0', 4);
                        memset(data+122, '~', 4);
                        memset(data+357, ' ', 20);
                        }
                    data[BYTEDHRD-2] = '\r';
                    data[BYTEDHRD-1] = '\n';
                    data[BYTEDHRD] = '\0';

					string_change_1(data, 76, 50);
                    recount = string_change_2(data, BYTEDHRD);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, BYTEDHRD-recount);
                    usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;

				case 32:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
	                    goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                        goto txend;

                    datepd = (insdata[19]-'0')*1000 + (insdata[20]-'0')*100 + (insdata[21]-'0') * 10 + (insdata[22]-'0');
                    if(datepd < 2000)
                        goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[24] - '0') * 10 + (insdata[25] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[27] - '0') * 10 + (insdata[28] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[30] - '0') * 10 + (insdata[31] - '0');
                    if(datepd > 24)
                        goto txend;

					datamovexs(datatime1,insdata,7);
					timedp1 = timedistance(datatime1);	
					datamovexs(datatime2,insdata,21);
					timedp2 = timedistance(datatime2);
					count1 = (timedp2 - timedp1) / 3600 + 1;

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<50;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[75] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,-3600);

                    for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehr,12,4);
                        if((fbcs = fopen(namehr,"r")) == NULL)
							{
							returnTorF(portfd, returntf, 0);
							goto txend;
							}
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHR;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata,BYTENUMHR,1,fbcs);
						fclose(fbcs);
	
                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                       	stringmove(sourcetime,4,data,19,2);
                       	stringmove(sourcetime,6,data,22,2);
//                       	stringmove(sourcetime,8,data,25,2);

                        stringmove(csdata, 204, data, 76, 50);                  // Quality Control
                        data[126] = ' ';

                        string_add_space(csdata, 4, data, 127, 4, 50);
                        data[BYTEDHRD-2] = '\r';
                        data[BYTEDHRD-1] = '\n';
                        data[BYTEDHRD] = '\0';
                        recount = string_change_2(data, BYTEDHRD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDHRD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
						timeconvert(sourcetime,3600);
                        }
					break;

				case 20:
				case 21:
				case 22:
				case 23:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 2000)
                    	goto txend;
                    if(datepd > 2100)
                        goto txend;
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
                        goto txend;
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
                        goto txend;
                    datepd = (insdata[16] - '0') * 10 + (insdata[17] - '0');
                    if(datepd > 24)
                    	goto txend;

//					datamovexs(datatime1,insdata,7);                                        
					switch(ii)
						{
						case 20: count1 = insdata[19] - '0';break;
						case 21: count1 = (insdata[19] - '0') * 10 + (insdata[20] - '0');break;
						case 22: count1 = (insdata[19] - '0') * 100 + (insdata[20] - '0') * 10 + (insdata[21] - '0');break;
						case 23: count1 = (insdata[19] - '0') * 1000 + (insdata[20] - '0') * 100 + (insdata[21] - '0') * 10 + (insdata[22] - '0');break;
						default:  returnTorF(portfd, returntf, 0);
						}

                    stringmove(insdata, 0, data, 0, 4);                     // Instruction
                    data[4] = ' ';
                    stationid(stid);                                        // Station Id
                    stringmove(stid, 0, data, 5, 5);
                    data[10] = ' ';
                    filetime(presenttime, td);                              // Time
                    stringmove(presenttime, 0, data, 11, 13);
                    data[24] = ' ';
                    for(x=0;x<50;x++)                                       // Data Index
                        {
                        data[25+x] = '1';
                        }
                    data[75] = ' ';

                    datamovexs(datatime1,insdata,7);
                    stringmove(datatime1,0,sourcetime,0,12);
                    timeconvert(datatime1,-3600);

                   for( ;count1>0;count1--)
                        {
                        stringmove(datatime1,0,namehr,12,4);
                        if((fbcs = fopen(namehr,"r")) == NULL)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        id = xiaoshi_bianhao(datatime1);
                        idn = (id + 1) * BYTENUMHR;
                        fseek(fbcs, idn, SEEK_SET);
                        fread(&csdata,BYTENUMHR,1,fbcs);
                        fclose(fbcs);

                        stringmove(sourcetime,0,data,13,2);
                        stringmove(sourcetime,2,data,16,2);
                        stringmove(sourcetime,4,data,19,2);
                        stringmove(sourcetime,6,data,22,2);
//                        stringmove(sourcetime,8,data,25,2);

                        stringmove(csdata, 204, data, 76, 50);                  // Quality Control
                        data[126] = ' ';

                        string_add_space(csdata, 4, data, 127, 4, 50);
                        data[BYTEDHRD-2] = '\r';
                        data[BYTEDHRD-1] = '\n';
                        data[BYTEDHRD] = '\0';
                        recount = string_change_2(data, BYTEDHRD);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, BYTEDHRD-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        timeconvert(datatime1,3600);
                        timeconvert(sourcetime,3600);
                        }
                    break;
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

        else if(strstr(insdata, "DHOD") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdhod, sizeof(helpdhod), MDELAY);
            else
                {
				returnTorF(portfd, returntf, 0);
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
//            	write(portfd, dhod, sizeof(dhod));
//            	usleep(MTIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
            }

	    else if(strstr(insdata, "SAMPLE") != 0)
	    	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsample, sizeof(helpsample), MDELAY);
            else
                {
/*
                                datepd=(insdata[10] - '0') * 1000 + (insdata[11] - '0') * 100 + (insdata[12] - '0') * 10 + (insdata[13] - '0');
                                if(datepd < 2000)
	                                goto txend;
                                if(datepd > 2100)
                                        goto txend;
                                datepd = (insdata[15] - '0') * 10 + (insdata[16] - '0');
                                if(datepd > 12)
                                        goto txend;
                                datepd = (insdata[18] - '0') * 10 + (insdata[19] - '0');
                                if(datepd > 31)
                                        goto txend;
                                datepd = (insdata[21] - '0') * 10 + (insdata[22] - '0');
                                if(datepd > 24)
                                        goto txend;
                                datepd = (insdata[24] - '0') * 10 + (insdata[25] - '0');
                                if(datepd > 60)
                                        goto txend;
*/
//		memset(csdata, '-', sizeof(csdata));
//		memset(data, '-', sizeof(data));

		switch(ii)
		    {
			case 25:
				if(insdata[7] == 'U')	samplex=1;
				else	samplex=0;
			
                stringmove(insdata, 0, data, 0, 8);                     // Instruction
            	data[6] = '_';
                data[8] = ' ';
                stationid(stid);                                        // Station Id
                stringmove(stid, 0, data, 9, 5);
                data[14] = ' ';
                stringmove(insdata, 9, data, 15, 16);
                data[31] = ' ';
//				printf("samplex=%d\n",samplex);
				switch(samplex)
					{		
                    case 1:
                    	if((fbsamp = fopen("/tmp/sample/samprh.tmp","r")) == NULL)
                        	break;
                        id = sample_bianhao(insdata, 25);
                        idn = (id - 1) * SAMPLE120;
                        fseek(fbsamp, idn, SEEK_SET);
                        fread(&csdata, SAMPLE120, 1, fbsamp);
						csdata[SAMPLE120]='\0';
//						printf("csdata=%s\n",csdata);
                        fclose(fbsamp);
                        string_add_space(csdata, 4, data, 32, 4, 30);
                        data[SAMPLEX30-2] = '\r';
                        data[SAMPLEX30-1] = '\n';
                        data[SAMPLEX30] = '\0';
                        recount = string_change_2(data, SAMPLEX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, &data, SAMPLEX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        break;
					default: returnTorF(portfd, returntf, 0);
					}
				break;

		    case 26:
			stringmove(insdata,7,samplexx,0,2);
//			printf("samplexx=%c%c\n",samplexx[0],samplexx[1]);
			if((strcmp(samplexx, "DR"))==0)	samplex=9;
            else if((strcmp(samplexx, "IR"))==0)   samplex=10;
            else if((strcmp(samplexx, "GR"))==0)   samplex=8;
            else if((strcmp(samplexx, "RH"))==0)   samplex=5;
            else if((strcmp(samplexx, "T0"))==0)   samplex=1;
            else if((strcmp(samplexx, "T1"))==0)   samplex=2;
            else if((strcmp(samplexx, "T2"))==0)   samplex=3;
            else if((strcmp(samplexx, "T3"))==0)   samplex=4;
            else if((strcmp(samplexx, "WD"))==0)   samplex=6;
            else if((strcmp(samplexx, "WS"))==0)   samplex=7;
			else	samplex=0;

	                stringmove(insdata, 0, data, 0, 9);                     // Instruction
			data[6] = '_';
        	        data[9] = ' ';
                	stationid(stid);                                        // Station Id
                        stringmove(stid, 0, data, 10, 5);
                        data[15] = ' ';
                        stringmove(insdata, 10, data, 16, 16);
                        data[32] = ' ';

//		printf("samplex=%d\n",samplex);
			switch(samplex)
			    {
				case 0:
						usleep(1);
						break;

			    case 1:
                                if((fbsamp = fopen("/tmp/sample/sampt.tmp","r")) == NULL)
					break;
				id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
//				printf("id=%d,idn=%ld\n",id,idn);
				fseek(fbsamp, idn, SEEK_SET);
				fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
				string_add_space(csdata, 4, data, 33, 4, 30);
				data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
				break;

                            case 2:
                                if((fbsamp = fopen("/tmp/sample/sampt1.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;
                                                                                                                                               
                            case 3:
                                if((fbsamp = fopen("/tmp/sample/sampt2.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;
                                                                                                                                               
                            case 4:
                                if((fbsamp = fopen("/tmp/sample/sampt3.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;
/*
                            case 5:
                                if((fbsamp = fopen("/tmp/sample/samprh.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;
*/
                            case 6:
                                if((fbsamp = fopen("/tmp/sample/sampwd.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE240;
//								printf("id=%d,idn=%ld\n",id,idn);
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE240, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 60);
                                data[SAMPLEXX60-2] = '\r';
                                data[SAMPLEXX60-1] = '\n';
                                data[SAMPLEXX60] = '\0';
                                recount = string_change_2(data, SAMPLEXX60);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX60-recount);
                        usleep(LTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

                            case 7:
                                if((fbsamp = fopen("/tmp/sample/sampws.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE960;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE960, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 240);
                                data[SAMPLEXX240-2] = '\r';
                                data[SAMPLEXX240-1] = '\n';
                                data[SAMPLEXX240] = '\0';
                                recount = string_change_2(data, SAMPLEXX240);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX240-recount);
                        usleep(GTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

                            case 8:
                                if((fbsamp = fopen("/tmp/sample/sampgr.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

                            case 9:
                                if((fbsamp = fopen("/tmp/sample/sampdr.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 26);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 33, 4, 30);
                                data[SAMPLEXX30-2] = '\r';
                                data[SAMPLEXX30-1] = '\n';
                                data[SAMPLEXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

			    default:  returnTorF(portfd, returntf, 0);
			    }
 			break;

		    case 27:
                        stringmove(insdata,7,samplexxx,0,3);
                    	if(strstr(samplexxx,"ST0"))	  	 samplex=1;
                    	else if(strstr(samplexxx,"ST1")) samplex=2;
                    	else if(strstr(samplexxx,"ST2")) samplex=3;
                    	else if(strstr(samplexxx,"ST3")) samplex=4;
                    	else if(strstr(samplexxx,"ST4")) samplex=5;
                    	else if(strstr(samplexxx,"ST5")) samplex=6;
                    	else if(strstr(samplexxx,"ST6")) samplex=7;
                    	else if(strstr(samplexxx,"ST7")) samplex=8;
                    	else if(strstr(samplexxx,"ST8")) samplex=9;
                        else if(strstr(samplexxx,"WS1")) samplex=10;
						else samplex=0;

                        stringmove(insdata, 0, data, 0, 10);                     // Instruction
                        data[6] = '_';
                        data[10] = ' ';
                        stationid(stid);                                        // Station Id
                        stringmove(stid, 0, data, 11, 5);
                        data[16] = ' ';
                        stringmove(insdata, 11, data, 17, 16);
                        data[33] = ' ';

//						printf("samplex=%d\n",samplex);
                        switch(samplex)
                            {
							case 0:
									usleep(1);
									break;

                            case 1:
                                if((fbsamp = fopen("/tmp/sample/sampd.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 27);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 34, 4, 30);
                                data[SAMPLEXXX30-2] = '\r';
                                data[SAMPLEXXX30-1] = '\n';
                                data[SAMPLEXXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

                            case 2:
                                if((fbsamp = fopen("/tmp/sample/sampsm1.tmp","r")) == NULL)
                                        break;
                                id = sample_bianhao(insdata, 27);
                                idn = (id - 1) * SAMPLE120;
                                fseek(fbsamp, idn, SEEK_SET);
                                fread(&csdata, SAMPLE120, 1, fbsamp);
                                fclose(fbsamp);
                                string_add_space(csdata, 4, data, 34, 4, 30);
                                data[SAMPLEXXX30-2] = '\r';
                                data[SAMPLEXXX30-1] = '\n';
                                data[SAMPLEXXX30] = '\0';
                                recount = string_change_2(data, SAMPLEXXX30);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                                write(portfd, &data, SAMPLEXXX30-recount);
                        usleep(MTIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                                break;

				case 10:
                    if((fbsamp = fopen("/tmp/sample/sampws1.tmp","r")) == NULL)
                        break;
                    id = sample_bianhao(insdata, 27);
                    idn = (id - 1) * SAMPLE240;
//                  printf("id=%d,idn=%ld\n",id,idn);
                    fseek(fbsamp, idn, SEEK_SET);
                    fread(&csdata, SAMPLE240, 1, fbsamp);
                    fclose(fbsamp);
                    string_add_space(csdata, 4, data, 33, 4, 60);
                    data[SAMPLEXX60-2] = '\r';
                    data[SAMPLEXX60-1] = '\n';
                    data[SAMPLEXX60] = '\0';
                    recount = string_change_2(data, SAMPLEXX60);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &data, SAMPLEXX60-recount);
                    usleep(LTIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;

			    default:  returnTorF(portfd, returntf, 0);
			    }

			break;
	
//   		    default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
		    }
		}


		else if(strstr(insdata, "DATE") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdate, sizeof(helpdate), MDELAY);
            else
                {
			switch(ii)
				{
				case 4:
//					time(&timep);
//        			p = localtime(&timep);
                    fd_rtc = open(DEV_RTC, O_RDWR);
                    if(fd_rtc == -1)
                        {
						returnTorF(portfd, returntf, 0);
                        break;
                        }
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    close(fd_rtc);
					hyyear = rtc_tm.tm_year;
					hymonth = rtc_tm.tm_mon;
					hyday = rtc_tm.tm_mday;
					rtc_tm.tm_year = 0;
                    rtc_tm.tm_mon = 0;
                    rtc_tm.tm_mday = 0;

					fd_rtc = open(DEV_RTC, O_RDWR);
                    if(fd_rtc == -1)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    close(fd_rtc);

					if(hyyear==rtc_tm.tm_year && hymonth==rtc_tm.tm_mon && hyday==rtc_tm.tm_mday)
						{
						date[0] = (rtc_tm.tm_year + 1900) / 1000 + '0';
						date[1] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
						date[2] = (rtc_tm.tm_year % 100) / 10 + '0';
						date[3] = rtc_tm.tm_year % 10 + '0';
						date[4] = '-';
						date[5] = (rtc_tm.tm_mon + 1) / 10 + '0';
						date[6] = (rtc_tm.tm_mon + 1) % 10 + '0';
						date[7] = '-';
						date[8] = rtc_tm.tm_mday / 10 + '0';
						date[9] = rtc_tm.tm_mday % 10 + '0';
						date[10] = '\r';
						date[11] = '\n';
						rs485clt(fd_485, R485_STA_CTL1, TX);
						write(portfd,&date,sizeof(date));
						usleep(STIME);
						rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					else
						{
						returnTorF(portfd, returntf, 0);
						}
					break;
				
				case 15:
                    datepd = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(datepd < 1900)
						{
						returnTorF(portfd, returntf, 0);
                    	goto txend;
						}
                    if(datepd > 2100)
						{
						returnTorF(portfd, returntf, 0);
                        goto txend;
						}	
                    datepd = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(datepd > 12)
						{
						returnTorF(portfd, returntf, 0);
                        goto txend;
						}
                    datepd = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(datepd > 31)
						{
						returnTorF(portfd, returntf, 0);
                        goto txend;
						}

//			        fd_rtc = open(DEV_RTC, O_RDONLY);
					fd_rtc = open(DEV_RTC, O_RDWR);
        			if(fd_rtc == -1)
            			{
                        returnTorF(portfd, returntf, 0);
           				break;
            			}
        			retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
        			if(retval == -1)
            			{
						close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
            			break;
            			}
//					close(fd_rtc);

				    rtc_tm.tm_year = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0') - 1900;  
				    rtc_tm.tm_mon = (insdata[10] - '0') * 10 + (insdata[11] - '0') - 1;   
				    rtc_tm.tm_mday = (insdata[13] - '0') * 10 + (insdata[14] - '0'); 
				    ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
//					printf("ret=%d\n",ret);
					close(fd_rtc);
					if(ret == 0)
						returntf[0] = 'T';
					else
						returntf[0] = 'F';
					returntf[1] = '\r';
					returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);

			        instime[5]  = (rtc_tm.tm_mon + 1) / 10 + '0';
        			instime[6]  = (rtc_tm.tm_mon + 1) % 10 + '0';
        			instime[7]  = (rtc_tm.tm_mday) / 10 + '0';
        			instime[8]  = (rtc_tm.tm_mday) % 10 + '0';
        			instime[9]  = (rtc_tm.tm_hour) / 10 + '0';
        			instime[10] = (rtc_tm.tm_hour) % 10 + '0';
        			instime[11] = (rtc_tm.tm_min) / 10 + '0';
        			instime[12] = (rtc_tm.tm_min) % 10 + '0';
        			instime[13] = (rtc_tm.tm_year + 1900) / 1000 + '0';
        			instime[14] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
        			instime[15] = (rtc_tm.tm_year - 100) / 10 + '0';
        			instime[16] = (rtc_tm.tm_year - 100) % 10 + '0';
        			instime[17] = '.';
        			instime[18] = (rtc_tm.tm_sec) / 10 + '0';
        			instime[19] = (rtc_tm.tm_sec) % 10 + '0';
        			instime[20] = '\0';
//      			printf("instime=%s\n",instime);
        			system(instime);
					break;

//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}


		else if(strstr(insdata, "TIME") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helptime, sizeof(helptime), MDELAY);
            else
                {
			switch(ii)
				{
				case 4:
					fd_rtc = open(DEV_RTC, O_RDWR);
                    if(fd_rtc == -1)
                        {
						returnTorF(portfd, returntf, 0);
                        break;
                        }
				    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
						returnTorF(portfd, returntf, 0);
                        break;
                        }
					close(fd_rtc);
					hyhour = rtc_tm.tm_hour;
					hymin = rtc_tm.tm_min;
					hysec = rtc_tm.tm_sec;
					rtc_tm.tm_hour = 0;
					rtc_tm.tm_min = 0;
					rtc_tm.tm_sec = 0;

					fd_rtc = open(DEV_RTC, O_RDWR);
                    if(fd_rtc == -1)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    close(fd_rtc);

					if(hyhour==rtc_tm.tm_hour && hymin==rtc_tm.tm_min)
						{
						timed[0] = rtc_tm.tm_hour / 10 + '0';
						timed[1] = rtc_tm.tm_hour % 10 + '0';
						timed[2] = ':';
						timed[3] = rtc_tm.tm_min / 10 + '0';
						timed[4] = rtc_tm.tm_min % 10 + '0';
						timed[5] = ':';
						timed[6] = rtc_tm.tm_sec / 10 + '0';
						timed[7] = rtc_tm.tm_sec % 10 + '0';
						timed[8] = '\r';
						timed[9] = '\n';
                    	rs485clt(fd_485, R485_STA_CTL1, TX);
						write(portfd,&timed,sizeof(timed));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					else
						{
						returnTorF(portfd, returntf, 0);
						}
					break;

				case 13:
                    datepd = (insdata[5] - '0') * 10 + (insdata[6] - '0');
                    if(datepd > 24)
						{
						returnTorF(portfd, returntf, 0);
                    	break;
						}
                    datepd = (insdata[8] - '0') * 10 + (insdata[9] - '0');
                    if(datepd > 60)
						{
						returnTorF(portfd, returntf, 0);
                    	break;
						}
                    datepd = (insdata[11] - '0') * 10 + (insdata[12] - '0');
                    if(datepd > 60)
						{
						returnTorF(portfd, returntf, 0);
                        break;
						}

                    fd_rtc = open(DEV_RTC, O_RDWR);
                    if(fd_rtc == -1)
                        {
						returnTorF(portfd, returntf, 0);
                        break;
                        }
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
						returnTorF(portfd, returntf, 0);
                        break;
                        }
//                  close(fd_rtc);

                    rtc_tm.tm_hour = (insdata[5] - '0') * 10 + (insdata[6] - '0');
                    rtc_tm.tm_min = (insdata[8] - '0') * 10 + (insdata[9] - '0');
                    rtc_tm.tm_sec = (insdata[11] - '0') * 10 + (insdata[12] - '0');
                    ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
                    close(fd_rtc);
                    if(ret == 0)
                        returntf[0] = 'T';
                    else
                        returntf[0] = 'F';
                    returntf[1] = '\r';
                    returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);

                    instime[5]  = (rtc_tm.tm_mon + 1) / 10 + '0';
                    instime[6]  = (rtc_tm.tm_mon + 1) % 10 + '0';
                    instime[7]  = (rtc_tm.tm_mday) / 10 + '0';
                    instime[8]  = (rtc_tm.tm_mday) % 10 + '0';
                    instime[9]  = (rtc_tm.tm_hour) / 10 + '0';
                    instime[10] = (rtc_tm.tm_hour) % 10 + '0';
                    instime[11] = (rtc_tm.tm_min) / 10 + '0';
                    instime[12] = (rtc_tm.tm_min) % 10 + '0';
                    instime[13] = (rtc_tm.tm_year + 1900) / 1000 + '0';
                    instime[14] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
                    instime[15] = (rtc_tm.tm_year - 100) / 10 + '0';
                    instime[16] = (rtc_tm.tm_year - 100) % 10 + '0';
                    instime[17] = '.';
                    instime[18] = (rtc_tm.tm_sec) / 10 + '0';
                    instime[19] = (rtc_tm.tm_sec) % 10 + '0';
                    instime[20] = '\0';
//                  printf("instime=%s\n",instime);
                    system(instime);
					break;
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}


        else if(strstr(insdata, "STATMAIN") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpstatmain, sizeof(helpstatmain), SDELAY);
            else
                {
//            memset(statmain, ' ', sizeof(statmain));
            if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, pss, sizeof(pss));
//              printf("pss=%s\n",pss);
                close(fdmaste);
                stringmove(pss, 3, statmain, 11, 2);
                statmain[13] = pss[6];
                if(pss[0]=='A')
                    statmain[15] = '0';
                else
                    statmain[15] = '1';
                }
            if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
                usleep(1);
            else
                {
                ret = read(fdmaste, &masttemp, 2);
                usleep(10000);
                ret = read(fdmaste, &masttemp, 2);
//              printf("temp=%d\n",masttemp);
                close(fdmaste);
                if(masttemp >= 0)
                    {
                    statmain[17] = ' ';
                    statmain[18] = masttemp / 100 + '0';
                    statmain[19] = masttemp % 100 / 10 + '0';
                    statmain[20] = masttemp % 10 + '0';
                    }
                else
                    {
                    statmain[17] = '-';
                    statmain[18] = masttemp * (-1) / 100 + '0';
                    statmain[19] = masttemp * (-1) % 100 / 10 + '0';
                    statmain[20] = masttemp * (-1) % 10 + '0';
                    }
                }
            system("df>/usr/df.tmp");
            if((fdmaste = open("/usr/df.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, df, sizeof(df));
                close(fdmaste);
                stringmove(df, 168, statmain, 28, 4);
                }
            if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, door, sizeof(door));
                close(fdmaste);
                statmain[35] = door[0];
                }
           if((fdmaste = open("/usr/statzfsw.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, statzfsw, sizeof(statzfsw));
                close(fdmaste);
                stringmove(statzfsw, 0, statmain, 43, 4);
                }
            if((fdmaste = open("/usr/statzfcc.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, statccsw, sizeof(statccsw));
                close(fdmaste);
                stringmove(statccsw, 0, statmain, 48, 4);
                }
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statmain, sizeof(statmain));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }
            }

        else if(strstr(insdata, "STATCLIM") != 0)
            {
            if((fdmaste = open("/usr/statclim.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, statclim, sizeof(statclim));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statclim, sizeof(statclim));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATRADI") != 0)
            {
            if((fdmaste = open("/usr/statradi.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, statradi, sizeof(statradi));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statradi, sizeof(statradi));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATEATH") != 0)
            {
            if((fdmaste = open("/usr/stateath.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, stateath, sizeof(stateath));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, stateath, sizeof(stateath));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATSOIL") != 0)
            {
            if((fdmaste = open("/usr/statsoil.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, statsoil, sizeof(statsoil));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statsoil, sizeof(statsoil));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATSEAA") != 0)
            {
            if((fdmaste = open("/usr/statseaa.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, statseaa, sizeof(statseaa));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statseaa, sizeof(statseaa));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATINTL 1") != 0)
            {
            if((fdmaste = open("/usr/statintl1.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, statintl, sizeof(statintl));
            close(fdmaste);
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, statintl, sizeof(statintl));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }

        else if(strstr(insdata, "STATSENSOR") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpstatsensor, sizeof(helpstatsensor), MDELAY);
            else
                {
                if((fdmaste = open("/tmp/paramete/senstate.txt",O_RDONLY)) < 1)
                    {
                    returnTorF(portfd, returntf, 0);
                    goto txend;
                    }
                read(fdmaste, senstate, sizeof(senstate));
                close(fdmaste);
				if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                    {
                    returnTorF(portfd, returntf, 0);
                    goto txend;
                    }
                read(fdmaste, senstonoff, sizeof(senstonoff));
                close(fdmaste);
				for(onoff=0;onoff<(sizeof(senstonoff)-2);onoff++)
					{
					if(senstonoff[onoff] =='0')
						senstate[onoff] = 'N';
					else
						senstate[onoff] = '0';
					}
            switch(ii)
                {
                case 10:
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, senstate, sizeof(senstate));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 12:
                    if(insdata[6] == 'P')       samplex=1;
                    else if(insdata[6] == 'U')  samplex=7;
                    else                        samplex=0;
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        sst[0] = senstate[samplex-1];
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    break;
                case 13:
                    stringmove(insdata,6,samplexx,0,2);
                    if(!(strcmp(samplexx, "T0")))       samplex=2;
                    else if(!(strcmp(samplexx, "T1")))  samplex=3;
                    else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    else if(!(strcmp(samplexx, "WS")))  samplex=13;
                    else if(!(strcmp(samplexx, "TG")))  samplex=18;
                    else if(!(strcmp(samplexx, "IR")))  samplex=19;
                    else if(!(strcmp(samplexx, "LE")))  samplex=29;
                    else if(!(strcmp(samplexx, "VI")))  samplex=30;
                    else if(!(strcmp(samplexx, "CH")))  samplex=31;
                    else if(!(strcmp(samplexx, "WW")))  samplex=34;
                    else if(!(strcmp(samplexx, "SD")))  samplex=35;
                    else if(!(strcmp(samplexx, "FR")))  samplex=36;
                    else if(!(strcmp(samplexx, "WI")))  samplex=37;
                    else if(!(strcmp(samplexx, "GR")))  samplex=40;
                    else if(!(strcmp(samplexx, "NR")))  samplex=41;
                    else if(!(strcmp(samplexx, "DR")))  samplex=42;
                    else if(!(strcmp(samplexx, "SR")))  samplex=43;
                    else if(!(strcmp(samplexx, "RR")))  samplex=44;
                    else if(!(strcmp(samplexx, "UR")))  samplex=45;
                    else if(!(strcmp(samplexx, "AR")))  samplex=48;
                    else if(!(strcmp(samplexx, "TR")))  samplex=50;
                    else if(!(strcmp(samplexx, "PR")))  samplex=52;
                    else if(!(strcmp(samplexx, "WT")))  samplex=62;
                    else if(!(strcmp(samplexx, "BA")))  samplex=63;
                    else if(!(strcmp(samplexx, "OT")))  samplex=64;
                    else if(!(strcmp(samplexx, "OS")))  samplex=65;
                    else if(!(strcmp(samplexx, "OC")))  samplex=66;
                    else if(!(strcmp(samplexx, "OH")))  samplex=67;
                    else if(!(strcmp(samplexx, "OP")))  samplex=68;
                    else if(!(strcmp(samplexx, "OV")))  samplex=69;
                    else if(!(strcmp(samplexx, "OD")))  samplex=70;
                    else if(!(strcmp(samplexx, "TL")))  samplex=71;
                    else                                samplex=0;
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        sst[0] = senstate[samplex-1];
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    break;
                case 14:
                    stringmove(insdata,6,samplexxx,0,3);
                    if(strstr(samplexxx,"SV1"))      samplex=9;
                    else if(strstr(samplexxx,"SV2")) samplex=10;
                    else if(strstr(samplexxx,"SV3")) samplex=11;
                    else if(strstr(samplexxx,"WS1")) samplex=14;
                    else if(strstr(samplexxx,"RAT")) samplex=15;
                    else if(strstr(samplexxx,"RAW")) samplex=17;
                    else if(strstr(samplexxx,"ST0")) samplex=20;
                    else if(strstr(samplexxx,"ST1")) samplex=21;
                    else if(strstr(samplexxx,"ST2")) samplex=22;
                    else if(strstr(samplexxx,"ST3")) samplex=23;
                    else if(strstr(samplexxx,"ST4")) samplex=24;
                    else if(strstr(samplexxx,"ST5")) samplex=25;
                    else if(strstr(samplexxx,"ST6")) samplex=26;
                    else if(strstr(samplexxx,"ST7")) samplex=27;
                    else if(strstr(samplexxx,"ST8")) samplex=28;
                    else if(strstr(samplexxx,"TCA")) samplex=32;
                    else if(strstr(samplexxx,"LCA")) samplex=33;
                    else if(strstr(samplexxx,"FSD")) samplex=38;
                    else if(strstr(samplexxx,"LNF")) samplex=39;
                    else if(strstr(samplexxx,"UVA")) samplex=46;
                    else if(strstr(samplexxx,"UVB")) samplex=47;
                    else if(strstr(samplexxx,"ART")) samplex=49;
                    else if(strstr(samplexxx,"TRT")) samplex=51;
                    else if(strstr(samplexxx,"SSD")) samplex=53;
                    else if(strstr(samplexxx,"SM1")) samplex=54;
                    else if(strstr(samplexxx,"SM2")) samplex=55;
                    else if(strstr(samplexxx,"SM3")) samplex=56;
                    else if(strstr(samplexxx,"SM4")) samplex=57;
                    else if(strstr(samplexxx,"SM5")) samplex=58;
                    else if(strstr(samplexxx,"SM6")) samplex=59;
                    else if(strstr(samplexxx,"SM7")) samplex=60;
                    else if(strstr(samplexxx,"SM8")) samplex=61;
                    else if(strstr(samplexxx,"OTU")) samplex=72;
                    else if(strstr(samplexxx,"OCC")) samplex=73;
                    else                             samplex=0;
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        sst[0] = senstate[samplex-1];
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    break;
                case 15:
                    if(strstr(insdata,"RAT1"))
                        {
                        samplex=16;
                        sst[0] = senstate[samplex-1];
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        break;
                        }
                    else
                        returnTorF(portfd, returntf, 0);
                    break;
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "STAT") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helprsta, sizeof(helprsta), SDELAY);
            else
                {
                fd_rtc = open(DEV_RTC, O_RDWR);
                if(fd_rtc == -1)
                    usleep(1);
                else
                    {
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        close(fd_rtc);
                    else
                        {
                        close(fd_rtc);
                        stat[5] = (rtc_tm.tm_year + 1900) / 1000 + '0';
                        stat[6] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
                        stat[7] = (rtc_tm.tm_year % 100) / 10 + '0';
                        stat[8] = rtc_tm.tm_year % 10 + '0';
                        stat[10] = (rtc_tm.tm_mon + 1) / 10 + '0';
                        stat[11] = (rtc_tm.tm_mon + 1) % 10 + '0';
                        stat[13] = rtc_tm.tm_mday / 10 + '0';
                        stat[14] = rtc_tm.tm_mday % 10 + '0';
                        stat[16] = rtc_tm.tm_hour / 10 + '0';
                        stat[17] = rtc_tm.tm_hour % 10 + '0';
                        stat[19] = rtc_tm.tm_min / 10 + '0';
                        stat[20] = rtc_tm.tm_min % 10 + '0';
                        }
                    }
                if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, pss, sizeof(pss));
                    close(fdmaste);
                    stringmove(pss, 3, stat, 24, 2);
                    stat[26] = pss[6];
                    if(pss[0]=='A')
                        stat[27] = '0';
                    else
                        stat[27] = '1';
                    }
                if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
                    usleep(1);
                else
                    {
                    ret = read(fdmaste, &masttemp, 2);
                    usleep(10000);
                    ret = read(fdmaste, &masttemp, 2);
                    close(fdmaste);
                    if(masttemp >= 0)
                        {
                        stat[28] = '0';
                        stat[29] = masttemp / 100 + '0';
                        stat[30] = masttemp % 100 / 10 + '0';
                        stat[31] = masttemp % 10 + '0';
                        }
                    else
                        {
                        stat[28] = '-';
                        stat[29] = masttemp * (-1) / 100 + '0';
                        stat[30] = masttemp * (-1) % 100 / 10 + '0';
                        stat[31] = masttemp * (-1) % 10 + '0';
                        }
                    }
                system("df>/usr/df.tmp");
                if((fdmaste = open("/usr/df.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, df, sizeof(df));
                    close(fdmaste);
                    stringmove(df, 168, stat, 35, 4);
                    }
                if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, door, sizeof(door));
                    close(fdmaste);
                    stat[40] = door[0];
                    }
                if((fdmaste = open("/usr/statclim.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, statclim, sizeof(statclim));
                    close(fdmaste);
                    stringmove(statclim, 11, stat, 46, 3);
                    }
                if((fdmaste = open("/usr/statradi.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, statradi, sizeof(statradi));
                    close(fdmaste);
                    stringmove(statradi, 11, stat, 58, 3);
                    }
                if((fdmaste = open("/usr/stateath.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, stateath, sizeof(stateath));
                    close(fdmaste);
                    stringmove(stateath, 11, stat, 70, 3);
                    }
                if((fdmaste = open("/usr/statintl1.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, statintl, sizeof(statintl));
                    close(fdmaste);
                    stringmove(statintl, 13, stat, 106, 3);
                    }
                if((fdmaste = open("/tmp/paramete/senstate.txt",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, senstate, sizeof(senstate));
                    close(fdmaste);
					if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
						usleep(1);
                  	else
						{
                		read(fdmaste, senstonoff, sizeof(senstonoff));
                		close(fdmaste);
                		for(onoff=0;onoff<(sizeof(senstonoff)-2);onoff++)
                    		{
                    		if(senstonoff[onoff] =='0')
                        		senstate[onoff] = 'N';
                    		else
                        		senstate[onoff] = '0';
                    		}
						}
                    stringmove(senstate, 0, stat, 176, 73);
                    }
                if((fdmaste = open("/usr/statzfsw.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, statzfsw, sizeof(statzfsw));
                    close(fdmaste);
                    stringmove(statzfsw, 0, stat, 249, 4);
                    }
                if((fdmaste = open("/usr/statzfcc.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, statccsw, sizeof(statccsw));
                    close(fdmaste);
                    stringmove(statccsw, 0, stat, 253, 4);
                    }
                rs485clt(fd_485, R485_STA_CTL1, TX);
                write(portfd, stat, sizeof(stat));
                usleep(MTIME);
                rs485clt(fd_485, R485_STA_CTL1, RX);
                }
            }


        else if(strstr(insdata, "SETCOM") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsetcom, sizeof(helpsetcom), MDELAY);
            else
                {
	        if((fdtongxun = open("/tmp/paramete/tongxun.txt", O_RDONLY)) < 1)
				goto txend;
            read(fdtongxun, tongxun, sizeof(tongxun));
            close(fdtongxun);

			switch(ii)
				{
				case 6:
//					stringmove(tongxun,0,baudd,0,1);
					baud = tongxun[0] -'0';
					switch(baud)
						{
						case 1: 
							baud3[0]='3';baud3[1]='0';baud3[2]='0';baudc=3;
							stringmove(baud3,0,tongxunn,0,baudc);
							break;
						case 2: 
							baud4[0]='1';baud4[1]='2';baud4[2]='0';baud4[3]='0';baudc=4;
                            stringmove(baud4,0,tongxunn,0,baudc);
							break;
						case 3: 
							baud4[0]='2';baud4[1]='4';baud4[2]='0';baud4[3]='0';baudc=4;
                            stringmove(baud4,0,tongxunn,0,baudc);
							break;
						case 4: 
							baud4[0]='4';baud4[1]='8';baud4[2]='0';baud4[3]='0';baudc=4;
                            stringmove(baud4,0,tongxunn,0,baudc);
							break;
						case 5: 
							baud4[0]='9';baud4[1]='6';baud4[2]='0';baud4[3]='0';baudc=4;
							stringmove(baud4,0,tongxunn,0,baudc);
							break;
						case 6: 
							baud5[0]='1';baud5[1]='9';baud5[2]='2';baud5[3]='0';baud5[4]='0';baudc=5;
                            stringmove(baud5,0,tongxunn,0,baudc);
							break;
                        case 7: 
							baud5[0]='3';baud5[1]='8';baud5[2]='4';baud5[3]='0';baud5[4]='0';baudc=5;
                            stringmove(baud5,0,tongxunn,0,baudc);
							break;
                        case 8: 
							baud5[0]='5';baud5[1]='7';baud5[2]='6';baud5[3]='0';baud5[4]='0';baudc=5;
                            stringmove(baud5,0,tongxunn,0,baudc);
							break;
                        case 9: 
							baud6[0]='1';baud6[1]='1';baud6[2]='5';baud6[3]='2';baud6[4]='0';baud6[5]='0';baudc=6;
                            stringmove(baud6,0,tongxunn,0,baudc);
							break;
					default: returnTorF(portfd, returntf, 0);
					}
					stringmove(tongxun,1,tongxunn,baudc,6);
					tongxunn[baudc+6]='\r';
					tongxunn[baudc+7]='\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&tongxunn,baudc+8);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					break;
				case 16:
					tongxun[0] = '1';
					baud = 1;
					stringmove(insdata,10,tongxun,1,6);
			        if((fdtongxun=open("/tmp/paramete/tongxun.txt",O_WRONLY)) < 1)
						break;
   					write(fdtongxun,&tongxun,sizeof(tongxun));
       				close(fdtongxun);
                    returntf[0] = 'T';
                    returntf[1] = '\r';
                    returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					close_port(portfd);
					portfd = open_port(UART1, baud);
					break;
				case 17:
					baudc = insdata[7] - '0';
					switch(baudc)
						{
						case 1: tongxun[0] = '2';baud=2;break;
                        case 2: tongxun[0] = '3';baud=3;break;
                        case 4: tongxun[0] = '4';baud=4;break;
                        case 9: tongxun[0] = '5';baud=5;break;
						default: returnTorF(portfd, returntf, 0);
						}
					stringmove(insdata,11,tongxun,1,6);
                    if((fdtongxun=open("/tmp/paramete/tongxun.txt",O_WRONLY)) < 1)
						break;
                    write(fdtongxun,&tongxun,sizeof(tongxun));
                    close(fdtongxun);
                    returntf[0] = 'T';
                    returntf[1] = '\r';
                    returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
					portfd = open_port(UART1, baud);
                    break;
                case 18:
                    baudc = insdata[7] - '0';
                    switch(baudc)
                    	{
                        case 1: tongxun[0] = '6';baud=6;break;
                        case 3: tongxun[0] = '7';baud=7;break;
                        case 5: tongxun[0] = '8';baud=8;break;
                        default: returnTorF(portfd, returntf, 0);
                        }
                    stringmove(insdata,12,tongxun,1,6);
                    if((fdtongxun=open("/tmp/paramete/tongxun.txt",O_WRONLY)) < 1)
						break;
                    write(fdtongxun,&tongxun,sizeof(tongxun));
                    close(fdtongxun);
                    returntf[0] = 'T';
                    returntf[1] = '\r';
                    returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
                    portfd = open_port(UART1, baud);
                    break;
                case 19:
                    tongxun[0] = '9';
					baud = 9;
                    stringmove(insdata,13,tongxun,1,6);
                    if((fdtongxun=open("/tmp/paramete/tongxun.txt",O_WRONLY)) < 1)
						break;
                    write(fdtongxun,&tongxun,sizeof(tongxun));
                    close(fdtongxun);
                    returntf[0] = 'T';
                    returntf[1] = '\r';
                    returntf[2] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
                    portfd = open_port(UART1, baud);
                    break;
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

		else if(strstr(insdata, "DEVMODE RAW") != 0)
			{
			switch(ii)
                {
                case 11:
					dmtime = datetosecond();
					if(dmtime < dmraw)
						{
						devmoderaw[2] = ((dmraw-dmtime)%3600) / 600 + '0';
						devmoderaw[3] = (((dmraw-dmtime)%3600)%600) / 60 + '0';
						rs485clt(fd_485, R485_STA_CTL1, TX);
                   		write(portfd, devmoderaw, sizeof(devmoderaw));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					else
						{
						rs485clt(fd_485, R485_STA_CTL1, TX);
                    	write(portfd, zero, sizeof(zero));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
						}
                    break;
                case 13:
					if(insdata[12] == '0')
						{
						dmraw = datetosecond();
						returnTorF(portfd, returntf, 1);
						}
					else
						returnTorF(portfd, returntf, 0);
                    break;
				case 16:
					dmtime = datetosecond();
//					time(&dmtime);
//        			p = localtime(&dmtime);
//        			dmtime = mktime(p);
        			dmraw = dmtime + ((insdata[14]-'0')*10+(insdata[15]-'0'))*60;
					returnTorF(portfd, returntf, 1);
					break;
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
				}
			}

		else if(strstr(insdata, "DEVMODE LE") != 0)
            {
            switch(ii)
                {
                case 10:
                    dmtime = datetosecond();
                    if(dmtime < dmraw)
                        {
                        devmodele[2] = ((dmle-dmtime)%3600) / 600 + '0';
                        devmodele[3] = (((dmle-dmtime)%3600)%600) / 60 + '0';
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, devmodele, sizeof(devmodele));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    else
                        {
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, zero, sizeof(zero));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    break;
                case 12:
                    if(insdata[11] == '0')
                        {
                        dmle = datetosecond();
                        returnTorF(portfd, returntf, 1);
                        }
                    else
                        returnTorF(portfd, returntf, 0);
                    break;
                case 15:
                    dmtime = datetosecond();
//                  time(&dmtime);
//                  p = localtime(&dmtime);
//                  dmtime = mktime(p);
                    dmle = dmtime + ((insdata[13]-'0')*10+(insdata[14]-'0'))*60;
                    returnTorF(portfd, returntf, 1);
                    break;
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
			}


		else if(strstr(insdata, "BASEINFO") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpbaseinfo, sizeof(helpbaseinfo), SDELAY);
            else
                {
//				memset(autocheck, ' ', sizeof(autocheck));
				if((fdstbd = open("/tmp/paramete/baseinfo.txt",O_RDONLY)) < 1)
					{
					returnTorF(portfd, returntf, 0);
					goto txend;
					}
        		read(fdstbd, baseinfo, sizeof(baseinfo));
        		close(fdstbd);
				baseinfo[19]='\r';
            	baseinfo[20]='\n';
            	rs485clt(fd_485, R485_STA_CTL1, TX);
				write(portfd, baseinfo, sizeof(baseinfo));
            	usleep(STIME);
            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
			}

		else if(strstr(insdata, "AUTOCHECK") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpautocheck, sizeof(helpautocheck), SDELAY);
            else
                {
            	fd_rtc = open(DEV_RTC, O_RDWR);										// Date Time
            	if(fd_rtc == -1)
	            	{
//                	returnTorF(portfd, returntf, 0);
                	goto check1;
                	}
            	retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
            	if(retval == -1)
                	{
                	close(fd_rtc);
//                	returnTorF(portfd, returntf, 0);
                	goto check1;
                	}
            	close(fd_rtc);
            	date[0] = (rtc_tm.tm_year + 1900) / 1000 + '0';
            	date[1] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
            	date[2] = (rtc_tm.tm_year % 100) / 10 + '0';
            	date[3] = rtc_tm.tm_year % 10 + '0';
            	date[4] = '-';
            	date[5] = (rtc_tm.tm_mon + 1) / 10 + '0';
            	date[6] = (rtc_tm.tm_mon + 1) % 10 + '0';
            	date[7] = '-';
            	date[8] = rtc_tm.tm_mday / 10 + '0';
            	date[9] = rtc_tm.tm_mday % 10 + '0';
            	timed[0] = rtc_tm.tm_hour / 10 + '0';
            	timed[1] = rtc_tm.tm_hour % 10 + '0';
            	timed[2] = ':';
            	timed[3] = rtc_tm.tm_min / 10 + '0';
            	timed[4] = rtc_tm.tm_min % 10 + '0';
            	timed[5] = ':';
            	timed[6] = rtc_tm.tm_sec / 10 + '0';
            	timed[7] = rtc_tm.tm_sec % 10 + '0';
				stringmove(date, 0, autocheck, 0, 10);
				autocheck[10] = ' ';
				stringmove(timed, 0, autocheck, 11, 8);
        	check1:
				autocheck[19] = ';';
	            autocheck[20] = 'G';                                                // GPS
    	        autocheck[21] = 'P';
				autocheck[22] = 'S';
				stringmove(normal, 0, autocheck, 24, 8);
				autocheck[32] = ';';

	            if((fdtongxun = open("/tmp/paramete/tongxun.txt", O_RDONLY)) < 1)	// Baudrate
    	            goto check2;
        	    read(fdtongxun, tongxun, sizeof(tongxun));
	            close(fdtongxun);
    	        baud = tongxun[0] -'0';
        	    switch(baud)
	                {
    	            case 1:
        	        	baud3[0]='3';baud3[1]='0';baud3[2]='0';baudc=3;
            	        stringmove(baud3,0,tongxunn,0,baudc);
                	    break;
	                case 2:
    	                baud4[0]='1';baud4[1]='2';baud4[2]='0';baud4[3]='0';baudc=4;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 3:
    	                baud4[0]='2';baud4[1]='4';baud4[2]='0';baud4[3]='0';baudc=4;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 4:
        	            baud4[0]='4';baud4[1]='8';baud4[2]='0';baud4[3]='0';baudc=4;
    	                stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 5:
    	                baud4[0]='9';baud4[1]='6';baud4[2]='0';baud4[3]='0';baudc=4;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 6:
    	                baud5[0]='1';baud5[1]='9';baud5[2]='2';baud5[3]='0';baud5[4]='0';baudc=5;
        	            stringmove(baud5,0,tongxunn,0,baudc);
            	        break;
	                case 7:
    	                baud5[0]='3';baud5[1]='8';baud5[2]='4';baud5[3]='0';baud5[4]='0';baudc=5;
        	            stringmove(baud5,0,tongxunn,0,baudc);
            	        break;
	                case 8:
    	                baud5[0]='5';baud5[1]='7';baud5[2]='6';baud5[3]='0';baud5[4]='0';baudc=5;
        	            stringmove(baud5,0,tongxunn,0,baudc);
            	        break;
	                case 9:
    	                baud6[0]='1';baud6[1]='1';baud6[2]='5';baud6[3]='2';baud6[4]='0';baud6[5]='0';baudc=6;
        	            stringmove(baud6,0,tongxunn,0,baudc);
            	        break;
	                default: 
						goto check2;
        	        }
	            stringmove(tongxun, 1, tongxunn, baudc, 6);
				stringmove(tongxunn, 0, autocheck, 33, 12);
        	check2:
				autocheck[45] = ';';
    	        if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
        	        {
	                goto check3;
    	            }
	            ret = read(fdmaste, &masttemp, 2);
    	        usleep(10000);
        	    ret = read(fdmaste, &masttemp, 2);
//          	printf("temp=%d\n",masttemp);
	            close(fdmaste);
    	        if(ret > 0)
        	        {
            	    if(masttemp >= 0)
                	    {
                    	mact[0] = ' ';
	                    mact[1] = masttemp / 100 + '0';
    	                mact[2] = masttemp % 100 / 10 + '0';
        	            mact[3] = '.';
            	        mact[4] = masttemp % 10 + '0';
                	    }
	                else
    	                {
        	            mact[0] = '-';
            	        mact[1] = masttemp * (-1) / 100 + '0';
                	    mact[2] = masttemp * (-1) % 100 / 10 + '0';
                    	mact[3] = '.';
	                    mact[4] = masttemp * (-1) % 10 + '0';
    	                }
        	        }
				else
					goto check3;
        	    stringmove(mact, 0, autocheck, 46, 5);
	        check3:
    	        autocheck[51] = ';';
        	    if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
            	    {
                	goto check4;
	                }
    	        read(fdmaste, pss, sizeof(pss));
        	    close(fdmaste);
				stringmove(pss, 0, autocheck, 52, 7);
	        check4:
    	        autocheck[59] = ';';
        	    stringmove(collect, 0, autocheck, 60, 64);
				autocheck[124] = ';';
	            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
    	            {
        	        goto check5;
					}
	            read(fdmaste, senstate, sizeof(senstate));
    	        close(fdmaste);
        	    stringmove(senstate, 0, autocheck, 125, 73);
			check5:			
//				autocheck[198] = '\r';
//				autocheck[199] = '\n';
    	        rs485clt(fd_485, R485_STA_CTL1, TX);
				write(portfd, autocheck, sizeof(autocheck));
            	usleep(LTIME);
	            rs485clt(fd_485, R485_STA_CTL1, RX);
				}
			}

		else if(strstr(insdata, "ID") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpid, sizeof(helpid), MDELAY);
            else
                {
			memset(&stid, ' ' ,sizeof(stid));
            switch(ii)
            	{
                case 2:
					if((fdstbd = open("/tmp/paramete/stid.txt", O_RDONLY)) < 1)
	                	{
        	            returnTorF(portfd, returntf, 0);
                	    goto txend;
                        }
					read(fdstbd, stid, sizeof(stid));
					close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, stid, strcspn(stid, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					break;
				case 4:
                case 5:
                case 6:
                case 7:
				case 8: 
					if((fdstbd = open("/tmp/paramete/stid.txt", O_RDWR)) < 1)
	                    {
						returnTorF(portfd, returntf, 0);
	                    goto txend;
	                    }
//					for(n=0; n<8-ii; n++)
//						stid[n] = ' ';
                    stringmove(ins, 3, stid, 0, strcspn(ins, "\n")-2);
					ret = write(fdstbd, stid, sizeof(stid));
					close(fdstbd);
					returnTorF(portfd, returntf, ret);

//					if((fdstbd = fopen("/usr/stid.tmp","r+")) < 1)
//                    	{
//                        write(portfd,&openerr,sizeof(openerr));
//                        goto txend;
//                        }
//                    fseek(fdstbd, 0, SEEK_SET);
//                    stringmove(insdata,3,stidd,0,5);
//                    fwrite(&stidd,5,1,fdstbd);
//					fclose(fdstbd);
                    break;
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

		else if(strstr(insdata, "LAT") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helplat, sizeof(helplat), MDELAY);
            else
                {
			memset(&stlat, ' ' ,sizeof(stlat));
            switch(ii)
        	    {
                case 3:
            	    if((fdstbd = open("/tmp/paramete/lat.txt", O_RDONLY)) < 1)
	                	{
						returnTorF(portfd, returntf, 0);
                	    goto txend;
                        }
                    read(fdstbd, stlat, sizeof(stlat));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stlat, strcspn(stlat, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 9:
                case 10:
                case 11:
                case 12:
					string1 = strchr(ins, ' ');
//					printf("du=%f	",atof(string1));
					if(string1==0 || atof(string1)>180 || atof(string1)<0)
						{
						returnTorF(portfd, returntf, 0);
						break;
						}
                	string2 = strchr(string1, '.');
					if(string2 == 0)
                       	{
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
					string2++;	
//                    printf("fen=%f	",atof(string2));
                    if(atof(string2)>60 || atof(string2)<0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    string1 = strchr(string2, '.');
                    if(string1 == 0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
					string1++;
//                    printf("miao=%f\n",atof(string1));
                    if(atof(string1)>60 || atof(string1)<0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }

					if((fdstbd = open("/tmp/paramete/lat.txt",O_RDWR)) < 1)
	                    {
						returnTorF(portfd, returntf, 0);
                        goto txend;
      	                }
					stringmove(ins, 4, stlat, 0, strcspn(ins, "\n")-3);
					ret = write(fdstbd, stlat, sizeof(stlat));
                    close(fdstbd);
					returnTorF(portfd, returntf, ret);
                    break;
//                default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "LONG") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helplong, sizeof(helplong), MDELAY);
            else
                {
			memset(&stlong, ' ' ,sizeof(stlong));
            switch(ii)
                {
                case 4:
            	    if((fdstbd = open("/tmp/paramete/long.txt",O_RDONLY)) < 1)
	                	{
						returnTorF(portfd, returntf, 0);
                	    goto txend;
                        }
                    read(fdstbd, stlong, sizeof(stlong));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &stlong, strcspn(stlong, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 10:
                case 11:
                case 12:
                case 13:
                case 14:
                    string1 = strchr(ins, ' ');
//                  printf("du=%f   ",atof(string1));
//					printf("%d\n",string1);
                    if(string1==0 || atof(string1)>180 || atof(string1)<0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    string2 = strchr(string1, '.');
                    if(string2 == 0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    string2++;
//                    printf("fen=%f    ",atof(string2));
                    if(atof(string2)>60 || atof(string2)<0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    string1 = strchr(string2, '.');
                    if(string1 == 0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    string1++;
//                    printf("miao=%f\n",atof(string1));
                    if(atof(string1)>60 || atof(string1)<0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }

					if((fdstbd = open("/tmp/paramete/long.txt",O_RDWR)) < 1)
                    	{
						returnTorF(portfd, returntf, 0);
        	            goto txend;
	                    }
                    stringmove(ins, 5, stlong, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, stlong, sizeof(stlong));
                    close(fdstbd);
					returnTorF(portfd, returntf, ret);
                    break;
//                default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if((strstr(insdata,"TD")!=0) && (insdata[0]=='T'))
            {
//			if(insdata[0]=='T')
//			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helptd, sizeof(helptd), MDELAY);
            else
                {
			memset(&sttd, ' ', sizeof(sttd));
            switch(ii)
            	{
                case 2:
                	if((fdstbd = open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
                    	{
						returnTorF(portfd, returntf, 0);
        	            goto txend;
	                    }
                	read(fdstbd, sttd, sizeof(sttd));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, sttd, strcspn(sttd, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
				case 4:
				case 5:
				case 6:
                case 7:
					string1 = strchr(ins, ' ');
                    if(string1==0)
                        {
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
					string1++;
//					printf("%d\n",atoi(string1));
					if(atof(string1)>180 || atof(string1)<-180)
						{
						returnTorF(portfd, returntf, 0);
                        break;
						}

					if((fdstbd = open("/tmp/paramete/td.txt", O_RDWR)) < 1)
	                	{
						returnTorF(portfd, returntf, 0);
                	    goto txend;
                        }
					stringmove(ins, 3, sttd, 0, strcspn(ins, "\n")-2);
//                    stringmove(insdata, 3, sttd, 0, ii-3);
//                    for(nn=7-ii; nn>0; nn--,ii++)
//                    	sttd[ii-3] = ' ';
//					sttd[4] = '\r';
//					sttd[5] = '\n';
                    ret = write(fdstbd, sttd, sizeof(sttd));
                    close(fdstbd);
					returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
			}


        else if(strstr(insdata, "ALTP") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpaltp, sizeof(helpaltp), MDELAY);
            else
                {
				memset(&staltp, ' ', sizeof(staltp));
            switch(ii)
            	{
                case 4:
                	if((fdstbd = open("/tmp/paramete/altp.txt", O_RDONLY)) < 1)
                    	{
						returnTorF(portfd, returntf, 0);
        	            goto txend;
	                    }
                    read(fdstbd, staltp, sizeof(staltp));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, staltp, strcspn(staltp, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
				case 8:
				case 9:
				case 10:
                case 11:
                    if((fdstbd = open("/tmp/paramete/altp.txt", O_RDWR)) < 1)
                        {
						returnTorF(portfd, returntf, 0);
        	            goto txend;
	                    }
					stringmove(ins, 5, staltp, 0, strcspn(ins, "\n")-4);
//                    stringmove(insdata, 5, staltp, 0, ii-5);
//                    for(nn=11-ii; nn>0; nn--,ii++)
//                    	staltp[ii-5] = ' ';
//					staltp[6] = '\r';
//					staltp[7] = '\n';
                    ret = write(fdstbd, staltp, sizeof(staltp));
                    close(fdstbd);
					returnTorF(portfd, returntf, ret);
                    break;
//                default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
//			goto txend;
            }

        else if(strstr(insdata, "ALT") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpalt, sizeof(helpalt), MDELAY);
            else
                {
				memset(stalt, ' ', sizeof(stalt));
            switch(ii)
                {
                case 3:
                    if((fdstbd = open("/tmp/paramete/alt.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, stalt, sizeof(stalt));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stalt, strcspn(stalt, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                case 8:
                case 9:
                case 10:
                    if((fdstbd = open("/tmp/paramete/alt.txt", O_RDWR|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
					stringmove(ins, 4, stalt, 0, strcspn(ins, "\n")-3);
//                    stringmove(insdata, 4, stalt, 0, ii-4);
//                    for(nn=10-ii; nn>0; nn--,ii++)
//                        stalt[ii-4] = ' ';
//                    stalt[6] = '\r';
//                    stalt[7] = '\n';
                    ret = write(fdstbd, stalt, sizeof(stalt));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "SENSI") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsensi, sizeof(helpsensi), MDELAY);
            else
                {
			stringmove(insdata, 6, samplexx, 0, 2);
            if(!(strcmp(samplexx, "GR")))   	samplex=1;
            else if(!(strcmp(samplexx, "DR")))	samplex=2;
            else if(!(strcmp(samplexx, "SR")))  samplex=3;
            else if(!(strcmp(samplexx, "RR")))  samplex=4;
            else if(!(strcmp(samplexx, "UR")))  samplex=5;
            else if(!(strcmp(samplexx, "AR")))  samplex=6;
            else if(!(strcmp(samplexx, "TR")))  samplex=7;
            else if(!(strcmp(samplexx, "PR")))  samplex=8;
            else if(!(strcmp(samplexx, "NR")))  samplex=9;
			else 								samplex=0;

			memset(sensigr, ' ', sizeof(sensigr));
            memset(sensidr, ' ', sizeof(sensidr));
            memset(sensinr, ' ', sizeof(sensinr));
			switch(ii)
				{
				case 8:
					switch(samplex)
						{
						case 1:
				        	if((fdsensit = open("/tmp/paramete/sensigr.txt",O_RDONLY)) < 1)
				           		{
								returnTorF(portfd, returntf, 0);
                				goto txend;
                				}
            				read(fdsensit, sensigr, sizeof(sensigr));
            				close(fdsensit);
                    		rs485clt(fd_485, R485_STA_CTL1, TX);
							write(portfd, sensigr, strcspn(sensigr, "\n")+1);
                    		usleep(STIME);
                    		rs485clt(fd_485, R485_STA_CTL1, RX);
							break;
                        case 2:
	                        if((fdsensit = open("/tmp/paramete/sensidr.txt",O_RDONLY)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							break;
                        case 3:
                            if((fdsensit = open("/tmp/paramete/sensisr.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							break;
                        case 4:
                            if((fdsensit = open("/tmp/paramete/sensirr.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 5:
                            if((fdsensit = open("/tmp/paramete/sensiur.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 6:
                            if((fdsensit = open("/tmp/paramete/sensiar.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 7:
                            if((fdsensit = open("/tmp/paramete/sensitr.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 8:
                            if((fdsensit = open("/tmp/paramete/sensipr.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensidr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 9:
                            if((fdsensit = open("/tmp/paramete/sensinr.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, sensinr, sizeof(sensinr));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, sensinr, strcspn(sensidr, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
					default:  returnTorF(portfd, returntf, 0);
					}
//					stringmove(sensitive,0,sensit,0,6);
//					sensit[6]='\r';
//					sensit[7]='\n';
//                                      write(portfd,&sensit,sizeof(sensit));
					break;
					
				case 10:
                case 11:
                case 12:
                case 13:
				case 14:
					switch(samplex)
						{
						case 1:
							if((fdsensit = open("/tmp/paramete/sensigr.txt",O_WRONLY|O_CREAT)) < 1)
                        		{
								returnTorF(portfd, returntf, 0);
        	                    goto txend;
	                            }
							stringmove(ins, 9, sensigr, 0, strcspn(ins, "\n")-8);
							ret = write(fdsensit, sensigr, sizeof(sensigr));
							close(fdsensit);
                            returnTorF(portfd, returntf, ret);
							break;
						case 2:
							if((fdsensit = open("/tmp/paramete/sensidr.txt",O_WRONLY|O_CREAT)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
						case 3:
                            if((fdsensit = open("/tmp/paramete/sensisr.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 4:
                            if((fdsensit = open("/tmp/paramete/sensirr.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 5:
                            if((fdsensit = open("/tmp/paramete/sensiur.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                        case 6:
                            if((fdsensit = open("/tmp/paramete/sensiar.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                        case 7:
                            if((fdsensit = open("/tmp/paramete/sensitr.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                        case 8:
                            if((fdsensit = open("/tmp/paramete/sensipr.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            stringmove(ins, 9, sensidr, 0, strcspn(ins, "\n")-8);
                            ret = write(fdsensit, sensidr, sizeof(sensidr));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
						default: returnTorF(portfd, returntf, 0);
						}
					break;

                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
				case 21:
                    if((fdsensit = open("/tmp/paramete/sensinr.txt",O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 9, sensinr, 0, strcspn(ins, "\n")-8);
                    ret = write(fdsensit, sensinr, sizeof(sensinr));
                    close(fdsensit);
                    returnTorF(portfd, returntf, ret);
					break;	
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
//				close(fdsensit);
			}

		else if(strstr(insdata, "SMC") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsmc, sizeof(helpsmc), MDELAY);
            else
                {
			if(insdata[6] == '1')		samplex=1;
			else if(insdata[6] == '2')	samplex=2;
            else if(insdata[6] == '3')  samplex=3;
            else if(insdata[6] == '4')  samplex=4;
            else if(insdata[6] == '5')  samplex=5;
            else if(insdata[6] == '6')  samplex=6;
            else if(insdata[6] == '7')  samplex=7;
            else if(insdata[6] == '8')  samplex=8;
			else						samplex=0;

			memset(&smcsm, ' ', sizeof(smcsm));
			switch(ii)
				{
				case 7:
					switch(samplex)
						{
						case 1:
							if((fdsensit = open("/tmp/paramete/smcsm1.txt",O_RDONLY)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							break;
                        case 2:
                            if((fdsensit = open("/tmp/paramete/smcsm2.txt",O_RDONLY)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 3:
                            if((fdsensit = open("/tmp/paramete/smcsm3.txt",O_RDONLY)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 4:
                            if((fdsensit = open("/tmp/paramete/smcsm4.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 5:
                            if((fdsensit = open("/tmp/paramete/smcsm5.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 6:
                            if((fdsensit = open("/tmp/paramete/smcsm6.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 7:
                            if((fdsensit = open("/tmp/paramete/smcsm7.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
                        case 8:
                            if((fdsensit = open("/tmp/paramete/smcsm8.txt",O_RDONLY)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            read(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
                            write(portfd, smcsm, strcspn(smcsm, "\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
                            break;
						default: returnTorF(portfd, returntf, 0);
						}
					break;
                case 13:
                case 14:
                case 15:
				case 16:
					stringmove(ins, 8, smcsm, 0, strcspn(ins, "\n")-7);
					switch(samplex)
                        {
                        case 1:
                            if((fdsensit = open("/tmp/paramete/smcsm1.txt",O_WRONLY|O_CREAT)) < 1)
                                {
								returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
							ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
							returnTorF(portfd, returntf, ret);
                            break;
                        case 2:
                            if((fdsensit = open("/tmp/paramete/smcsm2.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 3:
                            if((fdsensit = open("/tmp/paramete/smcsm3.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 4:
                            if((fdsensit = open("/tmp/paramete/smcsm4.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 5:
                            if((fdsensit = open("/tmp/paramete/smcsm5.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 6:
                            if((fdsensit = open("/tmp/paramete/smcsm6.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 7:
                            if((fdsensit = open("/tmp/paramete/smcsm7.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
                        case 8:
                            if((fdsensit = open("/tmp/paramete/smcsm8.txt",O_WRONLY|O_CREAT)) < 1)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            ret = write(fdsensit, smcsm, sizeof(smcsm));
                            close(fdsensit);
                            returnTorF(portfd, returntf, ret);
                            break;
						default: returnTorF(portfd, returntf, 0);
						}
					break;
//				default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}


		else if(strstr(insdata, "DOOR") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdoor, sizeof(helpdoor), SDELAY);
            else
                {
            	if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                	{
                	returnTorF(portfd, returntf, 0);
                	goto txend;
                	}
            	read(fdmaste, door, sizeof(door));
            	close(fdmaste);
            	rs485clt(fd_485, R485_STA_CTL1, TX);
				write(portfd, door, sizeof(door));
            	usleep(STIME);
            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
			}

        else if(strstr(insdata, "GPSSET") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpgpsset, sizeof(helpgpsset), SDELAY);
            else
                {
                switch(ii)
                    {
                    case 6:
                        if((fdset = open("/tmp/paramete/gpsset.txt", O_RDONLY)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        read(fdset, sst, sizeof(sst));
                        close(fdset);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        break;
                    case 8:
                        if((fdset = open("/tmp/paramete/gpsset.txt", O_WRONLY|O_CREAT)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        sst[0] = insdata[7];
                        ret = write(fdset, sst, sizeof(sst));
                        close(fdstbd);
                        returnTorF(portfd, returntf, ret);
                        break;
                    default:
                        returnTorF(portfd, returntf, 0);
                    }
                }
            }

        else if(strstr(insdata, "PSS") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helppss, sizeof(helppss), SDELAY);
            else
                {
            	if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
            		{
					returnTorF(portfd, returntf, 0);
                	goto txend;
                	}
            	read(fdmaste, pss, sizeof(pss));
            	close(fdmaste);
            	rs485clt(fd_485, R485_STA_CTL1, TX);
            	write(portfd, pss, sizeof(pss));
            	usleep(STIME);
            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
			}

        else if(strstr(insdata, "MACT") != 0)
        	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpmact, sizeof(helpmact), SDELAY);
            else
                {
		    	if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
    				{
                	returnTorF(portfd, returntf, 0);
	                goto txend;
					}
	        	ret = read(fdmaste, &masttemp, 2);
				usleep(10000);
        	    ret = read(fdmaste, &masttemp, 2);
//				printf("temp=%d\n",masttemp);
				close(fdmaste);
    	    	if (ret > 0)
					{
					if(masttemp >= 0)
						{
						mact[0] = ' ';
						mact[1] = masttemp / 100 + '0';
						mact[2] = masttemp % 100 / 10 + '0';
						mact[3] = '.';
						mact[4] = masttemp % 10 + '0'; 
						}
					else
						{
						mact[0] = '-';
                	    mact[1] = masttemp * (-1) / 100 + '0';
	                    mact[2] = masttemp * (-1) % 100 / 10 + '0';
    	                mact[3] = '.';
        	            mact[4] = masttemp * (-1) % 10 + '0';
						}
					mact[5] = '\r';
					mact[6] = '\n';
    	        	rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, mact, sizeof(mact));
            		usleep(STIME);
            		rs485clt(fd_485, R485_STA_CTL1, RX);
//            		printf("temp =  %d.%d \n",temp/10,temp%10);
    				}
				}
			}


        else if(strstr(insdata, "SENST") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsenst, sizeof(helpsenst), MDELAY);
            else
                {
//            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
//            	{
//				returnTorF(portfd, returntf, 0);
//				goto txend;
//                }
//            read(fdmaste, senstate, sizeof(senstate));
//            close(fdmaste);

			switch(ii)
				{
				case 5:
	        	    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
    	        	    {
        	        	returnTorF(portfd, returntf, 0);
                		goto txend;
                		}
            		read(fdmaste, senstate, sizeof(senstate));
            		close(fdmaste);
//					senstate[71] = '\r';
//					senstate[72] = '\n';
                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, senstate, sizeof(senstate));
            		usleep(STIME);
            		rs485clt(fd_485, R485_STA_CTL1, RX);
					break;
				case 79:
		            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_WRONLY|O_CREAT)) < 1)
                		{
						returnTorF(portfd, returntf, 0);
						goto txend;
                		}
					stringmove(ins, 6, senstate, 0, sizeof(senstate));
            		ret = write(fdmaste, senstate, sizeof(senstate));
            		close(fdmaste);
					returnTorF(portfd, returntf, ret);
					break;
				case 7:
                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdmaste, senstate, sizeof(senstate));
                    close(fdmaste);
					if(insdata[6] == 'P')		samplex=1;
					else if(insdata[6] == 'U')	samplex=7;
					else						samplex=0;
					if(samplex == 0)
						returnTorF(portfd, returntf, 0);
					else
						{
						sst[0] = senstate[samplex-1];
                    	rs485clt(fd_485, R485_STA_CTL1, TX);
						write(portfd, sst, sizeof(sst));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					break;
				case 8:
                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdmaste, senstate, sizeof(senstate));
                    close(fdmaste);
		            stringmove(insdata,6,samplexx,0,2);
                    if(!(strcmp(samplexx, "T0")))   	samplex=2;
                    else if(!(strcmp(samplexx, "T1")))	samplex=3;
                    else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    else if(!(strcmp(samplexx, "WS")))  samplex=13;
                    else if(!(strcmp(samplexx, "TG")))  samplex=18;
                    else if(!(strcmp(samplexx, "IR")))  samplex=19;
                    else if(!(strcmp(samplexx, "LE")))  samplex=29;
                    else if(!(strcmp(samplexx, "VI")))  samplex=30;
                    else if(!(strcmp(samplexx, "CH")))  samplex=31;
                    else if(!(strcmp(samplexx, "WW")))  samplex=34;
                    else if(!(strcmp(samplexx, "SD")))  samplex=35;
                    else if(!(strcmp(samplexx, "FR")))  samplex=36;
                    else if(!(strcmp(samplexx, "WI")))  samplex=37;
                    else if(!(strcmp(samplexx, "GR")))  samplex=40;
                    else if(!(strcmp(samplexx, "NR")))  samplex=41;
                    else if(!(strcmp(samplexx, "DR")))  samplex=42;
                    else if(!(strcmp(samplexx, "SR")))  samplex=43;
                    else if(!(strcmp(samplexx, "RR")))  samplex=44;
                    else if(!(strcmp(samplexx, "UR")))  samplex=45;
                    else if(!(strcmp(samplexx, "AR")))  samplex=48;
                    else if(!(strcmp(samplexx, "TR")))  samplex=50;
                    else if(!(strcmp(samplexx, "PR")))  samplex=52;
                    else if(!(strcmp(samplexx, "WT")))  samplex=62;
                    else if(!(strcmp(samplexx, "BA")))  samplex=63;
                    else if(!(strcmp(samplexx, "OT")))  samplex=64;
                    else if(!(strcmp(samplexx, "OS")))  samplex=65;
                    else if(!(strcmp(samplexx, "OC")))  samplex=66;
                    else if(!(strcmp(samplexx, "OH")))  samplex=67;
                    else if(!(strcmp(samplexx, "OP")))  samplex=68;
                    else if(!(strcmp(samplexx, "OV")))  samplex=69;
                    else if(!(strcmp(samplexx, "OD")))  samplex=70;
                    else if(!(strcmp(samplexx, "TL")))  samplex=71;
					else								samplex=0;	
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        sst[0] = senstate[samplex-1];
                    	rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
                    break;
                case 9:
                    stringmove(insdata,6,samplexxx,0,3);
                    if(strstr(samplexxx,"SV1"))      samplex=9;
                    else if(strstr(samplexxx,"SV2")) samplex=10;
                    else if(strstr(samplexxx,"SV3")) samplex=11;
                    else if(strstr(samplexxx,"WS1")) samplex=14;
                    else if(strstr(samplexxx,"RAT")) samplex=15;
                    else if(strstr(samplexxx,"RAW")) samplex=17;
                    else if(strstr(samplexxx,"ST0")) samplex=20;
                    else if(strstr(samplexxx,"ST1")) samplex=21;
                    else if(strstr(samplexxx,"ST2")) samplex=22;
                    else if(strstr(samplexxx,"ST3")) samplex=23;
                    else if(strstr(samplexxx,"ST4")) samplex=24;
                    else if(strstr(samplexxx,"ST5")) samplex=25;
                    else if(strstr(samplexxx,"ST6")) samplex=26;
                    else if(strstr(samplexxx,"ST7")) samplex=27;
                    else if(strstr(samplexxx,"ST8")) samplex=28;
                    else if(strstr(samplexxx,"TCA")) samplex=32;
                    else if(strstr(samplexxx,"LCA")) samplex=33;
                    else if(strstr(samplexxx,"FSD")) samplex=38;
                    else if(strstr(samplexxx,"LNF")) samplex=39;
                    else if(strstr(samplexxx,"UVA")) samplex=46;
                    else if(strstr(samplexxx,"UVB")) samplex=47;
                    else if(strstr(samplexxx,"ART")) samplex=49;
                    else if(strstr(samplexxx,"TRT")) samplex=51;
                    else if(strstr(samplexxx,"SSD")) samplex=53;
                    else if(strstr(samplexxx,"SM1")) samplex=54;
                    else if(strstr(samplexxx,"SM2")) samplex=55;
                    else if(strstr(samplexxx,"SM3")) samplex=56;
                    else if(strstr(samplexxx,"SM4")) samplex=57;
                    else if(strstr(samplexxx,"SM5")) samplex=58;
                    else if(strstr(samplexxx,"SM6")) samplex=59;
                    else if(strstr(samplexxx,"SM7")) samplex=60;
                    else if(strstr(samplexxx,"SM8")) samplex=61;
                    else if(strstr(samplexxx,"OTU")) samplex=72;
                    else if(strstr(samplexxx,"OCC")) samplex=73;
					else		                  	 samplex=0;
                    if(samplex == 0)
						{
	                    stringmove(insdata,6,samplexx,0,2);
    	                if(!(strcmp(samplexx, "P ")))       samplex=1;
        	            else if(!(strcmp(samplexx, "U ")))  samplex=7;
            	        else                                samplex=0;
                        if(samplex == 0)
                        	returnTorF(portfd, returntf, 0);
                    	else
                        	{
	                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
    	                        {
        	                    returnTorF(portfd, returntf, 0);
            	                goto txend;
                	            }
                    	    read(fdmaste, senstate, sizeof(senstate));
                        	close(fdmaste);
							senstate[samplex-1] = insdata[8];
		                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_WRONLY|O_CREAT)) < 1)
        		                {
                		        returnTorF(portfd, returntf, 0);
                        		goto txend;
                        		}
                    		ret = write(fdmaste, senstate, sizeof(senstate));
                    		close(fdmaste);
                    		returnTorF(portfd, returntf, ret);
							}
						}
                    else
                        {
	                    if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
    	                    {
        	                returnTorF(portfd, returntf, 0);
            	            goto txend;
                	        }
                    	read(fdmaste, senstate, sizeof(senstate));
                    	close(fdmaste);
                        sst[0] = senstate[samplex-1];
                    	rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                    	usleep(STIME);
                    	rs485clt(fd_485, R485_STA_CTL1, RX);
                        }
					break;
				case 10:
					if(strstr(insdata,"RAT1"))
						{
						samplex=16;
						sst[0] = senstate[samplex-1];
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
						break;
						}
                    stringmove(insdata,6,samplexx,0,2);
                    if(!(strcmp(samplexx, "T0")))       samplex=2;
                    else if(!(strcmp(samplexx, "T1")))  samplex=3;
                    else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    else if(!(strcmp(samplexx, "WS")))  samplex=13;
                    else if(!(strcmp(samplexx, "TG")))  samplex=18;
                    else if(!(strcmp(samplexx, "IR")))  samplex=19;
                    else if(!(strcmp(samplexx, "LE")))  samplex=29;
                    else if(!(strcmp(samplexx, "VI")))  samplex=30;
                    else if(!(strcmp(samplexx, "CH")))  samplex=31;
                    else if(!(strcmp(samplexx, "WW")))  samplex=34;
                    else if(!(strcmp(samplexx, "SD")))  samplex=35;
                    else if(!(strcmp(samplexx, "FR")))  samplex=36;
                    else if(!(strcmp(samplexx, "WI")))  samplex=37;
                    else if(!(strcmp(samplexx, "GR")))  samplex=40;
                    else if(!(strcmp(samplexx, "NR")))  samplex=41;
                    else if(!(strcmp(samplexx, "DR")))  samplex=42;
                    else if(!(strcmp(samplexx, "SR")))  samplex=43;
                    else if(!(strcmp(samplexx, "RR")))  samplex=44;
                    else if(!(strcmp(samplexx, "UR")))  samplex=45;
                    else if(!(strcmp(samplexx, "AR")))  samplex=48;
                    else if(!(strcmp(samplexx, "TR")))  samplex=50;
                    else if(!(strcmp(samplexx, "PR")))  samplex=52;
                    else if(!(strcmp(samplexx, "WT")))  samplex=62;
                    else if(!(strcmp(samplexx, "BA")))  samplex=63;
                    else if(!(strcmp(samplexx, "OT")))  samplex=64;
                    else if(!(strcmp(samplexx, "OS")))  samplex=65;
                    else if(!(strcmp(samplexx, "OC")))  samplex=66;
                    else if(!(strcmp(samplexx, "OH")))  samplex=67;
                    else if(!(strcmp(samplexx, "OP")))  samplex=68;
                    else if(!(strcmp(samplexx, "OV")))  samplex=69;
                    else if(!(strcmp(samplexx, "OD")))  samplex=70;
                    else if(!(strcmp(samplexx, "TL")))  samplex=71;
                    else                                samplex=0;
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
					else
						{
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                        	{
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        senstate[samplex-1] = insdata[9];
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_WRONLY|O_CREAT)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        ret = write(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        returnTorF(portfd, returntf, ret);
						}
					break;
				case 11:
                    stringmove(insdata,6,samplexxx,0,3);
                    if(strstr(samplexxx,"SV1"))      samplex=9;
                    else if(strstr(samplexxx,"SV2")) samplex=10;
                    else if(strstr(samplexxx,"SV3")) samplex=11;
                    else if(strstr(samplexxx,"WS1")) samplex=14;
                    else if(strstr(samplexxx,"RAT")) samplex=15;
                    else if(strstr(samplexxx,"RAW")) samplex=17;
                    else if(strstr(samplexxx,"ST0")) samplex=20;
                    else if(strstr(samplexxx,"ST1")) samplex=21;
                    else if(strstr(samplexxx,"ST2")) samplex=22;
                    else if(strstr(samplexxx,"ST3")) samplex=23;
                    else if(strstr(samplexxx,"ST4")) samplex=24;
                    else if(strstr(samplexxx,"ST5")) samplex=25;
                    else if(strstr(samplexxx,"ST6")) samplex=26;
                    else if(strstr(samplexxx,"ST7")) samplex=27;
                    else if(strstr(samplexxx,"ST8")) samplex=28;
                    else if(strstr(samplexxx,"TCA")) samplex=32;
                    else if(strstr(samplexxx,"LCA")) samplex=33;
                    else if(strstr(samplexxx,"FSD")) samplex=38;
                    else if(strstr(samplexxx,"LNF")) samplex=39;
                    else if(strstr(samplexxx,"UVA")) samplex=46;
                    else if(strstr(samplexxx,"UVB")) samplex=47;
                    else if(strstr(samplexxx,"ART")) samplex=49;
                    else if(strstr(samplexxx,"TRT")) samplex=51;
                    else if(strstr(samplexxx,"SSD")) samplex=53;
                    else if(strstr(samplexxx,"SM1")) samplex=54;
                    else if(strstr(samplexxx,"SM2")) samplex=55;
                    else if(strstr(samplexxx,"SM3")) samplex=56;
                    else if(strstr(samplexxx,"SM4")) samplex=57;
                    else if(strstr(samplexxx,"SM5")) samplex=58;
                    else if(strstr(samplexxx,"SM6")) samplex=59;
                    else if(strstr(samplexxx,"SM7")) samplex=60;
                    else if(strstr(samplexxx,"SM8")) samplex=61;
                    else if(strstr(samplexxx,"OTU")) samplex=72;
                    else if(strstr(samplexxx,"OCC")) samplex=73;
                    else                             samplex=0;
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        senstate[samplex-1] = insdata[10];
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_WRONLY|O_CREAT)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        ret = write(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        returnTorF(portfd, returntf, ret);
						}
					break;
				case 12:
					if(strstr(insdata,"RAT1"))
				      	{
						samplex=16;
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        read(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        senstate[samplex-1] = insdata[11];
                        if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_WRONLY|O_CREAT)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        ret = write(fdmaste, senstate, sizeof(senstate));
                        close(fdmaste);
                        returnTorF(portfd, returntf, ret);
                        }
					break;
//				default:	returnTorF(portfd, returntf, 0); 
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

        else if(strstr(insdata, "RSTA") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helprsta, sizeof(helprsta), SDELAY);
            else
                {
            memset(rsta, '1', sizeof(rsta));
            if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, door, sizeof(door));
            close(fdmaste);
            rsta[0] = door[0];
            rsta[1] = ' ';

            if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            ret = read(fdmaste, &masttemp, 2);
            usleep(10000);
            ret = read(fdmaste, &masttemp, 2);
//          printf("temp=%d\n",masttemp);
            close(fdmaste);
            if (ret > 0)
                {
                if(masttemp >= 0)
                    {
                    mact[0] = ' ';
                    mact[1] = masttemp / 100 + '0';
                    mact[2] = masttemp % 100 / 10 + '0';
                    mact[3] = '.';
                    mact[4] = masttemp % 10 + '0';
                    }
                else
                    {
                    mact[0] = '-';
                    mact[1] = masttemp * (-1) / 100 + '0';
                    mact[2] = masttemp * (-1) % 100 / 10 + '0';
                    mact[3] = '.';
                    mact[4] = masttemp * (-1) % 10 + '0';
                    }
				}
			stringmove(mact, 0, rsta, 2, 5);
            rsta[7] = ' ';

            if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, pss, sizeof(pss));
//			printf("pss=%s\n",pss);
            close(fdmaste);
			stringmove(pss, 0, rsta, 8, 7);
            rsta[15] = ' ';

            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
                {
                returnTorF(portfd, returntf, 0);
                goto txend;
                }
            read(fdmaste, senstate, sizeof(senstate));
            close(fdmaste);
			stringmove(senstate, 0, rsta, 16, 73);
            rsta[89] = '\r';
            rsta[90] = '\n';

            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, rsta, sizeof(rsta));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
            }
			}

		else if(strstr(insdata, "QCPS") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpqcps, sizeof(helpqcps), MDELAY);
            else
                {
			if(ii < 10)
				{
				switch(ii)
					{
					case 6:
	                    if(insdata[5] == 'P')       samplex=1;
	                    else if(insdata[5] == 'U')  samplex=7;
    	                else                        samplex=0;
        	            if(samplex == 0)
                        	returnTorF(portfd, returntf, 0);
            	        else
                	        {
							if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
								{
								returnTorF(portfd, returntf, 0);
								goto txend;
								}
							idn = (samplex - 1) * BYTEQCPS;
							fseek(fdqcp, idn, SEEK_SET);
							fread(qcps, BYTEQCPS, 1, fdqcp);
							fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//							ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                    		usleep(STIME);
                    		rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                        	}
						break;
					case 7:
                    	stringmove(insdata,5,samplexx,0,2);
                    	if(!(strcmp(samplexx, "T0")))       samplex=2;
                    	else if(!(strcmp(samplexx, "T1")))  samplex=3;
                    	else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    	else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    	else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    	else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    	else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    	else if(!(strcmp(samplexx, "WS")))  samplex=13;
	                    else if(!(strcmp(samplexx, "TG")))  samplex=18;
    	                else if(!(strcmp(samplexx, "IR")))  samplex=19;
        	            else if(!(strcmp(samplexx, "LE")))  samplex=29;
            	        else if(!(strcmp(samplexx, "VI")))  samplex=30;
                	    else if(!(strcmp(samplexx, "CH")))  samplex=31;
	                    else if(!(strcmp(samplexx, "WW")))  samplex=34;
    	                else if(!(strcmp(samplexx, "SD")))  samplex=35;
        	            else if(!(strcmp(samplexx, "FR")))  samplex=36;
            	        else if(!(strcmp(samplexx, "WI")))  samplex=37;
                	    else if(!(strcmp(samplexx, "GR")))  samplex=40;
		                else if(!(strcmp(samplexx, "NR")))  samplex=41;
	                    else if(!(strcmp(samplexx, "DR")))  samplex=42;
    	                else if(!(strcmp(samplexx, "SR")))  samplex=43;
        	            else if(!(strcmp(samplexx, "RR")))  samplex=44;
            	        else if(!(strcmp(samplexx, "UR")))  samplex=45;
                	    else if(!(strcmp(samplexx, "AR")))  samplex=48;
	                    else if(!(strcmp(samplexx, "TR")))  samplex=50;
    	                else if(!(strcmp(samplexx, "PR")))  samplex=52;
        	            else if(!(strcmp(samplexx, "WT")))  samplex=62;
            	        else if(!(strcmp(samplexx, "BA")))  samplex=63;
                	    else if(!(strcmp(samplexx, "OT")))  samplex=64;
	                    else if(!(strcmp(samplexx, "OS")))  samplex=65;
    	                else if(!(strcmp(samplexx, "OC")))  samplex=66;
        	            else if(!(strcmp(samplexx, "OH")))  samplex=67;
            	        else if(!(strcmp(samplexx, "OP")))  samplex=68;
                	    else if(!(strcmp(samplexx, "OV")))  samplex=69;
	                    else if(!(strcmp(samplexx, "OD")))  samplex=70;
    	                else if(!(strcmp(samplexx, "TL")))  samplex=71;
        	            else                                samplex=0;
//						printf("samplex=%d\n",samplex);
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPS;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcps, BYTEQCPS, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 8:
	                    stringmove(insdata,5,samplexxx,0,3);
	                    if(strstr(samplexxx,"SV1"))      samplex=9;
    	                else if(strstr(samplexxx,"SV2")) samplex=10;
        	            else if(strstr(samplexxx,"SV3")) samplex=11;
            	        else if(strstr(samplexxx,"WS1")) samplex=14;
                	    else if(strstr(samplexxx,"RAT")) samplex=15;
	                    else if(strstr(samplexxx,"RAW")) samplex=17;
    	                else if(strstr(samplexxx,"ST0")) samplex=20;
        	            else if(strstr(samplexxx,"ST1")) samplex=21;
            	        else if(strstr(samplexxx,"ST2")) samplex=22;
                	    else if(strstr(samplexxx,"ST3")) samplex=23;
	                    else if(strstr(samplexxx,"ST4")) samplex=24;
    	                else if(strstr(samplexxx,"ST5")) samplex=25;
        	            else if(strstr(samplexxx,"ST6")) samplex=26;
            	        else if(strstr(samplexxx,"ST7")) samplex=27;
                	    else if(strstr(samplexxx,"ST8")) samplex=28;
                    	else if(strstr(samplexxx,"TCA")) samplex=32;
	                    else if(strstr(samplexxx,"LCA")) samplex=33;
    	                else if(strstr(samplexxx,"FSD")) samplex=38;
        	            else if(strstr(samplexxx,"LNF")) samplex=39;
            	        else if(strstr(samplexxx,"UVA")) samplex=46;
                	    else if(strstr(samplexxx,"UVB")) samplex=47;
                    	else if(strstr(samplexxx,"ART")) samplex=49;
	                    else if(strstr(samplexxx,"TRT")) samplex=51;
    	                else if(strstr(samplexxx,"SSD")) samplex=53;
        	            else if(strstr(samplexxx,"SM1")) samplex=54;
            		    else if(strstr(samplexxx,"SM2")) samplex=55;
                    	else if(strstr(samplexxx,"SM3")) samplex=56;
	                    else if(strstr(samplexxx,"SM4")) samplex=57;
    	                else if(strstr(samplexxx,"SM5")) samplex=58;
        	            else if(strstr(samplexxx,"SM6")) samplex=59;
            	        else if(strstr(samplexxx,"SM7")) samplex=60;
                	    else if(strstr(samplexxx,"SM8")) samplex=61;
                    	else if(strstr(samplexxx,"OTU")) samplex=72;
	                    else if(strstr(samplexxx,"OCC")) samplex=73;
    	                else                             samplex=0;
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPS;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcps, BYTEQCPS, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 9:
						if(strstr(insdata,"RAT1"))
							{
							samplex=16;
							if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPS;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcps, BYTEQCPS, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							}
						break;
					default: returnTorF(portfd, returntf, 0);
					}
				}
			else
				{
				qcps1 = strchr(ins, ' ');
				qcps2 = strchr(qcps1+1, ' ');
                if(strstr(insdata, "RAT1 ") != 0)	  samplex=16;
                else if(strstr(insdata, "SV1 ") != 0) samplex=9;
                else if(strstr(insdata, "SV2 ") != 0) samplex=10;
                else if(strstr(insdata, "SV3 ") != 0) samplex=11;
                else if(strstr(insdata, "WS1 ") != 0) samplex=14;
                else if(strstr(insdata, "RAT ") != 0) samplex=15;
                else if(strstr(insdata, "RAW ") != 0) samplex=17;
                else if(strstr(insdata, "ST0 ") != 0) samplex=20;
                else if(strstr(insdata, "ST1 ") != 0) samplex=21;
                else if(strstr(insdata, "ST2 ") != 0) samplex=22;
                else if(strstr(insdata, "ST3 ") != 0) samplex=23;
                else if(strstr(insdata, "ST4 ") != 0) samplex=24;
                else if(strstr(insdata, "ST5 ") != 0) samplex=25;
                else if(strstr(insdata, "ST6 ") != 0) samplex=26;
                else if(strstr(insdata, "ST7 ") != 0) samplex=27;
                else if(strstr(insdata, "ST8 ") != 0) samplex=28;
                else if(strstr(insdata, "TCA ") != 0) samplex=32;
                else if(strstr(insdata, "LCA ") != 0) samplex=33;
                else if(strstr(insdata, "FSD ") != 0) samplex=38;
                else if(strstr(insdata, "LNF ") != 0) samplex=39;
                else if(strstr(insdata, "UVA ") != 0) samplex=46;
                else if(strstr(insdata, "UVB ") != 0) samplex=47;
                else if(strstr(insdata, "ART ") != 0) samplex=49;
                else if(strstr(insdata, "TRT ") != 0) samplex=51;
                else if(strstr(insdata, "SSD ") != 0) samplex=53;
                else if(strstr(insdata, "SM1 ") != 0) samplex=54;
                else if(strstr(insdata, "SM2 ") != 0) samplex=55;
                else if(strstr(insdata, "SM3 ") != 0) samplex=56;
                else if(strstr(insdata, "SM4 ") != 0) samplex=57;
                else if(strstr(insdata, "SM5 ") != 0) samplex=58;
                else if(strstr(insdata, "SM6 ") != 0) samplex=59;
                else if(strstr(insdata, "SM7 ") != 0) samplex=60;
                else if(strstr(insdata, "SM8 ") != 0) samplex=61;
                else if(strstr(insdata, "OTU ") != 0) samplex=72;
                else if(strstr(insdata, "OCC ") != 0) samplex=73;
				else if(strstr(insdata, "T0 ") != 0) samplex=2;
                else if(strstr(insdata, "T1 ") != 0) samplex=3;
                else if(strstr(insdata, "T2 ") != 0) samplex=4;
                else if(strstr(insdata, "T3 ") != 0) samplex=5;
                else if(strstr(insdata, "TW ") != 0) samplex=6;
                else if(strstr(insdata, "TD ") != 0) samplex=8;
                else if(strstr(insdata, "WD ") != 0) samplex=12;
                else if(strstr(insdata, "WS ") != 0) samplex=13;
                else if(strstr(insdata, "TG ") != 0) samplex=18;
                else if(strstr(insdata, "IR ") != 0) samplex=19;
                else if(strstr(insdata, "LE ") != 0) samplex=29;
                else if(strstr(insdata, "VI ") != 0) samplex=30;
                else if(strstr(insdata, "CH ") != 0) samplex=31;
                else if(strstr(insdata, "WW ") != 0) samplex=34;
                else if(strstr(insdata, "SD ") != 0) samplex=35;
                else if(strstr(insdata, "FR ") != 0) samplex=36;
                else if(strstr(insdata, "WI ") != 0) samplex=37;
                else if(strstr(insdata, "GR ") != 0) samplex=40;
                else if(strstr(insdata, "NR ") != 0) samplex=41;
                else if(strstr(insdata, "DR ") != 0) samplex=42;
                else if(strstr(insdata, "SR ") != 0) samplex=43;
                else if(strstr(insdata, "RR ") != 0) samplex=44;
                else if(strstr(insdata, "UR ") != 0) samplex=45;
                else if(strstr(insdata, "AR ") != 0) samplex=48;
                else if(strstr(insdata, "TR ") != 0) samplex=50;
                else if(strstr(insdata, "PR ") != 0) samplex=52;
                else if(strstr(insdata, "WT ") != 0) samplex=62;
                else if(strstr(insdata, "BA ") != 0) samplex=63;
                else if(strstr(insdata, "OT ") != 0) samplex=64;
                else if(strstr(insdata, "OS ") != 0) samplex=65;
                else if(strstr(insdata, "OC ") != 0) samplex=66;
                else if(strstr(insdata, "OH ") != 0) samplex=67;
                else if(strstr(insdata, "OP ") != 0) samplex=68;
                else if(strstr(insdata, "OV ") != 0) samplex=69;
                else if(strstr(insdata, "OD ") != 0) samplex=70;
                else if(strstr(insdata, "TL ") != 0) samplex=71;
                else if(strstr(insdata, "P ") != 0)  samplex=1;
                else if(strstr(insdata, "U ") != 0)  samplex=7;
				else								 samplex=0;

//				printf("samplex=%d\n",samplex);
                if(samplex == 0)
	                returnTorF(portfd, returntf, 0);
                else
                    {
                    if((fdqcp = fopen("/tmp/paramete/qcps.txt","r+")) == NULL)
                		{
                		if((fdqcp = fopen("/tmp/paramete/qcps.txt","w+")) == NULL)
                    		{
//                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
						}
                    idn = (samplex - 1) * BYTEQCPS;
//					printf("idn=%d, qcps2=%s\n",idn,qcps2);
//					fseek(fdqcp, idn, SEEK_SET);
//					memset(qcps, ' ', BYTEQCPS);
//					fwrite(qcps, BYTEQCPS, 1, fdqcp);
                    fseek(fdqcp, idn, SEEK_SET);
//                    ret = fwrite((qcps2+1), BYTEQCPS, 1, fdqcp);
                    ret = fwrite((qcps2+1), strlen(qcps2)-1, 1, fdqcp);
                    fclose(fdqcp);
                    returnTorF(portfd, returntf, ret);
                    }
				}
			}
		}

		else if(strstr(insdata, "QCPM") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpqcpm, sizeof(helpqcpm), MDELAY);
            else
                {
			if(ii < 10)
				{
				switch(ii)
					{
					case 6:
	                    if(insdata[5] == 'P')       samplex=1;
	                    else if(insdata[5] == 'U')  samplex=7;
    	                else                        samplex=0;
        	            if(samplex == 0)
                        	returnTorF(portfd, returntf, 0);
            	        else
                	        {
							if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
								{
								returnTorF(portfd, returntf, 0);
								goto txend;
								}
							idn = (samplex - 1) * BYTEQCPM;
							fseek(fdqcp, idn, SEEK_SET);
							fread(qcpm, BYTEQCPM, 1, fdqcp);
							fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//							ret = write(portfd, qcpm, BYTEQCPM);
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
							usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                        	}
						break;
					case 7:
                    	stringmove(insdata,5,samplexx,0,2);
                    	if(!(strcmp(samplexx, "T0")))       samplex=2;
                    	else if(!(strcmp(samplexx, "T1")))  samplex=3;
                    	else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    	else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    	else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    	else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    	else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    	else if(!(strcmp(samplexx, "WS")))  samplex=13;
	                    else if(!(strcmp(samplexx, "TG")))  samplex=18;
    	                else if(!(strcmp(samplexx, "IR")))  samplex=19;
        	            else if(!(strcmp(samplexx, "LE")))  samplex=29;
            	        else if(!(strcmp(samplexx, "VI")))  samplex=30;
                	    else if(!(strcmp(samplexx, "CH")))  samplex=31;
	                    else if(!(strcmp(samplexx, "WW")))  samplex=34;
    	                else if(!(strcmp(samplexx, "SD")))  samplex=35;
        	            else if(!(strcmp(samplexx, "FR")))  samplex=36;
            	        else if(!(strcmp(samplexx, "WI")))  samplex=37;
	                    else if(!(strcmp(samplexx, "GR")))  samplex=40;
    	                else if(!(strcmp(samplexx, "NR")))  samplex=41;
        	            else if(!(strcmp(samplexx, "DR")))  samplex=42;
            	        else if(!(strcmp(samplexx, "SR")))  samplex=43;
	                    else if(!(strcmp(samplexx, "RR")))  samplex=44;
    	                else if(!(strcmp(samplexx, "UR")))  samplex=45;
        	            else if(!(strcmp(samplexx, "AR")))  samplex=48;
            	        else if(!(strcmp(samplexx, "TR")))  samplex=50;
	                    else if(!(strcmp(samplexx, "PR")))  samplex=52;
    	                else if(!(strcmp(samplexx, "WT")))  samplex=62;
        	            else if(!(strcmp(samplexx, "BA")))  samplex=63;
            	        else if(!(strcmp(samplexx, "OT")))  samplex=64;
	                    else if(!(strcmp(samplexx, "OS")))  samplex=65;
    	                else if(!(strcmp(samplexx, "OC")))  samplex=66;
        	            else if(!(strcmp(samplexx, "OH")))  samplex=67;
            	        else if(!(strcmp(samplexx, "OP")))  samplex=68;
	                    else if(!(strcmp(samplexx, "OV")))  samplex=69;
    	                else if(!(strcmp(samplexx, "OD")))  samplex=70;
        	            else if(!(strcmp(samplexx, "TL")))  samplex=71;
            	        else                                samplex=0;
//						printf("samplex=%d\n",samplex);
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPM;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcpm, BYTEQCPM, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcpm, BYTEQCPM);
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 8:
	                    stringmove(insdata,5,samplexxx,0,3);
	                    if(strstr(samplexxx,"SV1"))      samplex=9;
    	                else if(strstr(samplexxx,"SV2")) samplex=10;
        	            else if(strstr(samplexxx,"SV3")) samplex=11;
            	        else if(strstr(samplexxx,"WS1")) samplex=14;
                	    else if(strstr(samplexxx,"RAT")) samplex=15;
                        else if(strstr(samplexxx,"RAW")) samplex=17;
                        else if(strstr(samplexxx,"ST0")) samplex=20;
                        else if(strstr(samplexxx,"ST1")) samplex=21;
                        else if(strstr(samplexxx,"ST2")) samplex=22;
                        else if(strstr(samplexxx,"ST3")) samplex=23;
                        else if(strstr(samplexxx,"ST4")) samplex=24;
                        else if(strstr(samplexxx,"ST5")) samplex=25;
                        else if(strstr(samplexxx,"ST6")) samplex=26;
                        else if(strstr(samplexxx,"ST7")) samplex=27;
                        else if(strstr(samplexxx,"ST8")) samplex=28;
                        else if(strstr(samplexxx,"TCA")) samplex=32;
                        else if(strstr(samplexxx,"LCA")) samplex=33;
                        else if(strstr(samplexxx,"FSD")) samplex=38;
                        else if(strstr(samplexxx,"LNF")) samplex=39;
                        else if(strstr(samplexxx,"UVA")) samplex=46;
                        else if(strstr(samplexxx,"UVB")) samplex=47;
                        else if(strstr(samplexxx,"ART")) samplex=49;
                        else if(strstr(samplexxx,"TRT")) samplex=51;
                        else if(strstr(samplexxx,"SSD")) samplex=53;
                        else if(strstr(samplexxx,"SM1")) samplex=54;
                        else if(strstr(samplexxx,"SM2")) samplex=55;
                        else if(strstr(samplexxx,"SM3")) samplex=56;
                        else if(strstr(samplexxx,"SM4")) samplex=57;
                        else if(strstr(samplexxx,"SM5")) samplex=58;
                        else if(strstr(samplexxx,"SM6")) samplex=59;
                        else if(strstr(samplexxx,"SM7")) samplex=60;
                        else if(strstr(samplexxx,"SM8")) samplex=61;
                        else if(strstr(samplexxx,"OTU")) samplex=72;
                        else if(strstr(samplexxx,"OCC")) samplex=73;
                        else                             samplex=0;
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPM;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcpm, BYTEQCPM, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcpm, BYTEQCPM);
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 9:
						if(strstr(insdata,"RAT1"))
							{
							samplex=16;
							if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTEQCPM;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(qcpm, BYTEQCPM, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcpm, BYTEQCPM);
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							}
						break;
					default: returnTorF(portfd, returntf, 0);
					}
				}
			else
				{
				qcpm1 = strchr(ins, ' ');
				qcpm2 = strchr(qcpm1+1, ' ');
                if(strstr(insdata, "RAT1 ") != 0)	  samplex=16;
                else if(strstr(insdata, "SV1 ") != 0) samplex=9;
                else if(strstr(insdata, "SV2 ") != 0) samplex=10;
                else if(strstr(insdata, "SV3 ") != 0) samplex=11;
                else if(strstr(insdata, "WS1 ") != 0) samplex=14;
                else if(strstr(insdata, "RAT ") != 0) samplex=15;
                else if(strstr(insdata, "RAW ") != 0) samplex=17;
                else if(strstr(insdata, "ST0 ") != 0) samplex=20;
                else if(strstr(insdata, "ST1 ") != 0) samplex=21;
                else if(strstr(insdata, "ST2 ") != 0) samplex=22;
                else if(strstr(insdata, "ST3 ") != 0) samplex=23;
                else if(strstr(insdata, "ST4 ") != 0) samplex=24;
                else if(strstr(insdata, "ST5 ") != 0) samplex=25;
                else if(strstr(insdata, "ST6 ") != 0) samplex=26;
                else if(strstr(insdata, "ST7 ") != 0) samplex=27;
                else if(strstr(insdata, "ST8 ") != 0) samplex=28;
                else if(strstr(insdata, "TCA ") != 0) samplex=32;
                else if(strstr(insdata, "LCA ") != 0) samplex=33;
                else if(strstr(insdata, "FSD ") != 0) samplex=38;
                else if(strstr(insdata, "LNF ") != 0) samplex=39;
                else if(strstr(insdata, "UVA ") != 0) samplex=46;
                else if(strstr(insdata, "UVB ") != 0) samplex=47;
                else if(strstr(insdata, "ART ") != 0) samplex=49;
                else if(strstr(insdata, "TRT ") != 0) samplex=51;
                else if(strstr(insdata, "SSD ") != 0) samplex=53;
                else if(strstr(insdata, "SM1 ") != 0) samplex=54;
                else if(strstr(insdata, "SM2 ") != 0) samplex=55;
                else if(strstr(insdata, "SM3 ") != 0) samplex=56;
                else if(strstr(insdata, "SM4 ") != 0) samplex=57;
                else if(strstr(insdata, "SM5 ") != 0) samplex=58;
                else if(strstr(insdata, "SM6 ") != 0) samplex=59;
                else if(strstr(insdata, "SM7 ") != 0) samplex=60;
                else if(strstr(insdata, "SM8 ") != 0) samplex=61;
                else if(strstr(insdata, "OTU ") != 0) samplex=72;
                else if(strstr(insdata, "OCC ") != 0) samplex=73;
                else if(strstr(insdata, "T0 ") != 0) samplex=2;
                else if(strstr(insdata, "T1 ") != 0) samplex=3;
                else if(strstr(insdata, "T2 ") != 0) samplex=4;
                else if(strstr(insdata, "T3 ") != 0) samplex=5;
                else if(strstr(insdata, "TW ") != 0) samplex=6;
                else if(strstr(insdata, "TD ") != 0) samplex=8;
                else if(strstr(insdata, "WD ") != 0) samplex=12;
                else if(strstr(insdata, "WS ") != 0) samplex=13;
                else if(strstr(insdata, "TG ") != 0) samplex=18;
                else if(strstr(insdata, "IR ") != 0) samplex=19;
                else if(strstr(insdata, "LE ") != 0) samplex=29;
                else if(strstr(insdata, "VI ") != 0) samplex=30;
                else if(strstr(insdata, "CH ") != 0) samplex=31;
                else if(strstr(insdata, "WW ") != 0) samplex=34;
                else if(strstr(insdata, "SD ") != 0) samplex=35;
                else if(strstr(insdata, "FR ") != 0) samplex=36;
                else if(strstr(insdata, "WI ") != 0) samplex=37;
                else if(strstr(insdata, "GR ") != 0) samplex=40;
                else if(strstr(insdata, "NR ") != 0) samplex=41;
                else if(strstr(insdata, "DR ") != 0) samplex=42;
                else if(strstr(insdata, "SR ") != 0) samplex=43;
                else if(strstr(insdata, "RR ") != 0) samplex=44;
                else if(strstr(insdata, "UR ") != 0) samplex=45;
                else if(strstr(insdata, "AR ") != 0) samplex=48;
                else if(strstr(insdata, "TR ") != 0) samplex=50;
                else if(strstr(insdata, "PR ") != 0) samplex=52;
                else if(strstr(insdata, "WT ") != 0) samplex=62;
                else if(strstr(insdata, "BA ") != 0) samplex=63;
                else if(strstr(insdata, "OT ") != 0) samplex=64;
                else if(strstr(insdata, "OS ") != 0) samplex=65;
                else if(strstr(insdata, "OC ") != 0) samplex=66;
                else if(strstr(insdata, "OH ") != 0) samplex=67;
                else if(strstr(insdata, "OP ") != 0) samplex=68;
                else if(strstr(insdata, "OV ") != 0) samplex=69;
                else if(strstr(insdata, "OD ") != 0) samplex=70;
                else if(strstr(insdata, "TL ") != 0) samplex=71;
                else if(strstr(insdata, "P ") != 0)  samplex=1;
                else if(strstr(insdata, "U ") != 0)  samplex=7;
				else								 samplex=0;

//				printf("samplex=%d\n",samplex);
                if(samplex == 0)
	                returnTorF(portfd, returntf, 0);
                else
                    {
                    if((fdqcp = fopen("/tmp/paramete/qcpm.txt","r+")) == NULL)
                		{
                		if((fdqcp = fopen("/tmp/paramete/qcpm.txt","w+")) == NULL)
                    		{
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
						}
                    idn = (samplex - 1) * BYTEQCPM;
                    fseek(fdqcp, idn, SEEK_SET);
//                    ret = fwrite((qcpm2+1), BYTEQCPM, 1, fdqcp);
                    ret = fwrite((qcpm2+1), strlen(qcpm2)-1, 1, fdqcp);
                    fclose(fdqcp);
                    returnTorF(portfd, returntf, ret);
                    }
				}
			}
			}

		else if(strstr(insdata, "SCV") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpscv, sizeof(helpscv), MDELAY);
            else
                {
			if(ii < 8)
				{
				switch(ii)
					{
					case 5:
	                    if(insdata[4] == 'P')       samplex=1;
	                    else if(insdata[4] == 'U')  samplex=7;
    	                else                        samplex=0;
        	            if(samplex == 0)
                        	returnTorF(portfd, returntf, 0);
            	        else
                	        {
							if((fdqcp = fopen("/tmp/paramete/scv.txt","r+")) == NULL)
								{
								returnTorF(portfd, returntf, 0);
								goto txend;
								}
							idn = (samplex - 1) * BYTESCV;
							fseek(fdqcp, idn, SEEK_SET);
							fread(scv, BYTESCV, 1, fdqcp);
							fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//							ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, scv, strcspn(scv,"\n")+1);
                    		usleep(STIME);
                    		rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                        	}
						break;
					case 6:
                    	stringmove(insdata,4,samplexx,0,2);
                    	if(!(strcmp(samplexx, "T0")))       samplex=2;
                    	else if(!(strcmp(samplexx, "T1")))  samplex=3;
                    	else if(!(strcmp(samplexx, "T2")))  samplex=4;
                    	else if(!(strcmp(samplexx, "T3")))  samplex=5;
                    	else if(!(strcmp(samplexx, "TW")))  samplex=6;
                    	else if(!(strcmp(samplexx, "TD")))  samplex=8;
                    	else if(!(strcmp(samplexx, "WD")))  samplex=12;
                    	else if(!(strcmp(samplexx, "WS")))  samplex=13;
                        else if(!(strcmp(samplexx, "TG")))  samplex=18;
                        else if(!(strcmp(samplexx, "IR")))  samplex=19;
                        else if(!(strcmp(samplexx, "LE")))  samplex=29;
                        else if(!(strcmp(samplexx, "VI")))  samplex=30;
                        else if(!(strcmp(samplexx, "CH")))  samplex=31;
                        else if(!(strcmp(samplexx, "WW")))  samplex=34;
                        else if(!(strcmp(samplexx, "SD")))  samplex=35;
                        else if(!(strcmp(samplexx, "FR")))  samplex=36;
                        else if(!(strcmp(samplexx, "WI")))  samplex=37;
                        else if(!(strcmp(samplexx, "GR")))  samplex=40;
                        else if(!(strcmp(samplexx, "NR")))  samplex=41;
                        else if(!(strcmp(samplexx, "DR")))  samplex=42;
                        else if(!(strcmp(samplexx, "SR")))  samplex=43;
                        else if(!(strcmp(samplexx, "RR")))  samplex=44;
                        else if(!(strcmp(samplexx, "UR")))  samplex=45;
                        else if(!(strcmp(samplexx, "AR")))  samplex=48;
                        else if(!(strcmp(samplexx, "TR")))  samplex=50;
                        else if(!(strcmp(samplexx, "PR")))  samplex=52;
                        else if(!(strcmp(samplexx, "WT")))  samplex=62;
                        else if(!(strcmp(samplexx, "BA")))  samplex=63;
                        else if(!(strcmp(samplexx, "OT")))  samplex=64;
                        else if(!(strcmp(samplexx, "OS")))  samplex=65;
                        else if(!(strcmp(samplexx, "OC")))  samplex=66;
                        else if(!(strcmp(samplexx, "OH")))  samplex=67;
                        else if(!(strcmp(samplexx, "OP")))  samplex=68;
                        else if(!(strcmp(samplexx, "OV")))  samplex=69;
                        else if(!(strcmp(samplexx, "OD")))  samplex=70;
                        else if(!(strcmp(samplexx, "TL")))  samplex=71;
                        else                                samplex=0;
//						printf("samplex=%d\n",samplex);
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/scv.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTESCV;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(scv, BYTESCV, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, scv, strcspn(scv,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 7:
	                    stringmove(insdata,4,samplexxx,0,3);
	                    if(strstr(samplexxx,"SV1"))      samplex=9;
    	                else if(strstr(samplexxx,"SV2")) samplex=10;
        	            else if(strstr(samplexxx,"SV3")) samplex=11;
            	        else if(strstr(samplexxx,"WS1")) samplex=14;
                	    else if(strstr(samplexxx,"RAT")) samplex=15;
                        else if(strstr(samplexxx,"RAW")) samplex=17;
                        else if(strstr(samplexxx,"ST0")) samplex=20;
                        else if(strstr(samplexxx,"ST1")) samplex=21;
                        else if(strstr(samplexxx,"ST2")) samplex=22;
                        else if(strstr(samplexxx,"ST3")) samplex=23;
                        else if(strstr(samplexxx,"ST4")) samplex=24;
                        else if(strstr(samplexxx,"ST5")) samplex=25;
                        else if(strstr(samplexxx,"ST6")) samplex=26;
                        else if(strstr(samplexxx,"ST7")) samplex=27;
                        else if(strstr(samplexxx,"ST8")) samplex=28;
                        else if(strstr(samplexxx,"TCA")) samplex=32;
                        else if(strstr(samplexxx,"LCA")) samplex=33;
                        else if(strstr(samplexxx,"FSD")) samplex=38;
                        else if(strstr(samplexxx,"LNF")) samplex=39;
                        else if(strstr(samplexxx,"UVA")) samplex=46;
                        else if(strstr(samplexxx,"UVB")) samplex=47;
                        else if(strstr(samplexxx,"ART")) samplex=49;
                        else if(strstr(samplexxx,"TRT")) samplex=51;
                        else if(strstr(samplexxx,"SSD")) samplex=53;
                        else if(strstr(samplexxx,"SM1")) samplex=54;
                        else if(strstr(samplexxx,"SM2")) samplex=55;
                        else if(strstr(samplexxx,"SM3")) samplex=56;
                        else if(strstr(samplexxx,"SM4")) samplex=57;
                        else if(strstr(samplexxx,"SM5")) samplex=58;
                        else if(strstr(samplexxx,"SM6")) samplex=59;
                        else if(strstr(samplexxx,"SM7")) samplex=60;
                        else if(strstr(samplexxx,"SM8")) samplex=61;
                        else if(strstr(samplexxx,"OTU")) samplex=72;
                        else if(strstr(samplexxx,"OCC")) samplex=73;
                        else                             samplex=0;
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
                            if((fdqcp = fopen("/tmp/paramete/scv.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTESCV;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(scv, BYTESCV, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, scv, strcspn(scv,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
//							returnTorF(portfd, returntf, ret);
                            }
						break;
					case 8:
						if(strstr(insdata,"RAT1"))
							{
							samplex=16;
                            if((fdqcp = fopen("/tmp/paramete/scv.txt","r+")) == NULL)
                                {
                                returnTorF(portfd, returntf, 0);
                                goto txend;
                                }
                            idn = (samplex - 1) * BYTESCV;
                            fseek(fdqcp, idn, SEEK_SET);
                            fread(scv, BYTESCV, 1, fdqcp);
                            fclose(fdqcp);
                            rs485clt(fd_485, R485_STA_CTL1, TX);
//                            ret = write(portfd, qcps, BYTEQCPS);
                            ret = write(portfd, scv, strcspn(scv,"\n")+1);
                            usleep(STIME);
                            rs485clt(fd_485, R485_STA_CTL1, RX);
							}
						break;
					default: returnTorF(portfd, returntf, 0);
					}
				}
			else
				{
				scv1 = strchr(ins, ' ');
				scv2 = strchr(scv1+1, ' ');
                if(strstr(insdata, "RAT1 ") != 0)	  samplex=16;
                else if(strstr(insdata, "SV1 ") != 0) samplex=9;
                else if(strstr(insdata, "SV2 ") != 0) samplex=10;
                else if(strstr(insdata, "SV3 ") != 0) samplex=11;
                else if(strstr(insdata, "WS1 ") != 0) samplex=14;
                else if(strstr(insdata, "RAT ") != 0) samplex=15;
                else if(strstr(insdata, "RAW ") != 0) samplex=17;
                else if(strstr(insdata, "ST0 ") != 0) samplex=20;
                else if(strstr(insdata, "ST1 ") != 0) samplex=21;
                else if(strstr(insdata, "ST2 ") != 0) samplex=22;
                else if(strstr(insdata, "ST3 ") != 0) samplex=23;
                else if(strstr(insdata, "ST4 ") != 0) samplex=24;
                else if(strstr(insdata, "ST5 ") != 0) samplex=25;
                else if(strstr(insdata, "ST6 ") != 0) samplex=26;
                else if(strstr(insdata, "ST7 ") != 0) samplex=27;
                else if(strstr(insdata, "ST8 ") != 0) samplex=28;
                else if(strstr(insdata, "TCA ") != 0) samplex=32;
                else if(strstr(insdata, "LCA ") != 0) samplex=33;
                else if(strstr(insdata, "FSD ") != 0) samplex=38;
                else if(strstr(insdata, "LNF ") != 0) samplex=39;
                else if(strstr(insdata, "UVA ") != 0) samplex=46;
                else if(strstr(insdata, "UVB ") != 0) samplex=47;
                else if(strstr(insdata, "ART ") != 0) samplex=49;
                else if(strstr(insdata, "TRT ") != 0) samplex=51;
                else if(strstr(insdata, "SSD ") != 0) samplex=53;
                else if(strstr(insdata, "SM1 ") != 0) samplex=54;
                else if(strstr(insdata, "SM2 ") != 0) samplex=55;
                else if(strstr(insdata, "SM3 ") != 0) samplex=56;
                else if(strstr(insdata, "SM4 ") != 0) samplex=57;
                else if(strstr(insdata, "SM5 ") != 0) samplex=58;
                else if(strstr(insdata, "SM6 ") != 0) samplex=59;
                else if(strstr(insdata, "SM7 ") != 0) samplex=60;
                else if(strstr(insdata, "SM8 ") != 0) samplex=61;
                else if(strstr(insdata, "OTU ") != 0) samplex=72;
                else if(strstr(insdata, "OCC ") != 0) samplex=73;
                else if(strstr(insdata, "T0 ") != 0) samplex=2;
                else if(strstr(insdata, "T1 ") != 0) samplex=3;
                else if(strstr(insdata, "T2 ") != 0) samplex=4;
                else if(strstr(insdata, "T3 ") != 0) samplex=5;
                else if(strstr(insdata, "TW ") != 0) samplex=6;
                else if(strstr(insdata, "TD ") != 0) samplex=8;
                else if(strstr(insdata, "WD ") != 0) samplex=12;
                else if(strstr(insdata, "WS ") != 0) samplex=13;
                else if(strstr(insdata, "TG ") != 0) samplex=18;
                else if(strstr(insdata, "IR ") != 0) samplex=19;
                else if(strstr(insdata, "LE ") != 0) samplex=29;
                else if(strstr(insdata, "VI ") != 0) samplex=30;
                else if(strstr(insdata, "CH ") != 0) samplex=31;
                else if(strstr(insdata, "WW ") != 0) samplex=34;
                else if(strstr(insdata, "SD ") != 0) samplex=35;
                else if(strstr(insdata, "FR ") != 0) samplex=36;
                else if(strstr(insdata, "WI ") != 0) samplex=37;
                else if(strstr(insdata, "GR ") != 0) samplex=40;
                else if(strstr(insdata, "NR ") != 0) samplex=41;
                else if(strstr(insdata, "DR ") != 0) samplex=42;
                else if(strstr(insdata, "SR ") != 0) samplex=43;
                else if(strstr(insdata, "RR ") != 0) samplex=44;
                else if(strstr(insdata, "UR ") != 0) samplex=45;
                else if(strstr(insdata, "AR ") != 0) samplex=48;
                else if(strstr(insdata, "TR ") != 0) samplex=50;
                else if(strstr(insdata, "PR ") != 0) samplex=52;
                else if(strstr(insdata, "WT ") != 0) samplex=62;
                else if(strstr(insdata, "BA ") != 0) samplex=63;
                else if(strstr(insdata, "OT ") != 0) samplex=64;
                else if(strstr(insdata, "OS ") != 0) samplex=65;
                else if(strstr(insdata, "OC ") != 0) samplex=66;
                else if(strstr(insdata, "OH ") != 0) samplex=67;
                else if(strstr(insdata, "OP ") != 0) samplex=68;
                else if(strstr(insdata, "OV ") != 0) samplex=69;
                else if(strstr(insdata, "OD ") != 0) samplex=70;
                else if(strstr(insdata, "TL ") != 0) samplex=71;
                else if(strstr(insdata, "P ") != 0)  samplex=1;
                else if(strstr(insdata, "U ") != 0)  samplex=7;
				else								 samplex=0;

//				printf("samplex=%d\n",samplex);
                if(samplex == 0)
	                returnTorF(portfd, returntf, 0);
                else
                    {
                    if((fdqcp = fopen("/tmp/paramete/scv.txt","r+")) == NULL)
                		{
                		if((fdqcp = fopen("/tmp/paramete/scv.txt","w+")) == NULL)
                    		{
//                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
						}
                    idn = (samplex - 1) * BYTESCV;
//					printf("idn=%d, qcps2=%s\n",idn,qcps2);
//					fseek(fdqcp, idn, SEEK_SET);
//					memset(qcps, ' ', BYTEQCPS);
//					fwrite(qcps, BYTEQCPS, 1, fdqcp);
                    fseek(fdqcp, idn, SEEK_SET);
//                    ret = fwrite((qcps2+1), BYTEQCPS, 1, fdqcp);
                    ret = fwrite((scv2+1), strlen(scv2)-1, 1, fdqcp);
                    fclose(fdqcp);
                    returnTorF(portfd, returntf, ret);
                    }
				}
			}
			}

        else if(strstr(insdata, "GALE") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpgale, sizeof(helpgale), SDELAY);
            else
                {
				memset(&alarm4, ' ', sizeof(alarm4));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/gale.txt", O_RDONLY)) < 1)
                        {
						returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
            		usleep(STIME);
            		rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/gale.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "TMAX") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helptmax, sizeof(helptmax), SDELAY);
            else
                {
				memset(alarm4, ' ', sizeof(alarm4));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/tmax.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
					usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/tmax.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "TMIN") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helptmin, sizeof(helptmin), SDELAY);
            else
                {
				memset(alarm5, ' ', sizeof(alarm5));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/tmin.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm5, sizeof(alarm5));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm5, strcspn(alarm5, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 8:
                    if((fdstbd = open("/tmp/paramete/tmin.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm5, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm5, sizeof(alarm5));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "RMAX") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helprmax, sizeof(helprmax), SDELAY);
            else
                {
				memset(alarm4, ' ', sizeof(alarm4));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/rmax.txt", O_RDONLY)) < 1)
                        {
						returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/rmax.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "DTLT") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdtlt, sizeof(helpdtlt), SDELAY);
            else
                {
				memset(alarm4, ' ', sizeof(alarm4));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/dtlt.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/dtlt.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "DTLV") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdtlv, sizeof(helpdtlv), SDELAY);
            else
                {
				memset(alarm4, ' ', sizeof(alarm4));
            switch(ii)
                {
                case 4:
                    if((fdstbd = open("/tmp/paramete/dtlv.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/dtlv.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }


		else if(strstr(insdata, "DZQIWEN") != 0)
			{
			switch(ii)
                {
                case 7:
                    if((fdstbd = open("/tmp/paramete/dztn.txt", O_RDONLY)) < 1)
                        {
						returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, dztn, sizeof(dztn));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, dztn, sizeof(dztn));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 20:
                    if((fdstbd = open("/tmp/paramete/dztn.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 8, dztn, 0, sizeof(dztn));
                    ret = write(fdstbd, dztn, sizeof(dztn));
                    close(fdstbd);
					returnTorF(portfd, returntf, ret);
                    break;
                default:  usleep(1);
                }
            }

        else if(strstr(insdata, "DZTUWEN") != 0)
            {
            switch(ii)
                {
                case 7:
                    if((fdstbd = open("/tmp/paramete/dzstn.txt", O_RDONLY)) < 1)
                        {
						returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, dzstn, sizeof(dzstn));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, dzstn, sizeof(dzstn));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 38:
                    if((fdstbd = open("/tmp/paramete/dzstn.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 8, dzstn, 0, sizeof(dzstn));
                    ret = write(fdstbd, dzstn, sizeof(dzstn));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
                default:  
                    returnTorF(portfd, returntf, 0);
                }
            }

        else if(strstr(insdata, "DZYULIANG") != 0)
            {
            switch(ii)
                {
                case 9:
                    if((fdstbd = open("/tmp/paramete/dzran.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, dzran, sizeof(dzran));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, dzran, sizeof(dzran));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 19:
                    if((fdstbd = open("/tmp/paramete/dzran.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 10, dzran, 0, sizeof(dzran));
                    ret = write(fdstbd, dzran, sizeof(dzran));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
                default:
                    returnTorF(portfd, returntf, 0);
                }
            }

        else if(strstr(insdata, "DZFUSHE") != 0)
            {
            switch(ii)
                {
                case 7:
                    if((fdstbd = open("/tmp/paramete/dzsensi.txt", O_RDONLY)) < 1)
                        {
//                        write(portfd, &openerr, sizeof(openerr));
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, dzsensi, sizeof(dzsensi));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, dzsensi, sizeof(dzsensi));
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 14:
                    if((fdstbd = open("/tmp/paramete/dzsensi.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 8, dzsensi, 0, sizeof(dzsensi));
                    ret = write(fdstbd, dzsensi, sizeof(dzsensi));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
                default:
                    returnTorF(portfd, returntf, 0);
                }
            }

        else if(strstr(insdata, "IP") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpip, sizeof(helpip), MDELAY);
            else
                {
				memset(stip, ' ', sizeof(stip));		
            switch(ii)
                {
                case 2:
                    if((fdstbd = open("/tmp/paramete/ip.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, stip, sizeof(stip));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stip, strcspn(stip, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    break;
                case 10:
                case 11:
                case 12:
                case 13:
                case 14:
                case 15:
                case 16:
                case 17:
                case 18:
                    if((fdstbd = open("/tmp/paramete/ip.txt", O_RDWR|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
					stringmove(ins, 3, stip, 0, strcspn(ins, "\n")-2);
//                    stringmove(insdata, 3, stip, 0, ii-3);
//                    for(nn=18-ii; nn>0; nn--,ii++)
//                        stalt[ii-3] = ' ';
//                    stip[15] = '\r';
//                    stip[16] = '\n';
                    ret = write(fdstbd, stip, sizeof(stip));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
//                default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
                }
            }

        else if(strstr(insdata, "HELP") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helphelp, sizeof(helphelp), SDELAY);
            else
                {
            	rs485clt(fd_485, R485_STA_CTL1, TX);
            	write(portfd, help, sizeof(help));
            	usleep(LTIME);
            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
            }

        else if(strstr(insdata, "SENCO") != 0)
            {
			memset(senco, ' ', sizeof(senco));
			if(strstr(insdata, "/?") != 0)
				returncommand(portfd, helpsenco, sizeof(helpsenco), SDELAY);
			else if(strstr(insdata, "WS1") != 0)
				{
				if(ii == 9)
					{
					if((fdstbd = open("/tmp/paramete/sencows1.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
//					for(senconum=0;senco[senconum]!='\r' && senconum<sizeof(senco);senconum++)
//						{
//						usleep(1);
//						}
                    rs485clt(fd_485, R485_STA_CTL1, TX);
//                    write(portfd, senco, senconum+2);
					write(portfd, senco, strcspn(senco, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					}
				else if(ii>17 && ii<30)
					{
                    if((fdstbd = open("/tmp/paramete/sencows1.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 10, senco, 0, sizeof(senco));
                    ret = write(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
					}
				else
					returnTorF(portfd, returntf, 0);
                }
			else if(strstr(insdata, "WS") != 0)
                {
                if(ii == 8)
                    {
                    if((fdstbd = open("/tmp/paramete/sencows.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, senco, strcspn(senco, "\n")+1);
					usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					}
                else if(ii>16 && ii<30)
                    {
                    if((fdstbd = open("/tmp/paramete/sencows.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 9, senco, 0, sizeof(senco));
                    ret = write(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    }
                else
                    returnTorF(portfd, returntf, 0);
                }
            else if(strstr(insdata, "RAT1") != 0)
                {
                if(ii == 10)
                    {
                    if((fdstbd = open("/tmp/paramete/sencorat1.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, senco, strcspn(senco, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					}
                else if(ii>18 && ii<30)
                    {
                    if((fdstbd = open("/tmp/paramete/sencorat1.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 11, senco, 0, sizeof(senco));
                    ret = write(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    }
                else
                    returnTorF(portfd, returntf, 0);
                }
           else if(strstr(insdata, "RAT") != 0)
                {
                if(ii == 9)
                    {
                    if((fdstbd = open("/tmp/paramete/sencorat.txt", O_RDONLY)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    read(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, senco, strcspn(senco, "\n")+1);
                    usleep(STIME);
                    rs485clt(fd_485, R485_STA_CTL1, RX);
					}
                else if(ii>17 && ii<30)
                    {
                    if((fdstbd = open("/tmp/paramete/sencorat.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        write(portfd, &openerr, sizeof(openerr));
                        goto txend;
                        }
                    stringmove(ins, 10, senco, 0, sizeof(senco));
                    ret = write(fdstbd, senco, sizeof(senco));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    }
                else
                    returnTorF(portfd, returntf, 0);
                }
			else
				returnTorF(portfd, returntf, 0);
            }

        else if(strstr(insdata, "DAUSET") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpdauset, sizeof(helpdauset), SDELAY);
			else
				{
				switch(ii)
					{
					case 11:
						if(strstr(insdata, "CLIM") != 0)		samplex=1;
						else if(strstr(insdata, "RADI") != 0)	samplex=2;
						else if(strstr(insdata, "EATH") != 0)   samplex=3;
						else if(strstr(insdata, "SOIL") != 0)   samplex=4;
                        else if(strstr(insdata, "SEAA") != 0)   samplex=5;
						else									samplex=0;
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
                            {
		                    if((fdset = open("/tmp/paramete/dauset.txt",O_RDONLY)) < 1)
                        		{
                        		returnTorF(portfd, returntf, 0);
                        		goto txend;
                        		}
                    		read(fdset, dauset, sizeof(dauset));
                    		close(fdset);
                        	sst[0] = dauset[samplex-1];
                        	rs485clt(fd_485, R485_STA_CTL1, TX);
                        	write(portfd, sst, sizeof(sst));
                        	usleep(STIME);
                        	rs485clt(fd_485, R485_STA_CTL1, RX);
                        	}
						break;
					case 13:
                        if(strstr(insdata, "CLIM") != 0)        samplex=1;
                        else if(strstr(insdata, "RADI") != 0)   samplex=2;
                        else if(strstr(insdata, "EATH") != 0)   samplex=3;
                        else if(strstr(insdata, "SOIL") != 0)   samplex=4;
                        else if(strstr(insdata, "SEAA") != 0)   samplex=5;
                        else                                    samplex=0;
                        if(samplex == 0)
                            returnTorF(portfd, returntf, 0);
                        else
							{
							if((fdset = open("/tmp/paramete/dauset.txt",O_RDONLY)) < 1)
                            	{
                            	returnTorF(portfd, returntf, 0);
                            	goto txend;
                            	}
                        	read(fdset, dauset, sizeof(dauset));
                        	close(fdset);
                        	dauset[samplex-1] = insdata[12];
                        	if((fdset = open("/tmp/paramete/dauset.txt",O_WRONLY|O_CREAT)) < 1)
                            	{
                            	returnTorF(portfd, returntf, 0);
                            	goto txend;
                            	}
                        	ret = write(fdmaste, dauset, sizeof(dauset));
                        	close(fdset);
                        	returnTorF(portfd, returntf, ret);
							}
						break;
					default:
						returnTorF(portfd, returntf, 0);
					}
				}
			}

        else if(strstr(insdata, "CFSET") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpcfset, sizeof(helpcfset), SDELAY);
            else
                {
                switch(ii)
                    {
                    case 5:
                        if((fdset = open("/tmp/paramete/cfset.txt", O_RDONLY)) < 1)
                            {
                            returnTorF(portfd, returntf, 0);
                            goto txend;
                            }
                        read(fdset, sst, sizeof(sst));
                        close(fdset);
                        rs485clt(fd_485, R485_STA_CTL1, TX);
                        write(portfd, sst, sizeof(sst));
                        usleep(STIME);
                        rs485clt(fd_485, R485_STA_CTL1, RX);
                        break;
                    case 7:
                    	if((fdset = open("/tmp/paramete/cfset.txt", O_WRONLY|O_CREAT)) < 1)
                        	{
                        	returnTorF(portfd, returntf, 0);
                        	goto txend;
                        	}
                        sst[0] = insdata[6];
                        ret = write(fdset, sst, sizeof(sst));
                        close(fdstbd);
                        returnTorF(portfd, returntf, ret);
                        break;
                    default:
                        returnTorF(portfd, returntf, 0);
                    }
                }
            }

		else if(strstr(insdata, "RESTART") != 0)
			{
            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, restart2, sizeof(restart2));
            usleep(MTIME);
            rs485clt(fd_485, R485_STA_CTL1, RX);
			printf("Now restart Linux! Please wait ...\n");
            close_port(portfd);
            sleep(5);
			rs485clt(fd_485, WDT_STA_CTL, 1);
			}

		else
			returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
		}	
txend:	
		ii = 0;
//	        memset(ins, '\0' ,sizeof(ins));
//	        memset(insdata, '\0' ,sizeof(insdata));
		fflush(NULL);
	}
	close(fd_485);
	close(portfd);
	return 0;
}
