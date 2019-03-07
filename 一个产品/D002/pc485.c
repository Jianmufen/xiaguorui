//pc485 2014-08-21 14:00 V6.9
//source from "pc485.c 2013-09-10 16:00 V6.8"

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

#define P       0       
#define T0      1
#define TD      2
#define U       3
#define WD      4
#define WS      5
#define RAT     6
#define RAW     7
#define VI      8

#define RECNUMHOUR      744
#define RECNUMMIN       1440
#define BYTENUMMZ       255

#define BYTEDZTN    	14

#define SAMPLE120       126
#define SAMPLE240       246
#define SAMPLE960       966
#define SAMPLEX30  		183
#define SAMPLEXX6		64
#define SAMPLEXX30		184
#define SAMPLEXX60      334
#define SAMPLEXX240     1234
#define SAMPLEXXX30     185
#define BYTEQCPS		20
#define BYTEQCPM        30
#define SDELAY		1
#define MDELAY		2
#define LDELAY		3

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

static long sample_bianhao(char *string, int n)
{
        int temp1, temp2, id;
        temp1 = (string[n-5] - '0') * 10 + (string[n-4] - '0');
        temp2 = (string[n-2] - '0') * 10 + (string[n-1] - '0');
//        id = (temp1 * 60 + temp2 + 240) % 1440;
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
//    rs485clt(fd_485, R485_STA_CTL1, TX);
	write(portfd, returntf, 3);
    usleep(STIME);
//    rs485clt(fd_485, R485_STA_CTL1, RX);
	}

static void returncommand(int portfd, char *command, int size, int delay)
    {
//    rs485clt(fd_485, R485_STA_CTL1, TX);
    write(portfd, command, size);
	if(delay == SDELAY)
    	usleep(STIME);
	else if(delay == MDELAY)
		usleep(MTIME);
	else
		usleep(LTIME);
//    rs485clt(fd_485, R485_STA_CTL1, RX);
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
	char senstate[11]={"111111111\r\n"};
	char pss[9];
	char insdmgd[4]={"FZSJ"};
	char stat[66]={"STAT J0001 2017-10-01 00:00 137 -108 1234 2444 0 0 0 0 0 0 0 0 1\r\n"};
	char df[300], door[3];
	int ret;
    signed short int masttemp;
	char stat_sensor[6] = {"000000"};

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
//		printf("sec=%d\n", sec);

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

            for(x=0;x<42;x++)                           // Data Index
	            {
                data[28+x] = '1';
                }
			memset(data+70, '0', 10);
            data[80] = ' ';

            if((fbcs = fopen("/usr/fzsj.tmp","r")) == NULL)
	            goto autoend;
            fread(&csdata, BYTENUMMZ, 1, fbcs);
            fclose(fbcs);

            stringmove(csdata,0,sstime,0,4);
            sstime[4]='\0';
//			printf("sstime=%s,  wjtime=%s,  sametime=%d\n",sstime,wjtime,sametime);
            if((strcmp(sstime, wjtime)) == 0)
	            {
                sametime++;
                printf("sametime=%d\n",sametime);
                if(sametime > 5)
	                {
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

            stringmove(csdata, 211, data, 81, 42);       // Quality Control
			data[123] = ' ';

            stringmove(csdata, 4, data, 124, 206);
			data[330] = '\r';
            data[331] = '\n';
            data[332] = '\0';

/*
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
            if(senstate[VI]=='1')
                {
                memset(data+58, '1', 2);
                }
            else
                {
                memset(data+58, '0', 2);
                memset(data+104, '~', 2);
                memset(data+271, ' ', 12);
                }
            memset(data+69, '0', 4);
            memset(data+115, '~', 4);
            memset(data+337, ' ', 20);
            data[BYTEDMGD-2] = '\r';
            data[BYTEDMGD-1] = '\n';
            data[BYTEDMGD] = '\0';
*/
            recount = string_change_2(data, strcspn(data, "\n")+1);
            write(portfd ,&data, strcspn(data, "\n")+1);
            usleep(GTIME);

            stringmove(data, 5, stat, 5, 5);
			stringmove(data, 11, stat, 11, 16);
            if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, pss, sizeof(pss));
                close(fdmaste);
                stringmove(pss, 3, stat, 28, 2);
                stat[30] = pss[6];
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
                    stat[32] = ' ';
                    stat[33] = masttemp / 100 + '0';
                    stat[34] = masttemp % 100 / 10 + '0';
                    stat[35] = masttemp % 10 + '0';
                    }
                else
                    {
                    stat[32] = '-';
                    stat[33] = masttemp * (-1) / 100 + '0';
                    stat[34] = masttemp * (-1) % 100 / 10 + '0';
                    stat[35] = masttemp * (-1) % 10 + '0';
                    }
                }
            system("df>/usr/df.tmp");
            if((fdmaste = open("/usr/df.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, df, sizeof(df));
                close(fdmaste);
                stringmove(df, 112, stat, 37, 4);
                stringmove(df, 168, stat, 42, 4);
                }
            if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, door, sizeof(door));
                close(fdmaste);
                stat[63] = door[0];
                }
            if((fdmaste = open("/usr/sensorstat.tmp",O_RDONLY)) < 1)
                usleep(1);
            else
                {
                read(fdmaste, stat_sensor, sizeof(stat_sensor));
                close(fdmaste);
                stat[51] = stat_sensor[0];
                stat[53] = stat_sensor[1];
                stat[55] = stat_sensor[2];
                stat[57] = stat_sensor[3];
                stat[59] = stat_sensor[4];
                stat[61] = stat_sensor[5];
                }
            write(portfd, stat, sizeof(stat));

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

	char datatime1[12], datatime2[12], sourcetime[12];
    long id, idn, count1, timedp1, timedp2;

 	struct rtc_time rtc, rtc_tm;
	int fdtime, fdstbd, fdmaste, fdset,ret;
	char date[12], timed[10], returntf[3]={"F\r\n"};
	char baseinfo[65]={"<BASEINFO 4>\r<mC CWQX>\r<MODEL DZZ3>\r<ID 041201010001>\r<Ver 1.0>\r\n"};
	char stidd[5], stlat[10], stlong[11], sttd[6], stalt[8], staltp[8], stip[17];
    char senstate[11]={"111111111\r\n"};
	char dauset[8]={"111001r\n"};
	char sst[3]={"1\r\n"};
	char mact[7], pss[9], door[3], rsta[91];
	char alarm4[4], alarm5[5];
	signed short int masttemp;

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
	long dmtime=0, dmraw=0, dmle=0;
//	int devmodetime=0;
	char devmoderaw[6]={"2 30\r\n"};
	char devmodele[6]={"2 30\r\n"};
	char zero[3]={"0\r\n"};

	FILE *fdqcp;
	char qcps[BYTEQCPS], qcpm[BYTEQCPM];
	char *qcps1 = {"                     "};				// size=21
	char *qcps2 = {"                     "};
	char *qcpm1 = {"                               "};		// size=31
	char *qcpm2 = {"                               "};
	char *scv1 = {"                                                                                                    "};
	char *scv2 = {"                                                                                                    "};
															// size=101
	char *string1 = {"                "};					// size=16;
    char *string2 = {"                "};                   // size=16;

	char dztn[BYTEDZTN];
	char autocheck[218]={"<AUTOCHECK 21>\r<DATE 2012-08-01>\r<TIME 10:28:58>\r<GPS FAIL>\r<COM 115200 8 N 1>\r<MACT +20.0>\r<DC 13.6>\r<TARH 1>\r<CLIM 0>\r<RADI 1>\r<EATH 1>\r<SOIL 0>\r<WD 1>\r<WS 1>\r<T0 1>\r<U 1>\r<RAT 1>\r<P 1>\r<VI 1>\r<LE 1>\r<GR 1>\r<RAW 1>\r\n"};
	char statsensor[11]={"111111111\r\n"};
	char senstonoff[11]={"111111111\r\n"};
	char stat[66]={"STAT J0001 2017-10-01 00:00 137 -108 1234 2444 0 0 0 0 0 0 0 0 1\r\n"};
	int onoff;

	char welcome[28] = {"Welcome To SHCWQX New-AWS!\r\n"};

	char help[]={"SETCOM,IP,BASEINFO,AUTOCHECK,DATE,TIME,ID,LAT,LONG,TD,ALTP,ALT,SENSI,SMC,DOOR,MACT,PSS,SENST,RSTA,SENCO,DEVMODE,DAUSET,GPSSET,CFSET,STATMAIN,STATCLIM,STATRADI,STATEATH,STATSOIL,STATSEAA,STATTARH,STATINTL,STATSENSOR,STAT,QCPS,QCPM,SCV,DMGD,DMCD,DMRD,DMSD,DMOD,DHGD,DHCD,DHRD,DHSD,DHOD,SAMPLE,GALE,TMAX,TMIN,RMAX,DTLT,DTLV\r\n"};

//	char badcommand[]={"BAD COMMAND\r\n"};
	char badcommand[]={"F\r\n"};
	char helpdmgd[]={"FZSJ\nFZSJ YYYY-MM-DD HH:MM YYYY-MM-DD HH:MM\nFZSJ YYYY-MM-DD HH:MM n\r\n"};
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
	char helpdoor[]={"DOOR\r\n"};
	char helpmact[]={"MACT\r\n"};
	char helppss[]={"PSS\r\n"};
	char helpsenst[]={"SENST\nSENST *\nSENST X/XX/XXX\nSENST X/XX/XXX Y(Y=0 or Y=1)\r\n"};
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
    char helpstat[]={"STAT\r\n"};

	int hyyear, hymonth, hyday, hyhour, hymin, hysec;
	int sourceyear, sourcemon, sourceday, sourcehour, sourcemin, sourcesec;
	int insyear, insmon, insday, inshour, insmin, inssec;

	pthread_t th_autosend;
	char autosend[2];
	char stautosend[]={"AUTOSEND 00\r\n"};

	char stat_sensor[6] = {"000000"};

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
//	portfd = open_port(SERIAL0, baud);              // GPS	

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

	printf("pc485 2014-08-21 14:00 V6.9\n");

//    rs485clt(fd_485, R485_STA_CTL1, TX);
    write(portfd ,welcome, sizeof(welcome));
    usleep(STIME);
//    rs485clt(fd_485, R485_STA_CTL1, RX);

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
		FD_ZERO(&fds);
		FD_SET(portfd, &fds);
        stimeout.tv_sec = 59;
        stimeout.tv_usec = 0;
        retval = select(portfd + 1, &fds, NULL, NULL, &stimeout);
//		retval = select(portfd + 1, &fds, NULL, NULL, NULL);
//		printf("select=%d\n",retval);
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
//              break;
		    	goto txend;
		    	}
            if(time>2000)
		    	{
//              break;
		    	goto txend;
		    	}
            }while(c!='\n');
			ins[ii] = '\0';
//        	printf("%s  %d\n",ins,ii);			
	    	ii=ii-2;
	    	stringmove(ins,0,insdata,0,ii);
	    	insdata[ii]='\0';
//	    	printf("%s	%d\n",insdata,ii);

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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stautosend, sizeof(stautosend));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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

        else if(strstr(insdata, "FZSJ") != 0)
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

                    for(x=0;x<42;x++)							// Data Index
                        {
                        data[28+x] = '1';
                        }
            		memset(data+70, '0', 10);
            		data[80] = ' ';

            		if((fbcs = fopen("/usr/fzsj.tmp","r")) == NULL)
                		goto txend;
            		fread(&csdata, BYTENUMMZ, 1, fbcs);
            		fclose(fbcs);

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
//		                    rs485clt(fd_485, R485_STA_CTL1, TX);
		                    write(portfd, restart1, sizeof(restart1));
		                    usleep(MTIME);
//		                    rs485clt(fd_485, R485_STA_CTL1, RX);
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

            		stringmove(csdata, 211, data, 81, 42);       // Quality Control
            		data[123] = ' ';

            		stringmove(csdata, 4, data, 124, 206);
            		data[330] = '\r';
            		data[331] = '\n';
            		data[332] = '\0';

/*
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
                    if(senstate[VI]=='1')
                        {
                        memset(data+58, '1', 2);
                        }
                    else
                        {
                        memset(data+58, '0', 2);
                        memset(data+104, '~', 2);
                        memset(data+271, ' ', 12);
                        }
                    memset(data+69, '0', 4);
                    memset(data+115, '~', 4);
                    memset(data+337, ' ', 20);
					data[BYTEDMGD-2] = '\r';
					data[BYTEDMGD-1] = '\n';
					data[BYTEDMGD] = '\0';
*/
                    recount = string_change_2(data, strcspn(data, "\n")+1);
                    write(portfd ,&data, strcspn(data, "\n")+1);
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
                    for(x=0;x<42;x++)                           // Data Index
                        {
                        data[28+x] = '1';
                        }
                    memset(data+70, '0', 10);
                    data[80] = ' ';

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

                    	stringmove(csdata, 211, data, 81, 42);       // Quality Control
                    	data[123] = ' ';

                        stringmove(sourcetime,0,data,13,2);         // year
                        stringmove(sourcetime,2,data,16,2);         // month
                        stringmove(sourcetime,4,data,19,2);         // day
                        stringmove(sourcetime,6,data,22,2);         // hour
                        stringmove(sourcetime,8,data,25,2);         // minute

                    	stringmove(csdata, 4, data, 124, 206);
                    	data[330] = '\r';
                    	data[331] = '\n';
                    	data[332] = '\0';

        	            recount = string_change_2(data, strcspn(data, "\n")+1);
						write(portfd ,&data, strcspn(data, "\n")+1);
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
                    for(x=0;x<42;x++)                           // Data Index
                        {
                        data[28+x] = '1';
                        }
					memset(data+70, '0', 10);
                    data[80] = ' ';

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

                        stringmove(csdata, 211, data, 81, 42);      // Quality Control
                        data[123] = ' ';

                        stringmove(sourcetime,0,data,13,2);			// year
                        stringmove(sourcetime,2,data,16,2);			// month
                        stringmove(sourcetime,4,data,19,2);			// day
                        stringmove(sourcetime,6,data,22,2);			// hour
                        stringmove(sourcetime,8,data,25,2);			// minute

                        stringmove(csdata, 4, data, 124, 206);
                        data[330] = '\r';
                        data[331] = '\n';
                        data[332] = '\0';
                        
						recount = string_change_2(data, strcspn(data, "\n")+1);
						write(portfd ,&data, strcspn(data, "\n")+1);
                        timeconvert(datatime1,60);
                        timeconvert(sourcetime,60);
                        }
					break;

//		    	default: returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
		    	}
				}
			}	// if(strstr(insdata, "DMGD") != 0)

	    else if(strstr(insdata, "SAMPLE") != 0)
	    	{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpsample, sizeof(helpsample), MDELAY);
            else
                {
				switch(ii)
		    		{
					case 25:
						if(insdata[7] == 'U')	samplex=1;
						else					samplex=0;
			
                		stringmove(insdata, 0, data, 0, 8);                     // Instruction
            			data[6] = '_';
                		data[8] = ' ';
		                stationid(stid);                                        // Station Id
        		        stringmove(stid, 0, data, 9, 5);
                		data[14] = ' ';
		                stringmove(insdata, 9, data, 15, 16);
        		        data[31] = ' ';
//						printf("samplex=%d\n",samplex);
						switch(samplex)
							{		
                    		case 1:
                    			if((fbsamp = fopen("/tmp/samp/UUYYMMDD.DAT","r")) == NULL)
                        			break;
                        		id = sample_bianhao(insdata, 25);
		                        idn = (id - 1) * SAMPLE120;
        			            fseek(fbsamp, idn, SEEK_SET);
		                        fread(&csdata, SAMPLE120, 1, fbsamp);
								csdata[SAMPLE120]='\0';
//								printf("csdata=%s\n",csdata);
                        		fclose(fbsamp);
		                        string_add_space(csdata, 4, data, 32, 4, 30);
        		                data[SAMPLEX30-2] = '\r';
                		        data[SAMPLEX30-1] = '\n';
                        		data[SAMPLEX30] = '\0';
		                        recount = string_change_2(data, SAMPLEX30);
                        		write(portfd, &data, SAMPLEX30-recount);
                        		break;
							default: returnTorF(portfd, returntf, 0);
							}
						break;

		    		case 26:
						stringmove(insdata,7,samplexx,0,2);
//						printf("samplexx=%c%c\n",samplexx[0],samplexx[1]);
			            if((strcmp(samplexx, "T0"))==0)   samplex=1;
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

//						printf("samplex=%d\n",samplex);
						switch(samplex)
			    			{
							case 0:
								usleep(1);
								break;
	
						    case 1:
                				if((fbsamp = fopen("/tmp/samp/T0YYMMDD.DAT","r")) == NULL)
									break;
								id = sample_bianhao(insdata, 26);
            			        idn = (id - 1) * SAMPLE120;
//								printf("id=%d,idn=%ld\n",id,idn);
								fseek(fbsamp, idn, SEEK_SET);
								fread(&csdata, SAMPLE120, 1, fbsamp);
			                    fclose(fbsamp);
								string_add_space(csdata, 4, data, 33, 4, 30);
								data[SAMPLEXX30-2] = '\r';
            			        data[SAMPLEXX30-1] = '\n';
			                    data[SAMPLEXX30] = '\0';
            			        recount = string_change_2(data, SAMPLEXX30);
			                    write(portfd, &data, SAMPLEXX30-recount);
            			        usleep(MTIME);
								break;

                            case 6:
                                if((fbsamp = fopen("/tmp/samp/WDYYMMDD.DAT","r")) == NULL)
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
                                write(portfd, &data, SAMPLEXX60-recount);
                                break;

                            case 7:
                                if((fbsamp = fopen("/tmp/samp/WSYYMMDD.DAT","r")) == NULL)
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
                                write(portfd, &data, SAMPLEXX240-recount);
                                break;
			    			default:  returnTorF(portfd, returntf, 0);
			    			}
 						break;
			    	default:  returnTorF(portfd, returntf, 0);
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
//						rs485clt(fd_485, R485_STA_CTL1, TX);
						write(portfd,&date,sizeof(date));
						usleep(STIME);
//						rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					else
						{
						returnTorF(portfd, returntf, 0);
						}
					break;
				
				case 15:
                    insyear = (insdata[5]-'0') * 1000 + (insdata[6]-'0') * 100 + (insdata[7]-'0') * 10 + (insdata[8]-'0');
                    if(insyear < 1900)
						{
						returnTorF(portfd, returntf, 0);
                    	goto txend;
						}
                    if(insyear > 2100)
						{
						returnTorF(portfd, returntf, 0);
                        goto txend;
						}	
                    insmon = (insdata[10] - '0') * 10 + (insdata[11] - '0');
                    if(insmon > 12)
						{
						returnTorF(portfd, returntf, 0);
                        goto txend;
						}
                    insday = (insdata[13] - '0') * 10 + (insdata[14] - '0');
                    if(insday > 31)
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

// Added by XGR at 2014-03-19
					sourceyear = rtc_tm.tm_year;
					sourcemon = rtc_tm.tm_mon;
					sourceday = rtc_tm.tm_mday;
					sourcehour = rtc_tm.tm_hour;
					sourcemin = rtc_tm.tm_min;
					sourcesec = rtc_tm.tm_sec;
                    rtc_tm.tm_year = 0;
                    rtc_tm.tm_mon = 0;
                    rtc_tm.tm_mday = 0;
                    rtc_tm.tm_hour = 0;
                    rtc_tm.tm_min = 0;
                    rtc_tm.tm_sec = 0;
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
					if(sourceyear==rtc_tm.tm_year && sourcemon==rtc_tm.tm_mon && sourceday==rtc_tm.tm_mday && sourcehour==rtc_tm.tm_hour && sourcemin==rtc_tm.tm_min)
						{	
                    	rtc_tm.tm_year = insyear - 1900;
                    	rtc_tm.tm_mon = insmon - 1;
                    	rtc_tm.tm_mday = insday;
				    	ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
//						printf("ret=%d\n",ret);
	                    if(ret != 0)
							{
							close(fd_rtc);
    	                    returnTorF(portfd, returntf, 0);
        	                break;	
							}
						else
							{
                    		retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    		if(retval == -1)
                        		{
                        		close(fd_rtc);
                        		returnTorF(portfd, returntf, 0);
                        		break;
                        		}
							if((insyear-1900)==rtc_tm.tm_year && (insmon-1)==rtc_tm.tm_mon && insday==rtc_tm.tm_mday && sourcehour==rtc_tm.tm_hour && sourcemin==rtc_tm.tm_min)
								{
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
//      						printf("instime=%s\n",instime);
	        					system(instime);
								close(fd_rtc);
								returnTorF(portfd, returntf, 1);
								}
							else
								{
								rtc_tm.tm_year = sourceyear;
	    	                	rtc_tm.tm_mon = sourcemon;
    	    	            	rtc_tm.tm_mday = sourceday;
                		    	rtc_tm.tm_hour = sourcehour;
                    			rtc_tm.tm_min = sourcemin;
                    			rtc_tm.tm_sec = sourcesec;
								ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
								returnTorF(portfd, returntf, 0);	
								close(fd_rtc);
								}
							}
						}	
					else
						{
						close(fd_rtc);
						returnTorF(portfd, returntf, 0);
						}
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
//                    	rs485clt(fd_485, R485_STA_CTL1, TX);
						write(portfd,&timed,sizeof(timed));
                    	usleep(STIME);
//                    	rs485clt(fd_485, R485_STA_CTL1, RX);
						}
					else
						{
						returnTorF(portfd, returntf, 0);
						}
					break;

				case 13:
                    inshour = (insdata[5] - '0') * 10 + (insdata[6] - '0');
                    if(inshour > 24)
						{
						returnTorF(portfd, returntf, 0);
                    	break;
						}
                    insmin = (insdata[8] - '0') * 10 + (insdata[9] - '0');
                    if(insmin > 60)
						{
						returnTorF(portfd, returntf, 0);
                    	break;
						}
                    inssec = (insdata[11] - '0') * 10 + (insdata[12] - '0');
                    if(inssec > 60)
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

// Added by XGR at 2014-03-19
                    sourceyear = rtc_tm.tm_year;
                    sourcemon = rtc_tm.tm_mon;
                    sourceday = rtc_tm.tm_mday;
                    sourcehour = rtc_tm.tm_hour;
                    sourcemin = rtc_tm.tm_min;
                    sourcesec = rtc_tm.tm_sec;
                    rtc_tm.tm_year = 0;
                    rtc_tm.tm_mon = 0;
                    rtc_tm.tm_mday = 0;
                    rtc_tm.tm_hour = 0;
                    rtc_tm.tm_min = 0;
                    rtc_tm.tm_sec = 0;
                    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                    if(retval == -1)
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        break;
                        }
                    if(sourceyear==rtc_tm.tm_year && sourcemon==rtc_tm.tm_mon && sourceday==rtc_tm.tm_mday && sourcehour==rtc_tm.tm_hour && sourcemin==rtc_tm.tm_min)
                        {
                        rtc_tm.tm_hour = inshour;
                        rtc_tm.tm_min = insmin;
                        rtc_tm.tm_sec = inssec;
                        ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
//                      printf("ret=%d\n",ret);
                        if(ret != 0)
                            {
                            close(fd_rtc);
                            returnTorF(portfd, returntf, 0);
                            break;
                            }
                        else
                            {
                            retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
                            if(retval == -1)
                                {
                                close(fd_rtc);
                                returnTorF(portfd, returntf, 0);
                                break;
                                }
                            if(sourceyear==rtc_tm.tm_year && sourcemon==rtc_tm.tm_mon && sourceday==rtc_tm.tm_mday && inshour==rtc_tm.tm_hour && insmin==rtc_tm.tm_min)
                                {
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
//                              printf("instime=%s\n",instime);
                                system(instime);
                                close(fd_rtc);
                                returnTorF(portfd, returntf, 1);
                                }
                            else
                                {
                                rtc_tm.tm_year = sourceyear;
                                rtc_tm.tm_mon = sourcemon;
                                rtc_tm.tm_mday = sourceday;
                                rtc_tm.tm_hour = sourcehour;
                                rtc_tm.tm_min = sourcemin;
                                rtc_tm.tm_sec = sourcesec;
                                ret = ioctl(fd_rtc, RTC_SET_TIME, &rtc_tm);
                                returnTorF(portfd, returntf, 0);
                                close(fd_rtc);
                                }
                            }
                        }
                    else
                        {
                        close(fd_rtc);
                        returnTorF(portfd, returntf, 0);
                        }
                    break;

//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

        else if(strstr(insdata, "STAT") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpstat, sizeof(helpstat), SDELAY);
            else
                {
				stationid(stid);                            // Station Id
                stringmove(stid, 0, stat, 5, 5);

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
                        stat[11] = (rtc_tm.tm_year + 1900) / 1000 + '0';
                        stat[12] = (rtc_tm.tm_year + 1900) / 100 % 10 + '0';
                        stat[13] = (rtc_tm.tm_year % 100) / 10 + '0';
                        stat[14] = rtc_tm.tm_year % 10 + '0';
                        stat[16] = (rtc_tm.tm_mon + 1) / 10 + '0';
                        stat[17] = (rtc_tm.tm_mon + 1) % 10 + '0';
                        stat[19] = rtc_tm.tm_mday / 10 + '0';
                        stat[20] = rtc_tm.tm_mday % 10 + '0';
                        stat[22] = rtc_tm.tm_hour / 10 + '0';
                        stat[23] = rtc_tm.tm_hour % 10 + '0';
                        stat[25] = rtc_tm.tm_min / 10 + '0';
                        stat[26] = rtc_tm.tm_min % 10 + '0';
                        }
                    }
                if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, pss, sizeof(pss));
                    close(fdmaste);
                    stringmove(pss, 3, stat, 28, 2);
                    stat[30] = pss[6];
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
                        stat[32] = ' ';
                        stat[33] = masttemp / 100 + '0';
                        stat[34] = masttemp % 100 / 10 + '0';
                        stat[35] = masttemp % 10 + '0';
                        }
                    else
                        {
                        stat[32] = '-';
                        stat[33] = masttemp * (-1) / 100 + '0';
                        stat[34] = masttemp * (-1) % 100 / 10 + '0';
                        stat[35] = masttemp * (-1) % 10 + '0';
                        }
                    }
                system("df>/usr/df.tmp");
                if((fdmaste = open("/usr/df.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, df, sizeof(df));
                    close(fdmaste);
					stringmove(df, 112, stat, 37, 4);	
                    stringmove(df, 168, stat, 42, 4);
                    }
                if((fdmaste = open("/usr/door.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
                    read(fdmaste, door, sizeof(door));
                    close(fdmaste);
                    stat[63] = door[0];
                    }
				if((fdmaste = open("/usr/sensorstat.tmp",O_RDONLY)) < 1)
                    usleep(1);
                else
                    {
					read(fdmaste, stat_sensor, sizeof(stat_sensor));
                    close(fdmaste);
                    stat[51] = stat_sensor[0];
                    stat[53] = stat_sensor[1];
                    stat[55] = stat_sensor[2];
                    stat[57] = stat_sensor[3];
                    stat[59] = stat_sensor[4];
                    stat[61] = stat_sensor[5];
					}
                write(portfd, stat, sizeof(stat));
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&tongxunn,baudc+8);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
					close_port(portfd);
					portfd = open_port(UART0, baud);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
					portfd = open_port(UART0, baud);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
                    portfd = open_port(UART0, baud);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd,&returntf,sizeof(returntf));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
                    close_port(portfd);
                    portfd = open_port(UART0, baud);
                    break;
//				default:  returnTorF(portfd, returntf, 0);
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
				}
			}

		else if(strstr(insdata, "BASEINFO") != 0)
			{
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helpbaseinfo, sizeof(helpbaseinfo), SDELAY);
            else
                {
//				memset(autocheck, ' ', sizeof(autocheck));
//				if((fdstbd = open("/tmp/paramete/baseinfo.txt",O_RDONLY)) < 1)
//					{
//					returnTorF(portfd, returntf, 0);
//					goto txend;
//					}
//        		read(fdstbd, baseinfo, sizeof(baseinfo));
//        		close(fdstbd);
//				baseinfo[19]='\r';
//            	baseinfo[20]='\n';
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
				write(portfd, baseinfo, sizeof(baseinfo));
            	usleep(STIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
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
				stringmove(date, 0, autocheck, 21, 10);
				autocheck[10] = ' ';
				stringmove(timed, 0, autocheck, 39, 8);
        	check1:
                if((fdset = open("/tmp/paramete/gpsset.txt", O_RDONLY)) < 1)		//GPS
                    goto check2;
                read(fdset, sst, sizeof(sst));
                close(fdset);
				if(sst[0]=='1')
					{
					autocheck[54] = ' ';
                    autocheck[55] = ' ';
                    autocheck[56] = 'O';
                    autocheck[57] = 'K';
					}
				else
                    {
                    autocheck[54] = 'F';
                    autocheck[55] = 'A';
                    autocheck[56] = 'I';
                    autocheck[57] = 'L';
                    }
			check2:
	            if((fdtongxun = open("/tmp/paramete/tongxun.txt", O_RDONLY)) < 1)	// Baudrate
    	            goto check3;
        	    read(fdtongxun, tongxun, sizeof(tongxun));
	            close(fdtongxun);
    	        baud = tongxun[0] -'0';
        	    switch(baud)
	                {
    	            case 1:
        	        	baud3[0]=' ';baud3[1]=' ';baud3[2]=' ';baud3[3]='3';baud3[4]='0';baud3[5]='0';baudc=6;
            	        stringmove(baud3,0,tongxunn,0,baudc);
                	    break;
	                case 2:
    	                baud4[0]=' ';baud4[1]=' ';baud4[2]='1';baud4[3]='2';baud4[4]='0';baud4[5]='0';baudc=6;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 3:
    	                baud4[0]=' ';baud4[1]=' ';baud4[2]='2';baud4[3]='4';baud4[4]='0';baud4[5]='0';baudc=6;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 4:
        	            baud4[0]=' ';baud4[1]=' ';baud4[2]='4';baud4[3]='8';baud4[4]='0';baud4[5]='0';baudc=6;
    	                stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 5:
    	                baud4[0]=' ';baud4[1]=' ';baud4[2]='9';baud4[3]='6';baud4[4]='0';baud4[5]='0';baudc=6;
        	            stringmove(baud4,0,tongxunn,0,baudc);
            	        break;
	                case 6:
    	                baud5[0]=' ';baud5[1]='1';baud5[2]='9';baud5[3]='2';baud5[4]='0';baud5[5]='0';baudc=6;
        	            stringmove(baud5,0,tongxunn,0,baudc);
            	        break;
	                case 7:
    	                baud5[0]=' ';baud5[1]='3';baud5[2]='8';baud5[3]='4';baud5[4]='0';baud5[5]='0';baudc=6;
        	            stringmove(baud5,0,tongxunn,0,baudc);
            	        break;
	                case 8:
    	                baud5[0]=' ';baud5[1]='5';baud5[2]='7';baud5[3]='6';baud5[4]='0';baud5[5]='0';baudc=6;
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
				stringmove(tongxunn, 0, autocheck, 65, 12);
        	check3:
    	        if((fdmaste=open("/dev/ds18b20",O_RDWR)) == -1)
        	        {
	                goto check4;
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
					goto check4;
        	    stringmove(mact, 0, autocheck, 85, 5);
	        check4:
        	    if((fdmaste = open("/usr/pss.tmp",O_RDONLY)) < 1)
            	    {
                	goto check5;
	                }
    	        read(fdmaste, pss, sizeof(pss));
        	    close(fdmaste);
				stringmove(pss, 0, autocheck, 93, 7);
	        check5:
                if((fdset = open("/tmp/paramete/dauset.txt",O_RDONLY)) < 1)
                	goto check6;
                read(fdset, dauset, sizeof(dauset));
                close(fdset);
				autocheck[108] = dauset[5];
                autocheck[117] = dauset[0];
                autocheck[126] = dauset[1];
                autocheck[135] = dauset[2];
                autocheck[144] = dauset[3];
			check6:
	            if((fdmaste = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)
        	        goto check7;
	            read(fdmaste, senstate, sizeof(senstate));
    	        close(fdmaste);
				autocheck[151] = senstate[4];
                autocheck[158] = senstate[5];
                autocheck[165] = senstate[1];
                autocheck[171] = senstate[3];
                autocheck[179] = senstate[6];
                autocheck[185] = senstate[8];
                autocheck[192] = senstate[7];
                autocheck[199] = senstate[0];
			check7:			
				write(portfd, autocheck, sizeof(autocheck));
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, stid, strcspn(stid, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stlat, strcspn(stlat, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, &stlong, strcspn(stlong, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, staltp, strcspn(staltp, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, stalt, strcspn(stalt, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
				write(portfd, door, sizeof(door));
            	usleep(STIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
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
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
            	write(portfd, pss, sizeof(pss));
            	usleep(STIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
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
//    	        	rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, mact, sizeof(mact));
            		usleep(STIME);
//            		rs485clt(fd_485, R485_STA_CTL1, RX);
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
					write(portfd, senstate, sizeof(senstate));
					break;
				case 15:
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
					else if(insdata[6] == 'U')	samplex=4;
					else						samplex=0;
					if(samplex == 0)
						returnTorF(portfd, returntf, 0);
					else
						{
						sst[0] = senstate[samplex-1];
						write(portfd, sst, sizeof(sst));
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
					samplexx[2] = '\0';
                    if(!(strcmp(samplexx, "T0")))   	samplex=2;
                    else if(!(strcmp(samplexx, "TD")))  samplex=3;
                    else if(!(strcmp(samplexx, "WD")))  samplex=5;
                    else if(!(strcmp(samplexx, "WS")))  samplex=6;
                    else if(!(strcmp(samplexx, "VI")))  samplex=9;
					else								samplex=0;	
                    if(samplex == 0)
                        returnTorF(portfd, returntf, 0);
                    else
                        {
                        sst[0] = senstate[samplex-1];
                        write(portfd, sst, sizeof(sst));
                        }
                    break;
                case 9:
                    stringmove(insdata,6,samplexxx,0,3);
					samplexxx[3] = '\0';
                    if(strstr(samplexxx,"RAT")) samplex=7;
                    else if(strstr(samplexxx,"RAW")) samplex=8;
					else		                  	 samplex=0;
                    if(samplex == 0)
						{
	                    stringmove(insdata,6,samplexx,0,2);
    	                if(!(strcmp(samplexx, "P ")))       samplex=1;
        	            else if(!(strcmp(samplexx, "U ")))  samplex=4;
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
                        write(portfd, sst, sizeof(sst));
                        }
					break;
				case 10:
                    stringmove(insdata,6,samplexx,0,2);
					samplexx[2] = '\0';
                    if(!(strcmp(samplexx, "T0")))       samplex=2;
                    else if(!(strcmp(samplexx, "TD")))  samplex=3;
                    else if(!(strcmp(samplexx, "WD")))  samplex=5;
                    else if(!(strcmp(samplexx, "WS")))  samplex=6;
                    else if(!(strcmp(samplexx, "VI")))  samplex=9;
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
					samplexxx[3] = '\0';
                    if(strstr(samplexxx,"RAT")) samplex=7;
                    else if(strstr(samplexxx,"RAW")) samplex=8;
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
                default: returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
                }
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
	                    else if(insdata[5] == 'U')  samplex=4;
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
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                        	}
						break;
					case 7:
                    	stringmove(insdata,5,samplexx,0,2);
						samplexx[2] = '\0';
                    	if(!(strcmp(samplexx, "T0")))       samplex=2;
                    	else if(!(strcmp(samplexx, "TD")))  samplex=3;
                    	else if(!(strcmp(samplexx, "WD")))  samplex=5;
                    	else if(!(strcmp(samplexx, "WS")))  samplex=6;
            	        else if(!(strcmp(samplexx, "VI")))  samplex=9;
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
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                            }
						break;
					case 8:
	                    stringmove(insdata,5,samplexxx,0,3);
						samplexxx[3] = '\0';
                	    if(strstr(samplexxx,"RAT")) samplex=7;
	                    else if(strstr(samplexxx,"RAW")) samplex=8;
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
                            ret = write(portfd, qcps, strcspn(qcps,"\n")+1);
                            }
						break;
					default: returnTorF(portfd, returntf, 0);
					}
				}
			else
				{
				qcps1 = strchr(ins, ' ');
				qcps2 = strchr(qcps1+1, ' ');
                if(strstr(insdata, "RAT ") != 0) samplex=7;
                else if(strstr(insdata, "RAW ") != 0) samplex=8;
				else if(strstr(insdata, "T0 ") != 0) samplex=2;
                else if(strstr(insdata, "TD ") != 0) samplex=3;
                else if(strstr(insdata, "WD ") != 0) samplex=5;
                else if(strstr(insdata, "WS ") != 0) samplex=6;
                else if(strstr(insdata, "VI ") != 0) samplex=9;
                else if(strstr(insdata, "P ") != 0)  samplex=1;
                else if(strstr(insdata, "U ") != 0)  samplex=4;
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
                            goto txend;
                            }
						}
                    idn = (samplex - 1) * BYTEQCPS;
                    fseek(fdqcp, idn, SEEK_SET);
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
	                    else if(insdata[5] == 'U')  samplex=4;
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
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                        	}
						break;
					case 7:
                    	stringmove(insdata,5,samplexx,0,2);
						samplexx[2] = '\0';
                    	if(!(strcmp(samplexx, "T0")))       samplex=2;
                    	else if(!(strcmp(samplexx, "TD")))  samplex=3;
                    	else if(!(strcmp(samplexx, "WD")))  samplex=5;
                    	else if(!(strcmp(samplexx, "WS")))  samplex=6;
            	        else if(!(strcmp(samplexx, "VI")))  samplex=9;
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
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                            }
						break;
					case 8:
	                    stringmove(insdata,5,samplexxx,0,3);
						samplexxx[3] = '\0';
                	    if(strstr(samplexxx,"RAT")) samplex=7;
                        else if(strstr(samplexxx,"RAW")) samplex=8;
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
                            ret = write(portfd, qcpm, strcspn(qcpm,"\n")+1);
                            }
						break;
					default: returnTorF(portfd, returntf, 0);
					}
				}
			else
				{
				qcpm1 = strchr(ins, ' ');
				qcpm2 = strchr(qcpm1+1, ' ');
                if(strstr(insdata, "RAT ") != 0) samplex=7;
                else if(strstr(insdata, "RAW ") != 0) samplex=8;
                else if(strstr(insdata, "T0 ") != 0) samplex=2;
                else if(strstr(insdata, "TD ") != 0) samplex=3;
                else if(strstr(insdata, "WD ") != 0) samplex=5;
                else if(strstr(insdata, "WS ") != 0) samplex=6;
                else if(strstr(insdata, "VI ") != 0) samplex=9;
                else if(strstr(insdata, "P ") != 0)  samplex=1;
                else if(strstr(insdata, "U ") != 0)  samplex=4;
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
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
                    break;
                case 7:
                    if((fdstbd = open("/tmp/paramete/gale.txt", O_WRONLY|O_CREAT)) < 1)
                        {
                        returnTorF(portfd, returntf, 0);
                        goto txend;
                        }
                    stringmove(ins, 5, alarm4, 0, strcspn(ins, "\n")-4);
                    ret = write(fdstbd, alarm4, sizeof(alarm4));
                    close(fdstbd);
                    returnTorF(portfd, returntf, ret);
                    break;
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
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
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
                    write(portfd, alarm5, strcspn(alarm5, "\n")+1);
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
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
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
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
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
                    write(portfd, alarm4, strcspn(alarm4, "\n")+1);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, dztn, sizeof(dztn));
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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

        else if(strstr(insdata, "HELP") != 0)
            {
            if(strstr(insdata, "/?") != 0)
                returncommand(portfd, helphelp, sizeof(helphelp), SDELAY);
            else
                {
//            	rs485clt(fd_485, R485_STA_CTL1, TX);
            	write(portfd, help, sizeof(help));
            	usleep(LTIME);
//            	rs485clt(fd_485, R485_STA_CTL1, RX);
				}
            }

        else if(strstr(insdata, "SENCO") != 0)
            {
			memset(senco, ' ', sizeof(senco));
			if(strstr(insdata, "/?") != 0)
				returncommand(portfd, helpsenco, sizeof(helpsenco), SDELAY);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
                    write(portfd, senco, strcspn(senco, "\n")+1);
					usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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
//                    rs485clt(fd_485, R485_STA_CTL1, TX);
					write(portfd, senco, strcspn(senco, "\n")+1);
                    usleep(STIME);
//                    rs485clt(fd_485, R485_STA_CTL1, RX);
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

		else if(strstr(insdata, "RESTART") != 0)
			{
//            rs485clt(fd_485, R485_STA_CTL1, TX);
            write(portfd, restart2, sizeof(restart2));
            usleep(MTIME);
//            rs485clt(fd_485, R485_STA_CTL1, RX);
			printf("Now restart Linux! Please wait ...\n");
            close_port(portfd);
            sleep(5);
			rs485clt(fd_485, WDT_STA_CTL, 1);
			}

        else if(strstr(insdata, "REMOVE") != 0)
            {
            system("rm /tmp/data/*");
            returnTorF(portfd, returntf, 1);
            }

		else
			returncommand(portfd, badcommand, sizeof(badcommand), SDELAY);
		}	
txend:	
		ii = 0;
		fflush(NULL);
	}
	close(fd_485);
	close(portfd);
	return 0;
}
