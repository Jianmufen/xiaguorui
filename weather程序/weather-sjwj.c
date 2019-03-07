// weather-sjwj 2012-10-09 11:00 V6.3
// source from "weather-sjwj 2011-11-11 14:00 V6.2 for 吕四测试"
// 不定义EN090423

#include <stdio.h>
#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <signal.h>

#define UART0   "/dev/ttyAT1"
#define UART1   "/dev/ttyAT2"
#define UART2   "/dev/ttyAT3"
#define SERIAL0 "/dev/ttyS0"
#define SERIAL1 "/dev/ttyS1"
#define SERIAL2 "/dev/ttyS2"
#define SERIAL3 "/dev/ttyS3"
#define DEV_RTC	"/dev/rtc"

#define P   0
#define T0  1
#define T1  2
#define T2  3
#define T3  4
#define TW  5
#define U   6
#define TD  7
#define SV1 8
#define SV2 9
#define SV3 10
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
#define CH  30
#define TCA 31
#define LCA 32
#define WW  33
#define SD  34
#define FR  35
#define WI  36
#define FSD 37
#define LNF 38
#define GR  39
#define NR  40
#define DR  41
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

#define RECNUMHOUR	744							// 24*31
#define RECNUMMIN	1440						// 60*24
#define BYTENUMMZ 	252
#define BYTENUMMS 	51
#define BYTENUMMR 	136
#define BYTENUMHZ 	430
#define BYTENUMHS 	251
#define BYTENUMHR 	256
#define SAMPLE120 	126
#define SAMPLE240 	246
#define SAMPLE960 	966

//#define EN090423 1
#define USB	1

//      zi ding yi han shu
static void stringmove(char *from, int i, char *to, int j, int k)
    {
    for( ;k>0;k--)
	{
        to[j]=from[i];
        i++;
        j++;
        }
    }
                                                                                                                                               
static void string_add_space(char *from, int i, char *to, long j, int k, int n)
        {
    int m=k;
    for( ;n>0;n--)
                {
        for( ;k>0;k--)
                        {
            to[j]=from[i];
            j++;
            i++;
            }
        to[j]=' ';
        j++;
        k=m;
        }
        }
                                                                                                                                               
static void string_sishewuru(char *from, int i, char *to, int j)
        {
        int data;
        data = (from[i] - '0') * 1000 + (from[i+1] - '0') * 100 + (from[i+2] - '0') * 10 + (from[i+3] - '0') + 5;
        to[j] = data / 1000 + '0';
        to[j+1] = (data - data / 1000 * 1000) / 100 + '0';
        to[j+2] = (data - data / 100 * 100) / 10 + '0';
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
//		printf("%d-%d-%d %d:%d:%d\n", p->tm_year, p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
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

/*
static void xiaoshi_init(char string[388], char datatime[12], FILE *fd)
    {
    long m = 1;
    int	 n = 4;//744;
    for( ;n>0;n--)
	{
        string[0] = datatime[4];
        string[1] = datatime[5];
        string[2] = datatime[6];
        string[3] = datatime[7];
        string[386] = '\r';
        string[387] = '\n';
	string[388] = '\0';
	printf("string=%s\n",string);
        fseek(fd, m, SEEK_SET);
        fwrite(&string,388,1,fd);
	m += 388;
        timeconvert(datatime, 3600);
    	}
    }
*/

static int fenzhong_bianhao(char *time)
    {
    int temp1, temp2, id;
    temp1 = (time[6] - '0') * 10 + (time[7] - '0');
    temp2 = (time[8] - '0') * 10 + (time[9] - '0');
    id = temp1 * 60 + temp2;
    return(id);
    }

static int xiaoshi_bianhao(char *time)
        {
        int temp1, temp2, id;
        temp1 = (time[4] - '0') * 10 + (time[5] - '0') - 1;
        temp2 = (time[6] - '0') * 10 + (time[7] - '0');
        id = temp1 * 24 + temp2;
        return(id);
        }

static void stationid(char *string)
    {
    int fdstbd;
                                                                                                                                               
    if((fdstbd = open("/tmp/paramete/stid.txt",O_RDONLY))< 1)
        {
        memset(string, '0', 5);
        }
    else
        {
        read(fdstbd, string, 5);
//      printf("id=%s\n",stidd);
        close(fdstbd);
        }
    }

static void stationlong(char *string)
    {
    int fdstbd;
    if((fdstbd = open("/tmp/paramete/long.txt",O_RDONLY))< 1)
        {
		memset(string, '0', 9);
        }
    else
        {
        read(fdstbd, string, 9);
//      printf("id=%s\n",stidd);
        close(fdstbd);
        }
    }

static void stationlat(char *string)
    {
    int fdstbd;
    if((fdstbd = open("/tmp/paramete/lat.txt",O_RDONLY))< 1)
        {
        memset(string, '0', 8);
        }
    else
        {
        read(fdstbd, string, 8);
//      printf("id=%s\n",stidd);
        close(fdstbd);
        }
    }

static void stationalt(char *string)
	{
    int fdstbd;
	long altitude;
	char high[6];
    if((fdstbd = open("/tmp/paramete/alt.txt",O_RDONLY))< 1)
        {
        string[0] = ' ';
        string[1] = ' ';
        string[2] = ' ';
        string[3] = ' ';
        string[4] = '0';
        }
    else
        {
        read(fdstbd, high, sizeof(high));
        close(fdstbd);
        high[6] = '\0';
        altitude = 10 * atof(high);
    	if(altitude >= 0)
        	{
        	string[0] = altitude / 10000 + '0';
        	string[1] = (altitude-altitude/10000*10000) / 1000 + '0';
        	string[2] = (altitude-altitude/1000*1000) / 100 + '0';
			string[3] = (altitude-altitude/100*100) / 10 + '0';
        	string[4] = altitude % 10 + '0';
        	if(string[0] == '0')
            	{
            	string[0] = ' ';
            	if(string[1] == '0')
                	{
                	string[1] = ' ';
					if(string[2] == '0')
                    	{
                    	string[2] = ' ';
		                if(string[3] == '0')
                    		{
                    		string[3] = ' ';
							}
						}
                	}
            	}
        	}
    	else
        	{
        	altitude = 0 - altitude;
        	string[0] = '-';
            string[1] = altitude / 1000 + '0';
            string[2] = (altitude-altitude/1000*1000) / 100 + '0';
            string[3] = (altitude-altitude/100*100) / 10 + '0';
			string[4] = altitude % 10 + '0';
        	if(string[1] == '0')
            	{
            	string[0] = ' ';
            	string[1] = '-';
            	if(string[2] == '0')
                	{
                	string[1] = ' ';
                	string[2] = '-';
                	if(string[3] == '0')
                    	{
                    	string[2] = ' ';
                    	string[3] = '-';
						}
					}
                }
            }
        }
	}

static void stationaltp(char *string)
	{
    int fdstbd;
	long altitude;
	char high[6];
    if((fdstbd = open("/tmp/paramete/altp.txt",O_RDONLY))< 1)
        {
        string[0] = ' ';
        string[1] = ' ';
        string[2] = ' ';
        string[3] = ' ';
        string[4] = '0';
        }
    else
        {
        read(fdstbd, high, sizeof(high));
        close(fdstbd);
        high[6] = '\0';
        altitude = 10 * atof(high);
    	if(altitude >= 0)
        	{
        	string[0] = altitude / 10000 + '0';
        	string[1] = (altitude-altitude/10000*10000) / 1000 + '0';
        	string[2] = (altitude-altitude/1000*1000) / 100 + '0';
			string[3] = (altitude-altitude/100*100) / 10 + '0';
        	string[4] = altitude % 10 + '0';
        	if(string[0] == '0')
            	{
            	string[0] = ' ';
            	if(string[1] == '0')
                	{
                	string[1] = ' ';
					if(string[2] == '0')
                    	{
                    	string[2] = ' ';
		                if(string[3] == '0')
                    		{
                    		string[3] = ' ';
							}
						}
                	}
            	}
        	}
    	else
        	{
        	altitude = 0 - altitude;
        	string[0] = '-';
            string[1] = altitude / 1000 + '0';
            string[2] = (altitude-altitude/1000*1000) / 100 + '0';
            string[3] = (altitude-altitude/100*100) / 10 + '0';
			string[4] = altitude % 10 + '0';
        	if(string[1] == '0')
            	{
            	string[0] = ' ';
            	string[1] = '-';
            	if(string[2] == '0')
                	{
                	string[1] = ' ';
                	string[2] = '-';
                	if(string[3] == '0')
                    	{
                    	string[2] = ' ';
                    	string[3] = '-';
						}
					}
                }
            }
        }
	}


int main(int argc, char **argv)
    {
	int fd_rtc=-1;
    int fdparame, fdrtc, retval;
    FILE *fdxsdata, *fdfzdata, *fdmin, *fdhour;
    struct rtc_time rtc_tm;
    char ctime[13], rtime[13], datatime[13];
    char datahz[BYTENUMHZ], datahs[BYTENUMHS], datahr[BYTENUMHR]; 
    char datamz[BYTENUMMZ], datams[BYTENUMMS], datamr[BYTENUMMR];
	char datamz0[BYTENUMMZ], datams0[BYTENUMMS], datamr0[BYTENUMMR];
	char datamz1[BYTENUMMZ-4], datams1[BYTENUMMS-4], datamr1[BYTENUMMR-4];
    char samp120[SAMPLE120], samp240[SAMPLE240], samp960[SAMPLE960];
    int recnum, bytenum;
    long seekset;
    int sleept;

    time_t timep;
    struct tm *p;
    int sec, min, hour, mday, mon, year;
	int localsec, localmin, localhour, localmday, localmon, localyear;
    int td=0, localt=0;
    char paramete[8];

    char namemz[]={"/tmp/data/GGYYMMDD.DAT"};
    char namems[]={"/tmp/data/SSYYMMDD.DAT"};
    char namemr[]={"/tmp/data/RRYYMMDD.DAT"};
    char namehz[]={"/tmp/data/hzYYMM.TXT"};
    char namehs[]={"/tmp/data/hsYYMM.TXT"};
    char namehr[]={"/tmp/data/hrYYMM.TXT"};

#ifdef USB
    char usbmz[]={"/usb/data/GGYYMMDD.DAT"};
    char usbms[]={"/usb/data/SSYYMMDD.DAT"};
    char usbmr[]={"/usb/data/RRYYMMDD.DAT"};
    char usbhz[]={"/usb/data/hzYYMM.TXT"};
    char usbhs[]={"/usb/data/hsYYMM.TXT"};
    char usbhr[]={"/usb/data/hrYYMM.TXT"};
#endif

    char instime[]={"date 123120002011.00"};
    char oldtime[]={"date 123120002011.00"};
	int nowtime, lasttime, nowyear, lastyear;

    char presenttime[19], stpara[12];
	char senstate[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};

	char mzfirst[BYTENUMMZ]={"SH001 2009   01   010000000E000000N    0   15  100   153111111101111111111111111111110000000V1.00---------------------------------------------------------------------------------------------------------------------------------------------------------\r\n"};
	char mrfirst[BYTENUMMR]={"SH001 2009   01   010000000E000000N    0   15    0   15    0    0    0    0    0    051010000001V1.00---------------------------------\r\n"};
	char msfirst[BYTENUMMS]={"SH001 2009   01 010000000E000000N    0 4000000000\r\n"};
    char hzfirst[BYTENUMHZ]={"SH001 2009   010000000E000000N    0   15  100   15    3    1    1    1    1    1    1    1    0    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    1    0    0    0    0    0    0    0V1.00--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n"};
	char hrfirst[BYTENUMHR]={"SH001 2009   010000000E000000N    0   15    0   15    0    0    0    0    0    0    5    1    0    1    0    0    0    0    0    0    1V1.00------------------------------------------------------------------------------------------------------------------\r\n"};
	char hsfirst[BYTENUMHS]={"SH001 2009   010000000E000000N    0    4   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0V1.00----------------------------------------\r\n"};

//  td
    if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)
        td = 0;
    else
        {
        read(fdparame,&paramete,4);
        close(fdparame);
        paramete[4]='\0';
//      printf("printf=%s\n",paramete);
        td = atof(paramete);
//      printf("td=%d\n",td);
        if(td<-180 || td>180)
            td = 0;
        }
    localt = (60-td%60) % 60;
//  printf("td=%d,localt=%d\n",td,localt);

    fd_rtc = open(DEV_RTC, O_RDONLY);
    if(fd_rtc == -1)
        {
        perror("Open RTC error!\n");
        goto begin;
        }
    retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
    if(retval == -1)
        {
        perror("RTC_RD_TIME ioctl");
        close(fd_rtc);
        goto begin;
        }
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
//  printf("instime=%s\n",instime);
    stringmove(instime, 0 , oldtime, 0, sizeof(instime));

begin:
    printf("weather-sjwj 2012-10-09 11:00 V6.3\n");

    while(1)
		{
start1:
        time(&timep);
        p = gmtime(&timep);
        sec  = p->tm_sec;
        min  = p->tm_min;
        hour = p->tm_hour;
        mday = p->tm_mday;
		mon  = p->tm_mon + 1;
		year = p->tm_year % 100;
//    	fprintf(stderr, "Current RTC date/time is %d-%d-%d, %02d:%02d:%02d.\n",year + 2000, mon, mday, hour, min, sec);

		timep = mktime(p);
		timep = timep + td*60;
		p = localtime(&timep);
        localsec  = p->tm_sec;
        localmin  = p->tm_min;
        localhour = p->tm_hour;
        localmday = p->tm_mday;
        localmon  = p->tm_mon + 1;
        localyear = p->tm_year % 100;

		if(sec == 3)
	    	{
	        if((fdparame=open("/tmp/paramete/td.txt",O_RDONLY)) < 1)					// TD
    	        td = 0;
        	else
            	{
            	read(fdparame,&paramete,4);
            	close(fdparame);
            	paramete[4]='\0';
//          	printf("printf=%s\n",paramete);
            	td = atof(paramete);
//          	printf("td=%d\n",td);
            	if(td<-180 || td>180)
                	td = 0;
            	}
        	localt = (60-td%60) % 60;
//      	printf("td=%d,localt=%d\n",td,localt);

        	if((fdparame = open("/tmp/paramete/senstonoff.txt",O_RDONLY)) < 1)			// Senstate
            	usleep(1);
			else
				{
        		read(fdparame, senstate, sizeof(senstate));
        		close(fdparame);
				}

	    	ctime[0] = year/10 + '0';													// Beijing Time
            ctime[1] = year%10 + '0';
	    	ctime[2] = mon/10 + '0';
            ctime[3] = mon%10 + '0';
	    	ctime[4] = mday/10 + '0';
            ctime[5] = mday%10 + '0';
            ctime[6] = hour/10 + '0';
            ctime[7] = hour%10 + '0';
            ctime[8] = min/10 + '0';
            ctime[9] = min%10 + '0';
            ctime[10] = sec/10 + '0';
            ctime[11] = sec%10 + '0';
	    	ctime[12] = '\0';
//	    	printf("ctime=%s\n",ctime);

            rtime[0] = localyear/10 + '0';												// Local Time
            rtime[1] = localyear%10 + '0';
            rtime[2] = localmon/10 + '0';
            rtime[3] = localmon%10 + '0';
            rtime[4] = localmday/10 + '0';
            rtime[5] = localmday%10 + '0';
            rtime[6] = localhour/10 + '0';
            rtime[7] = localhour%10 + '0';
            rtime[8] = localmin/10 + '0';
            rtime[9] = localmin%10 + '0';
            rtime[10] = localsec/10 + '0';
            rtime[11] = localsec%10 + '0';
            rtime[12] = '\0';

start11:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namemz, 12, 6);
            if((fdfzdata = fopen(namemz,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namemz,"w+")) == NULL)
                    {
//                    fzsign = 1;
#ifndef USB
                    goto start12;
#else
					stringmove(datatime, 0, usbmz, 12, 6);	
	            	if((fdfzdata = fopen(usbmz,"r+")) == NULL )
                		{
                		if((fdfzdata = fopen(usbmz,"w+")) == NULL)
                    		{
//                    		fzsign = 1;
							goto start12;
							}
						}
					else
						goto usbmz1;
#endif
                    }

//				memset(datamz, ' ', sizeof(datamz));
                stationid(stpara);												// Station Id
                stringmove(stpara, 0, mzfirst, 0, 5);
				filetime(presenttime, 239);										// Time
				stringmove(presenttime, 0, mzfirst, 6, 4);
				stringmove(presenttime, 5, mzfirst, 13, 2);
                stringmove(presenttime, 8, mzfirst, 18, 2);
                stationlong(stpara);											// Station Long
//				mzfirst[20] = 'E';
                stringmove(stpara, 0, mzfirst, 20, 3);
                stringmove(stpara, 4, mzfirst, 23, 2);
                stringmove(stpara, 7, mzfirst, 25, 2);
                stationlat(stpara);												// Station Lat
//              mzfirst[28] = 'N';
                stringmove(stpara, 0, mzfirst, 28, 2);
                stringmove(stpara, 3, mzfirst, 30, 2);
                stringmove(stpara, 6, mzfirst, 32, 2);
                stationalt(stpara);                                             // Station Alt
                stringmove(stpara, 0, mzfirst, 35, 5);
                stationaltp(stpara);                                            // Station AltP
                stringmove(stpara, 0, mzfirst, 40, 5);
				mzfirst[56] = senstate[T0];										// Sensor Flags
                mzfirst[57] = senstate[T1];
                mzfirst[58] = senstate[T2];
                mzfirst[59] = senstate[T3];
                mzfirst[60] = senstate[SV1];
                mzfirst[61] = senstate[SV2];
                mzfirst[62] = senstate[SV3];
                mzfirst[63] = senstate[TW];
                mzfirst[64] = senstate[U];
                mzfirst[65] = senstate[P];
                mzfirst[66] = senstate[WD];
                mzfirst[67] = senstate[WS];
                mzfirst[68] = senstate[WS1];
                mzfirst[69] = senstate[RAT];
				mzfirst[70] = senstate[RAT1];
                mzfirst[71] = senstate[RAW];
                mzfirst[72] = senstate[TG];
                mzfirst[73] = senstate[ST0];
                mzfirst[74] = senstate[IR];
                mzfirst[75] = senstate[ST1];
                mzfirst[76] = senstate[ST2];
                mzfirst[77] = senstate[ST3];
                mzfirst[78] = senstate[ST4];
                mzfirst[79] = senstate[ST5];
                mzfirst[80] = senstate[ST6];
                mzfirst[81] = senstate[ST7];
                mzfirst[82] = senstate[ST8];
                mzfirst[83] = senstate[LE];
                mzfirst[84] = senstate[VI];
                mzfirst[85] = senstate[CH];
                mzfirst[86] = senstate[TCA];
                mzfirst[87] = senstate[WW];
                mzfirst[88] = senstate[SD];
                mzfirst[89] = senstate[WI];
                mzfirst[90] = senstate[FSD];
                mzfirst[91] = senstate[LNF];
                mzfirst[BYTENUMMZ-2]='\r';
                mzfirst[BYTENUMMZ-1]='\n';
                fwrite(&mzfirst, sizeof(mzfirst), 1, fdfzdata);

                memset(datamz, '-', sizeof(datamz));
                datamz[BYTENUMMZ-2]='\r';
                datamz[BYTENUMMZ-1]='\n';
//                datamz[396]='\0';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=BYTENUMMZ,seekset=BYTENUMMZ;recnum>0;recnum--)
                    {
                    datamz[0] = datatime[6];
                    datamz[1] = datatime[7];
                    datamz[2] = datatime[8];
                    datamz[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&datamz, sizeof(datamz), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
               
//				stringmove(ctime, 0, datatime, 0, 12);
//				timeconvert(datatime, -864000);
//              stringmove(datatime,0,namemz,12,6);
				}

usbmz1:
            memset(datamz, '/', sizeof(datamz));
            datamz[BYTENUMMZ-2]='\r';
            datamz[BYTENUMMZ-1]='\n';
//            datamz[396]='\0';
	
	    	if((fdmin = fopen("/usr/mingg.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start12;
				}
            fread(&datamz,sizeof(datamz),1,fdmin);
            fclose(fdmin);

	    	stringmove(ctime, 0, datatime, 0, 12);
            datamz[0] = datatime[6];
            datamz[1] = datatime[7];
            datamz[2] = datatime[8];
            datamz[3] = datatime[9];
            timeconvert(datatime, 14340);
            seekset = BYTENUMMZ * fenzhong_bianhao(datatime) +  BYTENUMMZ;
//	    	printf("seekset1=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&datamz, sizeof(datamz), 1, fdfzdata);
#ifdef EN090423
			if(seekset >= (2*BYTENUMMZ))
				{
				seekset = seekset - BYTENUMMZ;
				fseek(fdfzdata, seekset, SEEK_SET);
				fread(&datamz0, sizeof(datamz0), 1, fdfzdata);
				if(strstr(datamz0, "-------------------------------------------------------------------------------") != 0)
					{
					stringmove(datamz, 4, datamz1, 0, sizeof(datamz1));
					seekset = seekset + 4;
					fseek(fdfzdata, seekset, SEEK_SET);
					fwrite(&datamz1, sizeof(datamz1), 1, fdfzdata);
//					printf("seekset5=%ld\n",seekset);
					}
				}
#endif
            fclose(fdfzdata);

start12:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namems, 12, 6);
            if((fdfzdata = fopen(namems,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namems,"w+")) == NULL)
                    {
//                    fzsign = 1;
#ifndef USB
                    goto start13;
#else
					stringmove(datatime, 0, usbms, 12, 6);
                    if((fdfzdata = fopen(usbms,"r+")) == NULL )
                        {
                        if((fdfzdata = fopen(usbms,"w+")) == NULL)
                            {
//                          fzsign = 1;
                            goto start13;
							}
						}
                    else
                        goto usbms1;
#endif
                    }

//            	memset(datams, '/', sizeof(datams));
                stationid(stpara);                                              // Station Id
                stringmove(stpara, 0, msfirst, 0, 5);
                filetime(presenttime, 239);                                     // Time
                stringmove(presenttime, 0, msfirst, 6, 4);
                stringmove(presenttime, 5, msfirst, 13, 2);
                stringmove(presenttime, 8, msfirst, 16, 2);
                stationlong(stpara);                                            // Station Long
//              msfirst[20] = 'E';
                stringmove(stpara, 0, msfirst, 18, 3);
                stringmove(stpara, 4, msfirst, 21, 2);
                stringmove(stpara, 7, msfirst, 23, 2);
                stationlat(stpara);                                             // Station Lat
//              msfirst[28] = 'N';
                stringmove(stpara, 0, msfirst, 26, 2);
                stringmove(stpara, 3, msfirst, 28, 2);
                stringmove(stpara, 6, msfirst, 30, 2);
                stationalt(stpara);                                             // Station Alt
                stringmove(stpara, 0, msfirst, 33, 5);
                msfirst[40] = senstate[SM1];                                    // Sensor Flags
                msfirst[41] = senstate[SM2];
                msfirst[42] = senstate[SM3];
                msfirst[43] = senstate[SM4];
                msfirst[44] = senstate[SM5];
                msfirst[45] = senstate[SM6];
                msfirst[46] = senstate[SM7];
                msfirst[47] = senstate[SM8];
                msfirst[48] = senstate[WT];
                msfirst[BYTENUMMS-2]='\r';
                msfirst[BYTENUMMS-1]='\n';
                fwrite(msfirst, sizeof(msfirst), 1, fdfzdata);

                memset(datams, '-', sizeof(datams));
                datams[BYTENUMMS-2]='\r';
                datams[BYTENUMMS-1]='\n';
//                datamz[396]='\0';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=BYTENUMMS,seekset=BYTENUMMS;recnum>0;recnum--)
                    {
                    datams[0] = datatime[6];
                    datams[1] = datatime[7];
                    datams[2] = datatime[8];
                    datams[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&datams, sizeof(datams), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
//              stringmove(ctime, 0, datatime, 0, 12);
//              timeconvert(datatime, -864000);
//              stringmove(datatime,0,namemz,12,6);
                }

usbms1:
            memset(datams, '/', sizeof(datams));
            datams[BYTENUMMS-2]='\r';
            datams[BYTENUMMS-1]='\n';
//            datamz[396]='\0';
        	if((fdmin = fopen("/usr/minss.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start13;
				}
            fread(&datams,sizeof(datams),1,fdmin);
            fclose(fdmin);

            stringmove(ctime, 0, datatime, 0, 12);
            datams[0] = datatime[6];
            datams[1] = datatime[7];
            datams[2] = datatime[8];
            datams[3] = datatime[9];
            timeconvert(datatime, 14340);
            seekset = BYTENUMMS * fenzhong_bianhao(datatime) +  BYTENUMMS;
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&datams, sizeof(datams), 1, fdfzdata);
#ifdef EN090423
            if(seekset >= (2*BYTENUMMS))
                {
                seekset = seekset - BYTENUMMS;
                fseek(fdfzdata, seekset, SEEK_SET);
                fread(&datams0, sizeof(datams0), 1, fdfzdata);
                if(strstr(datams0, "------------------------") != 0)
                    {
                    stringmove(datams, 4, datams1, 0, sizeof(datams1));
                    seekset = seekset + 4;
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&datams1, sizeof(datams1), 1, fdfzdata);
//                  printf("seekset5=%ld\n",seekset);
                    }
                }
#endif
            fclose(fdfzdata);

start13:
            stringmove(rtime, 0 , datatime, 0, 12);
            timeconvert(datatime, -60);
            stringmove(datatime, 0, namemr, 12, 6);
            if((fdfzdata = fopen(namemr,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namemr,"w+")) == NULL)
                    {
//                    fzsign = 1;
#ifndef USB
                    goto start2c;
#else
					stringmove(datatime, 0, usbmr, 12, 6);
                    if((fdfzdata = fopen(usbmr,"r+")) == NULL )
                        {
                        if((fdfzdata = fopen(usbmr,"w+")) == NULL)
                            {
//                          fzsign = 1;
                            goto start2c;
							}
						}
                    else
                        goto usbmr1;
#endif
                    }

                stationid(stpara);												// Station Id
                stringmove(stpara, 0, mrfirst, 0, 5);
                filetime(presenttime, td-1);									// Time
                stringmove(presenttime, 0, mrfirst, 6, 4);
                stringmove(presenttime, 5, mrfirst, 13, 2);
                stringmove(presenttime, 8, mrfirst, 18, 2);
                stationlong(stpara);											// Station Long
//                mrfirst[20] = 'E';
                stringmove(stpara, 0, mrfirst, 20, 3);
                stringmove(stpara, 4, mrfirst, 23, 2);
                stringmove(stpara, 7, mrfirst, 25, 2);
                stationlat(stpara);												// Station Lat
//                mrfirst[28] = 'N';
                stringmove(stpara, 0, mrfirst, 28, 2);
                stringmove(stpara, 3, mrfirst, 30, 2);
                stringmove(stpara, 6, mrfirst, 32, 2);
				stationalt(stpara);												// Station Alt
				stringmove(stpara, 0, mrfirst, 35, 5);
				mrfirst[86] = senstate[GR];										// Sensor Flags
                mrfirst[87] = senstate[NR];
                mrfirst[88] = senstate[DR];
                mrfirst[89] = senstate[SR];
                mrfirst[90] = senstate[RR];
                mrfirst[91] = senstate[UR];
                mrfirst[92] = senstate[AR];
                mrfirst[93] = senstate[TR];
                mrfirst[94] = senstate[PR];
                mrfirst[95] = senstate[SSD];
                mrfirst[BYTENUMMR-2]='\r';
                mrfirst[BYTENUMMR-1]='\n';
                fwrite(&mrfirst, sizeof(mrfirst), 1, fdfzdata);

                memset(datamr, '-', sizeof(datamr));
                datamr[BYTENUMMR-2]='\r';
                datamr[BYTENUMMR-1]='\n';
//                datamz[396]='\0';
                stringmove(rtime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=BYTENUMMR,seekset=BYTENUMMR;recnum>0;recnum--)
                    {
//                    datamr[0] = datatime[6];
//                    datamr[1] = datatime[7];
//                    datamr[2] = datatime[8];
//                    datamr[3] = datatime[9];
		            if(datatime[6]=='0' && datatime[7]=='0' && datatime[8]=='0' && datatime[9]=='0')
		                {
        		        datamr[0] = '2';
                		datamr[1] = '4';
		                datamr[2] = '0';
        		        datamr[3] = '0';
                		}
		            else
        		        {
                		datamr[0] = datatime[6];
		                datamr[1] = datatime[7];
        		        datamr[2] = datatime[8];
                		datamr[3] = datatime[9];
                		}
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&datamr, sizeof(datamr), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
//              stringmove(rtime, 0, datatime, 0, 12);
//              timeconvert(datatime, -864000);
//              stringmove(datatime,0,namemz,12,6);
                }

usbmr1:
            memset(datamr, '/', sizeof(datamr));
            datamr[BYTENUMMR-2]='\r';
            datamr[BYTENUMMR-1]='\n';
//            datamz[396]='\0';
        	if((fdmin = fopen("/usr/minrr.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
            	goto start2;
				}
            fread(&datamr,sizeof(datamr),1,fdmin);
            fclose(fdmin);

            stringmove(rtime, 0, datatime, 0, 12);
			if(datatime[6]=='0' && datatime[7]=='0' && datatime[8]=='0' && datatime[9]=='0')
				{
				datamr[0] = '2';
            	datamr[1] = '4';
                datamr[2] = '0';
                datamr[3] = '0';
				}
			else
				{
            	datamr[0] = datatime[6];
            	datamr[1] = datatime[7];
				datamr[2] = datatime[8];
            	datamr[3] = datatime[9];
				}
            timeconvert(datatime, -60);
            seekset = BYTENUMMR * fenzhong_bianhao(datatime) +  BYTENUMMR;
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&datamr, sizeof(datamr), 1, fdfzdata);
#ifdef EN090423
            if(seekset >= (2*BYTENUMMR))
                {
                seekset = seekset - BYTENUMMR;
                fseek(fdfzdata, seekset, SEEK_SET);
                fread(&datamr0, sizeof(datamr0), 1, fdfzdata);
                if(strstr(datamr0, "-------------------------------------------------------------------------------") != 0)
                    {
                    stringmove(datamr, 4, datamr1, 0, sizeof(datamr1));
                    seekset = seekset + 4;
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&datamr1, sizeof(datamr1), 1, fdfzdata);
//                  printf("seekset5=%ld\n",seekset);
                    }
                }
#endif
            fclose(fdfzdata);

start2:
            if((fdfzdata = fopen("/tmp/sample/sampt.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampt.tmp","w+")) == NULL)
                    {
                    goto start21;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampt.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start21;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start21:
            if((fdfzdata = fopen("/tmp/sample/sampt1.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampt1.tmp","w+")) == NULL)
                    {
                    goto start22;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampt1.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start22;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start22:
            if((fdfzdata = fopen("/tmp/sample/sampt2.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampt2.tmp","w+")) == NULL)
                    {
                    goto start23;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampt2.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start23;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start23:
            if((fdfzdata = fopen("/tmp/sample/sampt3.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampt3.tmp","w+")) == NULL)
                    {
                    goto start24;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampt3.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start24;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start24:
            if((fdfzdata = fopen("/tmp/sample/sampd.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampd.tmp","w+")) == NULL)
                    {
                    goto start25;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampd.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start25;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start25:
            if((fdfzdata = fopen("/tmp/sample/samprh.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/samprh.tmp","w+")) == NULL)
                    {
                    goto start26;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/samprh.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start26;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start26:
            if((fdfzdata = fopen("/tmp/sample/sampgr.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampgr.tmp","w+")) == NULL)
                    {
                    goto start27;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(rtime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampgr.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start27;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(rtime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start27:
            if((fdfzdata = fopen("/tmp/sample/sampdr.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampdr.tmp","w+")) == NULL)
                    {
                    goto start28;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(rtime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampdr.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start28;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
            stringmove(rtime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start28:
            if((fdfzdata = fopen("/tmp/sample/sampwd.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampwd.tmp","w+")) == NULL)
                    {
                    goto start29;
                    }
                memset(samp240, '-', sizeof(samp240));
                samp240[SAMPLE240-2]='\r';
                samp240[SAMPLE240-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE240,seekset=0; recnum>0; recnum--)
                    {
                    samp240[0] = datatime[6];
                    samp240[1] = datatime[7];
                    samp240[2] = datatime[8];
                    samp240[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp240, sizeof(samp240), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampwd.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start29;
				}
            fread(&samp240,sizeof(samp240),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
//			printf("time=%s\n",datatime);
            samp240[0] = datatime[6];
            samp240[1] = datatime[7];
            samp240[2] = datatime[8];
            samp240[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE240 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp240, sizeof(samp240), 1, fdfzdata);
            fclose(fdfzdata);

start29:
            if((fdfzdata = fopen("/tmp/sample/sampws.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampws.tmp","w+")) == NULL)
                    {
                    goto start2a;
                    }
                memset(samp960, '-', sizeof(samp960));
                samp960[SAMPLE960-2]='\r';
                samp960[SAMPLE960-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE960,seekset=0; recnum>0; recnum--)
                    {
                    samp960[0] = datatime[6];
                    samp960[1] = datatime[7];
                    samp960[2] = datatime[8];
                    samp960[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp960, sizeof(samp960), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampws.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start2a;
				}
            fread(&samp960,sizeof(samp960),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp960[0] = datatime[6];
            samp960[1] = datatime[7];
            samp960[2] = datatime[8];
            samp960[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE960 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp960, sizeof(samp960), 1, fdfzdata);
            fclose(fdfzdata);

start2a:
            if((fdfzdata = fopen("/tmp/sample/sampws1.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampws1.tmp","w+")) == NULL)
                    {
                    goto start2b;
                    }
                memset(samp240, '-', sizeof(samp240));
                samp240[SAMPLE240-2]='\r';
                samp240[SAMPLE240-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE240,seekset=0; recnum>0; recnum--)
                    {
                    samp240[0] = datatime[6];
                    samp240[1] = datatime[7];
                    samp240[2] = datatime[8];
                    samp240[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp240, sizeof(samp240), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
        		}
            if((fdmin = fopen("/usr/sampws1.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start2b;
				}
            fread(&samp240,sizeof(samp240),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp240[0] = datatime[6];
            samp240[1] = datatime[7];
            samp240[2] = datatime[8];
            samp240[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE240 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp240, sizeof(samp240), 1, fdfzdata);
            fclose(fdfzdata);

start2b:
            if((fdfzdata = fopen("/tmp/sample/sampsm1.tmp","r+")) == NULL )
                {
                if((fdfzdata = fopen("/tmp/sample/sampsm1.tmp","w+")) == NULL)
                    {
                    goto start2c;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='0';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE120,seekset=0; recnum>0; recnum--)
                    {
                    samp120[0] = datatime[6];
                    samp120[1] = datatime[7];
                    samp120[2] = datatime[8];
                    samp120[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
                }
            if((fdmin = fopen("/usr/sampsm1.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start2c;
				}
            fread(&samp120,sizeof(samp120),1,fdmin);
            fclose(fdmin);
//	    	samp120[126] = '\0';
//	    	printf("sampsm1=%s\n",samp120);
            stringmove(ctime, 0, datatime, 0, 12);
            samp120[0] = datatime[6];
            samp120[1] = datatime[7];
            samp120[2] = datatime[8];
            samp120[3] = datatime[9];
            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
	    	fclose(fdfzdata);


start2c:
			usleep(1);
	    	fd_rtc = open(DEV_RTC, O_RDONLY);
    		if(fd_rtc == -1) 
				{
        		perror("Open RTC error!\n");
        		goto start2d;
    			}
	    	retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
    		if(retval == -1) 
				{
        		perror("RTC_RD_TIME ioctl");
				close(fd_rtc);
        		goto start2d;
    			}
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
//			printf("instime=%s\n",instime);
			nowyear = (instime[13]-'0')*1000+(instime[14]-'0')*100+(instime[15]-'0')*10+(instime[16]-'0');
			lastyear = (oldtime[13]-'0')*1000+(oldtime[14]-'0')*100+(oldtime[15]-'0')*10+(oldtime[16]-'0');
			printf("nowyear=%d,lastyear=%d\n",nowyear,lastyear);
			if(abs(nowyear-lastyear) < 3)
				{
				nowtime = ((instime[9]-'0')*10+(instime[10]-'0'))*60+((instime[11]-'0')*10+(instime[12]-'0'));
				if(nowtime == 0)
					nowtime = 1440;
				lasttime = ((oldtime[9]-'0')*10+(oldtime[10]-'0'))*60+((oldtime[11]-'0')*10+(oldtime[12]-'0'));
				printf("nowtime=%d,lasttime=%d\n",nowtime,lasttime);
				if(abs(nowtime-lasttime) < 60)
					{
					printf("system:%s\n",instime);
					system(instime);
//					stringmove(instime, 0, oldtime, 0, sizeof(instime));
					}
//				stringmove(instime, 0, oldtime, 0, sizeof(instime));
				}
			stringmove(instime, 0, oldtime, 0, sizeof(instime));
start2d:			
			usleep(1);
	    	}    //if(rtc_tm.tm_sec == 0)
		else
        	{
	    	sleep(1);
	    	goto start1;
	    	}	    

start3:	
		if(min==0 || min==1)
//		if(min == 0)
	    	{
	    	stringmove(ctime, 0, datatime, 0, 12);
            timeconvert(datatime, 10800);
            stringmove(datatime, 0, namehz, 12, 4);
            if((fdxsdata=fopen(namehz,"r+")) == NULL )
				{            
				if((fdxsdata=fopen(namehz,"w+")) == NULL)
					{
#ifndef USB
                    goto start31;
#else
					stringmove(datatime, 0, usbhz, 12, 4);
                    if((fdxsdata = fopen(usbhz,"r+")) == NULL )
                        {
                        if((fdxsdata = fopen(usbhz,"w+")) == NULL)
                            {
//                          fzsign = 1;
                            goto start31;
							}
						}
                    else
                        goto usbhz1;
#endif
					}
//                memset(datahz, '/', sizeof(datahz));
                stationid(stpara);												// Station Id
                stringmove(stpara, 0, hzfirst, 0, 5);
				filetime(presenttime, 239);
				stringmove(presenttime, 0, hzfirst, 6, 4);
				stringmove(presenttime, 5, hzfirst, 13, 2);
                stationlong(stpara);											// Station Long
//				hzfirst[20] = 'E';
                stringmove(stpara, 0, hzfirst, 15, 3);
                stringmove(stpara, 4, hzfirst, 18, 2);
                stringmove(stpara, 7, hzfirst, 20, 2);
                stationlat(stpara);												// Station Lat
//              hzfirst[28] = 'N';
                stringmove(stpara, 0, hzfirst, 23, 2);
                stringmove(stpara, 3, hzfirst, 25, 2);
                stringmove(stpara, 6, hzfirst, 27, 2);
                stationalt(stpara);                                             // Station Alt
                stringmove(stpara, 0, hzfirst, 30, 5);
                stationaltp(stpara);                                            // Station AltP
                stringmove(stpara, 0, hzfirst, 35, 5);
				hzfirst[64] = senstate[T0];										// Sensor Flags
                hzfirst[69] = senstate[T1];
                hzfirst[74] = senstate[T2];
                hzfirst[79] = senstate[T3];
                hzfirst[84] = senstate[SV1];
                hzfirst[89] = senstate[SV2];
                hzfirst[94] = senstate[SV3];
                hzfirst[99] = senstate[TW];
                hzfirst[104] = senstate[U];
                hzfirst[109] = senstate[P];
                hzfirst[114] = senstate[WD];
                hzfirst[119] = senstate[WS];
                hzfirst[124] = senstate[WS1];
                hzfirst[129] = senstate[RAT];
                hzfirst[134] = senstate[RAT1];
                hzfirst[139] = senstate[RAW];
                hzfirst[144] = senstate[TG];
                hzfirst[149] = senstate[ST0];
                hzfirst[154] = senstate[IR];
                hzfirst[159] = senstate[ST1];
                hzfirst[164] = senstate[ST2];
                hzfirst[169] = senstate[ST3];
                hzfirst[174] = senstate[ST4];
                hzfirst[179] = senstate[ST5];
                hzfirst[184] = senstate[ST6];
                hzfirst[189] = senstate[ST7];
                hzfirst[194] = senstate[ST8];
                hzfirst[199] = senstate[LE];
                hzfirst[204] = senstate[VI];
                hzfirst[209] = senstate[CH];
                hzfirst[214] = senstate[TCA];
                hzfirst[219] = senstate[WW];
                hzfirst[224] = senstate[SD];
                hzfirst[229] = senstate[WI];
                hzfirst[234] = senstate[FSD];
                hzfirst[239] = senstate[LNF];
                hzfirst[BYTENUMHZ-2]='\r';
                hzfirst[BYTENUMHZ-1]='\n';
//                stringmove(ctime, 0, datatime, 0, 12);
//                timeconvert(datatime, 14340);
//                fenzhong_wj_init(autodata,datatime);
                fwrite(hzfirst, sizeof(hzfirst), 1, fdxsdata);

                memset(datahz, '-', sizeof(datahz));
                datahz[BYTENUMHZ-2]='\r';
                datahz[BYTENUMHZ-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[4]='0';
                datatime[5]='1';
                datatime[6]='0';
                datatime[7]='0';
                timeconvert(datatime, -10800);
//                xiaoshi_init(datahz,datatime,fdxsdata);
    			for(recnum=RECNUMHOUR,bytenum=BYTENUMHZ,seekset=BYTENUMHZ;recnum>0;recnum--)
        	    	{
        	    	datahz[0] = datatime[4];
        	    	datahz[1] = datatime[5];
        	    	datahz[2] = datatime[6];
        	    	datahz[3] = datatime[7];
        	    	fseek(fdxsdata, seekset, SEEK_SET);
        	    	fwrite(&datahz,sizeof(datahz),1,fdxsdata);
        	    	seekset += bytenum;
//					printf("datatime=%s,recnum=%d\n",datatime,recnum);
        	    	timeconvert(datatime, 3600);
        	    	}
                }

usbhz1:
//            memset(datahz, '/', sizeof(datahz));
//            datahz[BYTENUMHZ-2]='\r';
//            datahz[BYTENUMHZ-1]='\n';
            if((fdhour = fopen("/usr/hourzz.tmp","r")) == NULL)
				{
				fclose(fdxsdata);
                goto start31;
				}
            fread(&datahz,sizeof(datahz),1,fdhour);
            fclose(fdhour);
//			printf("datahz=%s\n",datahz);
            stringmove(ctime, 0, datatime, 0, 12);
            datahz[0] = datatime[4];
            datahz[1] = datatime[5];
            datahz[2] = datatime[6];
            datahz[3] = datatime[7];
            timeconvert(datatime, 10800);
            seekset = BYTENUMHZ * xiaoshi_bianhao(datatime) +  BYTENUMHZ;
//	    	printf("seekset=%ld\n",seekset);
            fseek(fdxsdata, seekset, SEEK_SET);
            fwrite(&datahz, sizeof(datahz), 1, fdxsdata);
            fclose(fdxsdata);


start31:
        	stringmove(ctime, 0, datatime, 0, 12);
        	timeconvert(datatime, 10800);
        	stringmove(datatime, 0, namehs, 12, 4);
        	if((fdxsdata=fopen(namehs,"r+")) == NULL )
        		{
        		if((fdxsdata=fopen(namehs,"w+")) == NULL)
					{
#ifndef USB
                    goto start4;
#else
					stringmove(datatime, 0, usbhs, 12, 4);
                    if((fdxsdata = fopen(usbhs,"r+")) == NULL )
                        {
                        if((fdxsdata = fopen(usbhs,"w+")) == NULL)
                            {
//                          fzsign = 1;
                            goto start4;
							}
						}
                    else
                        goto usbhs1;
#endif
					}	
//          	memset(datahs, '/', sizeof(datahs));
            	stationid(stpara);                                              // Station Id
            	stringmove(stpara, 0, hsfirst, 0, 5);
	            filetime(presenttime, 239);
    	        stringmove(presenttime, 0, hsfirst, 6, 4);
        	    stringmove(presenttime, 5, hsfirst, 13, 2);
	            stationlong(stpara);                                            // Station Long
//          	hsfirst[20] = 'E';
	            stringmove(stpara, 0, hsfirst, 15, 3);
    	        stringmove(stpara, 4, hsfirst, 18, 2);
        	    stringmove(stpara, 7, hsfirst, 20, 2);
            	stationlat(stpara);                                             // Station Lat
//          	hsfirst[28] = 'N';
	            stringmove(stpara, 0, hsfirst, 23, 2);
    	        stringmove(stpara, 3, hsfirst, 25, 2);
        	    stringmove(stpara, 6, hsfirst, 27, 2);
	            stationalt(stpara);                                             // Station Alt
    	        stringmove(stpara, 0, hsfirst, 30, 5);
        	    hsfirst[171] = senstate[SM1];                                   // Sensor Flags
	            hsfirst[175] = senstate[SM2];
    	        hsfirst[179] = senstate[SM3];
        	    hsfirst[183] = senstate[SM4];
	            hsfirst[187] = senstate[SM5];
    	        hsfirst[191] = senstate[SM6];
        	    hsfirst[195] = senstate[SM7];
	            hsfirst[199] = senstate[SM8];
    	        hsfirst[203] = senstate[WT];
        	    hsfirst[BYTENUMHS-2]='\r';
	            hsfirst[BYTENUMHS-1]='\n';
    	        fwrite(hsfirst, sizeof(hsfirst), 1, fdxsdata);

	            memset(datahs, '-', sizeof(datahs));
    	        datahs[BYTENUMHS-2]='\r';
        	    datahs[BYTENUMHS-1]='\n';
	            stringmove(ctime, 0, datatime, 0, 12);
    	        datatime[4]='0';
        	    datatime[5]='1';
	            datatime[6]='0';
    	        datatime[7]='0';
        	    timeconvert(datatime, -10800);
//          	xiaoshi_init(datahz,datatime,fdxsdata);
            	for(recnum=RECNUMHOUR,bytenum=BYTENUMHS,seekset=BYTENUMHS;recnum>0;recnum--)
                	{
                	datahs[0] = datatime[4];
	                datahs[1] = datatime[5];
    	            datahs[2] = datatime[6];
        	        datahs[3] = datatime[7];
            	    fseek(fdxsdata, seekset, SEEK_SET);
	                fwrite(&datahs,sizeof(datahs),1,fdxsdata);
    	            seekset += bytenum;
        	        timeconvert(datatime, 3600);
                	}
            	}

usbhs1:
//      	memset(datahz, '/', sizeof(datahz));
//      	datahz[BYTENUMHZ-2]='\r';
//      	datahz[BYTENUMHZ-1]='\n';
        	if((fdhour = fopen("/usr/hourss.tmp","r")) == NULL)
				{
				fclose(fdxsdata);
            	goto start4;
				}
	        fread(&datahs,sizeof(datahs),1,fdhour);
    	    fclose(fdhour);
        	stringmove(ctime, 0, datatime, 0, 12);
	        datahs[0] = datatime[4];
    	    datahs[1] = datatime[5];
        	datahs[2] = datatime[6];
	        datahs[3] = datatime[7];
    	    timeconvert(datatime, 10800);
        	seekset = BYTENUMHS * xiaoshi_bianhao(datatime) +  BYTENUMHS;
//      	printf("seekset=%ld\n",seekset);
	        fseek(fdxsdata, seekset, SEEK_SET);
    	    fwrite(&datahs, sizeof(datahs), 1, fdxsdata);
        	fclose(fdxsdata);
	    	}	    // if(min == 0)

start4:
        if(min == localt)
            {
            stringmove(rtime, 0, datatime, 0, 12);
            timeconvert(datatime, -3600);
            stringmove(datatime, 0, namehr, 12, 4);
            if((fdxsdata=fopen(namehr,"r+")) == NULL )
                {
                if((fdxsdata=fopen(namehr,"w+")) == NULL)
					{
#ifndef USB
                    goto txend;
#else
					stringmove(datatime, 0, usbhr, 12, 4);
                    if((fdxsdata = fopen(usbhr,"r+")) == NULL )
                        {
                        if((fdxsdata = fopen(usbhr,"w+")) == NULL)
                            {
//                          fzsign = 1;
                            goto txend;
							}
						}
                    else
                        goto usbhr1;
#endif
					}
//                memset(datahr, '/', sizeof(datahr));
                stationid(stpara);                                              // Station Id
                stringmove(stpara, 0, hrfirst, 0, 5);
                filetime(presenttime, td-1);                                    // Time
                stringmove(presenttime, 0, hrfirst, 6, 4);
                stringmove(presenttime, 5, hrfirst, 13, 2);
                stationlong(stpara);                                            // Station Long
//                hrfirst[20] = 'E';
                stringmove(stpara, 0, hrfirst, 15, 3);
                stringmove(stpara, 4, hrfirst, 18, 2);
                stringmove(stpara, 7, hrfirst, 20, 2);
                stationlat(stpara);                                             // Station Lat
//                hrfirst[28] = 'N';
                stringmove(stpara, 0, hrfirst, 23, 2);
                stringmove(stpara, 3, hrfirst, 25, 2);
                stringmove(stpara, 6, hrfirst, 27, 2);
                stationalt(stpara);                                             // Station Alt
                stringmove(stpara, 0, hrfirst, 30, 5);
                hrfirst[89] = senstate[GR];                                     // Sensor Flags
                hrfirst[94] = senstate[NR];
                hrfirst[99] = senstate[DR];
                hrfirst[104] = senstate[SR];
                hrfirst[109] = senstate[RR];
                hrfirst[114] = senstate[UR];
                hrfirst[119] = senstate[AR];
                hrfirst[124] = senstate[TR];
                hrfirst[129] = senstate[PR];
                hrfirst[134] = senstate[SSD];
                hrfirst[BYTENUMHR-2]='\r';
                hrfirst[BYTENUMHR-1]='\n';
//                fwrite(&hrfirst, sizeof(hrfirst), 1, fdfzdata);					// 090512 important
                fwrite(&hrfirst, sizeof(hrfirst), 1, fdxsdata);

                memset(datahr, '-', sizeof(datahr));
                datahr[BYTENUMHR-2]='\r';
                datahr[BYTENUMHR-1]='\n';
                stringmove(rtime, 0, datatime, 0, 12);
                datatime[4]='0';
                datatime[5]='1';
                datatime[6]='0';
                datatime[7]='0';
                timeconvert(datatime, 3600);
//                xiaoshi_init(datahz,datatime,fdxsdata);
                for(recnum=RECNUMHOUR,bytenum=BYTENUMHR,seekset=BYTENUMHR; recnum>0; recnum--)
                    {
                    datahr[0] = datatime[4];
                    datahr[1] = datatime[5];
                    datahr[2] = datatime[6];
                    datahr[3] = datatime[7];
//		            if(datatime[6]=='0' && datatime[7]=='0')
//                		{
//                		datahr[2] = '2';
//                		datahr[3] = '4';
//                		}
//            		else
//                		{
//                		datahr[2] = datatime[6];
//                		datahr[3] = datatime[7];
//                		}
                    fseek(fdxsdata, seekset, SEEK_SET);
                    fwrite(&datahr,sizeof(datahr),1,fdxsdata);
                    seekset += bytenum;
                    timeconvert(datatime, 3600);
                    }
                }

usbhr1:
//            memset(datahr, '/', sizeof(datahr));
//            datahr[BYTENUMHR-2]='\r';
//            datahr[BYTENUMHR-1]='\n';
            if((fdhour = fopen("/usr/hourrr.tmp","r")) == NULL)
				{
				fclose(fdxsdata);
            	goto txend;
				}
            fread(&datahr,sizeof(datahr),1,fdhour);
            fclose(fdhour);
            stringmove(rtime, 0, datatime, 0, 12);
            datahr[0] = datatime[4];
            datahr[1] = datatime[5];
            datahr[2] = datatime[6];
            datahr[3] = datatime[7];
//			if(datatime[6]=='0' && datatime[7]=='0')
//				{
//				datahr[2] = '2';
//              datahr[3] = '4';
//				}
//			else
//				{
//          	datahr[2] = datatime[6];
//            	datahr[3] = datatime[7];
//				}
            timeconvert(datatime, -3600);
            seekset = BYTENUMHR * xiaoshi_bianhao(datatime) +  BYTENUMHR;
//          printf("seekset=%ld\n",seekset);
            fseek(fdxsdata, seekset, SEEK_SET);
            fwrite(&datahr, sizeof(datahr), 1, fdxsdata);
            fclose(fdxsdata);
            }		// if(min == localt)

    txend:
        time(&timep);
        p = gmtime(&timep);
        sec  = p->tm_sec;
		sleept = 59 - sec; 
		sleep(sleept);
		}	//while(1)
    return 0;
    }

