//weather-sjwj 2014-07-29 13:00 V6.5
//source from "weather-sjwj 2013-06-08 15:00 V6.4"

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

#define P       0       
#define T0      1
#define TD      2
#define U       3
#define WD      4
#define WS      5
#define RAT     6
#define RAW     7
#define VI      8

#define RECNUMHOUR	744							// 24*31
#define RECNUMMIN	1440						// 60*24
#define BYTENUMMZ 	255
#define SAMPLE120 	126
#define SAMPLE150	126	//156
#define SAMPLE180	186
#define SAMPLE240 	246
#define SAMPLE960 	966	//1206

//#define EN090423 1

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

static int fenzhong_bianhao(char *time)
    {
    int temp1, temp2, id;
    temp1 = (time[6] - '0') * 10 + (time[7] - '0');
    temp2 = (time[8] - '0') * 10 + (time[9] - '0');
    id = temp1 * 60 + temp2;
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
    FILE *fdfzdata, *fdmin;
    struct rtc_time rtc_tm, rtc_so;
    char ctime[13], datatime[13];
    char datamz[BYTENUMMZ];
	char datamz0[BYTENUMMZ];
	char datamz1[BYTENUMMZ-4];
    char samp120[SAMPLE120], samp150[SAMPLE150], samp180[SAMPLE180], samp240[SAMPLE240], samp960[SAMPLE960];
    int recnum, bytenum;
    long seekset;
    int sleept;

    time_t timep;
    struct tm *p;
    int sec, min, hour, mday, mon, year;
    char paramete[8];

    char namemz[]={"/tmp/data/GGYYMMDD.DAT"};

	char namet0[]={"/tmp/samp/T0YYMMDD.DAT"};
    char nameuu[]={"/tmp/samp/UUYYMMDD.DAT"};
    char namepp[]={"/tmp/samp/PPYYMMDD.DAT"};
    char namewd[]={"/tmp/samp/WDYYMMDD.DAT"};
    char namews[]={"/tmp/samp/WSYYMMDD.DAT"};

	char removemz[]={"rm /tmp/data/GGYYMMDD.DAT"};

    char instime[]={"date 123120002011.00"};
    char oldtime[]={"date 123120002011.00"};
	int nowtime, lasttime, nowyear, lastyear;

    char presenttime[19], stpara[12];
	char senstate[75]={"1111101011111111111111111111110000000001010000000000111111111000000000000\r\n"};

	char mzfirst[BYTENUMMZ]={"SH001 2009   01   010000000E000000N    0   15  100   153111111111V1.00---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\r\n"};

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
    printf("weather-sjwj 2017-11-01 11:00 V0.1\n");

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

	if(sec == 3)
//        if(sec == 8)    // Changed by XGR at 2013-06-08		
	    	{
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

start11:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namemz, 12, 6);
            if((fdfzdata = fopen(namemz,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namemz,"w+")) == NULL)
                    {
                    goto start12;
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
		mzfirst[56] = senstate[P];										// Sensor Flags
                mzfirst[57] = senstate[T0];
                mzfirst[58] = senstate[TD];
                mzfirst[59] = senstate[U];
                mzfirst[60] = senstate[WD];
                mzfirst[61] = senstate[WS];
                mzfirst[62] = senstate[RAT];
                mzfirst[63] = senstate[RAW];
                mzfirst[64] = senstate[VI];
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

                stringmove(ctime, 0, datatime, 0, 12);
                timeconvert(datatime, -31536000);                           // 60*60*24*365
                stringmove(datatime, 0, removemz, 15, 6);
                printf("%s\n",removemz);
                system(removemz);             
				}

            memset(datamz, '/', sizeof(datamz));
            datamz[BYTENUMMZ-2]='\r';
            datamz[BYTENUMMZ-1]='\n';
//            datamz[396]='\0';
	
	    	if((fdmin = fopen("/usr/fzsj.tmp","r")) == NULL)
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
            stringmove(datatime, 0, namet0, 12, 6);
            if((fdfzdata = fopen(namet0,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namet0,"w+")) == NULL)
                    {
                    goto start21;
                    }
                memset(samp150, '-', sizeof(samp150));
                samp150[SAMPLE150-2]='\r';
                samp150[SAMPLE150-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE150,seekset=0; recnum>0; recnum--)
                    {
                    samp150[0] = datatime[6];
                    samp150[1] = datatime[7];
                    samp150[2] = datatime[8];
                    samp150[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp150, sizeof(samp150), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/sampt0.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start21;
				}
            fread(&samp150,sizeof(samp150),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp150[0] = datatime[6];
            samp150[1] = datatime[7];
            samp150[2] = datatime[8];
            samp150[3] = datatime[9];
            timeconvert(datatime, 14340);
//			timeconvert(datatime, -60);
            seekset = SAMPLE150 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp150, sizeof(samp150), 1, fdfzdata);
            fclose(fdfzdata);

start21:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, nameuu, 12, 6);
            if((fdfzdata = fopen(nameuu,"r+")) == NULL )
                {
                if((fdfzdata = fopen(nameuu,"w+")) == NULL)
                    {
                    goto start22;
                    }
                memset(samp120, '-', sizeof(samp120));
                samp120[SAMPLE120-2]='\r';
                samp120[SAMPLE120-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
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
            if((fdmin = fopen("/usr/sampuu.tmp","r")) == NULL)
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
            timeconvert(datatime, 14340);
//            timeconvert(datatime, -60);
            seekset = SAMPLE120 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp120, sizeof(samp120), 1, fdfzdata);
            fclose(fdfzdata);

start22:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namepp, 12, 6);
            if((fdfzdata = fopen(namepp,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namepp,"w+")) == NULL)
                    {
                    goto start23;
                    }
                memset(samp180, '-', sizeof(samp180));
                samp180[SAMPLE180-2]='\r';
                samp180[SAMPLE180-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
                datatime[7]='0';
                datatime[8]='0';
                datatime[9]='1';
                for(recnum=RECNUMMIN,bytenum=SAMPLE180,seekset=0; recnum>0; recnum--)
                    {
                    samp180[0] = datatime[6];
                    samp180[1] = datatime[7];
                    samp180[2] = datatime[8];
                    samp180[3] = datatime[9];
                    fseek(fdfzdata, seekset, SEEK_SET);
                    fwrite(&samp180, sizeof(samp180), 1, fdfzdata);
                    seekset += bytenum;
                    timeconvert(datatime, 60);
                    }
				}
            if((fdmin = fopen("/usr/samppp.tmp","r")) == NULL)
				{
				fclose(fdfzdata);
                goto start23;
				}
            fread(&samp180,sizeof(samp180),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp180[0] = datatime[6];
            samp180[1] = datatime[7];
            samp180[2] = datatime[8];
            samp180[3] = datatime[9];
            timeconvert(datatime, 14340);
//            timeconvert(datatime, -60);
            seekset = SAMPLE180 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp180, sizeof(samp180), 1, fdfzdata);
            fclose(fdfzdata);

start23:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namewd, 12, 6);
            if((fdfzdata = fopen(namewd,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namewd,"w+")) == NULL)
                    {
                    goto start24;
                    }
                memset(samp240, '-', sizeof(samp240));
                samp240[SAMPLE240-2]='\r';
                samp240[SAMPLE240-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
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
                goto start24;
				}
            fread(&samp240,sizeof(samp240),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
//			printf("time=%s\n",datatime);
            samp240[0] = datatime[6];
            samp240[1] = datatime[7];
            samp240[2] = datatime[8];
            samp240[3] = datatime[9];
            timeconvert(datatime, 14340);
//            timeconvert(datatime, -60);
            seekset = SAMPLE240 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp240, sizeof(samp240), 1, fdfzdata);
            fclose(fdfzdata);

start24:
            stringmove(ctime, 0 , datatime, 0, 12);
            timeconvert(datatime, 14340);
            stringmove(datatime, 0, namews, 12, 6);
            if((fdfzdata = fopen(namews,"r+")) == NULL )
                {
                if((fdfzdata = fopen(namews,"w+")) == NULL)
                    {
                    goto start25;
                    }
                memset(samp960, '-', sizeof(samp960));
                samp960[SAMPLE960-2]='\r';
                samp960[SAMPLE960-1]='\n';
                stringmove(ctime, 0, datatime, 0, 12);
                datatime[6]='2';
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
                goto start25;
				}
            fread(&samp960,sizeof(samp960),1,fdmin);
            fclose(fdmin);
            stringmove(ctime, 0, datatime, 0, 12);
            samp960[0] = datatime[6];
            samp960[1] = datatime[7];
            samp960[2] = datatime[8];
            samp960[3] = datatime[9];
            timeconvert(datatime, 14340);
//            timeconvert(datatime, -60);
            seekset = SAMPLE960 * fenzhong_bianhao(datatime);
//          printf("seekset=%ld\n",seekset);
            fseek(fdfzdata, seekset, SEEK_SET);
            fwrite(&samp960, sizeof(samp960), 1, fdfzdata);
            fclose(fdfzdata);

start25:
			usleep(1);
            fd_rtc = open(DEV_RTC, O_RDONLY);
            if(fd_rtc == -1)
                {
                perror("Open RTC error!\n");
                goto start3;
                }
            retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_so);
            if(retval == -1)
                {
                perror("RTC_RD_TIME ioctl");
                close(fd_rtc);
                goto start3;
                }
            close(fd_rtc);

			usleep(100);
	    	fd_rtc = open(DEV_RTC, O_RDONLY);
    		if(fd_rtc == -1) 
				{
        		perror("Open RTC error!\n");
        		goto start3;
    			}
	    	retval = ioctl(fd_rtc, RTC_RD_TIME, &rtc_tm);
    		if(retval == -1) 
				{
        		perror("RTC_RD_TIME ioctl");
				close(fd_rtc);
        		goto start3;
    			}
			close(fd_rtc);

			if(rtc_tm.tm_year==rtc_so.tm_year && rtc_tm.tm_mon==rtc_so.tm_mon && rtc_tm.tm_mday==rtc_so.tm_mday && rtc_tm.tm_hour==rtc_so.tm_hour && rtc_tm.tm_min==rtc_so.tm_min)
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
			}

			usleep(1);
	    	}    //if(rtc_tm.tm_sec == 0)
		else
        	{
	    	sleep(1);
	    	goto start1;
	    	}	    

start3:	
		usleep(1);

    txend:
        time(&timep);
        p = gmtime(&timep);
        sec  = p->tm_sec;
		sleept = 59 - sec; 
		sleep(sleept);
		}	//while(1)
    return 0;
    }

