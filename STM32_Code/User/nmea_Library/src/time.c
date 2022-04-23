/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 */

/*! \file time.h */

#include "nmea/time.h"

#ifdef NMEA_WIN
#   pragma warning(disable: 4201)
#   pragma warning(disable: 4214)
#   pragma warning(disable: 4115)
#   include <windows.h>
#   pragma warning(default: 4201)
#   pragma warning(default: 4214)
#   pragma warning(default: 4115)
#else
#   include <time.h>
#endif





#ifdef NMEA_WIN

void nmea_time_now(nmeaTIME *stm)
{
    SYSTEMTIME st;

    GetSystemTime(&st);

    stm->year = st.wYear - 1900;
    stm->mon = st.wMonth - 1;
    stm->day = st.wDay;
    stm->hour = st.wHour;
    stm->min = st.wMinute;
    stm->sec = st.wSecond;
    stm->hsec = st.wMilliseconds / 10;
}

#else /* NMEA_WIN */

//modify by fire 这个函数用于产生系统时间,可以通过RTC为它设置时间，现在设置为默认值
void nmea_time_now(nmeaTIME *stm)
{
//    time_t lt ;
//   struct tm *tt;

    //time(&lt);          //modify by fire 
   // tt = gmtime(&lt);

//    stm->year = tt->tm_year;
//    stm->mon = tt->tm_mon;
//    stm->day = tt->tm_mday;
//    stm->hour = tt->tm_hour;
//    stm->min = tt->tm_min;
//    stm->sec = tt->tm_sec;
//    stm->hsec = 0;
    stm->year = 2014-1900;
    stm->mon =  0;
    stm->day = 1;
    stm->hour = 0;
    stm->min = 0;
    stm->sec = 0;
    stm->hsec = 0;
}




#endif
