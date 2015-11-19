#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>

extern bool verbose_flag;

#define debug(M, args...) printf("DEBUG %s:%d: " M "\n", __FILE__, __LINE__, args)
#define info(M, ...) {\
  syslog(LOG_INFO, "INFO %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
  if (verbose_flag)\
    printf("INFO %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
}
#define warn(M, ...) {\
  syslog(LOG_INFO, "WARN %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
  if (verbose_flag)\
    printf("WARN %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
}
#define error(M, ...) {\
  syslog(LOG_ERR, "ERROR %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
  if (verbose_flag)\
    printf("ERROR %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__);\
}
#define errcheck(E) if(E) {printf("Error at %s:%d: code = %i\n", __FILE__, __LINE__, E);}
#define errguard(E, F) if(!E) {E = (F); errcheck(E);}

inline int idivup(int a, int b) {
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

#endif // __UTIL_H__
