
//#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef APP_H
#define APP_H

#define NVIC_DFLT_PRIORITY 1u

extern int sg3_init(void);
extern void sg3_start(void);

#endif /* #ifndef APP_H */
