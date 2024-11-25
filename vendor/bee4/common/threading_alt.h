#ifndef _THREADING_ALT_H
#define _THREADING_ALT_H

typedef struct
{
    void *mutex;
    int flag;
} mbedtls_threading_mutex_t;

#endif
