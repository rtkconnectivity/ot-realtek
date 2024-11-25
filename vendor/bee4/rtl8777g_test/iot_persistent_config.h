#include "flash_map.h"

#define ZIGBEE_PER_ADDR                 BKP_DATA1_ADDR
#define ZIGBEE_PER_SIZE                 0x00000000
#define THREAD_PER_ADDR                 ZIGBEE_PER_ADDR + ZIGBEE_PER_SIZE
#define THREAD_PER_SIZE                 0x00002000
#define KVS_ADDR                        THREAD_PER_ADDR + THREAD_PER_SIZE
#define KVS_SIZE                        0x00008000
