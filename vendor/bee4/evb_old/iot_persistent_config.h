#include "flash_map.h"

#define ZIGBEE_PER_ADDR                 BKP_DATA1_ADDR
#define ZIGBEE_PER_SIZE                 0x00000000
#define THREAD_PER_ADDR                 FTL_ADDR + 0x0000E000
#define THREAD_PER_SIZE                 0x00002000
#define KVS_ADDR                        FTL_ADDR
#define KVS_SIZE                        0x0000E000
