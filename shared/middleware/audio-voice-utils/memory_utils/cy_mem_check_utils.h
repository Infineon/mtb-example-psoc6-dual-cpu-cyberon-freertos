
#ifndef CY_MEM_CHECK_UTILS_H__
#define CY_MEM_CHECK_UTILS_H__

#include "cy_log.h"

#ifdef ENABLE_MEMORY_ALLOC_CHECK
#define CY_MEM_UTIL_PRINT_ADDR(__ID__,__ADDR__,__SIZE__) cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "[MEMCHECK] %s, addr:%p, size:%d, func:%s, line:%d \r\n",__ID__,__ADDR__,__SIZE__,__FUNCTION__,__LINE__);
#else
#define CY_MEM_UTIL_PRINT_ADDR(__ID__,__ADDR__,__SIZE__)
#endif

long malloc_info_command( void );
void cy_mem_get_allocated_memory(char* str);

#endif /* CY_MEM_CHECK_UTILS_H__ */
