#include "ch.h"
#include "hal.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/sockets.h"
#include "secsTask.h"


thread_t *thdSecs;
thread_reference_t secs_trp = NULL;



static THD_WORKING_AREA(waSecsTask,2048);
static THD_FUNCTION(procSecs ,p)
{
  (void*)p;
  
  
  
}


void secsTaskInit(void)
{
  
  
}