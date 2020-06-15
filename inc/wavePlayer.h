#ifndef _WAVEPLAYER_H_
#define _WAVEPLAYER_H_
#include "hal.h"

#define EVT_I2S_ERR     EVENT_MASK(1)
#define EVT_I2S_ISR     EVENT_MASK(2)

void stopPlay(void);
void playFile(char *fpath);
void playerInit(void);
#endif