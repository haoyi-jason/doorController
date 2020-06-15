#include "ch.h"
#include "hal.h"
#include "ff.h"
#include "wavePlayer.h"
#include "codec_DAC.h"
#include <string.h>
#include "segger_rtt.h"

#define WAVE_BUUFER_SIZE        1024
#define PLAY_PRIO (NORMALPRIO+1)
#define HEADERMAX      128
#define DEBUG   FALSE

#define FORMAT_PCM      1
#define RIFF			0x46464952		// 'FIRR' = RIFF in Little-Endian format
#define WAVE			0x45564157		// 'WAVE' in Little-Endian
#define DATA			0x61746164		// 'data' in Little-Endian
#define FMT				0x20746D66		// 'fmt ' in Little-Endian

#define DEBUG_PRINT(...)  SEGGER_RTT_printf(0,__VA_ARGS__)
extern bool fs_ready;
static uint16_t wavBuffer[WAVE_BUUFER_SIZE];
static FIL file;
uint8_t bitsPerSample;
uint16_t sampleRate;
uint32_t bytesToPlay;

thread_t *playThread;
uint16_t bufferOffset;
FATFS SDC_FS;
FRESULT fres;
FIL sdFile;
DIR dir;
char fileName[32];
bool fs_ready = false;

static void i2scallback(I2SDriver *i2sp,  size_t offset, size_t n);
static const I2SConfig i2scfg = {
  wavBuffer,
  NULL,
  WAVE_BUUFER_SIZE,
  i2scallback,
  0,
  12
};

static void i2scallback(I2SDriver *i2sp, size_t offset, size_t n)
{
  (void)i2sp;
  (void)offset;
  (void)n;
  
  bufferOffset = offset;
  chSysLockFromISR();
  chEvtSignalI(playThread,EVT_I2S_ISR);
  chSysUnlockFromISR();
}


typedef struct _chunk{
  uint32_t id;
  uint32_t size;
}chunk;

typedef struct _RIFFHeader
{
	 chunk       descriptor;  // "RIFF"
	 uint32_t    type;        // "WAVE"
} RIFFHeader;

typedef struct _WAVEHeader
{
     chunk       descriptor;
     uint16_t    audioFormat;	// 1 - PCM
     uint16_t    numChannels;	// 1 - mono
     uint32_t    sampleRate;	//
     uint32_t    byteRate;		// byteRate = SampleRate * BlockAlign
     uint16_t    blockAlign;	// BlockAlign = bitsPerSample / 8 * NumChannels
     uint16_t    bitsPerSample;
} WAVEHeader;

typedef struct _DATAHeader
{
	chunk       descriptor;
} DATAHeader;

typedef struct _FILEHeader
{
    RIFFHeader  riff;
    WAVEHeader  wave;
} FILEHeader;

uint8_t fileSysInit()
{
  FRESULT err;

  sdcStart(&SDCD1,NULL);
  sdcConnect(&SDCD1);
  chThdSleepMilliseconds(100);
  err = f_mount(&SDC_FS,"0",1);
  if(err != FR_OK){
    sdcDisconnect(&SDCD1);
    return 0;
  }
  fs_ready = true;
  return 1;
}

void ffTest()
{
  UINT Size;
  fres = f_open(&sdFile,"test.txt",FA_WRITE | FA_CREATE_ALWAYS);
  if(fres != FR_OK){
    // error
  }else{
    f_write(&sdFile,"HELLO, WORLD!",13,&Size);
    f_close(&sdFile);
  }
    
}


static void i16_conv(uint16_t buf[], uint16_t len)
{
  for(uint16_t i=0;i<len;i++){
    buf[i] += 0x8000;
  }
}

static THD_WORKING_AREA(waPlayThread,1024);
static THD_FUNCTION(wavePlayerThread,arg)
{
  //char *fname = (char*)arg;
  (void)arg;
  UINT btr;
  FRESULT err;
  uint16_t *pbuffer;
  char b[512];
  
  pbuffer = wavBuffer;
  memset(pbuffer,0,WAVE_BUUFER_SIZE*sizeof(uint16_t));
  err= f_read(&file,pbuffer,WAVE_BUUFER_SIZE*sizeof(uint16_t),&btr);
  if(err != FR_OK) goto end;
  
  i2sStartExchange(&I2SD3);
  while(bytesToPlay){
    if(chThdShouldTerminateX()) break;
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    
    if(evt & EVT_I2S_ERR) break;
    if(evt & EVT_I2S_ISR){
      err = f_read(&file,(char*)(pbuffer+bufferOffset*sizeof(uint16_t)),(WAVE_BUUFER_SIZE),&btr);
//      err = f_read(&file,&b,(WAVE_BUUFER_SIZE),&btr);
      if(err != FR_OK) break;
      if(btr < (WAVE_BUUFER_SIZE>>1))
        memset((char*)(pbuffer + btr + bufferOffset),0,(WAVE_BUUFER_SIZE>>1)-btr);
      
      bytesToPlay -= btr;
    }
    
    if(!btr) break;
  }
  
end: 
  f_close(&file);
  i2sStopExchange(&I2SD3);
  playThread = NULL;
  chThdExit((msg_t)0);
  
}

void playFile(char *fpath)
{
  uint16_t headerLength;
  FRESULT err;
  UINT btr;
  
  if(!fs_ready){
    return;
  }
  
  while(playThread){
    stopPlay();
  }
  
  err = f_open(&file,fpath,FA_READ);
  if(err != FR_OK){
    DEBUG_PRINT("Fail to open file\n");
    return;
  }
  
  headerLength = sizeof(FILEHeader);
  err = f_read(&file,&wavBuffer,headerLength,&btr);
  if(err != FR_OK){
    DEBUG_PRINT("Read file error\n");
    return;
  }
  
  FILEHeader* header = (FILEHeader*)wavBuffer;
  if(!(header->riff.descriptor.id == RIFF
       && header->riff.type == WAVE
       && header->wave.descriptor.id == FMT
       && header->wave.audioFormat == FORMAT_PCM))
  {
    DEBUG_PRINT("Error: Not a PCM file\n");
    return;
  }
  
  DEBUG_PRINT("Nof Channel:%d\n",header->wave.numChannels);
  DEBUG_PRINT("Nof Bits per sample:%d\n",header->wave.bitsPerSample);
  
  sampleRate = header->wave.sampleRate;
  bitsPerSample = header->wave.bitsPerSample;
  
  DATAHeader *dataHeader = (DATAHeader*)&wavBuffer;
  headerLength += sizeof(DATAHeader);
  err = f_read(&file,&wavBuffer,sizeof(DATAHeader),&btr);
  if(err != FR_OK){
    DEBUG_PRINT("Error read data header\n");
    return;
  }
  
  while((uint32_t)dataHeader->descriptor.id != DATA){
    if(headerLength > HEADERMAX) break;
    if(dataHeader->descriptor.size > 0){
      headerLength += dataHeader->descriptor.size;
      f_lseek(&file,headerLength);
    }
    headerLength += sizeof(DATAHeader);
    err = f_read(&file, &wavBuffer,sizeof(DATAHeader),&btr);
    if(err != FR_OK){
      DEBUG_PRINT("Error read file name=%s, err=%d\n",fpath,err);
      return;
    }
  }
  
  if((uint32_t)dataHeader->descriptor.id != DATA){
    DEBUG_PRINT("Error: chunk not found\n");
    return;
  }
  
  DEBUG_PRINT("Ready to play wave file\n");
  
  bytesToPlay = dataHeader->descriptor.size;
  DEBUG_PRINT("Sample Size:%d\n",bytesToPlay);
  
  playThread = chThdCreateStatic(waPlayThread,sizeof(waPlayThread),PLAY_PRIO,wavePlayerThread,NULL);
   
}

void stopPlay(void)
{
  if(playThread){
    chThdTerminate(playThread);
    chThdWait(playThread);
    if(chThdTerminatedX(playThread))
      playThread = NULL;
  }
}

void playerInit(void)
{  
  fileSysInit();
  i2sStart(&I2SD3,&i2scfg);  
}
