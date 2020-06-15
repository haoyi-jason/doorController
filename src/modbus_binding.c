#include "ch.h"
#include "hal.h"

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbs.h"
#include "common/mbtypes.h"
#include "common/mbutils.h"
#include "common/mbportlayer.h"
#include "mb_reg_map.h"
#include "modbus_binding.h"
#include "sysparam.h"
#include "app_doorcontrol.h"

#if defined(APP_COMM_MBRTU) || defined(APP_COMM_MBTCP)

thread_t *thread;

#define IN_RANGE(a,x,b) ((a <=x) && (x <=b))

static virtual_timer_t vt;
static void blinker_cb(void *arg)
{
  chSysLockFromISR();
  //palSetPad(GPIOB,12);
  chSysUnlockFromISR();
}


STATIC eMBException eMyRegInputCB( UBYTE * pubRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  
}
STATIC eMBException eMyRegHoldingCB( UBYTE * pubRegBuffer, USHORT usAddress,
                                    USHORT usNRegs, eMBSRegisterMode eRegMode )
{
  eMBException eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
  USHORT adr = usAddress;
  USHORT adr_end = usAddress + usNRegs;
  int8_t szRet;
  //palClearPad(GPIOB,12);
  chVTSet(&vt,MS2ST(50),blinker_cb,NULL); 
  if(eRegMode == MBS_REGISTER_WRITE){
      for(uint16_t k=adr;k<adr_end;k++){
        for(uint8_t i=0;i<NOF_REG_FUNC;i++){
          if(IN_RANGE(mb_reg_map_func[i].regBegin,k,mb_reg_map_func[i].regEnd)){
            if(mb_reg_map_func[i].write){
              szRet = mb_reg_map_func[i].write(k - mb_reg_map_func[i].regBegin,pubRegBuffer);
              pubRegBuffer += (1 << szRet);
              if(szRet == 2) k++;
            }
          }
        }
      }
  }
  else if(eRegMode == MBS_REGISTER_READ){
//    if(adr == 999){
//    
//    }
//    else{
      for(uint16_t k=adr;k<adr_end;k++){
        for(uint8_t i=0;i<NOF_REG_FUNC;i++){
          if(IN_RANGE(mb_reg_map_func[i].regBegin,k,mb_reg_map_func[i].regEnd)){
            if(mb_reg_map_func[i].read){
              szRet = mb_reg_map_func[i].read(k - mb_reg_map_func[i].regBegin,pubRegBuffer);
              //adr += szRet;
              pubRegBuffer += (1 << szRet);
              if(szRet == 2) k++;
              continue;
            }
          }
        }
      }
    }
//  }
  
  return MB_PDU_EX_NONE;
}
STATIC eMBException eMyDiscInputCB( UBYTE * pubRegBuffer, USHORT usAddress, USHORT usNRegs )
{
  
}

#define MBS_LISTEN_ADDRESS              "0.0.0.0"       /* Bind on all addresses */
#define MBS_LISTEN_PORT                 502

static THD_WORKING_AREA(waModbusBinding,512);
static THD_FUNCTION(procModbusBinding ,p)
{
  
    eMBErrorCode    eStatus,eStatus2;
    xMBSHandle      xMBSHdl;
    chVTObjectInit(&vt);
#ifdef APP_COMM_MBTCP
    if( MB_ENOERR != ( eStatus2 = eMBSTCPInit( &xMBSHdl, MBS_LISTEN_ADDRESS, MBS_LISTEN_PORT ) ) )
    {
      eStatus = eStatus2;
    }
    else if( MB_ENOERR != ( eStatus = eMBSRegisterHoldingCB( xMBSHdl, eMyRegHoldingCB ) ) )
    {
        ( void )eMBSClose( xMBSHdl );
    }
    else{
      do
      {
          /* Poll the communication stack. */
          eStatus = eMBSPoll( xMBSHdl );
          chThdSleepMilliseconds(50);
      }
      while( MB_ENOERR == eStatus && !chThdShouldTerminateX() );
      ( void )eMBSClose( xMBSHdl );
    }
#endif    
#ifdef APP_COMM_MBRTU
    if( MB_ENOERR !=
          ( eStatus =
            eMBSSerialInit( &xMBSHdl, MB_RTU, 0x1,
                            0, moduleParam.serial.baudrate_val, moduleParam.serial.parity ) ) )
      {
        
      }
      else if( MB_ENOERR != ( eStatus = eMBSRegisterHoldingCB( xMBSHdl, eMyRegHoldingCB ) ) )
      {
          ( void )eMBSClose( xMBSHdl );
      }
      else
      {
          do
          {
              /* Poll the communication stack. */
              eStatus = eMBSPoll( xMBSHdl );
              chThdSleepMilliseconds(50);
          }
          while( MB_ENOERR == eStatus && !chThdShouldTerminateX() );
          ( void )eMBSClose( xMBSHdl );
      }
#endif

//    if( MB_ENOERR !=
//          ( eStatus =
//            eMBSSerialInit( &xMBSHdl, MB_RTU, moduleParam.serial.slave_address,
//                            0, moduleParam.serial.baudrate_val, moduleParam.serial.parity ) ) )
//      {
//      }
//      else if( MB_ENOERR != ( eStatus = eMBSRegisterInputCB( xMBSHdl, eMyRegInputCB ) ) )
//      {
//          ( void )eMBSClose( xMBSHdl );
//      }
//      else if( MB_ENOERR != ( eStatus = eMBSRegisterHoldingCB( xMBSHdl, eMyRegHoldingCB ) ) )
//      {
//          ( void )eMBSClose( xMBSHdl );
//      }
//      else if( MB_ENOERR != ( eStatus = eMBSRegisterDiscreteCB( xMBSHdl, eMyDiscInputCB ) ) )
//      {
//          ( void )eMBSClose( xMBSHdl );
//      }
//      else
//      {
//          do
//          {
//              /* Poll the communication stack. */
//              eStatus = eMBSPoll( xMBSHdl );
//              chThdSleepMilliseconds(5);
//          }
//          while( MB_ENOERR == eStatus && !chThdShouldTerminateX() );
//          ( void )eMBSClose( xMBSHdl );
//      }
    
    chThdExit(MSG_OK);
}


void modbusBindingInit()
{
  vMBPInit(  );
  thread = chThdCreateStatic(waModbusBinding,sizeof(waModbusBinding),NORMALPRIO,procModbusBinding,NULL);
}

void modbusBindingStop()
{
  if(thread){
    chThdTerminate(thread);
    chThdWait(thread);
    //chThdRelease(thread);
    thread = NULL;
  }
}

#endif