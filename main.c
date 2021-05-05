#include <stdbool.h>
#include <stdio.h>
#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"

osThreadId_t tid_vControl;
osThreadId_t tid_sOnBoard;
osThreadId_t tid_sRadioFreq;
osThreadId_t tid_sLaser;
osThreadId_t tid_sUltra;        
osThreadId_t tid_GateKeeper;

osMessageQueueId_t mqid_sRead;
osMessageQueueId_t mqid_vControl;


const osThreadAttr_t vControl_attr = {
  .name = "Vehicle Control Thread"
};

const osThreadAttr_t sOnBoard_attr = {
  .name = "On-Board Camera Sensor Thread"
};

const osThreadAttr_t sRadioFreq_attr = {
  .name = "Radio Frequency Sensor Thread"
};

const osThreadAttr_t sLaser_attr = {
  .name = "Laser Sensor Thread"
};

const osThreadAttr_t sUltra_attr = {
  .name = "Ultrasound Sensor Thread"
};

const osThreadAttr_t Gatekeeper_attr = {
  .name = "Gatekeeper Thread"
};

typedef struct structSensor{
  uint8_t sensor;
  uint32_t delay;
  uint32_t flag;
} scructSensor;

//----------
// UART definitions
extern void UARTStdioIntHandler(void);

void UARTInit(void){
  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 9600, SystemCoreClock);

  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
} // UARTInit

void UART0_Handler(void){
  UARTStdioIntHandler();
} // UART0_Handler
//----------


__NO_RETURN void vControl(void *arg){
  uint32_t v_aceleration = 0;
  uint32_t v_rotation = 0; 
  
  while(1){
    v_aceleration++;
    
    if(v_aceleration == 5){
      v_aceleration = 0;
    }
    
    UARTprintf("A%d;\n", v_aceleration);  
        
    for(int d = 0; d < 10000; d++); 
    
    v_rotation = 15 * v_aceleration;

    UARTprintf("V%d;\n", v_rotation );

    osDelay(1000); 
  } // while
} // vControl


void sRead(void *arg){
  uint32_t tick;
  structSensor* selectedSensor = (structSensor*) arg;
  uint8_t sensor = selectedSensor->sensor;
  uint32_t delay = selectedSensor->delay;
  uint32_t flag_gk = selectedSensor->flag_gk;
  uint32_t flag_con = seletedSensor->flag_con;
  
  
  while(1){
  tick = osKernelGetTickCount();
  
  osThreadFlagSet(tid_Gatekeeper, flag_gk);
    
  osMessageQueueGet(mqid_sRead, msg, NULL, 0U);
  
  if(status==osOK){
    switch(sensor){
    case sLaser:
      global_sLaser = msg;
      break;
    case sUltra:
      global_sUltra = msg;
      break;
    case sOnBoard:
      global_sOnboard = msg;
      break;
    case sRadioFreq:
      global_sRadioFreq = msg;
      break;
    case default: 
      break;
    }
    osThreadFlagSet(tid_vControl, flag_con);
  }
  osDelayUntil(tick + delay);
  }     // while
} // sRead






               
void main(void){
  UARTInit();
  
  structSensor sOnBoard, sRadioFreq, sLaser, sUltra;
  
  
  
  osKernelInitialize();
  tid_vControl = osThreadNew(vControl, NULL, &vControl_attr);
  tid_sOnBoard = osThreadNew(sRead, &sOnBoard, &sOnBoard_att);     
  tid_sRadioFreq = osThreadNew(sRead, &sRadioFreq, &sRadioFreq_att);
  tid_sLaser = osThreadNew(sRead, &sLaser, &sLaser_att);
  tid_sUltra = osThreadNew(sRead, &sUltra, &sUltra_att);        
  tid_GateKeeper = osThreadNew(Gatekeeper, NULL, &Gatekeeper_att);


  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
} 
