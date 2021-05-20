#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"

//#define limitePista 15
#define BUFFER_SIZE 24

void enviaUART(char string[]);
void UARTInit(void);
void UART0_Handler(void);
void initObjects(void);
//void tratamentoDados(char buffer[BUFFER_SIZE]);
void enviaUART(char string[]);
void requisitaRfreq(void);
void requisitaLaser(void);


osThreadId_t tid_vDecision;
osThreadId_t tid_vControl;
osThreadId_t tid_recebeUart;
osThreadId_t tid_requisitaSensor;
osMutexId_t mutex_uart;
osMessageQueueId_t inputControl;
osMessageQueueId_t outputControl;

typedef enum tipoSensor
{
  LASER,
  RFREQ,
  NULO
} tipoSensor;

typedef enum tipoFlag
{
  OBJETO_DETECTADO,
  SAIU_DIREITA,
  SAIU_ESQUERDA,
  CAMINHO_LIVRE,
  SAIU_DO_LIMITE,
  SAIU_DIREITA_MUITO,
  SAIU_ESQUERDA_MUITO
} tipoFlag;


typedef struct structSensor
{
  tipoSensor tipo;
  float valor;
} structSensor;

typedef struct decisaoCarro
{
  char comando[20];
  uint8_t nro;
  uint8_t delay;
} decisaoCarro;

void UARTInit(void)
{
  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 115200, SystemCoreClock);

  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
} // UARTInit

void UART0_Handler(void)
{
  uint32_t status = UARTIntStatus(UART0_BASE, true);
  UARTIntClear(UART0_BASE, status);
  osThreadFlagsSet(tid_recebeUart, 0x0001);
} // UART0_Handler
//----------



void recebeUart(void *arg)
{
  char buffer[BUFFER_SIZE];
  char dado[BUFFER_SIZE];
  uint8_t i = 0;
  uint8_t charRecebido = 0;
  structSensor sensor;
  uint8_t flagValido = 0;

  while(1)
  {
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    osDelay(200);
    UARTFlushRx();
    
    do
    {
      charRecebido = UARTCharGet(UART0_BASE);
      buffer[i] = charRecebido;
      i++;
    } while(UARTCharsAvail(UART0_BASE));
    
//   osMutexRelease(mutex_uart);
   
   switch(buffer[1])
   {
   case 'r':     
     for(int i=3; i< strlen(buffer); i++)
    {
          dado[i-3] = buffer[i];
    }
    sensor.valor = strtod(dado,NULL);
    sensor.tipo = RFREQ;
    flagValido = 1;
    break; 
   default:
     break;
   }
   
   if(flagValido == 1)
   osMessageQueuePut(inputControl,&sensor,0,NULL);
    
   i = 0;
   memset(buffer, 0, BUFFER_SIZE);
  }
}

void threadvControl(void *arg)
{
  osStatus_t status;
  tipoFlag flag;
  structSensor sensor;
  uint8_t limitePista = 15;
  flag = CAMINHO_LIVRE;
  
  while(1)
  {
  status = osMessageQueueGet(inputControl, &sensor, NULL, NULL);
  if(status == osOK)
  {
    switch(sensor.tipo)
    {
    case RFREQ:
      if(sensor.valor >= limitePista)
      {
        flag = SAIU_DO_LIMITE;
      }
      else if(sensor.valor >= 0.5)
      {
        flag = SAIU_ESQUERDA;
      }
      else if(sensor.valor <= -0.5)
      {
        flag = SAIU_DIREITA;
      }
      else
      {
        flag = CAMINHO_LIVRE;
      }
      break;   
    case NULO:
        flag = CAMINHO_LIVRE;
      break;
    default:
        flag = CAMINHO_LIVRE;
      break;
  }
  osMessageQueuePut(outputControl,&flag,0,NULL);
  }
  osDelay(100);
}
}

void threadvDecision(void *arg)
{
  osStatus_t  status;
  tipoFlag flag;
  int j = 0, q = 0;
  osMutexAcquire(mutex_uart,osWaitForever);
  enviaUART("A2;");
  UARTFlushTx(false);
  osMutexRelease(mutex_uart); 
  osDelay(2500);
   
  while(1)
  {
    status = osMessageQueueGet(outputControl,&flag,NULL,NULL);
    if(status == osOK)
    {
      switch(flag)
      {
        case SAIU_DO_LIMITE:
          osMutexAcquire(mutex_uart,osWaitForever);
          enviaUART("S;");
          enviaUART("A-5;");
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
          break;
        case SAIU_DIREITA:
          osMutexAcquire(mutex_uart,osWaitForever);
          enviaUART("V0.5;");
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
          break;
        case SAIU_ESQUERDA:
          osMutexAcquire(mutex_uart,osWaitForever);
          enviaUART("V-0.5;");
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
          break;
        case CAMINHO_LIVRE:
          enviaUART("A0;");
          break;
      default:
          break;
      } 
      }        
  } // while
} // if


void requisitaSensor(void *arg)
{  
  while(1)
  {
        
    osMutexAcquire(mutex_uart,osWaitForever);
    enviaUART("Prf;");
    UARTFlushTx(false);
    osMutexRelease(mutex_uart);    
    //      if(i==0) enviaUART("Pl;");
    osDelay(200);    
  } 
}

void enviaUART(char string[])
{
   for(int i = 0; i < strlen(string); i++)
   {
     UARTCharPut(UART0_BASE,string[i]);
   }
}

void initObjects(void)
{
  inputControl = osMessageQueueNew(20, sizeof(structSensor), NULL);

  outputControl = osMessageQueueNew(20, sizeof(tipoFlag), NULL);
  
  mutex_uart = osMutexNew(NULL);

  tid_vControl = osThreadNew(threadvControl, NULL, NULL);

  tid_vDecision = osThreadNew(threadvDecision, NULL, NULL);

  tid_recebeUart = osThreadNew(recebeUart, NULL, NULL);
  
  tid_requisitaSensor = osThreadNew(requisitaSensor, NULL,NULL);
  
}

void main(void)
{
  UARTInit();
  osKernelInitialize();
  initObjects();

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
}
