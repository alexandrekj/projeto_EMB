
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

    do
    {
      UARTFlushRx();
      charRecebido = UARTCharGet(UART0_BASE);
      buffer[i] = charRecebido;
      i++;
    } while(UARTCharsAvail(UART0_BASE));

   switch(buffer[0])
   {
   case 'f':     
     for(int i=1; i< strlen(buffer); i++)
    {
          dado[i-1] = buffer[i];
    }
    sensor.valor = atof(dado);
    sensor.tipo = RFREQ;
    flagValido = 1;
    break;
   case '0':
     sensor.valor = 0;
     sensor.tipo = LASER;
     flagValido = 1;
     break;
   case '-':
     if(buffer[1] == 1)
     {
     sensor.valor = -1;
     sensor.tipo = LASER;
     flagValido = 1;
     } else
     {
       flagValido = 0;
     }
     break;
   default:
     flagValido = 0;
     break;
   }
   
   if(flagValido == 1) 
   osMessageQueuePut(inputControl,&sensor,0,NULL);
    
   sensor.valor = NULL;
   sensor.tipo = NULO;
    
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
  
  while(1)
  {
  status = osMessageQueueGet(inputControl, &sensor, NULL, NULL);
  if(status == osOK)
  {
    if(sensor.tipo == LASER && sensor.valor == 0)
    {
      flag = CAMINHO_LIVRE;
    }
    else if(sensor.tipo == RFREQ && sensor.valor >= limitePista)
    {
      flag = SAIU_DO_LIMITE;
    }
    else if(sensor.tipo == RFREQ && sensor.valor < -5)
    {
      flag = SAIU_DIREITA;
    }
    else if(sensor.tipo == RFREQ && sensor.valor > 5)
    {
      flag = SAIU_ESQUERDA;
    } 
    else if(sensor.tipo == LASER && sensor.valor == 3)
    {
      flag = OBJETO_DETECTADO;
    }
  }
  osMessageQueuePut(outputControl,&flag,0,NULL);
  osDelay(250);
  }
}

void threadvDecision(void *arg)
{
  osStatus_t  status;
  decisaoCarro decisao;
  char decisaoParcial[BUFFER_SIZE];
  tipoFlag flag;

  while(1)
  {
    status = osMessageQueueGet(outputControl,&flag,NULL,NULL);
    if(status == osOK)
    {
      switch(flag)
      {
        case OBJETO_DETECTADO:
          decisao.nro = 5;
          decisao.comando[1]='E';
          decisao.comando[2]=';';
          decisao.comando[3]='V';
          decisao.comando[4]='9';
          decisao.comando[5]='0';
          decisao.comando[6]=';';
          decisao.comando[7]='A';
          decisao.comando[8]='2';
          decisao.comando[9]=';';
          decisao.comando[10]='E';
          decisao.comando[11]=';';
          decisao.comando[12]='V';
          decisao.comando[13]='-';
          decisao.comando[14]='9';
          decisao.comando[15]='0';
          decisao.comando[16]=';';
          break;
        case SAIU_DO_LIMITE:
          decisao.nro = 1;
          decisao.comando[1]='A';
          decisao.comando[2]='-';
          decisao.comando[3]='5';
          decisao.comando[4]=';';
          break;
        case SAIU_DIREITA:
          decisao.nro = 2;
          decisao.comando[1]='V';
          decisao.comando[2]='-';
          decisao.comando[3]='3';
          decisao.comando[4]='0';
          decisao.comando[5]=';';
          decisao.comando[6]='A';
          decisao.comando[7]='2';
          decisao.comando[8]=';';
          break;
        case SAIU_ESQUERDA:
          decisao.nro = 2;
          decisao.comando[1]='V';
          decisao.comando[2]='-';
          decisao.comando[3]='3';
          decisao.comando[4]='0';
          decisao.comando[5]=';';
          decisao.comando[1]='A';
          decisao.comando[2]='2';
          decisao.comando[3]=';';
          break;
        case CAMINHO_LIVRE:
          decisao.nro = 1;
          decisao.comando[1]='A';
          decisao.comando[2]='2';
          decisao.comando[3]=';';
      }
            
        if(decisao.nro == 5)
        {
            for(int i=0; i <= decisao.nro-3; i++)
            {
                decisaoParcial[i]=decisao.comando[i];
            }
          osMutexAcquire(mutex_uart,osWaitForever);
          enviaUART(decisaoParcial);
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
          osDelay(1000);

            int j = 0;
            for(int i=decisao.nro-3; i<= decisao.nro; i++)
            {
                  decisaoParcial[j]=decisao.comando[i];
                  j++;
            }
          osMutexAcquire(mutex_uart,osWaitForever);
          enviaUART(decisaoParcial);
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
        }

        osMutexAcquire(mutex_uart,osWaitForever);
        enviaUART(decisao.comando);
        UARTFlushTx(false);
        osMutexRelease(mutex_uart);
    }
  osDelay(500);
  } // while
}

void requisitaSensor(void *arg)
{  
  while(1)
  {
        for(int i=0; i<=2;i++)
        {
          osMutexAcquire(mutex_uart,osWaitForever);
          if(i==0) enviaUART("Pl;");
          UARTFlushTx(false);
          if(i==1) enviaUART("Prf;");
          UARTFlushTx(false);
          osMutexRelease(mutex_uart);
          osDelay(200);
        }
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
