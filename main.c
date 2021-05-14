#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"



osThreadId_t tid_sRead;
osThreadId_t tid_vControl;
osThreadId_t tid_vDecision;
osMutexId_t mutexUart;
osMessageQueueId_t inpvControl_MsgQueue;
osMessageQueueId_t outvControl_MsgQueue;


void sRead(void *arg); 
void vControl (void *argument);
void vDecision (void *argument); 
int32_t initObjects(void);


#define MSGQUEUE_OBJTS 16
#define BUFFER_SIZE 16
#define FLAG1 1    // objeto na frente
#define FLAG2 2    // limite da pista atingido (pela direita)
#define FLAG3 4    // limite da pista atingido (pela esquerda)
#define FLAG4 16    // saindo pela direita
#define FLAG5 32    // saindo pela esquerda
#define FLAG6 64    // caminho livre
#define limitePista 15



osThreadId_t tid_teste;
osThreadId_t tid_recebeUart;

void enviaUART(char string[]);

#define BUFFER_SIZE 16


typedef struct {
  char* sensor[BUFFER_SIZE];
  int32_t valor[BUFFER_SIZE];
  int32_t idx;
} MSGQUEUE_OBJ;

typedef struct {
  int32_t valor[BUFFER_SIZE];
  
} MSGQUEUE_OBJout;

typedef struct {
  char* comando[BUFFER_SIZE];
  int32_t nrInstr;
} structComando;



//----------
// UART definitions
//extern void UARTStdioIntHandler(void);

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



// Função initObjects
// Faz a inicialização dos objetos (mutexes, tarefas, filas de mensagens)

int32_t initObjects(void){
  inpvControl_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJTS, sizeof(MSGQUEUE_OBJTS), NULL);
  if (inpvControl_MsgQueue == NULL) {
    ; // Message Queue object not created, handle failure
  }
  
  outvControl_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJTS, sizeof(MSGQUEUE_OBJTS), NULL);
  if (outvControl_MsgQueue == NULL) {
    ; // Message Queue object not created, handle failure
  }
  
  mutexUart = osMutexNew(NULL);
  
  tid_sRead = osThreadNew(sRead, NULL, NULL);
  if (tid_sRead == NULL) {
    return(-1);
  }
  
  tid_vControl = osThreadNew(vControl, NULL, NULL);
  if (tid_vControl == NULL) {
    return(-1);
  }
  
  tid_vDecision = osThreadNew(vDecision, NULL, NULL);
  if (tid_vDecision == NULL) {
    return(-1);
  }
 return(0);
}

// Thread sRead
// Input: Dados da uart a respeito da leitura dos sensores
// Output: Escreve na UART solicitando os dados de leitura do sensor e envia periodicamente estes dados para a thread vControl por fila de mensagens

void sRead(void *arg){
  int32_t laser = 0, rfreq = 0;
  char* dado = "";
  char ch_rfreq[BUFFER_SIZE];
  MSGQUEUE_OBJ msg;
  uint32_t index_i = 0;
  
  while(1){

    osMutexAcquire(mutexUart,osWaitForever);            // Leitura do sensor Laser
    UARTprintf("Pl;");
    UARTgets(dado,sizeof(dado));
    laser = (int)dado;
    osMutexRelease(mutexUart);
    
    msg.sensor[index_i] = "rfreq";
    msg.valor[index_i] = rfreq;
    msg.idx = index_i;
    
    osMessageQueuePut(inpvControl_MsgQueue, &msg, 0U, 0U);
    
    index_i++;
    if(index_i >= BUFFER_SIZE)
      index_i = 0;
    
    osMutexAcquire(mutexUart,osWaitForever);           // Leitura do sensor radio frequência
    UARTprintf("Prf;");
    UARTgets(dado,sizeof(dado));
    osMutexRelease(mutexUart);
    for(int i = 0; i <=sizeof(dado); i++){             // Pegando somente o valor da leitura
      ch_rfreq[i] = dado[i+3]; 
    }
    rfreq = (int)ch_rfreq;                              // Converte somente o número da leitura
    
    msg.sensor[index_i] = "laser";
    msg.valor[index_i] = laser;
    msg.idx = index_i;
    
    index_i++;
      if(index_i >= BUFFER_SIZE)
        index_i = 0;
    
    osMessageQueuePut(inpvControl_MsgQueue, &msg, 0U, 0U);    
  } // while
}

// Thread vControl
// Input: Recebe a leitura dos sensores através de fila de mensagens da thread sRead
// Output: Envia para a thread vDecision uma flag por fila de mensagens a respeito da melhor decisão de aceleração/rotação do carro com base nas leituras dos sensores

void vControl(void *arg){
  osStatus_t status;
  MSGQUEUE_OBJ msg;
  MSGQUEUE_OBJout msgout;
  uint8_t index_o = 0;
  uint32_t dado_laser = 0, dado_rfreq = 0;
  uint8_t index_out = 0;

  //Recebimento dos dados pela fila de mensagens
  
  while(1){
  status = osMessageQueueGet(inpvControl_MsgQueue, &msg, NULL, 0U);   // wait for message
    if (status == osOK) {
      //index_o = msg.idx;                                      // precisa disso? ou puxa pelo index_o direto
      if(msg.sensor[index_o] == "laser"){
        dado_laser = msg.valor[index_o];
      } else if(msg.sensor[index_o] == "rfreq"){
          dado_rfreq = msg.valor[index_o];
        }
     index_o ++;
     if(index_o >= BUFFER_SIZE)
       index_o = 0;
    }
    
    // Tratamento dos dados recebidos
  
    if(dado_laser != 0){
      msgout.valor[index_out] = FLAG1;
      }else if(dado_rfreq >= limitePista){
        msgout.valor[index_out] = FLAG2;
      } else if(dado_rfreq <= limitePista){
        msgout.valor[index_out] = FLAG3;
      } else if(dado_rfreq >= 5){
        msgout.valor[index_out] = FLAG4;
      } else if(dado_rfreq <= -5){
        msgout.valor[index_out] = FLAG5;
      } else {
        msgout.valor[index_out] = FLAG6;
      }   
    
      index_out++;
      if(index_out >= BUFFER_SIZE)
        index_out = 0;
      
    osMessageQueuePut(outvControl_MsgQueue, &msgout, 0U, 0U);
      
  } // while
}

// Thread vDecision
// Input: Flag de comando setada pela thread vControl e transmitida através de fila de mensagens
// Output Comando para a UART definindo aceleração e rotação do carro

void vDecision(void *arg){
  MSGQUEUE_OBJout msgout;
  osStatus_t status;
  uint8_t index_o = 0;
  structComando com;
  uint8_t index_c = 0;
  // utlizar indices nos comandos?
  
  while(1){
    
    status = osMessageQueueGet(outvControl_MsgQueue, &msgout, NULL, 0U);   // wait for message
    
    com.nrInstr = 1;
    if (status == osOK) { 
      switch(msgout.valor[index_o]){
      case FLAG1:
        com.comando[index_c] = "E;V90;A2;";
        com.comando[index_c + 1] = "E;V-90";
        com.nrInstr = 2;
        break;
      case FLAG2:
        com.comando[index_c] = "A-2;";          // preencher caractere ou string direto?
        break;
      case FLAG3:
        com.comando[index_c] = "A-2;";
        break;
      case FLAG4:
        com.comando[index_c] = "V-30;A2;";
        break;
      case FLAG5:
        com.comando[index_c] = "V30;A2;";
        break;
      case FLAG6:
        com.comando[index_c] = "A2;";
        break;
      default :
        com.comando[index_c] = "A2;";
        break;
      } // switch case
                    
   osMutexAcquire(mutexUart, osWaitForever);
   for(int i = 0; i <= index_c ; i++){
      osMutexAcquire(mutexUart, osWaitForever);
      UARTprintf("%s",com.comando[i]);
      osMutexRelease(mutexUart);
      osDelay(100);
      }
                 
   } // if
  } // while                        
}



void ThreadTeste(void *arg)
{
  //osStatus_t = status;
  char leitura[BUFFER_SIZE];
  char valorLeitura[BUFFER_SIZE];
  
  while(1)
  { 
    
  UARTprintf("Prf;\r"); 
 /* 
    switch(leitura[0])
    {
      case 'L':
   
        if(leitura[1] == 'I')
        {
          for(int i=0; i <= BUFFER_SIZE; i++)
            valorLeitura[i] = leitura[i+2];
        }
      break;
    default:
      break;
    }

    */
  osDelay(1000);  
  }  
}



void recebeUart(void *arg)
{
  char buffer[BUFFER_SIZE];
  uint8_t i = 0;
  uint8_t charRecebido = 0;

  while(1)
  {
    
    osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
    
    do
    {
      charRecebido = UARTCharGet(UART0_BASE);
      buffer[i] = charRecebido;
      i++;   
    } while(charRecebido != '\n' || UARTCharsAvail(UART0_BASE) != 1);
    
    //while(UARTCharsAvail(UART0_BASE)) UARTCharGet(UART0_BASE);
    
    enviaUART(buffer);
    i = 0;
  }
}

void enviaUART(char string[])
{
   for(int i = 0; i < strlen(string); i++)
   {
     UARTCharPut(UART0_BASE,string[i]);
   }
}


void main(void)
{
  UARTInit();
  osKernelInitialize();
  initObjects();

  tid_teste = osThreadNew(ThreadTeste, NULL, NULL);
  tid_recebeUart = osThreadNew(recebeUart, NULL, NULL);
  
  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
}
