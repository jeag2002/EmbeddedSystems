

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "LPC17xx.h"

/*libreria uart proporcionada por foro de sistemas empotrados*/
#include "uart.h"

/*prioridad tareas recepcion prioridad mayor 2*/
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )

/*prioridades tareas de envio prioridad 1*/
#define	mainQUEUE_SEND_TASK_PRIORITY_1		( tskIDLE_PRIORITY + 1 )
#define	mainQUEUE_SEND_TASK_PRIORITY_2		( tskIDLE_PRIORITY + 1 )

/* Ratio de envio de informacion */
#define mainQUEUE_FREQUENCY_MS_SND_1			50
#define mainQUEUE_FREQUENCY_MS_SND_2			100

/* Ratio de recepción de información */
#define mainQUEUE_FREQUENCY_MS_RECV			    75

/* Tamano de la cola */
#define mainQUEUE_LENGTH					( 1 )

static void initHardware();
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask_1( void *pvParameters );
static void prvQueueSendTask_2( void *pvParameters );
//static void sendDataToUART(uint32_t data);

//mutex
static xSemaphoreHandle xMutex = NULL;

//semaforo
static xSemaphoreHandle xEmpty = NULL;
static xSemaphoreHandle xFull = NULL;

//queue
static xQueueHandle xQueue = NULL;

/*-----------------------------------------------------------*/

int main(void)
{

	/*Arrancar puerto serie, protocolo UART*/
	initHardware();

	/*creando mutex, empty, full*/
	xMutex = xSemaphoreCreateMutex();

	xEmpty = xSemaphoreCreateCounting(5,5);
	xFull =  xSemaphoreCreateCounting(5,0);

	/* cola */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

		if ((xMutex != NULL) && (xEmpty != NULL) && (xFull != NULL))
		{
			/*Creo la tarea de recepción. Se asigna espacio de memoria de stack de 240. Se ejecuta cada 75 ms*/
			if (xTaskCreate( prvQueueReceiveTask, ( signed char * ) "RX_1", (unsigned long)240, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL )!=pdPASS){
				printf("\n<MAIN>TAREA RX_1 NO SE HA CREADO");
			}

			/*Creo la tarea de envío. Se asigna espacio de memoria de stack de 240. Se ejecuta cada 50 ms*/
			if (xTaskCreate( prvQueueSendTask_1, ( signed char * ) "TX_1",  (unsigned long)240, NULL, mainQUEUE_SEND_TASK_PRIORITY_1, NULL )!=pdPASS){
				printf("\n<MAIN>TAREA TX_1 NO SE HA CREADO");
			}

			/*Creo la tarea de envío. Se asigna espacio de memoria de stack de 240. Se ejecuta cada 100 ms*/
			if (xTaskCreate( prvQueueSendTask_2, ( signed char * ) "TX_2",  (unsigned long)240, NULL, mainQUEUE_SEND_TASK_PRIORITY_2, NULL )!=pdPASS){
				printf("\n<MAIN>TAREA TX_2 NO SE HA CREADO");
			}

			/*Arranca el scheduler de las tareas */
			vTaskStartScheduler();
		}else{
			printf("\n<MAIN>Mutex no se ha creado correctamente");
		}

	/*RETORNA OK*/

	return EXIT_SUCCESS;

}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*Configuramos los datos del hardware*/
static void initHardware(){

	/*inicializamos el módulo UART para enviar por puerto serie los datos del receptor*/
	//UARTInit(3, 9600);

	printf("\n<MAIN>hardware inicializado");
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

/*Enviamos mensaje de texto via UART */
/*
static void sendDataToUART(uint32_t data){
	char buffer[256];
	sprintf (buffer,"\ndata_received: %d",data);
	UARTSend(3, (uint8_t *)buffer , sizeof(char)*256 );
}
*/
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*Task-1 Productor (envía indice del vector)*/
static void prvQueueSendTask_1( void *pvParameters )
{

	unsigned int data = 3;
	while (1){

		//define un delay de 50 ms
		vTaskDelay(mainQUEUE_FREQUENCY_MS_SND_1);

		//pregunta si puede tomar el control de la estructura compartida
		if (xSemaphoreTake(xEmpty, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-1 INSERT>Mutex xEmpty no capturado ");
		}else if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-1 INSERT>Mutex xMutex no capturado ");
		}else{

			printf("\n<TASK-1> SEND 3");
			xQueueSend( xQueue, &data, 0 );

		//libera el control
			if (xSemaphoreGive(xMutex)!= pdTRUE){
				printf("\n<TASK-1 INSERT> NO Give semaphoro xMutex");
			}

			if (xSemaphoreGive(xFull)!= pdTRUE){
				printf("\n<TASK-1 INSERT> NO Give semaphoro xFull");
			}
		}


	}//end-while

}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*Task-2 Productor (envía indice del vector)*/
static void prvQueueSendTask_2( void *pvParameters )
{

	unsigned int data = 2;

	while (1){

		//define un delay de 100 ms
		vTaskDelay(mainQUEUE_FREQUENCY_MS_SND_2);

		//pregunta si puede tomar el control de la estructura compartida
		if (xSemaphoreTake(xEmpty, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-2 INSERT>Mutex xEmpty no capturado ");
		}else if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-2 INSERT>Mutex xMutex no capturado ");
		}else{


			printf("\n<TASK-2> SEND 2");
			xQueueSend( xQueue, &data, 0 );


		//libera el control
			if (xSemaphoreGive(xMutex)!= pdTRUE){
				printf("\n<TASK-2 INSERT> NO Give semaphoro xMutex");
			}

			if (xSemaphoreGive(xFull)!= pdTRUE){
				printf("\n<TASK-2 INSERT> NO Give semaphoro xFull");
			}

		}

	}//end-while
}

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
/*Task-3 Receptor*/
static void prvQueueReceiveTask( void *pvParameters )
{
	unsigned int ulReceivedValue;

	while(1){

		//pregunta si puede tomar el control de la estructura compartida
		if (xSemaphoreTake(xFull, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-1 READ>Mutex xFull no capturado ");
		}else if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE){
			printf("\n<TASK-1 READ>Mutex xMutex no capturado ");
		}else{

			xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

			if ((ulReceivedValue == 2) ||
					(ulReceivedValue == 3)){
				printf("\n<TASK-1 READ>enviando dato al UART %d",ulReceivedValue);
			}

		//libera el control
			if (xSemaphoreGive(xMutex)!= pdTRUE){
				printf("\n<TASK-1 READ> NO Give semaphoro xMutex");
			}

			if (xSemaphoreGive(xEmpty)!= pdTRUE){
				printf("\n<TASK-1 READ> NO Give semaphoro xEmpty");
			}

		}

		//define un delay de 75 ms
		vTaskDelay(mainQUEUE_FREQUENCY_MS_RECV);

	}//end-while
}

void vApplicationIdleHook( void )
{
}

void vApplicationTickHook( void )
{
}

void vApplicationMallocFailedHook( void )
{
	for( ;; );
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}



