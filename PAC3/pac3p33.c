/*
 * Pregunta 3 PAC 3
 *
 * Reuno en un mismo codigo, los 3 apartados de la pregunta (1,2,3)
 * el ejemplo periph_adc incluye un control por UART que descarto para centrarme en ADC
 * He implementado 2 metodos (encuesta/interrupciones) de los 3 propuestos en periph_adc
 * (encuesta/interrupciones/DMA)
 */

/*Habilitamos el modo semihosting de debug. (Parseamos la información en dispositivo, vemos los resultados por consola)*/
#define DEBUG_SEMIHOSTING 1

#include "board.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"


/*prioridad tareas recepcion prioridad mayor 2*/
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/*prioridades tareas de envio prioridad 1*/
#define	mainQUEUE_SEND_TASK_PRIORITY_1		( tskIDLE_PRIORITY + 1 )


#define TASK1_SEND_FREQUENCY			( 100 / portTICK_RATE_MS )
#define TASK2_RECEIVE_FREQUENCY			( 100 / portTICK_RATE_MS )

#define SEND_TASK_TICKS_TO_WAIT         ( 1000 )
#define RECEIVE_TASK_TICKS_TO_WAIT      ( 100 )


/* Tamano de la cola */
#define mainQUEUE_LENGTH					( 1 )

/* Configuracion del chip ADC */
/*******************************************************************/
#ifdef BOARD_NXP_LPCXPRESSO_1769
#define _ADC_CHANNLE ADC_CH0
#else
#define _ADC_CHANNLE ADC_CH2
#endif

#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn
static ADC_CLOCK_SETUP_T ADCSetup;
/*******************************************************************/

//queue
static xQueueHandle xQueue = NULL;

//semaphore (binary)

xSemaphoreHandle xSemaphore = NULL;


//ADCSetup
static volatile uint8_t Burst_Mode_Flag = 0;
static volatile uint32_t _bitRate = 0;

static volatile uint8_t Interrupt_Continue_Flag = 0;
static volatile uint8_t ADC_Interrupt_Done_Flag = 0;

//determina si se quiere modo encuesta (0) o modo interrupcion (0xFF)
static volatile uint32_t _IRQMode = !0;


static void enableIRQ();
static void initHardware();
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
static uint16_t App_ADC_Polling_Test();
static void App_ADC_Interruption_Test();
static void ADC_IRQHandler();
static void print_ADC_value_with_delay(uint16_t data);

int main(void)
{

	/*Configuramos el procesador para que arranque el conversor analogico-digital*/
	initHardware();

	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	if (xQueue != NULL){
		if (!_IRQMode){

			DEBUGOUT("<MAIN>MODO POLLING\n\r");

			/*Creo la tarea de envío. Se asigna espacio de memoria de stack de 128. Se ejecuta cada 100 ms*/
			if (xTaskCreate( prvQueueSendTask, ( signed char * ) "TX_1",  (unsigned long)128, NULL, mainQUEUE_SEND_TASK_PRIORITY_1, NULL )!=pdPASS){
				DEBUGOUT("<MAIN>TAREA TX_1 NO SE HA CREADO\r\n");
			}

			/*Creo la tarea de recepción. Se asigna espacio de memoria de stack de 128. Se ejecuta cada 100 ms*/
			if (xTaskCreate( prvQueueReceiveTask, ( signed char * ) "RX_1", (unsigned long)128, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL )!=pdPASS){
				DEBUGOUT("<MAIN>TAREA RX_1 NO SE HA CREADO\r\n");
			}

			vTaskStartScheduler();

		}else{

			DEBUGOUT("<MAIN>MODO IRQ\n\r");

			/*Habilitamos la interrupción del ADC (utilizamos un semaforo binario)*/
			vSemaphoreCreateBinary(xSemaphore);

			if (xSemaphore != NULL){

				/*Creo la tarea que activa la interrupcion. Se asigna espacio de memoria de stack de 128. Se ejecuta cada 100 ms*/
				if (xTaskCreate( enableIRQ, ( signed char * ) "TX_1", (unsigned long)128, NULL, 1, NULL )!=pdPASS){
					DEBUGOUT("<MAIN>INTERRUPCION RX_1 NO SE HA CREADO\r\n");
				}

				/*Creo la tarea cliente que recibe los datos de la interrupcion IRQ. Se asigna espacio de memoria de stack de 128. Se ejecuta cada 100 ms*/
				if (xTaskCreate( App_ADC_Interruption_Test, ( signed char * ) "RX_1", (unsigned long)128, NULL, 1, NULL )!=pdPASS){
					DEBUGOUT("<MAIN>INTERRUPCION RX_1 NO SE HA CREADO\r\n");
				}

				vTaskStartScheduler();
			}
		}



		/*Arranca el scheduler de las tareas */


	}else{
		DEBUGOUT("<MAIN>xQueue no se ha creado correctamente\r\n");
	}

	return 1;
}

static void enableIRQ(){

	//Habilito interrupcion
	NVIC_EnableIRQ(_LPC_ADC_IRQ);
	
	//Habilito canal (puerto 15 de la placa LCPXpresso)
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);

	//activo modo "Burst" en caso de necesitarlo
	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);
	}
	
	ADC_Interrupt_Done_Flag = 1;

	DEBUGOUT("<MAIN-enableIRQ> Interrupciones ADC_IRQ habilitada");

	while(1){
		if ((!Burst_Mode_Flag) && (ADC_Interrupt_Done_Flag)) {
			ADC_Interrupt_Done_Flag = 0;
			/*activo disparador, (una vez) hasta primera lectura*/
			Chip_ADC_SetStartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		}
	}

	//desactivo modo "Burst" en caso de necesitarlo
	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);
	}

	//deshabilito interrupcion
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
}



static void initHardware(){

	//activamos clock interno
	SystemCoreClockUpdate();
	DEBUGOUT("<MAIN-initHardware>System Core Clock inicializado\r\n");

	//activamos perifericos de la placa
	Board_Init();
	DEBUGOUT("<MAIN-initHardware>Board inicializado (habilitamos todos los perifericos)\r\n");

	//desactivamos led principal
	Board_LED_Set(0, true);
	DEBUGOUT("<MAIN-initHardware>LED 0\r\n");

	//activamos ADC
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	
	//activamos canal ADC
	Chip_ADC_EnableChannel(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);

	DEBUGOUT("<MAIN-initHardware>ADC inicializado\r\n");

	//Activamos/Desactivamos el Conversor Analogico Digital modo Burst (actualiza el valor de forma incremental)
	ADCSetup.burstMode = Burst_Mode_Flag;

	//Activamos el bitrate al maximo por defecto
	_bitRate = ADC_MAX_SAMPLE_RATE;

	Chip_ADC_SetSampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
	DEBUGOUT("<MAIN-initHardware>ADC-Modo Burst inicializado. Definicion del _bitRate\r\n");

	DEBUGOUT("<MAIN-initHardware>hardware inicializado\r\n");
}

/*-----------------------------------------------------------*/
static void print_ADC_value_with_delay(uint16_t data)
{
	//volatile uint32_t j;
	//j = 5000000;
	DEBUGOUT("<ADC GET DATA>valor_capturado : 0x%04x\r\n", data);
	//while (j--) {}
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
static void prvQueueSendTask( void *pvParameters )
{

	portTickType xNextWakeTime;

	unsigned int data = 3;

	xNextWakeTime = xTaskGetTickCount();

	while (1){

		//capturamos info ADC por encuesta
		data = App_ADC_Polling_Test();

		DEBUGOUT("<TASK-1 WRITE POLL>SEND %d; 0x%04x\r\n",data, data);
		
		//enviamos informacion encuesta por cola
		xQueueSend( xQueue, &data, 0 );

		vTaskDelayUntil(&xNextWakeTime, TASK1_SEND_FREQUENCY);
	}//end-while

}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
static uint16_t App_ADC_Polling_Test(void)
{
	uint16_t dataADC;

	/* Seleccionamos modo Burst */
	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);
	}else {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);
	}

	if (!Burst_Mode_Flag) {
		Chip_ADC_SetStartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	}

	/* Esperamos por encuesta, hasta que el dato del ADC sea capturado */
	while (Chip_ADC_ReadStatus(_LPC_ADC_ID, _ADC_CHANNLE, ADC_DR_DONE_STAT) != SET) {}

	/* leemos el datos del ADC*/
	Chip_ADC_ReadValue(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);

	print_ADC_value_with_delay(dataADC);

	/* deshabilitamos el modo Burst */
	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);
	}

	return dataADC;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/*
static void App_ADC_Interruption_Test(void)
{

	//portTickType xNextWakeTime;
	//xNextWakeTime = xTaskGetTickCount();

	NVIC_EnableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);


	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);
	}


	Interrupt_Continue_Flag = 1;
	ADC_Interrupt_Done_Flag = 1;


	while(Interrupt_Continue_Flag){

		if (!Burst_Mode_Flag && ADC_Interrupt_Done_Flag) {
			ADC_Interrupt_Done_Flag = 0;
			Chip_ADC_SetStartMode(_LPC_ADC_ID, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		}

		//vTaskDelayUntil(&xNextWakeTime, TASK1_SEND_FREQUENCY);
	}

	if (Burst_Mode_Flag) {
		Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);
	}

	NVIC_DisableIRQ(_LPC_ADC_IRQ);
}
*/

static void App_ADC_Interruption_Test(void)
{

	portTickType xNextWakeTime;
	unsigned int ulReceivedValue = 0;

	xNextWakeTime = xTaskGetTickCount();

	while (1){

		if (xSemaphoreTake(xSemaphore, portMAX_DELAY)!= pdTRUE){
			DEBUGOUT("<TASK-1 READ IRQ> Semaforo xSemaphore no capturado");
		}else{

			if (xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY )!=pdTRUE){
				DEBUGOUT("<TASK-1 READ IRQ> Queue xQueue no capturado");
			}else{
				if (ulReceivedValue==NULL){ulReceivedValue=0;}
				DEBUGOUT("<TASK-1 READ IRQ>RECEIVE %d: (0x%04x)\r\n", ulReceivedValue, ulReceivedValue);
			}
			xSemaphoreGive( xSemaphore );
		}

		vTaskDelayUntil(&xNextWakeTime, TASK2_RECEIVE_FREQUENCY);
	}

}


/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void ADC_IRQHandler(void)
{
	uint16_t dataADC;

	portBASE_TYPE HigherPriorityTaskWoken = pdFALSE;

	//deshabilitamos interrupcion IRQ
	NVIC_DisableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, DISABLE);

	//leemos datos del ADC
	Chip_ADC_ReadValue(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);

	//Captura los datos a traves de la interrupcion y los envia por la queue
	print_ADC_value_with_delay(dataADC);
	DEBUGOUT("<TASK-1 IRQ>SEND %d; 0x%04x\r\n",dataADC, dataADC);

	ADC_Interrupt_Done_Flag = 1;

	//enviamos informacion por cola
	xQueueSendFromISR( xQueue, &dataADC, &HigherPriorityTaskWoken);

	//si IRQ toma maxima prioridad, liberamos prioridad para el resto de las tareas
	if(HigherPriorityTaskWoken){
		portYIELD_FROM_ISR (HigherPriorityTaskWoken);
	}

	//liberamos semaforo (para el resto de tareas)
	xSemaphoreGiveFromISR(xSemaphore,NULL);
	
	NVIC_EnableIRQ(_LPC_ADC_IRQ);
	Chip_ADC_Int_SetChannelCmd(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);

}
/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/
static void prvQueueReceiveTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	unsigned int ulReceivedValue;
	xNextWakeTime = xTaskGetTickCount();

	while(1){

		//recibimos informacion por la cola y lo printamos por pantalla
		if (xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY ) == pdTRUE){
			DEBUGOUT("<TASK-1 READ POLL>RECEIVE %d: (0x%04x)\r\n", ulReceivedValue, ulReceivedValue);
		}

		vTaskDelayUntil(&xNextWakeTime, TASK2_RECEIVE_FREQUENCY);
	}//end-while
}
/*-----------------------------------------------------------*/


