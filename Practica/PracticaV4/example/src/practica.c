/*
 * PRACTICA Sistemas Embebidos.
 */

//Trabajamos con el UART3. Bloqueamos el semihosting para evitar conflictos entre UART-Wifly y el DEBUGOUT
//#define DEBUG_SEMIHOSTING 1

#include "board.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "uart.h"

#include "stdio.h"
#include "stdlib.h"

//DEFINES
/***********************************************************************/
#define mainQUEUE_LENGTH					( 5 )						//tamaño de la cola

/*PRIORIDAD TAREAS GESTION DAC-ACCELEROMETRO*/
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )	//prioridad de las tareas de envio

/*PRIORIDAD TAREA RECEPCION - ENVIO UART*/
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )	//prioridad de las tareas de recepcion

/*FRECUENCIAS*/
#define TASK1_SEND_FREQUENCY			( 1000 / portTICK_RATE_MS )		//frecuencia tarea ChannelX
#define TASK2_SEND_FREQUENCY			( 500 / portTICK_RATE_MS )		//frecuencia tarea ChannelY
#define TASK3_SEND_FREQUENCY			( 100 / portTICK_RATE_MS )		//frecuencia tarea ChannelZ

#define TASK1_RECEIVE_FREQUENCY			( 100 / portTICK_RATE_MS )		//frecuencia tarea recepcion

/*ACCELEROMETER*/
#define GPIO_ADC_0     23
#define GPIO_ADC_1	   24
#define GPIO_ADC_2	   25

/***********************************************************************/

//PARAMETERS
/***********************************************************************/

//Estructura de datos de un "momentum" de un canal
typedef struct {
	uint16_t channel; /*serán 0,1,2 según venga del canal 0,1,2*/
	uint16_t data;	  /*información del estado*/
} DataQueue_t;


//Clock canal ADC
static ADC_CLOCK_SETUP_T ADCSetup;


//Queue comunicacion Tasks productoras (canales accelerometro) y Task consumidoras (envío Wifly-UART)
static xQueueHandle xQueue = NULL;

//Semaforo entre tasks
static xSemaphoreHandle xSemaphore = NULL;


static volatile uint32_t configUART = 1; 		/*Ejecutar comandos Configuracion UART*/
static volatile uint32_t configWIFLY = 1; 		/*Ejecutar comandos configuracion WIFLY*/
static volatile uint32_t eraseWIFLY = 0; 		/*Ejecutar borrado ROM WIFLY*/
static volatile uint32_t sendWIFLY = 1; 		/*Ejecutar envio WIFLY*/
/***********************************************************************/


//FUNCTIONS
/*********************************************************************************/
static void initHardware();
static void initAccelerometro();
static void initDAC();
static void initUART();
static int initWiFly();
static void getChannelX(void *pvParameters );
static void getChannelY(void *pvParameters );
static void getChannelZ(void *pvParameters );
static void sendResultDataWIFI(void *pvParameters);
static void sendDataByUARTIdle(const char *data, unsigned int millis);
static int sendDataByUART(const char *data, const char *response, unsigned int delay, unsigned int millis);
static void sendDataToWiFly(unsigned int data, unsigned int millis);
static void Delay_MS(int ms);
static void hardResetWifly();
static void softResetWifly();
static void Lock_Wifly();
static void UnLock_Wifly();
/*********************************************************************************/


//MAIN
/*********************************************************************************/
int main(void)
{


	initHardware();

	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( DataQueue_t ) );
	if ((xQueue != NULL)){

		//vSemaphoreCreateBinary(xSemaphore);
		xSemaphore = xSemaphoreCreateMutex();

		if (xSemaphore != NULL){
			//crea la tarea que lee del canal X
			if (xTaskCreate(getChannelX, ( signed char * ) "TX_1",  (unsigned long)512, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL )!=pdPASS){
			}
			//crea la tarea que lee del canal Y
			if (xTaskCreate(getChannelY, ( signed char * ) "TX_2",  (unsigned long)512, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL )!=pdPASS){
			}
			//crea la tarea que lee del canal Z
			if (xTaskCreate(getChannelZ, ( signed char * ) "TX_3",  (unsigned long)512, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL )!=pdPASS){
			}
			//crea la tarea que recibe los datos del canal y los envia al WIFLY
			if (xTaskCreate(sendResultDataWIFI, ( signed char * ) "RX_1",  (unsigned long)512, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL )!=pdPASS){
			}

			vTaskStartScheduler();
		}
	}

	return 1;
}
/*********************************************************************************/


//INI HARDWARE
/********************************************************************************/
static void initHardware(){

	//activamos clock interno
	SystemCoreClockUpdate();

	//incializamos el board
	Board_Init();

	//desactivamos led principal
	Board_LED_Set(0, true);

	//inicializamos Accelerometro
	initAccelerometro();

	//inicializamos DAC
	initDAC();
}
/********************************************************************************/

//CONFIGURACION ACCELEROMETRO (APERTURA PUERTOS DAC)
/********************************************************************************/
static void initAccelerometro(){
	//resistencias pull-up abajo:
	Chip_IOCON_PinMuxSet(LPC_IOCON,0,23,IOCON_MODE_PULLDOWN); //resistencias en pull-down
	Chip_IOCON_PinMuxSet(LPC_IOCON,0,24,IOCON_MODE_PULLDOWN); //resistencias en pull-down
	Chip_IOCON_PinMuxSet(LPC_IOCON,0,25,IOCON_MODE_PULLDOWN); //resistencias en pull-down

	//modo GPIO Accelerometro (configuramos puertos de entrada)
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 23, false); //GPIO.0.23	(pin 15 de la placa)
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 24, false); //GPIO.0.24	(pin 16 de la placa)
	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 25, false); //GPIO.0.25	(pin 17 de la placa)

	//configuracion como PINSEL ADC-1/ADC-2; funcionamiento diferente a ADC-0 (aportacion de Ernesto Bori Robert)
	Chip_IOCON_PinMux(LPC_IOCON, 0, 23, IOCON_MODE_INACT, IOCON_FUNC1); /* ADC0 */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 24, IOCON_MODE_INACT, IOCON_FUNC1); /* ADC1 */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 25, IOCON_MODE_INACT, IOCON_FUNC1); /* ADC2 */
}
/********************************************************************************/

//CONFIGURACION DAC
/********************************************************************************/
static void initDAC(){

	Chip_ADC_Init(LPC_ADC, &ADCSetup);

	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, DISABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, DISABLE);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH2, DISABLE);
}
/********************************************************************************/

//CONFIGURACION UART
/********************************************************************************/
//UART-3, 9600 bauds, 8 bits de cabecera, 1 de paridad.
static void initUART(){
	UART_Init(3,9600);
}
/********************************************************************************/

//ENVIO DATOS UART
//@data = "cadena de la entrada"
//@response = "cadena de salida esperada"
//@delay = "tiempo de espera entre entrada/salida"
//@millis = "tiempo asignado para recuperar la respuesta del buffer"
/********************************************************************************/
static int sendDataByUART(const char *data, const char *response, unsigned int delay, unsigned int millis){

	static char Buffer[256];
	{

		Lock_Wifly();

		int i=0;
		for(i=0;i<256;i++){Buffer[i]=' ';}

		UART_Flush(3);

		/*
		 * Funcion UART crash+course. Funciones Chip_UART_ReadRB, Chip_UART_Read, Chip_UART_ReadBlocking (aportación de Alex Tirado) descartadas.
		 */
		UART_Send(3, data);

		Delay_MS(delay);

		if (UART_Read(3, Buffer, response, millis)>0){
			if (strstr(Buffer, response) != NULL){
				return 1;
			}else{
				return 0;
			}

		}else{
			return 0;
		}

		UnLock_Wifly();

	}
}
/********************************************************************************/

//ENVIO DATOS UART sin respuesta remota
/********************************************************************************/
static void sendDataByUARTIdle(const char *data, unsigned int millis){

	static char Buffer[256];


	int i=0;
	for(i=0;i<256;i++){Buffer[i]=' ';}

	UART_Flush(3);
	UART_Send(3, data);

	Delay_MS(millis);
}
/********************************************************************************/


//GESTION INICIAL WIFLY - Intentar entrar en Wifly en Modo Debug.
/********************************************************************************/
static int iniDataWiFly(const char *data)
{
	int response = sendDataByUART(data,"CMD",20,20000);					//primer intento de consola $$$

	if (response == 0){													//fracaso

		response = sendDataByUART("exit\r\n","EXIT",20,20000);			//sale de la sesión activa anterior (exit)

		if (response == 1){

			response = sendDataByUART(data,"CMD",20,20000);				//segundo intento de consola $$$
			return response;

		}else{
			return response;
		}

	}else{
		return response;
	}
}
/********************************************************************************/

//CONFIGURACION CONSOLA WIFLY
/********************************************************************************/
static int initWiFly(){

	/*
	 * Wifly se conecta con Comtrend C5813 en modo ad-hoc y dhcp con el primer canal Wifi libre que encuentre.
	 */

	int configFailed = 0;

	/*
	 * Configuramos el Wifly para conectar con el wifi local.
	 */

	configFailed = iniDataWiFly("$$$");																										//modo CMD

	configFailed = sendDataByUART("set wlan join 1\r\n","AOK",20,20000);																	//asociar al primer canal libre
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set wlan ssid <SSID>\r\n","AOK",20,20000);																//ssid de la red wifi
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set wlan phrase <PHRASE>\r\n","AOK",20,20000);															//clave de la red wifi (WPA-PSK)
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set ip dhcp 1\r\n","AOK",20,20000);																		//dhcp
	if(configFailed == 0){return configFailed;}

	/*
	 * Comando Wifly forwarding datos recibidos por UART a arpalab.appspot.com
	 */
	configFailed = sendDataByUART("set ip proto 18\r\n","AOK",20,20000);																	//dns+ip
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set dns name arpalab.appspot.com\r\n","AOK",20,20000);													//dns remoto
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set ip host 0\r\n","AOK",20,20000);																		//puerto de salida (cualquiera)
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set ip remote 80\r\n","AOK",20,20000);																	//puerto de entrada remoto (80)
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set com remote GET$/rest/stats?nid=46960465&ip=77.231.188.148&app=pract2&data=\r\n","AOK",20,20000);		//forwarding a la direccion remota X
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set option format 1\r\n","AOK",20,20000);																//datos binarios a Ascii
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set uart mode 2\r\n","AOK",20,20000);																	//captura y reenvio de la informacion que viene del UAR
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("set comm time 2000\r\n","AOK",20,20000);																	//envio de datos cada 2 segundos
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("save\r\n","Storing",20,20000);																			//salvar la configuracion
	if(configFailed == 0){return configFailed;}
	configFailed = sendDataByUART("reboot\r\n","*Reboot*",20,20000);																		//salvar la configuracion
	if(configFailed == 0){return configFailed;}


	return 1;
}
/********************************************************************************/

//ENVIO DATOS ACCELEROMETRO A ARPALAB MEDIANTE WIFLY
/********************************************************************************/
static void sendDataToWiFly(unsigned int data, unsigned int millis){

	char buffer[]="123456";
	sprintf (buffer, "%-6d", data);
	sendDataByUART(buffer,"",20,millis);
}

/********************************************************************************/

//TASK-1: Gestión canal X Accelerometro.
/********************************************************************************/

static void getChannelX( void *pvParameters )
{

	uint16_t dataADC;
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	while (1){

		if (xSemaphoreTake(xSemaphore, portMAX_DELAY)!= pdTRUE){

		}else{

			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);								//activa canal 0

			Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);											//activa el modo Burst (modo incremental)

			while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH0, ADC_DR_DONE_STAT) != SET) {}		//obtiene por encuesta los datos del canal 0

			Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &dataADC);									//leemos el dato obtenido del canal 0

			Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);											//desactivamos el modo Burst

			DataQueue_t data;

			data.channel=1;
			data.data = dataADC;

			if (xQueueSend( xQueue, &data, 0)!= pdPASS ){									//enviamos el dato a la task de envio por la Queue
			}

			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, DISABLE);
			xSemaphoreGive( xSemaphore );


		}

		vTaskDelayUntil(&xNextWakeTime, TASK1_SEND_FREQUENCY);
	}

}
/********************************************************************************/

//TASK-2: Gestión canal Y Accelerometro.
/********************************************************************************/
static void getChannelY( void *pvParameters )
{


	uint16_t dataADC = 0;
	uint16_t dataADC_0 = 0;
	uint16_t res = 0;

	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	while (1){

		if (xSemaphoreTake(xSemaphore, portMAX_DELAY)!= pdTRUE){
		}else{

			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, ENABLE);							//activa canal 1

			Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

			while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH1, ADC_DR_DONE_STAT) != SET) {}

			Chip_ADC_ReadValue(LPC_ADC, ADC_CH1, &dataADC);

			Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

			//Procesa muestra, con la muestra anterior.
			//////////////////////////////////////////////////////////////////////////////////////////
			res = dataADC - dataADC_0;
			dataADC_0 = dataADC;

			DataQueue_t data;
			data.channel=2;
			data.data = res;
			//////////////////////////////////////////////////////////////////////////////////////////

			if (xQueueSend( xQueue, &data, 0 )!= pdPASS ){
			}

			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH1, DISABLE);
			xSemaphoreGive( xSemaphore );

		}
		vTaskDelayUntil(&xNextWakeTime, TASK2_SEND_FREQUENCY);
	}


}
/********************************************************************************/


//TASK-3: Gestión canal Z Accelerometro.
/********************************************************************************/
static void getChannelZ( void *pvParameters )
{
	uint16_t dataADC;
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();

	uint16_t vector[10];
	uint16_t sum = 0;
	uint16_t avg = 0;

	//inicializamos vector
	int i=0;
	for(i=0;i<10;i++){vector[i]=0;}

	while (1){

		if (xSemaphoreTake(xSemaphore, portMAX_DELAY)!= pdTRUE){
		}else{

			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH2, ENABLE);							//activa canal 2

			Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

			while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH2, ADC_DR_DONE_STAT) != SET) {}

			Chip_ADC_ReadValue(LPC_ADC, ADC_CH2, &dataADC);

			Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

			//Recoge 10 muestras, y hace la media aritmetica de todas ellas.
			//////////////////////////////////////////////////////////////////////////////////////////
			sum=0;
			avg=0;
			int j=0;

			for(j=10;j>0;j--){vector[j]=vector[j-1]; sum+=vector[j];}

			vector[0]=dataADC;
			sum+=dataADC;
			avg=sum/10;

			DataQueue_t data;
			data.channel=3;
			data.data = avg;

			//////////////////////////////////////////////////////////////////////////////////////////

			if (xQueueSend( xQueue, &data, 0)!= pdPASS ){
			}


			Chip_ADC_EnableChannel(LPC_ADC, ADC_CH2, DISABLE);
			xSemaphoreGive( xSemaphore );
		}
		vTaskDelayUntil(&xNextWakeTime, TASK3_SEND_FREQUENCY);
	}
}
/********************************************************************************/


//TASK DE SALIDA. Gestion UART, Configuracion WIFLY. Envio Datos Remotos.
/********************************************************************************/
static void sendResultDataWIFI(void *pvParameters )
{

	portTickType xNextWakeTime;
	DataQueue_t ulReceivedValue;
	xNextWakeTime = xTaskGetTickCount();

	//reseteamos WIFLY
	if (eraseWIFLY==1){
		if (xSemaphoreTake(xSemaphore, ( portTickType ) 10)==pdTRUE){
			hardResetWifly();
			xSemaphoreGive( xSemaphore );
		}
	}

	//activamos UART
	if (configUART==1){
		if (xSemaphoreTake(xSemaphore, ( portTickType ) 10)==pdTRUE){
			initUART();
			xSemaphoreGive( xSemaphore );
		}
	}

	//configuramos WIFLY
	if (configWIFLY==1){
		if (xSemaphoreTake(xSemaphore,( portTickType ) 10)==pdTRUE){
			sendWIFLY = initWiFly();
			xSemaphoreGive( xSemaphore );
		}
	}

	while(1){
			if (xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY ) == pdTRUE){
				if ((configUART == 1) && (sendWIFLY == 1)){ 						//si UART configurado y WIFLY configurado. Enviamos datos cola a Wifly.
					unsigned int valorSend = ulReceivedValue.data;
					if (xSemaphoreTake(xSemaphore, ( portTickType ) 10)==pdTRUE){
						sendDataToWiFly(valorSend,20000);
						xSemaphoreGive( xSemaphore );
					}
				}else{
				}
			}
		vTaskDelayUntil(&xNextWakeTime, TASK1_RECEIVE_FREQUENCY);
	}


	//cerramos comunicacion UART
	if (configUART==1){
			if (xSemaphoreTake(xSemaphore, ( portTickType ) 10)==pdTRUE){
				UART_Close(3);
				xSemaphoreGive( xSemaphore );
			}
	}


}
/***********************************************************************************/


//FUNCIONES AUXILIARES
/***********************************************************************************/

//reinicio configuracion Wifly HARD
//señal pin 22 de la placa al puerto 5 del Wifly
static void hardResetWifly(){

	Chip_GPIO_WriteDirBit(LPC_GPIO, 0, 2, true);
	Chip_IOCON_PinMuxSet(LPC_IOCON,0,2,IOCON_MODE_PULLUP);

	Chip_GPIO_SetPinOutHigh(LPC_GPIO, 0, 2);
	Delay_MS(100);
	Chip_GPIO_SetPortOutLow(LPC_GPIO, 0, 2);

}

//reinicio configuracion Wifly SOFT
static void softResetWifly(){
	sendDataByUART("factory Reset\r\n","Reset",20,20000);			//factory Reset
	sendDataByUART("save\r\n","Storing",20,20000);					//save
	sendDataByUART("reboot\r\n","*Reboot*",20,20000);				//Reboot
}


//Delay_MS (definir un delay de X MS)
static void Delay_MS(int ms){
	int i=0;
	for(i=0; i<(((uint32_t)7140)*ms);i++);
}

//bloqueamos el funcionamiento del ADC mientras enviamos los datos por Wifly mediante UART
static void Lock_Wifly(){
	if (xSemaphore == NULL ) {
		xSemaphore = xSemaphoreCreateRecursiveMutex();
	}
	xSemaphoreTakeRecursive(xSemaphore, 10);
}

//desbloqueamos para seguir funcionando.
static void UnLock_Wifly(){
	xSemaphoreGiveRecursive(xSemaphore);
}

/***********************************************************************************/

