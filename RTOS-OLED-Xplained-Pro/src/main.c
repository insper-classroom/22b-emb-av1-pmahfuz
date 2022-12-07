#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// # 1
#define BUT_1_PIO			PIOD
#define BUT_1_PIO_ID		ID_PIOD
#define BUT_1_IDX		28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

// # 2
#define BUT_2_PIO			PIOC
#define BUT_2_PIO_ID		ID_PIOC
#define BUT_2_IDX		31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

// # 3
#define BUT_3_PIO			PIOA
#define BUT_3_PIO_ID		ID_PIOA
#define BUT_3_IDX		19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

// motor 1
#define PIO_1			PIOD
#define PIO_1_ID		ID_PIOD
#define PIO_1_IDX		30
#define PIO_1_IDX_MASK (1u << PIO_1_IDX)

// motor 2
#define PIO_2			PIOA
#define PIO_2_ID		ID_PIOA
#define PIO_2_IDX		6
#define PIO_2_IDX_MASK (1u << PIO_2_IDX)

// motor 3
#define PIO_3			PIOC
#define PIO_3_ID		ID_PIOC
#define PIO_3_IDX		19
#define PIO_3_IDX_MASK (1u << PIO_3_IDX)

// motor 4
#define PIO_4			PIOA
#define PIO_4_ID		ID_PIOA
#define PIO_4_IDX		2
#define PIO_4_IDX_MASK (1u << PIO_4_IDX)


/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);
void pio_inits(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

QueueHandle_t QueueModo;
QueueHandle_t QueueSteps;
SemaphoreHandle_t xSemaphoreRTT;


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

void but1_callback(void) {
	char a = 180;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// libera semáforo
	xQueueSendFromISR(QueueModo, &a, &xHigherPriorityTaskWoken);
}

void but2_callback(void) {
	char b = 90;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// libera semáforo
	xQueueSendFromISR(QueueModo, &b, &xHigherPriorityTaskWoken);
}

void but3_callback(void) {
	char c = 45;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// libera semáforo
	xQueueSendFromISR(QueueModo, &c, &xHigherPriorityTaskWoken);
}

int flag_rtt = 0;

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		xSemaphoreGiveFromISR(xSemaphoreRTT,0);
	}
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/




static void task_modo(void *pvParameters){
	
	/* iniciliza botao */
	BUT_init();
	char passos;
	char id;
	
	for (;;) {
		if(xQueueReceive (QueueModo, &id, (TickType_t) 500 )){
			
			char consta; 
			consta = 0.17578125;
			passos = id/consta;
			
			if(id == 180){
				gfx_mono_ssd1306_init();
				gfx_mono_draw_string("180 graus" , 0, 0, &sysfont);
			} else if (id == 90){
				gfx_mono_ssd1306_init();
				gfx_mono_draw_string("90 graus " , 0, 0, &sysfont);
			} else if (id == 45){
				gfx_mono_ssd1306_init();
				gfx_mono_draw_string("45 graus " , 0, 0, &sysfont);
			}
		}
		
		/* envia nova frequencia para a task_led */
		xQueueSend(QueueSteps, &passos, 10);
		
		printf("passos: %d \n", passos);
		
	}
}

static void task_motor(void *pvParameters){
	int rodador = 0;
	pio_inits();
	char passos;
	for (;;) {
		if(xQueueReceive(QueueSteps,  &passos, (TickType_t) 500)){
				
				for(int i = 0; i < passos; i+=1){
					RTT_init(1000,5,RTT_MR_ALMIEN);
					if (xSemaphoreTake(xSemaphoreRTT, 1000)){
						rodador += 1;
						if(rodador > 4){
							rodador = 1;
						}
						if (rodador == 1){
						pio_set(PIO_1, PIO_1_IDX_MASK);
						pio_clear(PIO_2, PIO_2_IDX_MASK);
						pio_clear(PIO_3, PIO_3_IDX_MASK);
						pio_clear(PIO_4, PIO_4_IDX_MASK);
						}
						else if (rodador == 2){
						pio_clear(PIO_1, PIO_1_IDX_MASK);
						pio_set(PIO_2, PIO_2_IDX_MASK);
						pio_clear(PIO_3, PIO_3_IDX_MASK);
						pio_clear(PIO_4, PIO_4_IDX_MASK);
						}
						else if(rodador == 3){
						pio_clear(PIO_1, PIO_1_IDX_MASK);
						pio_clear(PIO_2, PIO_2_IDX_MASK);
						pio_set(PIO_3, PIO_3_IDX_MASK);
						pio_clear(PIO_4, PIO_4_IDX_MASK);
						}
						else if(rodador == 4){
						pio_clear(PIO_1, PIO_1_IDX_MASK);
						pio_clear(PIO_2, PIO_2_IDX_MASK);
						pio_clear(PIO_3, PIO_3_IDX_MASK);
						pio_set(PIO_4, PIO_4_IDX_MASK);
						}
				}
			}
		}
	
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP);
	pio_set_debounce_filter(BUT_1_PIO, BUT_1_IDX_MASK, 60);
	pio_handler_set(BUT_1_PIO,	BUT_1_PIO_ID,	BUT_1_IDX_MASK,	PIO_IT_FALL_EDGE,	but1_callback);
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);
	
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP);
	pio_set_debounce_filter(BUT_2_PIO, BUT_2_IDX_MASK, 60);
	pio_handler_set(BUT_2_PIO,	BUT_2_PIO_ID,	BUT_2_IDX_MASK,	PIO_IT_FALL_EDGE,	but2_callback);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2_PIO);
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);
	
	pmc_enable_periph_clk(BUT_3_PIO_ID);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP);
	pio_set_debounce_filter(BUT_3_PIO, BUT_3_IDX_MASK, 60);
	pio_handler_set(BUT_3_PIO,	BUT_3_PIO_ID,	BUT_3_IDX_MASK,	PIO_IT_FALL_EDGE,	but3_callback);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3_PIO);
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
	
}

void pio_inits(void){
	sysclk_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(PIO_1_ID);
	pmc_enable_periph_clk(PIO_2_ID);
	pmc_enable_periph_clk(PIO_3_ID);
	pmc_enable_periph_clk(PIO_4_ID);
	
	pio_set_output(PIO_1, PIO_1_IDX_MASK, 0, 0, 0);
	pio_set_output(PIO_2, PIO_2_IDX_MASK, 0, 0, 0);
	pio_set_output(PIO_3, PIO_3_IDX_MASK, 0, 0, 0);
	pio_set_output(PIO_4, PIO_4_IDX_MASK, 0, 0, 0);
	
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL){
		printf("falha em criar o semaforo \n");
	}
	
	QueueModo = xQueueCreate(32, sizeof(char));
	QueueSteps = xQueueCreate(32, sizeof(char));
	
	if (QueueModo == NULL)
	printf("falha em criar a queue \n");
	if (QueueSteps == NULL)
	printf("falha em criar a queue \n");
	
	if (xTaskCreate(task_modo, "modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create motor task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
