#include "conf_board.h"
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* LED1 da placa oled */
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 0
#define LED1_PIO_IDX_MASK (1 << LED1_PIO_IDX)

/* LED2 da placa oled */
#define LED2_PIO PIOC
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO_IDX 30
#define LED2_PIO_IDX_MASK (1 << LED2_PIO_IDX)

// RTT
#define RTT_PRESCALE 10 // 100 milisegundo de resolução
#define RTT_MAX_WAIT_TIME 40 // 4 segundos

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_TC_STACK_SIZE (1024 * 4 / sizeof(portSTACK_TYPE))
#define TASK_TC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_RTT_STACK_SIZE (1024 * 4 / sizeof(portSTACK_TYPE))
#define TASK_RTT_STACK_PRIORITY (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphoreRTT;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void LED_init(int estado);
void LED2_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void led_toggle(Pio *pio, uint32_t mask);
static void configure_console(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTT_Handler(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow \n");
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
  gfx_mono_draw_string("oii", 0, 20, &sysfont);

  for (;;) {
  }
}

static void task_tc(void *pvParameters) {
  LED_init(0);
  TC_init(TC0, ID_TC1, 1, 4);
  tc_start(TC0, 1);
  for (;;) {
  }
}

static void task_rtt (void *pvParameters) {
  LED2_init(0);
  for (;;) {
		RTT_init(RTT_PRESCALE, RTT_MAX_WAIT_TIME, RTT_MR_ALMIEN);
		for (;;) {
			if (xSemaphoreTake(xSemaphoreRTT, 1000) == pdTRUE) {
				led_toggle(LED2_PIO, LED2_PIO_IDX_MASK);
        break;
			} 
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void led_toggle(Pio *pio, uint32_t mask) {
  if(pio_get_output_data_status(pio, mask)){
    pio_clear(pio, mask);
  } else {
    pio_set(pio, mask);
  }
}

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

  /* conf bot�o como entrada */
  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
  pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
  pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE, but_callback);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	led_toggle(LED1_PIO, LED1_PIO_IDX_MASK);
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0);
};

void LED2_init(int estado) {
  pmc_enable_periph_clk(LED2_PIO_ID);
  pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, estado, 0, 0);
};

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);

  if (rttIRQSource & RTT_MR_ALMIEN) {
    uint32_t ul_previous_time;
    ul_previous_time = rtt_read_timer_value(RTT);
    while (ul_previous_time == rtt_read_timer_value(RTT));
    rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
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

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}  
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

  /* Create semaphore to RTT */
	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL) {
		printf("Error creating the semaphore");
	}

  /* Create task to control oled */
  if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL,
                  TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create oled task\r\n");
  }

  /* Create task to control tc */
  if (xTaskCreate(task_tc, "tc", TASK_TC_STACK_SIZE, NULL,
                  TASK_TC_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create tc task\r\n");
  }

  /* Create task to control rtt */
  if (xTaskCreate(task_rtt, "rtt", TASK_RTT_STACK_SIZE, NULL,
                  TASK_RTT_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create rtt task\r\n");
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
