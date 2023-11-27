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

/* LED da placa */
#define LED_PIO PIOC       // periferico que controla o LED
#define LED_PIO_ID ID_PIOC // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX 8      // ID do LED no PIO
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX) // Mascara para CONTROLARMOS o LED

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

/* LED3 da placa oled */
#define LED3_PIO PIOB
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO_IDX 2
#define LED3_PIO_IDX_MASK (1 << LED3_PIO_IDX)

/* Butao1 da placa oled */
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1 << BUT1_PIO_IDX)

// RTT
#define RTT_PRESCALE 10 // 100 milisegundo de resolução
#define RTT_MAX_WAIT_TIME 40 // 4 segundos

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_TC_STACK_SIZE (1024 * 2 / sizeof(portSTACK_TYPE))
#define TASK_TC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_RTT_STACK_SIZE (1024 * 2 / sizeof(portSTACK_TYPE))
#define TASK_RTT_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_RTC_STACK_SIZE (1024 * 2 / sizeof(portSTACK_TYPE))
#define TASK_RTC_STACK_PRIORITY (tskIDLE_PRIORITY)

SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xSemaphoreRTC;
SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreUpdateTime;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void LED_init(int estado);
void LED1_init(int estado);
void LED2_init(int estado);
void LED3_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void led_toggle(Pio *pio, uint32_t mask);
static void configure_console(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTT_Handler(void);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void RTC_Handler(void);

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


void but1_callback(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  uint32_t current_hour, current_min, current_sec;
  char time[6];

  for (;;) {
    if (xSemaphoreTake(xSemaphoreUpdateTime, 1) == pdTRUE) {
      rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
      sprintf(time, "%u:%u:%u", current_hour, current_min, current_sec);
      gfx_mono_draw_string(time, 0, 17, &sysfont);
    }
  }
}

static void task_tc(void *pvParameters) {
  LED_init(0);
  LED1_init(0);
  TC_init(TC0, ID_TC1, 1, 4);
  TC_init(TC1, ID_TC4, 1, 5);
  tc_start(TC0, 1);
  tc_start(TC1, 1);
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

static void task_rtc(void *pvParameters) {
  LED3_init(1);
  BUT_init();
  /** Configura RTC */                                                                            
  calendar rtc_initial = {2023, 3, 19, 12, 15, 45 ,1};                                            
  RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);                                              
                                                                                                    
  /* Leitura do valor atual do RTC */           
  uint32_t current_hour, current_min, current_sec;
  uint32_t current_year, current_month, current_day, current_week;

  for (;;) {
    for (;;) {
      if (xSemaphoreTake(xSemaphoreBut1, 1000) == pdTRUE) {
        rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
        rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
        /* configura alarme do RTC para daqui 20 segundos */                                                                   
        rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
        rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
        break;
      }
    }

    for (;;) {
      if (xSemaphoreTake(xSemaphoreRTC, 1000) == pdTRUE) {
        led_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
        vTaskDelay(100);
        led_toggle(LED3_PIO, LED3_PIO_IDX_MASK);
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
  /* liga o clock do button */
  pmc_enable_periph_clk(BUT1_PIO_ID);

  /* conf botao como entrada */
  pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
  pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);

  pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback);

  pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
  pio_get_interrupt_status(BUT1_PIO);

  /* configura prioridae */
  NVIC_EnableIRQ(BUT1_PIO_ID);
  NVIC_SetPriority(BUT1_PIO_ID, 4);
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

void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);

	/** Muda o estado do LED (pisca) **/
	led_toggle(LED_PIO, LED_PIO_IDX_MASK);
}

void LED_init(int estado) {
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, estado, 0, 0);
};

void LED1_init(int estado) {
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, estado, 0, 0);
};

void LED2_init(int estado) {
  pmc_enable_periph_clk(LED2_PIO_ID);
  pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, estado, 0, 0);
};

void LED3_init(int estado) {
  pmc_enable_periph_clk(LED3_PIO_ID);
  pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, estado, 0, 0);
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

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void RTC_Handler(void) {
  uint32_t ul_status = rtc_get_status(RTC);

  /* seccond tick */
  if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreUpdateTime, &xHigherPriorityTaskWoken);
  }

  /* Time or date alarm */
  if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    // o código para irq de alame vem aqui
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreRTC, &xHigherPriorityTaskWoken);
  }

  rtc_clear_status(RTC, RTC_SCCR_SECCLR);
  rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
  rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
  rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
  rtc_clear_status(RTC, RTC_SCCR_CALCLR);
  rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
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

  /* Create semaphore to RTC */
  xSemaphoreRTC = xSemaphoreCreateBinary();
  if (xSemaphoreRTC == NULL) {
    printf("Error creating the semaphore");
  }

  /* Create semaphore to But1 */
  xSemaphoreBut1 = xSemaphoreCreateBinary();
  if (xSemaphoreBut1 == NULL) {
    printf("Error creating the semaphore");
  }

  /* Create semaphore to update time */
  xSemaphoreUpdateTime = xSemaphoreCreateBinary();
  if (xSemaphoreUpdateTime == NULL) {
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

  /* Create task to control rtc */
  if (xTaskCreate(task_rtc, "rtc", TASK_RTC_STACK_SIZE, NULL,
                  TASK_RTC_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create rtc task\r\n");
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
