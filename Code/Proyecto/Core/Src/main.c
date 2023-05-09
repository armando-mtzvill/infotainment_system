/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This program creates two Tasks using the CMSIS_V1 API (FreeRTOS).
  * A sender Task puts a message in a Queue and the receiver Task gets the
  * message from the Queue.
  * The Tasks prints every one second using the UART protocol.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "myprintf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
#define BUTTON0 (GPIOA->IDR & GPIO_IDR_IDR0)
#define BUTTON1 (GPIOA->IDR & GPIO_IDR_IDR1)
#define BUTTON2 (GPIOA->IDR & GPIO_IDR_IDR2)
#define BUTTON3 (GPIOA->IDR & GPIO_IDR_IDR3)

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 //osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId sTaskHandle;
osThreadId rTaskHandle;
osMessageQId MsgQueueHandle;
osThreadId bTaskHandle;
osThreadId lTaskHandle;
osMessageQId MsgQueueHandle1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void USER_RCC_Init(void);
void USER_GPIO_Init(void);
void USER_USART1_Init(void);
void buttonTask(void const * argument);
void senderTask(void const * argument);
void receiverTask(void const * argument);
void LCDTask(void const * argument);
void USER_USART1_Transmit(uint8_t *pData, uint16_t size);
uint8_t USER_USART1_Receive(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int b=1;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  USER_RCC_Init( );
  USER_GPIO_Init( );
  USER_USART1_Init();

  LCD_Init( );//				inicializamos la libreria del LCD
  LCD_Cursor_ON( );//			cursor visible activo
  LCD_Clear( );//			borra la pantalla
  LCD_Set_Cursor( 1, 0 );//		posiciona cursor en la fila 1 columna 0
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(MsgQueueHandle, 1, int);
  MsgQueueHandle = osMessageCreate(osMessageQ(MsgQueueHandle), NULL);

  osMessageQDef(MsgQueueHandle1, 1, int);
  MsgQueueHandle1 = osMessageCreate(osMessageQ(MsgQueueHandle1), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Create the task, storing the handle. */
  osThreadDef(sTaskHandle, senderTask, osPriorityAboveNormal, 0, 128);
  sTaskHandle = osThreadCreate(osThread(sTaskHandle), NULL);

  osThreadDef(rTaskHandle, receiverTask, osPriorityAboveNormal, 0, 128);
  rTaskHandle = osThreadCreate(osThread(rTaskHandle), NULL);

  osThreadDef(bTaskHandle, buttonTask, osPriorityHigh, 0, 128);
  bTaskHandle = osThreadCreate(osThread(bTaskHandle), NULL);

  osThreadDef(lTaskHandle, LCDTask, osPriorityBelowNormal, 0, 128);
  lTaskHandle = osThreadCreate(osThread(lTaskHandle), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void USER_RCC_Init(void){
	RCC->APB2ENR	|=	 RCC_APB2ENR_USART1EN// 	USART 1 clock enable
					| 	 RCC_APB2ENR_IOPAEN;//		I/O port A clock enable

	RCC->APB2ENR    |=	 RCC_APB1ENR_TIM2EN;
}

void USER_GPIO_Init(void){
	//Input pull up
			GPIOA->BSRR = GPIO_BSRR_BS0;
			GPIOA->CRL &= ~GPIO_CRL_CNF0_0 & ~GPIO_CRL_MODE0;
			GPIOA->CRL |= GPIO_CRL_CNF0_1;

			GPIOA->BSRR = GPIO_BSRR_BS1;
			GPIOA->CRL &= ~GPIO_CRL_CNF1_0 & ~GPIO_CRL_MODE1;
			GPIOA->CRL |= GPIO_CRL_CNF1_1;

			GPIOA->BSRR = GPIO_BSRR_BS2;
			GPIOA->CRL &= ~GPIO_CRL_CNF2_0 & ~GPIO_CRL_MODE2;
			GPIOA->CRL |= GPIO_CRL_CNF2_1;

			GPIOA->BSRR = GPIO_BSRR_BS3;
			GPIOA->CRL &= ~GPIO_CRL_CNF3_0 & ~GPIO_CRL_MODE3;
			GPIOA->CRL |= GPIO_CRL_CNF3_1;


			//Output push-pull.
			GPIOA->BSRR = GPIO_BSRR_BS4;
			GPIOA->CRL &= ~GPIO_CRL_CNF4 & ~GPIO_CRL_MODE4_1;
			GPIOA->CRL |= GPIO_CRL_MODE4_0;

			GPIOA->BSRR = GPIO_BSRR_BS5;
			GPIOA->CRL &= ~GPIO_CRL_CNF5 & ~GPIO_CRL_MODE5_1;
			GPIOA->CRL |= GPIO_CRL_MODE5_0;


			GPIOA->BSRR = GPIO_BSRR_BS6;
			GPIOA->CRL &= ~GPIO_CRL_CNF6 & ~GPIO_CRL_MODE6_1;
			GPIOA->CRL |= GPIO_CRL_MODE6_0;


			GPIOA->BSRR = GPIO_BSRR_BS7;
			GPIOA->CRL &= ~GPIO_CRL_CNF7 & ~GPIO_CRL_MODE7_1;
			GPIOA->CRL |= GPIO_CRL_MODE7_0;

			//USART
			GPIOA->CRH	&=	~GPIO_CRH_CNF9_0 & ~GPIO_CRH_MODE9_1;
			GPIOA->CRH	|=	 GPIO_CRH_CNF9_1 |	GPIO_CRH_MODE9_0;

			GPIOA->CRH	&=	~GPIO_CRH_CNF10_1 & ~GPIO_CRH_MODE10;
			GPIOA->CRH	|=	 GPIO_CRH_CNF10_0;

}

void USER_USART1_Init(void){
  //complete the configurations for 9600 baud rate, 1 start_8-bit data_1 stop bits
	USART1->BRR	=	0X1D4C;//			9600 BPS -> 468.75, 0% ERROR
		USART1->CR1	&=	~USART_CR1_M//		1 start bit, 8 data bits
					&	~USART_CR1_WAKE//	idle line
					&	~USART_CR1_PCE//	parity control disabled
					&	~USART_CR1_TXEIE//	interrupt disabled (empty transmit reg)
					&	~USART_CR1_TCIE//	interrupt disabled (transmission complete)
					&	~USART_CR1_RXNEIE//	interrupt disabled (data received)
					&	~USART_CR1_IDLEIE//	interrupt disabled (idle line detected)
					&	~USART_CR1_RWU//	receiver active mode
					&	~USART_CR1_SBK;//	no break character transmitted
		USART1->CR1	|=	USART_CR1_UE//		USART enabled
					|	USART_CR1_TE//		transmitter enabled
					|	USART_CR1_RE;//		receiver enabled
		USART1->CR2	&=	~USART_CR2_STOP;//	1 stop bit
}

void buttonTask(void const * argument)
{
  osStatus s_event1;
  osEvent r_event1;
  /* Infinite loop */
  int a;

  for(;;)
  {
	  	  //b = 1;
	  	  a = 0;
	  	  //Teclado matricial
	  	  GPIOA->ODR |= GPIO_ODR_ODR4;
	  	  GPIOA->ODR &= ~GPIO_ODR_ODR5;
	  	  GPIOA->ODR |= GPIO_ODR_ODR6;
	  	  GPIOA->ODR |= GPIO_ODR_ODR7;

	  	  if (BUTTON2 == 0){
	  	  	HAL_Delay(10);
	  	  	a = 56;
	  	  }

	  	  GPIOA->ODR |= ~GPIO_ODR_ODR4;
	  	  GPIOA->ODR |= GPIO_ODR_ODR5;
	  	  GPIOA->ODR &= ~GPIO_ODR_ODR6;
	  	  GPIOA->ODR |= GPIO_ODR_ODR7;

	  	  if (BUTTON1 == 0){
	  	  	HAL_Delay(10);
	  	  	a = 54;
	  	  }
	  	  else if (BUTTON2 == 0){
	  	  	HAL_Delay(50);
	  	  	a = 53;
	  	  }
	  	  else if (BUTTON3 == 0){
	  	  	HAL_Delay(10);
	  	  	a = 52;
	  	  }

	  	  GPIOA->ODR |= GPIO_ODR_ODR4;
	  	  GPIOA->ODR |= GPIO_ODR_ODR5;
	  	  GPIOA->ODR |= GPIO_ODR_ODR6;
	  	  GPIOA->ODR &= ~GPIO_ODR_ODR7;

	  	  if (BUTTON2 == 0){
	  	  	HAL_Delay(10);
	  	  	a = 50;
	  	  }

	  	  if(a!=0 || b==0){
	  		s_event1 = osMessagePut(MsgQueueHandle, a, 1000);
	  		osDelay(100);
	  		r_event1 = osMessageGet(MsgQueueHandle, 1000);
	  	  }
  }
}

void senderTask(void const * argument)
{
	  /* USER CODE BEGIN 5 */
	osStatus s_event;
	osEvent r_event;
	  /* Infinite loop */
	for(;;){
		  r_event = osMessageGet(MsgQueueHandle, 1000);
		  if( r_event.status == osEventMessage ){
			  if(b==1){
				  if (r_event.value.v == 50){
					  USER_USART1_Transmit("2",sizeof("2"));
					  b = 0;
				  }
				  else if (r_event.value.v == 56){
					  USER_USART1_Transmit("8",sizeof("8"));
					  b = 0;
				  }
				  else if (r_event.value.v == 53){
					  USER_USART1_Transmit("5",sizeof("5"));
				  }
				  else if (r_event.value.v == 54){
					  USER_USART1_Transmit("6",sizeof("6"));
					  b = 0;
				  }
				  else if (r_event.value.v == 52){
					  USER_USART1_Transmit("4",sizeof("4"));
					  b = 0;
					  }
				  else if (r_event.value.v == 0){
							  }
			  }


			  //USER_USART1_Transmit(r_event.value.v, sizeof(r_event.value.v));
			  osDelay(100);
			  s_event = osMessagePut(MsgQueueHandle, 0, 1);
		  }
	}
	  /* USER CODE END 5 */
}


void receiverTask(void const * argument)
{
	  /* USER CODE BEGIN 5 */
	osStatus s_event3;
	osEvent r_event3;

	  /* Infinite loop */
	  uint8_t d;

	  for(;;)
	  {
		  	  HAL_Delay(90);
		  	  d = USER_USART1_Receive();
		  	  if(d!='{' && b==0){
		  		s_event3 = osMessagePut(MsgQueueHandle1, d, 1000);
		  		osDelay(100);
		  		r_event3 = osMessageGet(MsgQueueHandle1, 2000);
		  	  }
	  }
}

void LCDTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
osStatus s_event4;
osEvent r_event4;
uint8_t data;
int x = 0;
int y = 1;
char a;
  /* Infinite loop */
for(;;){
	  r_event4 = osMessageGet(MsgQueueHandle1, 1);
	  if( r_event4.status == osEventMessage ){
		  data = r_event4.value.v;
		  a = (char) data;
		  HAL_Delay(110);
		  if (data == 46){
			  b = 1;
		  }
		  if (data != 46){
			  if (data == 125){
				  LCD_Clear( );//			borra la pantalla
				  LCD_Set_Cursor( 1, 0 );
				  x = 0;
			  }
			  else if (data == 124){
				  if (sizeof(data))
				  LCD_Put_Char("a" );
				  x++;
					  }
			  else if (data == 13){
				  y = 2;
				  x = 0;
				  LCD_Set_Cursor( y, x);
				  LCD_Put_Str("                ");
				  LCD_Set_Cursor( y, x);
			  }
			  else if (data == 95){
				  y = 2;
				  LCD_Set_Cursor( y, 0);
			  }
			  else {
				  if (x < 16){

					  if (sizeof(a))
					  LCD_Put_Char(a );
					  x++;
				  }
				  else{

					  if (sizeof(data))
					  LCD_Put_Char( data );

					  x++;
				  }
			  }
		  }
	  s_event4 = osMessagePut(MsgQueueHandle1, 1, 1);
	  }
  /* USER CODE END 5 */
}

}
/* USER CODE END 4 */

void USER_USART1_Transmit(uint8_t *pData, uint16_t size){
	for(int i=0; i < size; i++){
		while((USART1->SR & USART_SR_TXE)==0){
			return;
		}
		USART1->DR = *pData++;
	}
}

uint8_t USER_USART1_Receive(void){
	while ((USART1->SR & USART_SR_RXNE)==0){
		return '{';
	}
	return USART1->DR;
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1)
	  {
	  }
	  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
