/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"

//#include "hal_stm_lvgl/tft/tft.h"
#include "../lvgl/lvgl.h"
#include "../lv_examples/lv_examples.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t timestamp = 0;
void SysTick_Handler (void)
{
   //сюда попадаем каждую 1 миллисекунду?
   timestamp++;
   HAL_IncTick();
   lv_tick_inc(1);    // 1 ms
}

int busySPIDMA = 0;

void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx);
//    HAL_GPIO_WritePin(GPIOC, CS_Pin, GPIO_PIN_SET);
	busySPIDMA = 0;
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
	while (busySPIDMA) {
		  HAL_Delay(1);
	}
	busySPIDMA = 1;

	uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

	ILI9341_Set_Address(area->x1, area->y1, area->x2, area->y2);

	HAL_GPIO_WritePin(GPIOC, CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, DC_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit_DMA(&hspi1, (unsigned char*)color_p, w*h*2);

//	HAL_SPI_Transmit(&hspi1, (unsigned char*)color_p, w*h*2, 10);
//	HAL_GPIO_WritePin(GPIOC, CS_Pin, GPIO_PIN_SET);
    lv_disp_flush_ready(disp);
}

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX/4];

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);

  lv_init();

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);
  HAL_Delay(25);

//  tft_init();

  lv_demo_widgets();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  HAL_Delay(25);
	  lv_task_handler();
  }

  while (1)
  {

	    ILI9341_Fill_Screen(WHITE);
	    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	    ILI9341_Draw_Text("FPS TEST, 40 loop 2 screens", 10, 10, BLACK, 1, WHITE);
	    HAL_Delay(2000);
	    ILI9341_Fill_Screen(GREEN);

        uint32_t Timer_Counter = 0;

        char counter_buff[30];
        ILI9341_Fill_Screen(WHITE);
        ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
        sprintf(counter_buff, "Timer counter value: %d", Timer_Counter*2);
        ILI9341_Draw_Text(counter_buff, 10, 10, BLACK, 1, WHITE);
	    HAL_Delay(1000);

        double seconds_passed = 2*((float)Timer_Counter / 20000);
        sprintf(counter_buff, "Time: %.3f Sec", seconds_passed);
        ILI9341_Draw_Text(counter_buff, 10, 30, BLACK, 2, WHITE);
	    HAL_Delay(1000);

        double timer_float = 20/(((float)Timer_Counter)/20000); //Frames per sec

	    ILI9341_Fill_Screen(YELLOW);
	    HAL_Delay(1000);

        static uint16_t x = 0;
        static uint16_t y = 0;

        char Temp_Buffer_text[40];

        for(uint16_t i = 0; i <= 10; i++)
        {
        sprintf(Temp_Buffer_text, "Counting: %d", i);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 10, BLACK, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 30, BLUE, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 50, RED, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 70, GREEN, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 90, BLACK, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 110, BLUE, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 130, RED, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 150, GREEN, 2, WHITE);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 170, WHITE, 2, BLACK);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 190, BLUE, 2, BLACK);
        ILI9341_Draw_Text(Temp_Buffer_text, 10, 210, RED, 2, BLACK);
        }

        HAL_Delay(1000);

//        //----------------------------------------------------------LINES EXAMPLE
//		ILI9341_Fill_Screen(WHITE);
//		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//		ILI9341_Draw_Text("Horizontal and Vertical lines", 10, 20, BLACK, 1, WHITE);
//		HAL_Delay(2000);
//		ILI9341_Fill_Screen(WHITE);
//
//		for(uint32_t i = 0; i < 30000; i++)
//		{
//				uint32_t random_num = 0;
//				uint16_t xr = 0;
//				uint16_t yr = 0;
//				uint16_t radiusr = 0;
//				uint16_t colourr = 0;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				xr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				yr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				radiusr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				colourr = random_num;
//
//				xr &= 0x01FF;
//				yr &= 0x01FF;
//				radiusr &= 0x001F;
//				//ili9341_drawpixel(xr, yr, WHITE);
//				ILI9341_Draw_Horizontal_Line(xr, yr, radiusr, colourr);
//				ILI9341_Draw_Vertical_Line(xr, yr, radiusr, colourr);
//		}
//
//		HAL_Delay(1000);
		//----------------------------------------------------------FILLED CIRCLES EXAMPLE
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("Filled Circles", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);
		ILI9341_Fill_Screen(WHITE);

		for(uint32_t i = 0; i < 1000; i++)
		{
				uint32_t random_num = 0;
				uint16_t xr = 0;
				uint16_t yr = 0;
				uint16_t radiusr = 0;
				uint16_t colourr = 0;
				random_num = HAL_RNG_GetRandomNumber(&hrng);
				xr = random_num;
				random_num = HAL_RNG_GetRandomNumber(&hrng);
				yr = random_num;
				random_num = HAL_RNG_GetRandomNumber(&hrng);
				radiusr = random_num;
				random_num = HAL_RNG_GetRandomNumber(&hrng);
				colourr = random_num;

				xr &= 0x01FF;
				yr &= 0x01FF;
				radiusr &= 0x001F;
				//ili9341_drawpixel(xr, yr, WHITE);
				ILI9341_Draw_Filled_Circle(xr, yr, radiusr/2, colourr);
		}
		HAL_Delay(1000);

//		//----------------------------------------------------------HOLLOW RECTANGLES EXAMPLE
//		ILI9341_Fill_Screen(WHITE);
//		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//		ILI9341_Draw_Text("Randomly placed and sized", 10, 10, BLACK, 1, WHITE);
//		ILI9341_Draw_Text("Rectangles", 10, 20, BLACK, 1, WHITE);
//		HAL_Delay(2000);
//		ILI9341_Fill_Screen(WHITE);
//
//		for(uint32_t i = 0; i < 20000; i++)
//		{
//				uint32_t random_num = 0;
//				uint16_t xr = 0;
//				uint16_t yr = 0;
//				uint16_t radiusr = 0;
//				uint16_t colourr = 0;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				xr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				yr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				radiusr = random_num;
//				random_num = HAL_RNG_GetRandomNumber(&hrng);
//				colourr = random_num;
//
//				xr &= 0x01FF;
//				yr &= 0x01FF;
//				radiusr &= 0x001F;
//				//ili9341_drawpixel(xr, yr, WHITE);
//				ILI9341_Draw_Hollow_Rectangle_Coord(xr, yr, xr+radiusr, yr+radiusr, colourr);
//		}
//		HAL_Delay(1000);


		//----------------------------------------------------------565 COLOUR EXAMPLE, Grayscale
		ILI9341_Fill_Screen(WHITE);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
		ILI9341_Draw_Text("Colour gradient", 10, 10, BLACK, 1, WHITE);
		ILI9341_Draw_Text("Grayscale", 10, 20, BLACK, 1, WHITE);
		HAL_Delay(2000);


		for(uint16_t i = 0; i <= (320); i++)
		{
				uint16_t Red = 0;
				uint16_t Green = 0;
				uint16_t Blue = 0;

				Red = i/(10);
				Red <<= 11;
				Green = i/(5);
				Green <<= 5;
				Blue = i/(10);



				uint16_t RGB_color = Red + Green + Blue;
				ILI9341_Draw_Rectangle(i, x, 1, 240, RGB_color);

		}
		HAL_Delay(2000);


		//----------------------------------------------------------IMAGE EXAMPLE, Snow Tiger
//		ILI9341_Fill_Screen(WHITE);
//		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
//		ILI9341_Draw_Text("RGB Picture", 10, 10, BLACK, 1, WHITE);
//		ILI9341_Draw_Text("TIGER", 10, 20, BLACK, 1, WHITE);
//		HAL_Delay(2000);
//		ILI9341_Draw_Image((const char*)snow_tiger, SCREEN_VERTICAL_2);
//		ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
//		HAL_Delay(5000);


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
