/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100 // change between 100,250,500,750,1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BITS 16
#define FP_UNITY (1 << BITS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint64_t globalCheckSum = 0;
volatile uint32_t globalStartTime = 0;
volatile uint32_t globalEndTime = 0;
volatile uint32_t executionTime = 0;
// New variables for Task 3
volatile uint64_t globalClockCycles = 0;
volatile uint32_t throughput_pixels_per_sec = 0;

static const int image_dimensions[] = {128, 160, 192, 224, 256};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Performs an optimized 32x32->64 bit multiplication.
 * @note   This is faster than a standard C (int64_t) cast on a Cortex-M0
 * because it uses hardware-friendly 16x16 bit multiplications.
 * @param  x: The first 32-bit signed integer.
 * @param  y: The second 32-bit signed integer.
 * @retval The 64-bit signed result of x * y.
 */
static inline int64_t multiply_32x32_to_64(int32_t x, int32_t y) {
    // Break down each 32-bit number into 16-bit high and low parts
    int16_t x_high = x >> 16;
    uint16_t x_low = x & 0xFFFF;
    int16_t y_high = y >> 16;
    uint16_t y_low = y & 0xFFFF;

    // Perform the four 16x16 bit multiplications
    int32_t prod_hh = (int32_t)x_high * y_high;
    int32_t prod_hl = (int32_t)x_high * y_low;
    int32_t prod_lh = (int32_t)x_low * y_high;
    uint32_t prod_ll = (uint32_t)x_low * y_low;

    // Combine the partial products to get the full 64-bit result
    int64_t cross_products = (int64_t)prod_hl + (int64_t)prod_lh;
    int64_t result = ((int64_t)prod_hh << 32) + ((int64_t)cross_products << 16) + prod_ll;

    return result;
}
// Helper functions for fixed-point arithmetic
static inline int32_t qmul(int32_t x, int32_t y) {
    // Use the optimized helper function for the multiplication
    int64_t product = multiply_32x32_to_64(x, y);
    // Add rounding factor and shift as before
    return (int32_t)((product + (1LL << (BITS - 1))) >> BITS);
}
static inline int32_t qmul2(int32_t x, int32_t y) {
    int64_t product = multiply_32x32_to_64(x, y);
    return (int32_t)((product + (1LL << (BITS - 2))) >> (BITS - 1));
}
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for (int i = 0; i < (sizeof(image_dimensions) / sizeof(image_dimensions[0])); i++) {
        int dim = image_dimensions[i];

        globalStartTime = HAL_GetTick();
        uint32_t start_systick_val = SysTick->VAL;
        // Fixed-point test
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        globalCheckSum = calculate_mandelbrot_fixed_point_arithmetic(dim, dim, MAX_ITER);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        // Capture end time (wall-clock and cycles)
        uint32_t end_systick_val = SysTick->VAL;
        globalEndTime = HAL_GetTick();
        executionTime = globalEndTime - globalStartTime;

        // 2. CPU Clock Cycles
		uint32_t ticks_elapsed = executionTime;
		// The SysTick->LOAD register holds the number of cycles per tick minus 1.
		// For a 48MHz clock and 1ms tick, LOAD is 47999, so there are 48000 cycles per tick.
		uint32_t cycles_per_tick = SysTick->LOAD + 1;
		// Since SysTick is a down-counter, start_systick_val will be greater than end_systick_val
		globalClockCycles = (uint64_t)ticks_elapsed * cycles_per_tick + (start_systick_val - end_systick_val);

		// 3. Throughput (pixels per second)
		if (executionTime > 0) {
			uint64_t total_pixels = (uint64_t)dim * dim;
			// Formula: throughput = (total_pixels * 1000) / executionTime_ms
			throughput_pixels_per_sec = (uint32_t)((total_pixels * 1000) / executionTime);
		} else {
			throughput_pixels_per_sec = 0; // Avoid division by zero
		}

        HAL_Delay(1000); // Small delay for visual cue

        // Double-precision test
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//        globalStartTime = HAL_GetTick();
//        globalCheckSum = calculate_mandelbrot_double(dim, dim, MAX_ITER);
//        globalEndTime = HAL_GetTick();
//        executionTime = globalEndTime - globalStartTime;
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//        HAL_Delay(500); // Small delay for visual cue
//
//        HAL_Delay(1000); // Pause for 1s between test runs
    }
    /* USER CODE END 3 */
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

  /* 1) Enable HSI and configure PLL (HSI 8MHz * 6 = 48MHz) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  /* Configure PLL: use HSI as PLL source and MUL6 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;      // HSI used
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;              // 8MHz * 6 = 48MHz

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2) Configure Flash latency: 48 MHz needs 1 wait state on F0 */
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /* 3) Select PLL as system clock source and configure bus dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Use PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // HCLK = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;          // PCLK1 = HCLK

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {
    uint64_t mandelbrot_sum = 0;

    const int32_t x_left = -((5 * FP_UNITY) / 2); // -2.5 in BIT16
    const int32_t y_top = -FP_UNITY;              // -1.0 in BIT16

    const int32_t x_step = (int32_t)(((int64_t)7 * FP_UNITY + (2 * width) / 2) / (2 * width));
    const int32_t y_step = (int32_t)(((int64_t)2 * FP_UNITY + height / 2) / height);
    const int32_t FOUR = 4 * FP_UNITY;

    int32_t y0 = y_top;
    for (int y = 0; y < height; ++y) {
        int32_t x0 = x_left;
        for (int x = 0; x < width; ++x) {
            int32_t xi = 0, yi = 0;
            int i = 0;
            while (i < max_iterations) {
                int32_t xi2 = qmul(xi, xi);
                int32_t yi2 = qmul(yi, yi);
                if (xi2 + yi2 > FOUR) break;
                int32_t temp = xi2 - yi2 + x0;
                yi = qmul2(xi, yi) + y0;
                xi = temp;
                ++i;
            }
            mandelbrot_sum += i;
            x0 += x_step;
        }
        y0 += y_step;
    }
    return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations)
{
	uint64_t sum = 0;

		    double x_min   = -2.5;
		    double x_range =  3.5;
		    double y_min   = -1.0;
		    double y_range =  2.0;

		    // step sizes (computed once per dimension)
		    double step_x = x_range / (double)width;
		    double step_y = y_range / (double)height;

		    for (int y = 0; y < height; y++) {
		        double y0 = y_min + (double)y * step_y;

		        double x0 = x_min;
		        for (int x = 0; x < width; x++) {
		            double xi = 0.0, yi = 0.0;
		            int iter = 0;

		            while (iter < max_iterations && (xi*xi + yi*yi) <= 4.0) {
		                double temp = (xi * xi) - (yi * yi) + x0;
		                yi = (2.0 * xi * yi) + y0;
		                xi = temp;
		                iter++;
		            }

		            sum += (uint64_t)iter;
		            x0 += step_x;
		        }
		    }

		    return sum;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
