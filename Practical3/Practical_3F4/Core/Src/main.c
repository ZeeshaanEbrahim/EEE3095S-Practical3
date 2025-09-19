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
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Task 4 Constraint: MAX_ITER must be 100
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Defines for Task 4 ---
// Set your target image size here. Start with smaller values (e.g., 640x480) and increase.
#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
// The number of rows to process in each chunk.
#define TILE_HEIGHT 10
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
volatile uint64_t globalClockCycles = 0;
volatile uint32_t throughput_pixels_per_sec = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Modified for Task 4 to calculate a "chunk" of the image
uint64_t calculate_mandelbrot_chunk(int full_width, int full_height, int max_iterations, int y_start, int y_end);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- Manual DWT Cycle Counter Definitions ---
#define CoreDebug_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define TRCENA_Msk (1UL << 24)
#define CYCEN_Msk (1UL << 0)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  // Initialize DWT Cycle Counter
  CoreDebug_DEMCR |= TRCENA_Msk;
  DWT_CYCCNT = 0;
  DWT_CTRL |= CYCEN_Msk;

  // --- Task 4: Tiled Mandelbrot Calculation ---

  // Accumulators for the total calculation
  uint64_t total_checksum = 0;
  uint64_t total_execution_time_ms = 0;
  uint64_t total_clock_cycles = 0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Set pin high for entire calculation

  // Loop through the image in horizontal tiles
  for (int y_start = 0; y_start < IMG_HEIGHT; y_start += TILE_HEIGHT) {
      int y_end = y_start + TILE_HEIGHT;
      if (y_end > IMG_HEIGHT) {
          y_end = IMG_HEIGHT;
      }

      // Time each chunk individually
      globalStartTime = HAL_GetTick();
      uint32_t start_cycles = DWT_CYCCNT;

      uint64_t chunk_checksum = calculate_mandelbrot_chunk(IMG_WIDTH, IMG_HEIGHT, MAX_ITER, y_start, y_end);

      uint32_t end_cycles = DWT_CYCCNT;
      globalEndTime = HAL_GetTick();

      // Accumulate results from the chunk
      total_checksum += chunk_checksum;
      total_execution_time_ms += (globalEndTime - globalStartTime);
      total_clock_cycles += (end_cycles - start_cycles);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Set pin low after completion

  // Final results stored in global variables for debugging
  globalCheckSum = total_checksum;
  executionTime = total_execution_time_ms;
  globalClockCycles = total_clock_cycles;

  if (executionTime > 0) {
      uint64_t total_pixels = (uint64_t)IMG_WIDTH * IMG_HEIGHT;
      throughput_pixels_per_sec = (uint32_t)((total_pixels * 1000) / executionTime);
  } else {
      throughput_pixels_per_sec = 0;
  }

  /* Infinite loop */
  while (1)
  {

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
// This function now calculates a horizontal "chunk" of the Mandelbrot set,
// from row y_start to y_end. The width and height parameters refer to the FULL image.
uint64_t calculate_mandelbrot_chunk(int full_width, int full_height, int max_iterations, int y_start, int y_end)
{
    uint64_t sum = 0;
    int FRACT = 29;
    long long scale_factor = (1LL << FRACT);
    long long FOUR = (4LL << FRACT);

    long long x_min = -((5LL * scale_factor) >> 1);
    long long x_range = (7LL * scale_factor) >> 1;
    long long y_min = -(1LL * scale_factor);
    long long y_range = (2LL * scale_factor);

    long long step_x = x_range / full_width;
    long long step_y = y_range / full_height;

    // Calculate the starting y-coordinate for this specific chunk
    long long y0 = y_min + (long long)y_start * step_y;

    // Loop only over the rows assigned to this chunk
    for (int y = y_start; y < y_end; y++)
    {
        long long x0 = x_min;
        for (int x = 0; x < full_width; x++)
        {
            long long xi = 0, yi = 0;
            int iter = 0;

            while (iter < max_iterations)
            {
                long long xi2 = (xi * xi) >> FRACT;
                long long yi2 = (yi * yi) >> FRACT;
                if ((xi2 + yi2) > FOUR) break;

                long long temp = xi2 - yi2 + x0;
                yi = ((2LL * xi * yi) >> FRACT) + y0;
                xi = temp;
                iter++;
            }
            sum += (uint64_t)iter;
            x0 += step_x;
        }
        y0 += step_y; // Move to the next line for the next iteration
    }
    return sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            double x0 = ((double)x/width)*3.5 - 2.5;
            double y0 = (double)y/height * 2.0 - 1.0;
            double xi = 0, yi = 0;
            int i = 0;
            while (i < max_iterations)
            {
                double xi_sq = xi*xi;
                double yi_sq = yi*yi;
                if(xi_sq + yi_sq > 4.0) break;

                double temp = xi_sq - yi_sq;
                yi = 2.0*xi*yi + y0;
                xi = temp + x0;
                i++;
            }
            mandelbrot_sum += i;
        }
    }
    return mandelbrot_sum;
}
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* Can report file/line here */
}
#endif
