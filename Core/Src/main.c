/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ntc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 通道、引脚、采样点数、参考电压等板级参数由 bsp_adc.h 统一定义，
 * main 不再重复声明。
 *
 * 1 = 开机扫描所有候选模拟脚（诊断用，会多打 16 行、多花 64ms）；0 = 关闭 */
#define ADC_DIAG_SCAN     0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA 缓冲区、通道选择、校准、缓存失效全部由 BSP 层（bsp_adc.c）负责，
 * main 不再持有缓冲区，也不再直接调用 HAL 的 ADC 接口。 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if (ADC_DIAG_SCAN != 0)
/**
  * @brief  模拟脚占用扫描。
  *         把 ADC1 能接的全部候选脚（PA/PB/PC 共 16 个）逐个从模拟输入临时切成
  *         数字输入，先开内部上拉读 IDR、再开内部下拉读 IDR，然后恢复成模拟输入。
  *         内部上/下拉约 40kΩ，不用接任何外部信号就能判断每根脚的真实状态：
  *           up=1 dn=0 -> 高阻/空闲，可以用
  *           up=0 dn=0 -> 被外部低阻拉到 GND（板上电路占了这根脚）
  *           up=1 dn=1 -> 被外部驱动为高
  *         全程约 64ms，扫完 ADC 才启动 DMA，不影响采集。
  * @retval None
  */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       adc1_ch;
  const char   *name;
} ADC_PinProbe;

static const ADC_PinProbe adc_probe_list[] =
{
  { GPIOA, GPIO_PIN_0, 16, "PA0" },
  { GPIOA, GPIO_PIN_1, 17, "PA1" },
  { GPIOA, GPIO_PIN_2, 14, "PA2" },
  { GPIOA, GPIO_PIN_3, 15, "PA3" },
  { GPIOA, GPIO_PIN_4, 18, "PA4" },
  { GPIOA, GPIO_PIN_5, 19, "PA5" },
  { GPIOA, GPIO_PIN_6,  3, "PA6" },
  { GPIOA, GPIO_PIN_7,  7, "PA7" },
  { GPIOB, GPIO_PIN_0,  9, "PB0" },
  { GPIOB, GPIO_PIN_1,  5, "PB1" },
  { GPIOC, GPIO_PIN_0, 10, "PC0" },
  { GPIOC, GPIO_PIN_1, 11, "PC1" },
  { GPIOC, GPIO_PIN_2, 12, "PC2" },
  { GPIOC, GPIO_PIN_3, 13, "PC3" },
  { GPIOC, GPIO_PIN_4,  4, "PC4" },
  { GPIOC, GPIO_PIN_5,  8, "PC5" },
};

#define ADC_PROBE_COUNT   (sizeof(adc_probe_list) / sizeof(adc_probe_list[0]))

static void ADC_ProbeOnePin(const ADC_PinProbe *p)
{
  GPIO_InitTypeDef g = {0};
  uint32_t up;
  uint32_t dn;

  g.Pin  = p->pin;
  g.Mode = GPIO_MODE_INPUT;

  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(p->port, &g);
  HAL_Delay(2);
  up = (HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET) ? 1U : 0U;

  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(p->port, &g);
  HAL_Delay(2);
  dn = (HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET) ? 1U : 0U;

  /* 恢复成模拟输入（高阻，不驱动任何东西） */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(p->port, &g);

  printf("  %s (ADC1_INP%02u) up=%lu dn=%lu -> ",
         p->name, (unsigned int)p->adc1_ch,
         (unsigned long)up, (unsigned long)dn);

  if ((up == 1U) && (dn == 0U))
  {
    printf("空闲 FREE\r\n");
  }
  else if ((up == 0U) && (dn == 0U))
  {
    printf("外部拉到 GND\r\n");
  }
  else if ((up == 1U) && (dn == 1U))
  {
    printf("外部拉高\r\n");
  }
  else
  {
    printf("异常：下拉仍为高\r\n");
  }
}

static void ADC_ScanPins(void)
{
  uint32_t k;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  printf("\r\n--- ADC1 候选脚占用扫描 ---\r\n");
  for (k = 0U; k < ADC_PROBE_COUNT; k++)
  {
    ADC_ProbeOnePin(&adc_probe_list[k]);
  }
  printf("--- 上面标 FREE 的脚都可以接信号 ---\r\n\r\n");
}
#endif /* ADC_DIAG_SCAN */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int16_t temperature_x10;

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 只需要调 NTC 模块：探头型号、R25、B 值、接哪个 ADC 通道，
   * 全部在 ntc.c 末尾的接线表里 */
  if (Ntc_Board_Init() != 0U)
  {
    printf("NTC init failed\r\n");
    Error_Handler();
  }

#if (ADC_DIAG_SCAN != 0)
  /* 诊断用：扫一遍 ADC1 候选脚，找出哪些没被板子占用 */
  ADC_ScanPins();
#endif

  printf("\r\n<<< NTC 测温 start >>>\r\n");
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 适配层：刷新 ADC 数据 → 更新所有 NTC 实例 */
    Ntc_Board_Update();

    printf("\r\n==== NTC 采集 ====\r\n");
    printf("Uadc: %4u mV   Unode: %4u mV\r\n",
           (unsigned int)ntc[NTC_INST_TEMP].pin_voltage_mv,
           (unsigned int)ntc[NTC_INST_TEMP].node_voltage_mv);
    printf("NTC : %6lu ohm\r\n", (unsigned long)ntc[NTC_INST_TEMP].res_ohm);

    temperature_x10 = ntc[NTC_INST_TEMP].temp_x10;

    if (ntc[NTC_INST_TEMP].valid == 0U)
    {
      printf("TEMP: -- (fault %u)\r\n", (unsigned int)ntc[NTC_INST_TEMP].fault);
    }
    else if (temperature_x10 < 0)
    {
      printf("TEMP: -%d.%d C\r\n",
             (int)((-temperature_x10) / 10), (int)((-temperature_x10) % 10));
    }
    else
    {
      printf("TEMP: %d.%d C\r\n",
             (int)(temperature_x10 / 10), (int)(temperature_x10 % 10));
    }

    HAL_Delay(1000);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
#ifdef USE_FULL_ASSERT
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
