/**
 * @file    bsp_adc.c
 * @author  华畅
 * @brief   ADC 板级支持包实现（可移植）：单 ADC 多通道扫描 + DMA 循环
 * @version 0.5
 * @date    2026-03-30
 *
 * @copyright Copyright (c) 2026
 *
 * @note    本文件里唯一与具体板子相关的部分，是下面的 s_ch_cfg[] 通道表。
 *          其余逻辑（自检、模式校正、校准、启动、平均、换算）与硬件无关，
 *          换板子时不用改。
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"

#include "adc.h"    /* CubeMX 生成：BSP_ADC_HANDLE 对应的 ADC 句柄 */

/* Private define ------------------------------------------------------------*/

/* DMA 缓冲区总长度：通道数 × 每通道点数 */
#define BSP_ADC_BUF_WORDS               ((uint32_t)BSP_ADC_CH_SUM * BSP_ADC_SAMPLE_LEN)

/* 毫伏换算的中间量类型：用 64 位避免 raw × vref 溢出 */
#define BSP_ADC_VOLTAGE_T               uint64_t

/* H7 才有 ConversionDataManagement；老型号（F1/F4 等）没有这个字段 */
#if defined(ADC_CONVERSIONDATA_DMA_CIRCULAR)
    #define BSP_ADC_HAS_DATA_MGMT       (1)
#else
    #define BSP_ADC_HAS_DATA_MGMT       (0)
#endif

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief  单个逻辑通道的物理配置
 *
 * @note   本结构体只在 BSP 内部使用，头文件不暴露 HAL 类型。
 */
typedef struct Bsp_Adc_ChannelCfg_Struct_s
{
    uint32_t      adc_channel;      /* ADC_CHANNEL_x                   */
    GPIO_TypeDef *port;             /* 模拟引脚所在端口                 */
    uint16_t      pin;              /* 模拟引脚                         */
    uint32_t      sampling_time;    /* ADC_SAMPLETIME_xxx              */
} Bsp_Adc_ChannelCfg;

/* ====================== 板级差异：通道表（移植时改这里） ======================
 *
 * 顺序必须与 Bsp_Adc_Channel_Enum 一致，也必须与 CubeMX 里的 Rank 顺序一致。
 * 每行 = { 物理通道号, 引脚端口, 引脚号, 采样时间 }
 *
 * 采样时间怎么选（按信号源阻抗）：
 *   < 1 kΩ      : ADC_SAMPLETIME_8CYCLES_5
 *   1 ~ 10 kΩ   : ADC_SAMPLETIME_47CYCLES_5
 *   10 ~ 50 kΩ  : ADC_SAMPLETIME_387CYCLES_5
 * ============================================================================ */
static const Bsp_Adc_ChannelCfg s_ch_cfg[BSP_ADC_CH_SUM] =
{
    /* 逻辑通道 0：NTC 温度探头 —— PC1 / ADC1_INP11，源阻抗较高用长采样 */
    { 11U, GPIOC, GPIO_PIN_1, ADC_SAMPLETIME_387CYCLES_5 },
};

/* Private variables ---------------------------------------------------------*/

/* DMA 目标缓冲区
 * 三个必须：
 *   ① 静态/全局：DMA 会持续写入，放栈上会被踩
 *   ② 32 字节对齐：SCB_InvalidateDCache_by_Addr 要求
 *   ③ volatile：强制 CPU 每次都从内存读
 * 它落在 .bss 开头（本板 0x24000000 起），该区域被 MPU_Config() 配成非缓存。
 *
 * 布局（扫描模式）：ch0 ch1 ... chN-1 | ch0 ch1 ... | ...（共 SAMPLE_LEN 轮）
 */
__ALIGNED(32) static volatile uint16_t s_adc_dma_buf[BSP_ADC_BUF_WORDS];

/* 换算结果 */
Bsp_Adc_Data_Struct bsp_adc_data;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  检查 DMA 缓冲区是否落在非缓存窗口内
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
static Bsp_Adc_Status_Enum bsp_adc_check_buffer(void)
{
    uint32_t addr = (uint32_t)(uintptr_t)&s_adc_dma_buf[0];

    if ((addr < BSP_ADC_NC_WINDOW_BASE)
        || (addr >= (BSP_ADC_NC_WINDOW_BASE + BSP_ADC_NC_WINDOW_SIZE)))
    {
        return BSP_ADC_ERR_BUFFER;
    }

    return BSP_ADC_OK;
}

/**
 * @brief  保证 ADC 工作在「扫描 + 连续 + DMA 循环」模式
 *
 * @note   若 CubeMX 里的设置与通道数不一致，这里直接改写并重新初始化，
 *         避免"只转换一个通道导致 DMA 永不完成"。
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
static Bsp_Adc_Status_Enum bsp_adc_config_mode(void)
{
    uint8_t need_reinit = 0U;
    uint32_t want_scan;

    want_scan = (BSP_ADC_CH_SUM > 1) ? ADC_SCAN_ENABLE : ADC_SCAN_DISABLE;

    if (BSP_ADC_HANDLE.Init.NbrOfConversion != (uint32_t)BSP_ADC_CH_SUM)
    {
        BSP_ADC_HANDLE.Init.NbrOfConversion = (uint32_t)BSP_ADC_CH_SUM;
        need_reinit = 1U;
    }

    if (BSP_ADC_HANDLE.Init.ScanConvMode != want_scan)
    {
        BSP_ADC_HANDLE.Init.ScanConvMode = want_scan;
        need_reinit = 1U;
    }

    if (BSP_ADC_HANDLE.Init.ContinuousConvMode != ENABLE)
    {
        BSP_ADC_HANDLE.Init.ContinuousConvMode = ENABLE;
        need_reinit = 1U;
    }

    if (BSP_ADC_HANDLE.Init.Overrun != ADC_OVR_DATA_OVERWRITTEN)
    {
        BSP_ADC_HANDLE.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
        need_reinit = 1U;
    }

#if (BSP_ADC_HAS_DATA_MGMT != 0)
    if (BSP_ADC_HANDLE.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DMA_CIRCULAR)
    {
        BSP_ADC_HANDLE.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
        need_reinit = 1U;
    }
#endif

    if (need_reinit != 0U)
    {
        if (HAL_ADC_Init(&BSP_ADC_HANDLE) != HAL_OK)
        {
            return BSP_ADC_ERR_CHANNEL;
        }
    }

    return BSP_ADC_OK;
}

/**
 * @brief  按通道表配置引脚与 Rank
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
static Bsp_Adc_Status_Enum bsp_adc_config_channels(void)
{
    GPIO_InitTypeDef      gpio;
    ADC_ChannelConfTypeDef channel;
    uint32_t              index;

    /* 三个端口时钟都打开，换脚时不必再关心是哪个口 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    for (index = 0U; index < (uint32_t)BSP_ADC_CH_SUM; index++)
    {
        gpio = (GPIO_InitTypeDef){0};
        gpio.Pin = s_ch_cfg[index].pin;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(s_ch_cfg[index].port, &gpio);

        channel = (ADC_ChannelConfTypeDef){0};
        /* HAL 里 ADC_CHANNEL_n 的数值就等于通道号 n，可直接强转 */
        channel.Channel = s_ch_cfg[index].adc_channel;
        channel.Rank = index + 1U;                  /* Rank 从 1 开始 */
        channel.SamplingTime = s_ch_cfg[index].sampling_time;
        channel.SingleDiff = ADC_SINGLE_ENDED;
        channel.OffsetNumber = ADC_OFFSET_NONE;
        channel.Offset = 0U;
        channel.OffsetSignedSaturation = DISABLE;

        if (HAL_ADC_ConfigChannel(&BSP_ADC_HANDLE, &channel) != HAL_OK)
        {
            return BSP_ADC_ERR_CHANNEL;
        }
    }

    return BSP_ADC_OK;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化并启动 ADC
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
Bsp_Adc_Status_Enum Bsp_Adc_Init(void)
{
    Bsp_Adc_Status_Enum status;
    uint32_t index;

    /* 结果结构体清零 */
    for (index = 0U; index < (uint32_t)BSP_ADC_CH_SUM; index++)
    {
        bsp_adc_data.raw_avg[index] = 0U;
        bsp_adc_data.raw_min[index] = 0U;
        bsp_adc_data.raw_max[index] = 0U;
        bsp_adc_data.voltage_mv[index] = 0U;
    }
    bsp_adc_data.valid = 0U;

    /* ① 缓冲区位置自检（放错地方后面全是 0，先查） */
    status = bsp_adc_check_buffer();
    if (status != BSP_ADC_OK)
    {
        return status;
    }

    /* ② 工作模式：扫描 + 连续 + DMA 循环 */
    status = bsp_adc_config_mode();
    if (status != BSP_ADC_OK)
    {
        return status;
    }

    /* ③ 引脚与各 Rank */
    status = bsp_adc_config_channels();
    if (status != BSP_ADC_OK)
    {
        return status;
    }

    /* ④ 校准：H7 必须做，否则误差可达几十 LSB */
    if (HAL_ADCEx_Calibration_Start(&BSP_ADC_HANDLE,
                                    BSP_ADC_CALIB_MODE_CODE,
                                    ADC_SINGLE_ENDED) != HAL_OK)
    {
        return BSP_ADC_ERR_CALIB;
    }

    /* ⑤ 启动 DMA 循环搬运；ADC 连续转换，一次启动即自由运行 */
    return Bsp_Adc_Start();
}

/**
 * @brief  重新启动 DMA 搬运
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
Bsp_Adc_Status_Enum Bsp_Adc_Start(void)
{
    if (HAL_ADC_Start_DMA(&BSP_ADC_HANDLE,
                          (uint32_t *)(uintptr_t)&s_adc_dma_buf[0],
                          BSP_ADC_BUF_WORDS) != HAL_OK)
    {
        return BSP_ADC_ERR_START;
    }

    return BSP_ADC_OK;
}

/**
 * @brief  停止 ADC 转换与 DMA 搬运
 *
 * @param  无
 * @return 无
 */
void Bsp_Adc_Stop(void)
{
    (void)HAL_ADC_Stop_DMA(&BSP_ADC_HANDLE);
}

/**
 * @brief  更新一轮结果
 *
 * @param  无
 * @return 无
 */
void Bsp_Adc_Update(void)
{
    uint32_t raw_sum;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t raw;
    uint32_t ch;
    uint32_t i;
    uint32_t pos;
    BSP_ADC_VOLTAGE_T voltage;

    /* 缓冲区在非缓存区时这句是空操作；若将来挪到缓存区，它保证读到新数据 */
    BSP_ADC_CACHE_INVALIDATE((uint32_t *)(uintptr_t)&s_adc_dma_buf[0],
                             sizeof(s_adc_dma_buf));

    for (ch = 0U; ch < (uint32_t)BSP_ADC_CH_SUM; ch++)
    {
        raw_min = 0xFFFFU;
        raw_max = 0U;
        raw_sum = 0U;

        /* 扫描模式下该通道位于每轮的偏移 ch 处，共 SAMPLE_LEN 轮 */
        for (i = 0U; i < BSP_ADC_SAMPLE_LEN; i++)
        {
            pos = (i * (uint32_t)BSP_ADC_CH_SUM) + ch;
            raw = s_adc_dma_buf[pos];

            raw_sum += raw;

            if (raw < raw_min)
            {
                raw_min = raw;
            }

            if (raw > raw_max)
            {
                raw_max = raw;
            }
        }

        bsp_adc_data.raw_avg[ch] = (uint16_t)(raw_sum / BSP_ADC_SAMPLE_LEN);
        bsp_adc_data.raw_min[ch] = raw_min;
        bsp_adc_data.raw_max[ch] = raw_max;

        /* 码值 → 毫伏 */
        voltage = (((BSP_ADC_VOLTAGE_T)bsp_adc_data.raw_avg[ch]
                    * (BSP_ADC_VOLTAGE_T)BSP_ADC_VREF_MV)
                   / (BSP_ADC_VOLTAGE_T)BSP_ADC_FULL_SCALE);
        bsp_adc_data.voltage_mv[ch] = (uint16_t)voltage;
    }

    bsp_adc_data.valid = 1U;
}

/**
 * @brief  读取指定通道的电压
 *
 * @param[in] channel  逻辑通道号
 * @return             电压，单位 mV
 */
uint16_t Bsp_Adc_Get_VoltageMv(Bsp_Adc_Channel_Enum channel)
{
    if ((uint32_t)channel >= (uint32_t)BSP_ADC_CH_SUM)
    {
        return 0U;
    }

    return bsp_adc_data.voltage_mv[channel];
}

/**
 * @brief  读取指定通道的平均码值
 *
 * @param[in] channel  逻辑通道号
 * @return             平均码值
 */
uint16_t Bsp_Adc_Get_RawAvg(Bsp_Adc_Channel_Enum channel)
{
    if ((uint32_t)channel >= (uint32_t)BSP_ADC_CH_SUM)
    {
        return 0U;
    }

    return bsp_adc_data.raw_avg[channel];
}

/**
 * @brief  读取 DMA 缓冲区首地址
 *
 * @param  无
 * @return 缓冲区地址
 */
uint32_t Bsp_Adc_Get_BufferAddress(void)
{
    return (uint32_t)(uintptr_t)&s_adc_dma_buf[0];
}
