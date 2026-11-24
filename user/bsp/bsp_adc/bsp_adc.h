/**
 * @file    bsp_adc.h
 * @author  华畅
 * @brief   ADC 板级支持包（可移植）：单 ADC 多通道扫描 + DMA 循环 + 缓存维护
 * @version 0.5
 * @date    2026-03-30
 *
 * @copyright Copyright (c) 2026
 *
 * @par 分层定位
 *      本层只负责「让这块板子的 ADC 出数」：
 *      通道扫描配置、校准、启动 DMA、缓存失效、逐通道求平均、码值→毫伏。
 *      不含任何传感器知识（NTC / 压力 / 血氧一律在模块层）。
 *
 * @par 可移植性
 *      本文件顶部有「板级配置区」，移植到其它板子只需要改那一节，
 *      以及 bsp_adc.c 里的 s_ch_cfg[] 通道表；下面的逻辑代码完全不用动。
 *
 * @par 移植步骤
 *      1. 拷 bsp_adc.c / bsp_adc.h 到新工程，加入编译；
 *      2. 改「板级配置区」：
 *           BSP_ADC_HANDLE       —— CubeMX 生成的 ADC 句柄名（hadc1/hadc2…）
 *           BSP_ADC_VREF_MV      —— 实测 VREF+，单位 mV
 *           BSP_ADC_FULL_SCALE   —— 按分辨率：12 位 4095，16 位 65535
 *           BSP_ADC_SAMPLE_LEN   —— 每通道采样点数
 *           BSP_ADC_NC_WINDOW_*  —— DMA 缓冲区所在的非缓存地址窗口
 *           BSP_ADC_CACHE_INVALIDATE —— 无 D-Cache 的 MCU 定义成空
 *      3. 改逻辑通道枚举（要采哪几路）；
 *      4. 改 bsp_adc.c 里的 s_ch_cfg[]（每路的物理通道、引脚、采样时间），
 *         顺序必须与枚举、与 CubeMX 里的 Rank 一致；
 *      5. CubeMX 里把 Number Of Conversion 设成通道数，DMA 设为 Circular。
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 一、板级配置区（移植只改这一节） ============================ */

/* ADC 句柄：CubeMX 生成的名字，如 hadc1 / hadc2 / hadc3 */
#define BSP_ADC_HANDLE                  hadc1

/* ADC 参考电压（VDDA / VREF+），单位 mV —— 用万用表实测后填写 */
#define BSP_ADC_VREF_MV                 (3282U)

/* 满量程码值：12 位 4095、14 位 16383、16 位 65535 */
#define BSP_ADC_FULL_SCALE              (65535U)

/* 每个通道的采样点数（一轮 DMA 里每通道多少个点） */
#define BSP_ADC_SAMPLE_LEN              (50U)

/* DMA 缓冲区必须落在 MPU 配置的非缓存窗口内（本板：AXI SRAM 前 16 KB） */
#define BSP_ADC_NC_WINDOW_BASE          (0x24000000UL)
#define BSP_ADC_NC_WINDOW_SIZE          (0x00004000UL)

/* 缓存失效动作
 *   Cortex-M7（本板，开 D-Cache）：调用 CMSIS 函数
 *   无 Cache 的 MCU（M0/M3/M4/F1/F4 等）：改成 ((void)0) 即可 */
#define BSP_ADC_CACHE_INVALIDATE(addr, size) \
    SCB_InvalidateDCache_by_Addr((uint32_t *)(addr), (int32_t)(size))

/* 校准模式
 *   STM32H7（本板）        ：ADC_CALIB_OFFSET_LINEARITY
 *   STM32F4 / F7 / L4 等   ：ADC_CALIB_OFFSET（没有线性度选项）*/
#define BSP_ADC_CALIB_MODE_CODE         ADC_CALIB_OFFSET_LINEARITY

/* ============================ 二、逻辑通道定义 ============================ */

/**
 * @brief  逻辑通道编号
 *
 * @note   顺序必须与 bsp_adc.c 里 s_ch_cfg[] 的顺序、
 *         以及 CubeMX 里各通道的 Rank 顺序（Rank1、Rank2…）完全一致。
 *         要加一路信号：枚举加一项 + 通道表加一行 + CubeMX 的
 *         Number Of Conversion 加 1（Rank 会自动重排）。
 */
typedef enum Bsp_Adc_Channel_Enum_e
{
    BSP_ADC_CH_NTC = 0,        /* 逻辑通道 0：NTC 温度探头（PC1 / ADC1_INP11） */
    /* 后续按需增加：
     * BSP_ADC_CH_PRESS,        压力
     * BSP_ADC_CH_VSAT,         血氧
     */
    BSP_ADC_CH_SUM             /* 通道总数哨兵，勿删 */
} Bsp_Adc_Channel_Enum;

/* ============================ 三、类型定义 ============================ */

/**
 * @brief  返回状态
 */
typedef enum Bsp_Adc_Status_Enum_e
{
    BSP_ADC_OK = 0,          /* 成功                                    */
    BSP_ADC_ERR_CALIB,       /* ADC 校准失败                             */
    BSP_ADC_ERR_START,       /* 启动 DMA 失败                            */
    BSP_ADC_ERR_CHANNEL,     /* 通道/工作模式配置失败                     */
    BSP_ADC_ERR_BUFFER       /* 缓冲区不在非缓存窗口（DMA 写入不可见）     */
} Bsp_Adc_Status_Enum;

/**
 * @brief  各通道换算结果
 *
 * @note   数组下标即逻辑通道号。
 */
typedef struct Bsp_Adc_Data_Struct_s
{
    uint16_t raw_avg[BSP_ADC_CH_SUM];    /* 该通道平均码值 */
    uint16_t raw_min[BSP_ADC_CH_SUM];    /* 该通道最小码值 */
    uint16_t raw_max[BSP_ADC_CH_SUM];    /* 该通道最大码值 */
    uint16_t voltage_mv[BSP_ADC_CH_SUM]; /* 该通道电压，单位 mV */
    uint8_t  valid;                      /* 1 = 本轮结果有效 */
} Bsp_Adc_Data_Struct;

/* ============================ 四、对外变量 ============================ */

extern Bsp_Adc_Data_Struct bsp_adc_data;

/* ============================ 五、对外接口 ============================ */

/**
 * @brief  初始化并启动 ADC（多通道扫描 + 连续转换 + DMA 循环）
 *
 * @note   内部依次完成：缓冲区自检 → 工作模式校正 → 逐通道配置 →
 *         校准 → 启动 DMA。系统启动时调用一次即可。
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
Bsp_Adc_Status_Enum Bsp_Adc_Init(void);

/**
 * @brief  重新启动 DMA 搬运（Stop 之后可用）
 *
 * @param  无
 * @return 见 Bsp_Adc_Status_Enum
 */
Bsp_Adc_Status_Enum Bsp_Adc_Start(void);

/**
 * @brief  停止 ADC 转换与 DMA 搬运
 *
 * @param  无
 * @return 无
 */
void Bsp_Adc_Stop(void);

/**
 * @brief  更新一轮结果（取缓冲区 → 逐通道平均/极值 → 电压）
 *
 * @note   内部负责缓存失效。上层周期性调用后读 bsp_adc_data 或各 Get 接口。
 *
 * @param  无
 * @return 无
 */
void Bsp_Adc_Update(void);

/**
 * @brief  读取指定通道的电压
 *
 * @param[in] channel  逻辑通道号
 * @return             电压，单位 mV；通道号非法返回 0
 */
uint16_t Bsp_Adc_Get_VoltageMv(Bsp_Adc_Channel_Enum channel);

/**
 * @brief  读取指定通道的平均码值
 *
 * @param[in] channel  逻辑通道号
 * @return             平均码值；通道号非法返回 0
 */
uint16_t Bsp_Adc_Get_RawAvg(Bsp_Adc_Channel_Enum channel);

/**
 * @brief  读取 DMA 缓冲区首地址（用于上电自检日志）
 *
 * @param  无
 * @return 缓冲区地址
 */
uint32_t Bsp_Adc_Get_BufferAddress(void);

#ifdef __cplusplus
}
#endif
#endif /* __BSP_ADC_H */
