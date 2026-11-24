/**
 * @file    ntc.c
 * @author  华畅
 * @brief   NTC 热敏电阻模块实现（纯算法，不依赖 BSP/HAL）
 * @version 6.0
 * @date    2026-11-05
 *
 * @copyright Copyright (c) 2026
 *
 * 换算过程：
 *   V_node = V_pin / 8.5
 *   R_ntc  = V_node / 50uA - 510R
 *   T      = 1/(1/T25 + ln(R/R25)/B) - 273.15
 */

/* Includes ------------------------------------------------------------------*/
#include "ntc.h"

#include <math.h>

/* 本板适配段（文件末尾）需要从 BSP 取电压。
 * 移植到别的工程时：删掉这一行 + 文件末尾那个「本板适配段」整块即可。 */
#include "bsp_adc.h"

/* Private variables ---------------------------------------------------------*/

/* 所有实例，下标就是实例号 */
Ntc_Instance_t ntc[NTC_INST_SUM];

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  清空某实例的滤波窗口，并把结果置为无效
 *
 * @param[in] p      实例指针
 * @param[in] fault  故障码
 */
static void ntc_reset(Ntc_Instance_t *p, Ntc_Fault_Enum fault)
{
    uint8_t i;

    for (i = 0U; i < NTC_FILTER_LEN; i++)
    {
        p->filt[i] = 0;
    }

    p->filt_sum = 0;
    p->filt_pos = 0U;
    p->filt_cnt = 0U;

    p->node_voltage_mv = 0U;
    p->res_ohm = 0U;
    p->temp_x10 = 0;
    p->valid = 0U;
    p->fault = fault;
}

/**
 * @brief  压入一个温度样本，返回该实例的窗口平均值
 *
 * @param[in] p      实例指针
 * @param[in] value  温度，0.1 ℃
 * @return           平均值，0.1 ℃
 */
static int16_t ntc_filter_push(Ntc_Instance_t *p, int16_t value)
{
    if (p->filt_cnt < NTC_FILTER_LEN)
    {
        p->filt_cnt++;
    }
    else
    {
        p->filt_sum -= p->filt[p->filt_pos];
    }

    p->filt[p->filt_pos] = value;
    p->filt_sum += value;

    p->filt_pos++;
    if (p->filt_pos >= NTC_FILTER_LEN)
    {
        p->filt_pos = 0U;
    }

    return (int16_t)(p->filt_sum / (int32_t)p->filt_cnt);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  B 值公式：阻值 → 温度
 *
 * @note   T = 1/(1/T25 + ln(R/R25)/B) − 273.15
 *
 * @param[in] r_ohm    阻值，单位 Ω
 * @param[in] beta_k   B 常数
 * @param[in] r25_ohm  25 ℃ 时的阻值，单位 Ω
 * @return             温度，单位 0.1 ℃（超量程时被限幅）
 */
int16_t Ntc_Ohm_To_Temp_X10(uint32_t r_ohm, float beta_k, uint16_t r25_ohm)
{
    float   inv_t;
    float   temp_c;
    int16_t temp_x10;

    if ((r_ohm == 0U) || (r25_ohm == 0U) || (beta_k == 0.0f))
    {
        return NTC_TEMP_MIN_X10;
    }

    inv_t = (1.0f / NTC_T25_K) + (logf((float)r_ohm / (float)r25_ohm) / beta_k);

    if (inv_t <= 0.0f)
    {
        return NTC_TEMP_MIN_X10;
    }

    temp_c = (1.0f / inv_t) - 273.15f;

    /* 四舍五入到 0.1 ℃ */
    if (temp_c >= 0.0f)
    {
        temp_x10 = (int16_t)((temp_c * 10.0f) + 0.5f);
    }
    else
    {
        temp_x10 = (int16_t)((temp_c * 10.0f) - 0.5f);
    }

    if (temp_x10 < NTC_TEMP_MIN_X10)
    {
        temp_x10 = NTC_TEMP_MIN_X10;
    }
    else if (temp_x10 > NTC_TEMP_MAX_X10)
    {
        temp_x10 = NTC_TEMP_MAX_X10;
    }

    return temp_x10;
}

/**
 * @brief  初始化一个 NTC 实例（只记参数，不做硬件操作）
 *
 * @param[in] inst     实例号
 * @param[in] r25_ohm  25 ℃ 时的阻值，单位 Ω
 * @param[in] beta_k   B 常数
 * @return             0 = 成功，1 = 实例号或参数非法
 */
uint8_t Ntc_Init(Ntc_Instance_Enum inst, uint16_t r25_ohm, float beta_k)
{
    Ntc_Instance_t *p;

    if (inst >= NTC_INST_SUM)
    {
        return 1U;
    }

    p = &ntc[inst];

    if ((r25_ohm == 0U) || (beta_k == 0.0f))
    {
        ntc_reset(p, NTC_FAULT_PARAM);
        return 1U;
    }

    p->r25_ohm = r25_ohm;
    p->beta_k = beta_k;
    p->pin_voltage_mv = 0U;

    ntc_reset(p, NTC_FAULT_NONE);

    return 0U;
}

/**
 * @brief  计算：引脚电压 → 阻值 → 温度
 *
 * @param[in] inst            实例号
 * @param[in] pin_voltage_mv  ADC 引脚电压（放大后），单位 mV
 */
void Ntc_Process(Ntc_Instance_Enum inst, uint16_t pin_voltage_mv)
{
    Ntc_Instance_t *p;
    float    v_node_mv;
    float    r_total_ohm;
    uint32_t r_ntc_ohm;

    if (inst >= NTC_INST_SUM)
    {
        return;
    }

    p = &ntc[inst];
    p->pin_voltage_mv = pin_voltage_mv;

    /* 参数没设置就没法算 */
    if ((p->r25_ohm == 0U) || (p->beta_k == 0.0f))
    {
        ntc_reset(p, NTC_FAULT_PARAM);
        return;
    }

    /* 饱和：放大级已经削顶，反推出来的值不可信（阻值上限见 ntc.h） */
    if (pin_voltage_mv >= NTC_SAT_MV)
    {
        ntc_reset(p, NTC_FAULT_SATURATED);
        return;
    }

    /* ① 去掉前端放大：V_node = V_pin / 8.5 */
    v_node_mv = (float)pin_voltage_mv / NTC_AMP_GAIN;
    p->node_voltage_mv = (uint16_t)(v_node_mv + 0.5f);

    if (p->node_voltage_mv == 0U)
    {
        ntc_reset(p, NTC_FAULT_NO_EXCITE);
        return;
    }

    /* ② 去掉恒流：R_total = V_node / I = (mV) / (mA) = Ω */
    r_total_ohm = v_node_mv / NTC_EXCITE_MA;

    /* ③ 去掉串联的 510 Ω：R_ntc = R_total − R106 */
    if (r_total_ohm <= NTC_R_FIX_OHM)
    {
        ntc_reset(p, NTC_FAULT_RES_INVALID);
        return;
    }

    r_ntc_ohm = (uint32_t)(r_total_ohm - NTC_R_FIX_OHM + 0.5f);
    p->res_ohm = r_ntc_ohm;

    /* ④ B 值公式算温度（把 R25 与 B 值传进去），再做滑动平均 */
    p->temp_x10 = ntc_filter_push(p, Ntc_Ohm_To_Temp_X10(r_ntc_ohm,
                                                          p->beta_k,
                                                          p->r25_ohm));
    p->valid = 1U;
    p->fault = NTC_FAULT_NONE;
}

/**
 * @brief  读取某实例的温度
 *
 * @param[in] inst  实例号
 * @return          温度，单位 0.1 ℃
 */
int16_t Ntc_Get_Temp_X10(Ntc_Instance_Enum inst)
{
    if (inst >= NTC_INST_SUM)
    {
        return 0;
    }

    return ntc[inst].temp_x10;
}

/* ==================================================================
 * 本板适配段（依赖 bsp_adc）
 * ------------------------------------------------------------------
 * 这一段的唯一职责：把 NTC 实例和 BSP 的通道对应起来。
 * 上面的算法部分与本段无关；移植到别的工程时，整段删掉、自己调
 * Ntc_Process(inst, mv) 即可。
 * ================================================================== */

/**
 * @brief  本板初始化：启动 BSP 的 ADC，并初始化 NTC 实例
 *
 * @return 0 = 成功，1 = BSP 初始化失败，2 = NTC 参数非法
 */
uint8_t Ntc_Board_Init(void)
{
    /* ① 把 BSP 的 ADC 拉起来：选通道 → 校准 → 启动 DMA */
    if (Bsp_Adc_Init() != BSP_ADC_OK)
    {
        return 1U;
    }

    /* ② 初始化每一路探头
     *    10 kΩ 通用 NTC  ：10000U, 3950.0f
     *    2.2 kΩ 体温探头 ： 2209U, 3884.0f */
    if (Ntc_Init(NTC_INST_TEMP, 10000U, 3950.0f) != 0U)
    {
        return 2U;
    }

    /* 以后多一路就再加两行：
     *   (void)Ntc_Init(NTC_INST_PRESS, 2209U, 3884.0f);
     */

    return 0U;
}

/**
 * @brief  本板周期更新：刷新 BSP 数据，再更新 NTC 实例
 */
void Ntc_Board_Update(void)
{
    /* BSP 只刷新一次：读 DMA 缓冲 → 求平均 → 算出各通道电压 */
    Bsp_Adc_Update();

    /* 每一路探头：实例号 ↔ BSP 通道，写在一行里，一眼能核对 */
    Ntc_Process(NTC_INST_TEMP, Bsp_Adc_Get_VoltageMv(BSP_ADC_CH_NTC));

    /* 以后多一路就再加一行：
     *   Ntc_Process(NTC_INST_PRESS, Bsp_Adc_Get_VoltageMv(BSP_ADC_CH_PRESS));
     */
}
