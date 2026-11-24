/**
 * @file    ntc.h
 * @author  华畅
 * @brief   NTC 热敏电阻（多实例、纯算法）：引脚电压 → 阻值 → 温度
 * @version 6.0
 * @date    2026-11-05
 *
 * @copyright Copyright (c) 2026
 *
 * @note    本模块不依赖任何 BSP / HAL / 具体硬件，只做数学换算。
 *          电压从哪里来由调用方决定：
 *            - 本板用 user/modules/NTC/ntc_port.c 里的适配层
 *            - 其它工程直接把电压传给 Ntc_Process()
 *
 * 换算三步：
 *   V_node = V_pin / 8.5
 *   R_ntc  = V_node / 50uA - 510R
 *   T      = 1/(1/298.15 + ln(R/R25)/B) - 273.15
 */

#ifndef __NTC_H
#define __NTC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================================================================
 * 一、宏定义
 * ================================================================== */

/* ---------------- 1.1 电路参数 ---------------- */

#define NTC_AMP_GAIN            (8.5f)      /* 前端放大倍数 8.5 倍          */
#define NTC_EXCITE_MA           (0.05f)     /* 恒流 50µA，写成 mA（mV/mA=Ω）*/
#define NTC_R_FIX_OHM           (510.0f)    /* 回路串联电阻 R106，Ω         */
#define NTC_T25_K               (298.15f)   /* 25℃ 的开尔文温度             */
#define NTC_U_REF_MV            (3282U)     /* 参考电压实测值，mV           */
#define NTC_SAT_MV              (NTC_U_REF_MV - 30U)  /* 饱和阈值，mV    */
#define NTC_FILTER_LEN          (8U)        /* 滑动平均点数                 */
#define NTC_TEMP_MIN_X10        (-400)      /* 温度下限，0.1℃              */
#define NTC_TEMP_MAX_X10        (1250)      /* 温度上限，0.1℃              */

/* ---------------- 1.2 可测量的阻值上限说明 ----------------
 *
 * 本电路能准确测量的 NTC 阻值上限为 7208 Ω（约 7.2 kΩ），超过后节点电压
 * 被放大级削顶（运放到轨约 3.28 V），测出的阻值和温度都不准。
 *
 *   3.28 V ÷ 8.5          = 385.9 mV     （节点电压上限）
 *   385.9 mV ÷ 50 µA      = 7717.6 Ω     （回路总电阻）
 *   7717.6 Ω − 510 Ω      = 7207.6 Ω     （NTC 上限 ≈ 7208 Ω）
 *
 * 通用公式：R_ntc_max = (U_ADC_max / 增益 / 激励电流) − R106
 *
 * 对应最低可测温度：
 *   10 kΩ NTC（B=3950）   ：7208 Ω ≈ 32.6 ℃ → 只能测 32.6 ℃ 以上
 *   2.2 kΩ 探头（B=3884） ：7208 Ω ≈  0.2 ℃ → 能测 0.2 ℃ 以上
 *
 * 想扩大范围：减小 R76 降增益（8.5→7208Ω，4→15890Ω，2→32290Ω）、
 *             减小激励电流、或换阻值更小的 NTC。
 * ---------------------------------------------------------- */

/* ==================================================================
 * 二、枚举定义
 * ================================================================== */

/**
 * @brief  NTC 实例编号（用于数组索引和遍历）
 *
 * @note   要加一路 NTC：在 NTC_INST_SUM 前面加一个成员即可，
 *         实例数组会自动跟着变大。
 */
typedef enum __Ntc_Instance_Enum
{
    NTC_INST_TEMP = 0,          /* 温度探头 */
    NTC_INST_SUM                /* 实例总数（自动计算） */
} Ntc_Instance_Enum;

/**
 * @brief  故障码
 */
typedef enum __Ntc_Fault_Enum
{
    NTC_FAULT_NONE = 0,         /* 正常                        */
    NTC_FAULT_PARAM,            /* R25 或 B 值没设置            */
    NTC_FAULT_SATURATED,        /* 饱和：温度低于量程下限        */
    NTC_FAULT_NO_EXCITE,        /* 节点电压为 0：激励异常        */
    NTC_FAULT_RES_INVALID       /* 算出阻值 ≤ 0：接线异常        */
} Ntc_Fault_Enum;

/* ==================================================================
 * 三、结构体定义
 * ================================================================== */

/**
 * @brief  单路 NTC 的全部数据
 */
typedef struct Ntc_Instance_Struct_s
{
    /* ---------- 配置（Ntc_Init 时给出） ---------- */
    uint16_t       r25_ohm;          /* 25 ℃ 时的阻值，Ω            */
    float          beta_k;           /* B 常数                       */

    /* ---------- 计算结果 ---------- */
    uint16_t       pin_voltage_mv;   /* 输入电压（放大后），mV        */
    uint16_t       node_voltage_mv;  /* NTC 节点电压（放大前），mV    */
    uint32_t       res_ohm;          /* NTC 阻值，Ω                  */
    int16_t        temp_x10;         /* 温度，0.1℃（253=25.3℃）     */
    uint8_t        valid;            /* 1 = 结果有效                 */
    Ntc_Fault_Enum fault;            /* 故障码                       */

    /* ---------- 内部滤波状态（外部不用管） ---------- */
    int16_t        filt[NTC_FILTER_LEN];
    int32_t        filt_sum;
    uint8_t        filt_pos;
    uint8_t        filt_cnt;
} Ntc_Instance_t;

/* ==================================================================
 * 四、全局变量
 * ================================================================== */

/* 所有实例（下标即实例枚举值） */
extern Ntc_Instance_t ntc[NTC_INST_SUM];

/* ==================================================================
 * 五、函数接口
 * ================================================================== */

/**
 * @brief  初始化一个 NTC 实例（只记参数，不做任何硬件操作）
 *
 * @param[in] inst     实例号，见 Ntc_Instance_Enum
 * @param[in] r25_ohm  25 ℃ 时的阻值，Ω（10 kΩ 填 10000，2.2 kΩ 填 2209）
 * @param[in] beta_k   B 常数（常见 3950、3884、3435）
 * @return             0 = 成功，1 = 实例号或参数非法
 */
uint8_t Ntc_Init(Ntc_Instance_Enum inst, uint16_t r25_ohm, float beta_k);

/**
 * @brief  计算：给一个引脚电压，算出该实例的阻值和温度
 *
 * @param[in] inst            实例号
 * @param[in] pin_voltage_mv  ADC 引脚电压（放大后），mV
 */
void Ntc_Process(Ntc_Instance_Enum inst, uint16_t pin_voltage_mv);

/**
 * @brief  读取某实例的温度
 *
 * @param[in] inst  实例号
 * @return          温度，0.1 ℃；实例号非法返回 0
 */
int16_t Ntc_Get_Temp_X10(Ntc_Instance_Enum inst);

/**
 * @brief  B 值公式：阻值 → 温度（参数由调用方给出，可单独使用）
 *
 * @note   T = 1/(1/T25 + ln(R/R25)/B) − 273.15
 *
 * @param[in] r_ohm    阻值，Ω
 * @param[in] beta_k   B 常数
 * @param[in] r25_ohm  25 ℃ 时的阻值，Ω
 * @return             温度，0.1 ℃；参数非法返回 NTC_TEMP_MIN_X10
 */
int16_t Ntc_Ohm_To_Temp_X10(uint32_t r_ohm, float beta_k, uint16_t r25_ohm);

/* ==================================================================
 * 六、本板适配接口（依赖 bsp_adc，移植时可删）
 * ================================================================== */

/**
 * @brief  本板初始化：启动 BSP 的 ADC，并按接线表初始化全部 NTC 实例
 *
 * @note   探头型号、R25、B 值、接哪个 ADC 通道，都在 ntc.c 末尾的接线表里。
 *
 * @return 0 = 成功，1 = BSP 初始化失败，2 = 某个实例参数非法
 */
uint8_t Ntc_Board_Init(void);

/**
 * @brief  本板周期更新：刷新 BSP 数据，再更新全部 NTC 实例
 */
void Ntc_Board_Update(void);

#ifdef __cplusplus
}
#endif
#endif /* __NTC_H */
