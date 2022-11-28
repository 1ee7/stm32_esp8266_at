/**
 ******************************************************************************
 * @file    timer.c
 * @author  const-zpc
 * @date    2020-12-1
 * @brief   该文件提供功能模块层管理功能，以管理 TIMER 驱动的以下功能：
 *           + timer底层驱动，引脚配置
 *           + timer的中断服务函数
 *
 *
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "timer.h"
#include "sys.h"
#include <string.h>

/**
 * @brief 可注册的定时回调函数最大数目
 */
#define SUPPORT_FUN_MAX_NUM 10

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief TIME 句柄信息结构体定义
 */
typedef struct
{
  TimeFun pTimeFun; /*!< 回调函数 */

  uint32_t ui1msTic; /*!< 1ms定时计时 */

  uint32_t ui1msMaxTime; /*!< 回调周期时间 */
} TIME_FunType;

/**
 * @brief TIME 句柄信息结构体定义
 */
typedef struct
{
  uint16_t ucSupportCnt;                      /*!< 注册的定时回调函数的数目 */
  TIME_FunType tCallFun[SUPPORT_FUN_MAX_NUM]; /*!< 注册的定时回调函数 */
  uint32_t ui1msTic;                          /*!< 1ms定时 */
  uint32_t uimsDelayTic;                      /*!< 1ms定时 */
} TIME_HandleType;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/** 定时标志 */
Tim_TimeSignType g_tTimeSign = {0};

static TIME_HandleType sg_tTIME_Handle = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private function ----------------------------------------------------------*/

/**
 * @brief      PIT CH1 的初始化启动
 * @note       这里时钟选择为APB1的2倍，而APB1为36M
 * @param[in]  arr 自动重装值
 * @param[in]  psc 时钟预分频数
 * @retval     None
 */
void TIME_Init(uint16_t arr, uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  //定时器TIM3初始化
  TIM_TimeBaseStructure.TIM_Period = arr;
  TIM_TimeBaseStructure.TIM_Prescaler = psc;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  //中断优先级NVIC设置
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief  PIT CH1 的初始化启动.
 * @retval None
 */
void FML_TIME_Init(void)
{
  // 100Khz的计数频率，计数到100为1ms
  TIME_Init(99, 719);

  memset(&sg_tTIME_Handle, 0, sizeof(sg_tTIME_Handle));
}

/**
 * @brief  定时器3中断服务程序
 * @retval None
 */
void TIM3_IRQHandler(void)
{
  uint8_t i;

  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    for (i = 0; i < sg_tTIME_Handle.ucSupportCnt && i < SUPPORT_FUN_MAX_NUM; i++)
    {
      sg_tTIME_Handle.tCallFun[i].ui1msTic++;
      if (sg_tTIME_Handle.tCallFun[i].ui1msTic >= sg_tTIME_Handle.tCallFun[i].ui1msMaxTime)
      {
        sg_tTIME_Handle.tCallFun[i].ui1msTic = 0;
        if (sg_tTIME_Handle.tCallFun[i].pTimeFun != NULL)
        {
          sg_tTIME_Handle.tCallFun[i].pTimeFun();
        }
      }
    }

    sg_tTIME_Handle.uimsDelayTic++;
    sg_tTIME_Handle.ui1msTic++;

    sg_tTIME_Handle.ui1msTic % 1 == 0 ? g_tTimeSign.bTic1msSign = TRUE : 0;

    sg_tTIME_Handle.ui1msTic % 10 == 0 ? g_tTimeSign.bTic10msSign = TRUE : 0;

    sg_tTIME_Handle.ui1msTic % 20 == 0 ? g_tTimeSign.bTic20msSign = TRUE : 0;

    sg_tTIME_Handle.ui1msTic % 100 == 0 ? g_tTimeSign.bTic100msSign = TRUE : 0;

    sg_tTIME_Handle.ui1msTic % 500 == 0 ? g_tTimeSign.bTic500msSign = TRUE : 0;

    sg_tTIME_Handle.ui1msTic % 1000 == 0 ? (g_tTimeSign.bTic1secSign = TRUE, sg_tTIME_Handle.ui1msTic = 0) : 0;
  }
}

/**
 * @brief      注册定时回调函数.
 * @note       注册的定时函数执行时间必须很短
 * @param[in]  pTimeFun 回调函数
 * @param[in]  time     回调周期时间, 单位毫秒
 * @retval     0,成功; -1,失败
 */
int FML_TIME_Register(TimeFun pTimeFun, uint32_t time)
{
  if (sg_tTIME_Handle.ucSupportCnt < SUPPORT_FUN_MAX_NUM)
  {
    sg_tTIME_Handle.tCallFun[sg_tTIME_Handle.ucSupportCnt].ui1msMaxTime = time;
    sg_tTIME_Handle.tCallFun[sg_tTIME_Handle.ucSupportCnt].pTimeFun = pTimeFun;
    sg_tTIME_Handle.ucSupportCnt++;
    return 0;
  }

  return -1;
}

/**
 * @brief      注销定时回调函数.
 * @param[in]  pTimeFun 回调函数
 * @retval     None
 */
void FML_TIME_UnRegister(TimeFun pTimeFun)
{
  uint16_t i;
  uint8_t ucSupportCnt = 0;
  TIME_FunType tCallFun[SUPPORT_FUN_MAX_NUM] = {0};

  for (i = 0; i < sg_tTIME_Handle.ucSupportCnt && i < SUPPORT_FUN_MAX_NUM; i++)
  {
    if (sg_tTIME_Handle.tCallFun[i].pTimeFun != pTimeFun)
    {
      tCallFun[ucSupportCnt] = sg_tTIME_Handle.tCallFun[i];
      ucSupportCnt++;
    }
  }

  for (i = 0; i < SUPPORT_FUN_MAX_NUM; i++)
  {
    sg_tTIME_Handle.tCallFun[i] = tCallFun[i];
  }

  sg_tTIME_Handle.ucSupportCnt = ucSupportCnt;
}
