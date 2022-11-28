/**
  **********************************************************************************************************************
  * @file    esp8266.c
  * @brief   该文件提供 ESP8266 驱动相关功能
  * @author  周鹏程    any question please send mail to const_zpc@163.com
  * @version V1.0.0
  * @date    2021-3-13
  *
  * @details  功能详细说明：
  *           + 
  *
  **********************************************************************************************************************
  *
  * 使用方式:
  *    1、使用前初始化函数 ESP_Init, 之后使用 ESP_SetCWMode 设置工作模式
  *    2、周期调用函数 ESP_RunTask, 用来和 ESP8266 模块通信
  *
  **********************************************************************************************************************
  */

/* Includes ----------------------------------------------------------------------------------------------------------*/
#include "esp8266.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sys.h"

/* AT 任务列表支持数目 */
#define ESP_CMD_TASK_LIST_NUM   30

/* Private typedef ---------------------------------------------------------------------------------------------------*/
 
/**
  * @brief AT 指令枚举
  * @note  有效指令枚举和 AT 指令表 sg_ktCmdTable 索引对应
  */
typedef enum
{
    ESP_AT_CMD_INVALID = -1,            /*!< 无效指令 */
    
    /* 常用指令集：基础指令 */
    ESP_AT_CMD_AT_TEST = 0,             /*!< 测试 AT 启动 */
    ESP_AT_CMD_RST,                     /*!< 重启模块 */
    ESP_AT_CMD_SEE_VERSION,             /*!< 查看版本信息 */
    ESP_AT_CMD_ATE,                     /*!< 回显开启/关闭 */
    /* 常用指令集：wifi 功能指令 */
    ESP_AT_CMD_SET_MODE,                /*!< 选择 WIFI 应用模式 */
    ESP_AT_CMD_JOIN_AP,                 /*!< 加入 AP */
    ESP_AT_CMD_LISTS_AP,                /*!< 列出当前可用 AP */
    ESP_AT_CMD_QUIT_AP,                 /*!< 退出与 AP 的连接 */
    ESP_AT_CMD_SET_AP_PARAM,            /*!< 设置 AP 模式下的参数 */
    ESP_AT_CMD_LISTS_JOINED_IP,         /*!< 查看已接入设备的 IP */
    /* 常用指令集：TCP/IP 指令 */
    ESP_AT_CMD_GET_CONNECT_STATE,       /*!< 获得连接状态 */
    ESP_AT_CMD_BUILD_TCP_OR_UDP,        /*!< 建立 TCP 连接或注册 UDP 端口号 */
    ESP_AT_CMD_SEND_DATA,               /*!< 发送数据指令 */
    ESP_AT_TRANSFER_DATA,               /*!< 透传数据 */
    ESP_AT_CMD_CLOSE_TCP_OR_UDP,        /*!< 关闭 TCP 或 UDP */
    ESP_AT_CMD_GET_LOCAL_IP,            /*!< 获取本地 IP 地址 */
    ESP_AT_CMD_START_MULTI_CONNECT,     /*!< 启动多连接 */
    ESP_AT_CMD_CONFIG_SERVICE,          /*!< 配置为服务器 */
    ESP_AT_CMD_SET_TRANSFER_MODE,       /*!< 设置模块传输模式 */
    ESP_AT_CMD_SET_SERVICE_OVERTIME,    /*!< 设置服务器超时时间 */
    /* 安信可指令集：配置指令 */
    ESP_AT_CMD_ENABLE_AUTO_LINK_WIFI,   /*!< 使能上电自动连接AP */
    ESP_AT_CMD_ENTER_SMART_CONFIG,      /*!< 进入 ESP-Touch 或 Airkiss 智能配网 */
    ESP_AT_CMD_EXIT_SMART_CONFIG,       /*!< 退出 ESP-Touch 或 Airkiss 智能配网 */
} eATCmd;

/**
  * @breif AT指令发送响应信息结构体定义
  */
typedef struct
{
    char*    pszATCmd;                              /*!< AT 指令指针 */
    
    char*    pszATAck;                              /*!< AT 指令返回预期结果 */
    
    uint8_t resendMaxCnt;                           /*!< 重发最大次数, 为 0 表示不需要重发 */
    
    uint8_t sendCnt;                                /*!< 重发次数, 初始为 0 */
    
    uint8_t isQuery;                                /*!< 是否是查询 */
    
    uint32_t maxWaitTime;                           /*!< 响应等待时间 */
    
    void (*pfnPackCmd)(void *);                     /*!< 重新打包 AT 指令函数, 为 NULL 则按照默认方式处理 */
    
    void (*pfnUnpackAck)(const char *, uint16_t);   /*!< 重新解析 AT 指令响应处理函数, 为 NULL 则按照默认方式处理 */
    
    void (*pfnOverTimeHandle)(void);                /*!< AT 指令响应超时处理函数, 为 NULL 则不处理 */
}ATCmdInfo_t;

/**
  * @breif AT指令任务列表结构体定义
  */
typedef struct
{
    uint8_t  state;                                 /*!< 控制状态 */
    
    uint8_t  end;                                   /*!< 循环队列尾哨兵 */
    
    uint8_t  head;                                  /*!< 循环队列首哨兵 */
    
    uint8_t  num;                                   /*!< 循环队列中能存储的最多组数 */
    
    uint8_t  isFirst;                               /*!< 循环队列首哨兵首次读取 */
    
    eATCmd  aCmdList[ESP_CMD_TASK_LIST_NUM];        /*!< 任务列表 */
    
    EspResCallFun pfnResCallFun[ESP_CMD_TASK_LIST_NUM];     /*!< 任务列表结果回调函数 */
}ATCmdTask_t;

/**
  * @breif TCP 状态信息结构体定义
  */
typedef struct
{
    uint8_t isConnect;                              /*!< 连接状态 */
    
    eEspIpType eIPConnectType;                      /*!< IP 连接类型 */
    
    char szNetAddr[64];                             /*!< TCP/UDP 网络地址/域名 */
    
    uint32_t netPort;                               /*!< TCP/UDP 网络通信端口号 */

    EspRecvCallFun pfnRecvCallFun;                  /*!< 数据接收回调处理函数 */
}NetManager_t;

/**
  * @breif 服务器信息结构体定义
  */
typedef struct
{
    uint8_t isEnable;                         /*!< 是否使能 server 模式 */
    
    uint16_t overtime;                        /*!< 服务器超时时间, 范围为0~28800, 单位为 s */
    
    uint32_t port;                            /*!< 监听的端口号 */
}ServerInfo_t;

/**
  * @brief  ESP8266 模块AT指令管理结构体信息定义.
  */
typedef struct tag_EspHandle
{
    ATCmdInfo_t  tATCmdInfo;                        /*!< 当前使用中的 AT 指令信息 */
    
    char szAtCmdBuf[64];                            /*!< AT 指令数据缓冲 */

    char sendBufData[200];                         /*!< 发送TCP数据的缓冲 */
    
    uint16_t sendBufDataLenth;                      /*!< 发送TCP数据的长度 */
    
    eEspCWMode workMode;                            /*!< 工作应用模式 */
    
    uint8_t currNetTcpId;                           /*!< 当前使用中的 id */

    uint8_t isWifiConnected;                        /*!< WIFI 是否已连接 */
    
    uint8_t isATAckWait;                            /*!< AT 指令是否正在等待响应 */
    
    uint8_t isSeriaNet;                             /*!< 是否是透传模式 */
    
    uint32_t ATAckWaitTic;                          /*!< AT 指令等待计时 */
    
    uint8_t isMultiConnect;                         /*!< 是否是单路连接模式 */
    
    uint8_t isConfigSmartState;                     /*!< 是否处于 SMART CONFIG 模式 */
    
    uint8_t isMultiConnectBak;                      /*!< 是否是单路连接模式备份 */
    
    ATCmdTask_t tATCmdTask;                         /*!< AT指令任务列表 */

    ESP_WifiInfo_t tWifiInfo;                       /*!< 需要连接的 WIFI 信息 */
    
    ESP_WifiInfo_t tWifiHostInfo;                   /*!< AP 模式下(热点)的 WIFI 信息 */
    
    ServerInfo_t tServerInfo;                       /*!< 服务器信息 */
    
    NetManager_t  arrtNetManager[5];                /*!< 网络通信的连接状态和数据, 单连接模式下使用0 */
}EspHandle_t;

/* Private define ----------------------------------------------------------------------------------------------------*/
/* Private macro -----------------------------------------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------------------------------------*/
static EspHandle_t sg_tEspHandle;

/* Private function prototypes ---------------------------------------------------------------------------------------*/

static int InitCmdTaskToList(ATCmdTask_t* info, uint8_t num);
static void OnPackCWModeCmd(void *);
static void OnPackJoinAPCmd(void *);
static void OnPackSetAPParamCmd(void *);
static void OnPackBuildTCPOrUDPCmd(void *);
static void OnPackTCPOrUDPDataCmd(void *);
static void OnPackTCPOrUDPData(void *);
static void OnPackCloseTCPOrUDPCmd(void *);
static void OnPackStartMultiConnectCmd(void *);
static void OnPackConfigServiceCmd(void *);
static void OnPackSetTransferCmd(void *);
static void OnPackSetServiceOvertimeCmd(void *);

static void OnUnpackGeneralAck(const char *kBufData, uint16_t len);
static void OnUnpackATEAck(const char *kBufData, uint16_t len);
static void OnUnpackBuildTCPOrUDPAck(const char *kBufData, uint16_t len);
static void OnUnpackGetConnectStatusAck(const char *kBufData, uint16_t len);
static void OnUnpackTCPOrUDPDataCmdAck(const char *kBufData, uint16_t len);
static void OnUnpackTCPOrUDPDataSendAck(const char *kBufData, uint16_t len);
static void OnUnpackStartMultiConnectAck(const char *kBufData, uint16_t len);
static void OnUnpackEnterConfigSmartAck(const char *kBufData, uint16_t len);
static void OnUnpackExitConfigSmartAck(const char *kBufData, uint16_t len);
    
/* Private function --------------------------------------------------------------------------------------------------*/

/**
  * @brief   指令集内容
  */  
static ATCmdInfo_t sg_ktCmdTable[] = {
    {"AT",          "OK",               50, 0, 0, 100, NULL, NULL,  NULL},
    //{"AT+RST",      "jump to",          0, 0, 0, 2000,  NULL, NULL,  NULL},
    {"AT+RST",      "OK",          0, 0, 0, 2000,  NULL, NULL,  NULL},
    {"AT+GMR",      "OK",               0, 0, 0, 2000,  NULL, NULL,  NULL},
    {"ATE0",        "OK",               2, 0, 0, 500,  NULL, OnUnpackATEAck,  NULL},
    {"AT+CWMODE",   "OK",               1, 0, 0, 1000, OnPackCWModeCmd, NULL,  NULL},
    {"AT+CWJAP",    "OK",               3, 0, 0, 10000,  OnPackJoinAPCmd, NULL,  NULL},
    {"AT+CWLAP",    "OK",               0, 0, 0, 5000,  NULL, NULL,  NULL},
    {"AT+CWQAP",    "OK",               1, 0, 0, 500,  NULL, NULL, NULL},
    {"AT+CWSAP",    "OK",               1, 0, 0, 500,  OnPackSetAPParamCmd, NULL,  NULL},
    {"AT+CWLIF",    "OK",               1, 0, 0, 500,  NULL, NULL,  NULL},
    {"AT+CIPSTATUS","OK",               1, 0, 0, 500,  NULL, OnUnpackGetConnectStatusAck,  NULL},
    {"AT+CIPSTART", "OK",               1, 0, 0, 1500,  OnPackBuildTCPOrUDPCmd, OnUnpackBuildTCPOrUDPAck,  NULL},
    {"AT+CIPSEND",  ">",                0, 0, 0, 1000,  OnPackTCPOrUDPDataCmd, OnUnpackTCPOrUDPDataCmdAck,  NULL},
    {"",            "SEND OK",          0, 0, 0, 3000,  OnPackTCPOrUDPData, OnUnpackTCPOrUDPDataSendAck,  NULL},
    {"AT+CIPCLOSE", "OK",               1, 0, 0, 500,  OnPackCloseTCPOrUDPCmd, NULL,  NULL},
    {"AT+CIFSR",    "OK",               1, 0, 0, 500,  NULL, NULL,  NULL},
    {"AT+CIPMUX",    "OK",              1, 0, 0, 2000,  OnPackStartMultiConnectCmd, OnUnpackStartMultiConnectAck,  NULL},
    {"AT+CIPSERVER",    "OK",           1, 0, 0, 2000,  OnPackConfigServiceCmd, NULL,  NULL},
    {"AT+CIPMODE",  "OK",               1, 0, 0, 500,  OnPackSetTransferCmd, NULL,  NULL},
    {"AT+CIPSTO",   "OK",               1, 0, 0, 500,  OnPackSetServiceOvertimeCmd, NULL,  NULL},
    /* 安信可指令集：配置指令 */
    {"AT+CWAUTOCONN=1",   "OK",         0, 0, 0, 1000,  NULL, NULL,  NULL},
    {"AT+CWSTARTSMART=3", "OK",         0, 0, 0, 1000,  NULL, OnUnpackEnterConfigSmartAck,  NULL},
    {"AT+CWSTOPSMART",   "OK",          0, 0, 0, 1000,  NULL, OnUnpackExitConfigSmartAck,  NULL},
};

/**
  * @brief      初始化AT指令任务
  * @param[in]  cmd - AT 指令任务
  * @param[in][out] info - ATCmdTask_t 任务控制列表
  * @retval     返回的值含义如下
  *             @arg -1: 写入失败
  *             @arg 0: 写入成功
  */
static int InitCmdTaskToList(ATCmdTask_t* info, uint8_t num)
{
    info->end = 0;
    info->head = 0;
    info->isFirst = 0;
    info->num = num;
    info->state = 0x01;
    
    memset(info->aCmdList, 0, sizeof(info->aCmdList));
    
    return 0;
}

/**
  * @brief      添加一个AT指令任务
  * @param[in]  cmd - AT 指令任务
  * @param[in][out] info - ATCmdTask_t 任务控制列表
  * @param[in]  pfnCallFun - 指令任务结果回调函数
  * @retval     返回的值含义如下
  *             @arg -1: 写入失败
  *             @arg 0: 写入成功
  */
static int AddCmdTaskToList(ATCmdTask_t* info, eATCmd cmd, EspResCallFun pfnCallFun)
{
    if (0x00 == ((info->state) & 0x02))                      /* 允许向缓冲区写入数据 */
    {
        info->aCmdList[info->end] = cmd;
        info->pfnResCallFun[info->end] = pfnCallFun;
        (info->end)++;

        if ((info->end) >= (info->num))
        {
            (info->end) = 0;
        }

        if (((info->state) & 0x01) != 0x00)                 /* 缓冲区中无数据待发送或处理 */
        {
            (info->state) &= ~0x01;                         /* 缓冲区中有数据待发送或处理 */
        }
        else if (((info->state) & 0x04) != 0x00)            /* 缓存区已满 */
        {
            (info->head) = (info->end);                     /* 随尾哨兵移动首哨兵 */
        }
        else if ((info->end) == (info->head))
        {
            if (info->state & 0x80)                         /* 是否锁定旧数据 */
            {
                (info->state) |= 0x02;                      /* 禁止向缓冲区写入数据 */
            }
            else
            {
                (info->state) |= 0x04;                      /* 缓冲区已满 */
            }
        }

        return 0;
    }

    return -1;
}

///**
//  * @brief      读取当前执行AT指令任务
//  * @param[in][out] info - ATCmdTask_t 任务控制列表
//  * @retval     -1: 没有AT指令任务; 任务 ID
//  */
//static eATCmd ReadCmdTaskFromList(ATCmdTask_t* info)
//{
//    if (((info->state) & 0x01) == 0x00)
//    {
//        return info->aCmdList[info->head];
//    }

//    return ESP_AT_CMD_INVALID;
//}

/**
  * @brief      读取当前执行AT指令结果的回调函数任务
  * @param[in][out] info - ATCmdTask_t 任务控制列表
  * @retval     -1: 没有AT指令任务; 任务 ID
  */
static EspResCallFun ReadCmdTaskCallFunFromList(ATCmdTask_t* info)
{
    if (((info->state) & 0x01) == 0x00)
    {
        return info->pfnResCallFun[info->head];
    }

    return NULL;
}

/**
  * @brief      读取当前执行AT指令信息
  * @param[out] peCmd - AT指令
  * @param[out] info - ATCmdTask_t 任务控制列表
  * @retval     -1: 没有AT指令任务;
  *              0: 无新的指令
  *              1: 新的指令
  */
static int ReadCmdInfoFromList(ATCmdTask_t* info, eATCmd *peCmd)
{
    *peCmd = ESP_AT_CMD_INVALID;
    
    if (((info->state) & 0x01) == 0x00)
    {
        if (info->isFirst == 0)
        {
            info->isFirst = 1;
            *peCmd = info->aCmdList[info->head];
            return 1;
        }
        else
        {
            *peCmd = info->aCmdList[info->head];
            return 0;
        }
    }

    return -1;
}

/**
  * @brief      删除当前的AT指令任务
  * @param[in][out] info - ATCmdTask_t 任务控制列表
  * @retval     0
  */
static int DeletCurrCmdTaskFromList(ATCmdTask_t* info)
{
    if (((info->state) & 0x01) == 0x00)
    {
        (info->head) ++;                            /* 头哨兵 */
        info->isFirst = 0;

        if ((info->head) >= (info->num))
        {
            info->head = 0;
        }

        if ((info->head) == (info->end))            /* 头尾哨兵重合 */
        {
            if (((info->state) & 0x02) == 0x00)     /* 允许向缓冲区中存入数据 */
            {
                (info->state) |= 0x01;              /* 缓冲区内无数据等待处理或发送 */
            }
        }
    }
    
    (info->state) &= ~0x02;                         /* 允许向缓冲区中存入数据 */
    (info->state) &= ~0x04;                         /* 缓冲区未满 */
    
    return 0;
}

/**
  * @brief      根据任务列表发送指令.
  * @param[in]  pHandle 句柄
  * @retval     None
  */
static void SendCmdByTaskList(EspHandle_t *pHandle)
{
    EspResCallFun pfnCallFun;
    eATCmd currATCmd;// = ReadCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    static eATCmd s_lastATCmd = ESP_AT_CMD_INVALID;
    
    int ret = ReadCmdInfoFromList(&sg_tEspHandle.tATCmdTask, &currATCmd);

    if (ret >= 0 && currATCmd != ESP_AT_CMD_INVALID)
    {
        if (ret > 0)
        {
            pHandle->tATCmdInfo = sg_ktCmdTable[currATCmd];
        }
        
        if (pHandle->tATCmdInfo.sendCnt > pHandle->tATCmdInfo.resendMaxCnt)
        {
            if ((pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask)) != NULL)
            {
                pfnCallFun(ESP_ACK_FAIL, NULL, 0);
            }
            
            DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        }

        if (pHandle->tATCmdInfo.sendCnt == 0 && pHandle->tATCmdInfo.pfnPackCmd != NULL)
        {
            pHandle->tATCmdInfo.pfnPackCmd(pHandle);
        }
    
        if (strlen(pHandle->tATCmdInfo.pszATCmd))
        {
            ESP_USART_SEND((const uint8_t *)pHandle->tATCmdInfo.pszATCmd, strlen(pHandle->tATCmdInfo.pszATCmd));
            ESP_USART_SEND((const uint8_t *)"\r\n", 2);
            
            if (pHandle->tATCmdInfo.sendCnt == 0)
            {
                DEBUG_PRINTF("Send AT Cmd By Task[%d]: %s\r\n", strlen(pHandle->tATCmdInfo.pszATCmd) + 2, pHandle->tATCmdInfo.pszATCmd);
            }
            else
            {
                DEBUG_PRINTF("ReSend AT Cmd By Task[%d]: %s\r\n", strlen(pHandle->tATCmdInfo.pszATCmd) + 2, pHandle->tATCmdInfo.pszATCmd);
            }
        }

        pHandle->ATAckWaitTic = 0;
        pHandle->isATAckWait = 1;
        pHandle->tATCmdInfo.sendCnt++;
    }
    
    if (s_lastATCmd != currATCmd)
    {
        s_lastATCmd = currATCmd;
    }
}

/**
  * @brief      处理网络数据.
  * @param[in]  kpszBufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
static void ProcessingSmartInfo(const void *kpszBufData, uint16_t len)
{
    const char *kpszBufDataPtr = kpszBufData;
    char *pTmpPtr;
    
    if (sg_tEspHandle.isConfigSmartState)
    {
        if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "Smart get wifi info")) != 0)
        {
            kpszBufDataPtr += 21;
            
            if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "ssid:")) != 0 &&
                (pTmpPtr = strstr(kpszBufDataPtr, "\r\n")) != 0)
            {
                kpszBufDataPtr += 5;
                memcpy(sg_tEspHandle.tWifiInfo.szSsid, kpszBufDataPtr, pTmpPtr - kpszBufDataPtr);
            }
            
            if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "password:")) != 0 &&
                (pTmpPtr = strstr(kpszBufDataPtr, "\r\n")) != 0)
            {
                kpszBufDataPtr += 9;
                memcpy(sg_tEspHandle.tWifiInfo.szPassword, kpszBufDataPtr, pTmpPtr - kpszBufDataPtr);
            }
            
            ESP_DEBUG_PRINTF("Config Wifi:\r\nssid: %s\r\npassword: %s\r\n", 
                    sg_tEspHandle.tWifiInfo.szSsid, sg_tEspHandle.tWifiInfo.szPassword);
        }
    }
}

/**
  * @brief      处理网络数据.
  * @param[in]  kpszBufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
static void ProcessingNetworkData(const void *kpszBufData, uint16_t len)
{
    uint8_t id;
    const char *kpszBufDataPtr = kpszBufData;
    uint16_t tcpDataLenth;
    
    if (sg_tEspHandle.isSeriaNet)
    {
        if (sg_tEspHandle.arrtNetManager[0].isConnect)
        {
            tcpDataLenth = len;
            ESP_DEBUG_PRINTF("TCP: DATA[%d]:\r\n%s\r\n\r\n", tcpDataLenth, kpszBufDataPtr);
            
            if (sg_tEspHandle.arrtNetManager[0].pfnRecvCallFun != NULL)
            {
                sg_tEspHandle.arrtNetManager[0].pfnRecvCallFun(0, kpszBufDataPtr, tcpDataLenth);
            }
        }
    }
    else
    {
        if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "+IPD,")) != 0)// 接收数据
        {
            kpszBufDataPtr += 5;  // 跳过"+IPD,"
            
            if (sg_tEspHandle.isMultiConnect)
            {
                id = atoi(kpszBufDataPtr);
                kpszBufDataPtr += 2;
            }
            else
            {
                id = 0;
            }
            
            if (id < 5)
            {
                sg_tEspHandle.arrtNetManager[id].isConnect = 1;
                tcpDataLenth = atoi(kpszBufDataPtr);
                kpszBufDataPtr = strstr(kpszBufDataPtr, ":");
                kpszBufDataPtr++;

                ESP_DEBUG_PRINTF("TCP: ID[%d].DATA[%d]:\r\n%s\r\n\r\n", id, tcpDataLenth, kpszBufDataPtr);
                
                if (sg_tEspHandle.arrtNetManager[id].pfnRecvCallFun != NULL)
                {
                    sg_tEspHandle.arrtNetManager[id].pfnRecvCallFun(id, kpszBufDataPtr, tcpDataLenth);
                }
            }
        }
    }
}

/**
  * @brief      判断WIFI连接状态函数.
  * @param[in]  kpszBufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
static void JudgeWifiConnectionStatus(const void *kpszBufData, uint16_t len)
{
    if (sg_tEspHandle.workMode == ESP_MODE_STATION || sg_tEspHandle.workMode == ESP_MODE_AP_STATION)
    {
        if (sg_tEspHandle.isWifiConnected)
        {
            if (strstr(kpszBufData, "WIFI DISCONNECT") != 0)
            {
                sg_tEspHandle.isWifiConnected = 0;
                DEBUG_PRINTF("wifi disconnected\r\n");
            }
        }
        else
        {
            //if (strstr(kpszBufData, "WIFI CONNECTED") != 0)
            if (strstr(kpszBufData, "WIFI GOT IP") != 0)
            {
                sg_tEspHandle.isWifiConnected = 1;
                DEBUG_PRINTF("wifi connected\r\n");
            }
        }
    }
}

/**
  * @brief      判断TCP连接状态函数.
  * @param[in]  kpszBufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
static void JudgeNetConnectionStatus(const void *kpszBufData, uint16_t len)
{
    uint8_t id;
    const char *kpszBufDataPtr = kpszBufData;
    
    if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "CONNECT\r\n")) != 0 &&
        *(kpszBufDataPtr - 1) != ' ' && *(kpszBufDataPtr - 1) != 'S')  // 排除"WIFI CONNECT" 和 "WIFI DISCONNECT"
    {
        if (sg_tEspHandle.isMultiConnect)
        {
            id = atoi(kpszBufDataPtr - 2);
        }
        else
        {
            id = 0;
        }
  
        if (id < 5 && sg_tEspHandle.arrtNetManager[id].isConnect == 0)
        {
            DEBUG_PRINTF("TCP: ID [%d] CONNECT\r\n", id);
            sg_tEspHandle.arrtNetManager[id].isConnect = 1;
        }
    }
    
    if ((kpszBufDataPtr = strstr(kpszBufData, "CLOSED\r\n")) != 0)
    {
        if (sg_tEspHandle.isMultiConnect)
        {
            id = atoi(kpszBufDataPtr - 2);
        }
        else
        {
            id = 0;
        }

        if (id < 5 && sg_tEspHandle.arrtNetManager[id].isConnect == 1)
        {
            DEBUG_PRINTF("TCP: ID [%d] CLOSED\r\n", id);
            sg_tEspHandle.arrtNetManager[id].isConnect = 0;
        }
    }
}

/**
  * @brief      接收并解析串口数据函数.
  * @param[in]  kpszBufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
void ESP_OnReciveParseUsartData(const void *kpszBufData, uint16_t len)
{
    const char *kpszBufDataPtr = kpszBufData;

    if (kpszBufDataPtr == NULL || len == 0)
    {
        return;
    }
    
    DEBUG_PRINTF("ESP Usart Recv Data[%d]:\r\n%s\r\n\r\n", len, kpszBufDataPtr);
    
    if (sg_tEspHandle.isATAckWait)
    { 
        if (sg_tEspHandle.tATCmdInfo.pfnUnpackAck != NULL)
        {
            sg_tEspHandle.tATCmdInfo.pfnUnpackAck(kpszBufDataPtr, len);
        }
        else
        {
            OnUnpackGeneralAck(kpszBufDataPtr, len);
        }
    }
    
    ProcessingNetworkData(kpszBufData, len);
    ProcessingSmartInfo(kpszBufData, len);
    JudgeWifiConnectionStatus(kpszBufData, len);
    JudgeNetConnectionStatus(kpszBufData, len);
}

/**
  * @brief      处理响应超时函数.
  * @param[in]  pHandle 句柄
  * @retval     None
  */
static void ProcessAckOverTime(EspHandle_t *pHandle)
{
    EspResCallFun pfnCallFun;
    
    if (pHandle->isATAckWait)
    {
        pHandle->ATAckWaitTic += 10;
        
        if (pHandle->ATAckWaitTic > pHandle->tATCmdInfo.maxWaitTime)
        {
            pHandle->isATAckWait = 0;
            
            if (pHandle->tATCmdInfo.sendCnt > pHandle->tATCmdInfo.resendMaxCnt)
            {
                if ((pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask)) != NULL)
                {
                    pfnCallFun(ESP_ACK_OVERTIME, NULL, 0);
                }
        
                if (pHandle->tATCmdInfo.pfnOverTimeHandle != NULL)
                {
                    pHandle->tATCmdInfo.pfnOverTimeHandle();
                }
                else
                {
                    DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
                }
            }
        }
    }
}

/**
  * @brief      硬件底层初始化.
  * @retval     None.
  */
static void EspDriveInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    /* 使能端口复用时钟 GPIOA GPIOB 时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

/**
  * @brief   ESP8266初始化
  * @return  None
  */
void ESP_Init(void)
{
    EspDriveInit();
    
    memset(&sg_tEspHandle, 0, sizeof(sg_tEspHandle));
    sg_tEspHandle.workMode = ESP_MODE_AP_STATION;
    sg_tEspHandle.isWifiConnected = 0;

//    memcpy(sg_tEspHandle.tWifiInfo.szSsid, "espTest", 7);
    InitCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_CMD_TASK_LIST_NUM);
}













/**
  * @brief      设置 TCP/UDP 网络通信的数据接收回调函数.
  * @param[in]  id  连接的 id 号, 范围 0-4 
  * @param[in]  CallFun  数据接收回调函数
  * @retval     返回的值含义如下
  *             @arg -1: 失败
  *             @arg  0: 成功
  */
int ESP_SetNetDataRecv(uint8_t id, EspRecvCallFun CallFun)
{
    if (id < 5)
    {
        sg_tEspHandle.arrtNetManager[id].pfnRecvCallFun = CallFun;
        return 0;
    }
    
    return -1;
}

int ESP_SendNetData(uint8_t id, const void *pBody, uint16_t lenth, EspResCallFun CallFun)
{
    int ret = 0;
    
    if (!sg_tEspHandle.isMultiConnect)
    {
        id = 0;
    }
    
    sg_tEspHandle.currNetTcpId = id;
    memcpy(sg_tEspHandle.sendBufData, pBody, lenth);
    sg_tEspHandle.sendBufDataLenth = lenth;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SEND_DATA, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_TRANSFER_DATA, CallFun);
    
    return ret;
}

/**
  * @brief      处理WIFI应用模式的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
void OnPackCWModeCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->tATCmdInfo.isQuery)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s?", pHandle->tATCmdInfo.pszATCmd);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->workMode);
    }

    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      处理设置连接 WIFI 的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackJoinAPCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->tATCmdInfo.isQuery)
    {
        pHandle->tATCmdInfo.pszATAck = "OK";
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s?", pHandle->tATCmdInfo.pszATCmd);
    }
    else
    {
        pHandle->tATCmdInfo.pszATAck = "WIFI CONNECTED";
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=\"%s\",\"%s\"", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->tWifiInfo.szSsid, pHandle->tWifiInfo.szPassword);
    }

    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      设置AP模式下参数的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackSetAPParamCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->tATCmdInfo.isQuery)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s?", pHandle->tATCmdInfo.pszATCmd);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=\"%s\",\"%s\",%d,%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->tWifiHostInfo.szSsid, pHandle->tWifiHostInfo.szPassword, 
            pHandle->tWifiHostInfo.channel, pHandle->tWifiHostInfo.eEncWay);
    }

    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      建立 TCP 连接或注册 UDP 端口号的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackBuildTCPOrUDPCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    uint8_t id = pHandle->currNetTcpId;
    char *pszType[2] = {"TCP", "UDP"};
    uint8_t type = pHandle->arrtNetManager[id].eIPConnectType == ESP_TYPE_TCP ? 0 : 1;
    
    
    if (!pHandle->isMultiConnect)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=\"%s\",\"%s\",%d", pHandle->tATCmdInfo.pszATCmd, 
            pszType[type], pHandle->arrtNetManager[id].szNetAddr, pHandle->arrtNetManager[0].netPort);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d,\"%s\",\"%s\",%d", pHandle->tATCmdInfo.pszATCmd, 
            id, pszType[type], pHandle->arrtNetManager[id].szNetAddr, pHandle->arrtNetManager[id].netPort);
    }
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      TCP 或 UDP 发送数据的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackTCPOrUDPDataCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->isMultiConnect)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d,%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->currNetTcpId, pHandle->sendBufDataLenth);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->sendBufDataLenth);
    }
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      TCP 或 UDP 透传数据的发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackTCPOrUDPData(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->sendBufData;
}

/**
  * @brief      关闭 TCP 或 UDP 的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackCloseTCPOrUDPCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d", pHandle->tATCmdInfo.pszATCmd, 
        pHandle->currNetTcpId);
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      启动多连接的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackStartMultiConnectCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->isMultiConnect)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=1", pHandle->tATCmdInfo.pszATCmd);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=0", pHandle->tATCmdInfo.pszATCmd);
    }
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      配置为服务器的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackConfigServiceCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->tServerInfo.isEnable)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=1,%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->tServerInfo.port);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=0,%d", pHandle->tATCmdInfo.pszATCmd, 
            pHandle->tServerInfo.port);
    }
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      选择传输模式的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackSetTransferCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    if (pHandle->isSeriaNet)
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=1", pHandle->tATCmdInfo.pszATCmd);
    }
    else
    {
        snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=0", pHandle->tATCmdInfo.pszATCmd);
    }
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      设置服务器超时时间的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnPackSetServiceOvertimeCmd(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    snprintf(pHandle->szAtCmdBuf, sizeof(pHandle->szAtCmdBuf), "%s=%d", pHandle->tATCmdInfo.pszATCmd, 
        pHandle->tServerInfo.overtime);
    pHandle->tATCmdInfo.pszATCmd = pHandle->szAtCmdBuf;
}

/**
  * @brief      处理通用响应函数.
  * @param[in]  bufData 接收的数据
  * @param[in]  len 接收的数据长度
  * @retval     None
  */
static void OnUnpackGeneralAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
    
    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL ||
        strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

/**
  * @brief      设置AT指令回显开关的AT指令发送函数.
  * @param[in]  pBody  EspHandle_t句柄
  * @retval     None
  */
static void OnUnpackATEAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;

    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL || 
        (strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0 &&
        strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATCmd) == 0))
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}


/**
  * @brief      转换信号值
  * @param[in]  signalValue  原始信号值
  * @retval     转换后信号值, 值越大信号越好
  */
static uint8_t ConversionSignalIntensity(int signalValue)
{
    uint8_t signalIntensity;
    
    if (signalValue >= -55)
    {
        signalIntensity = 4;
    }
    else if (signalValue >= -66 && signalValue < -55)
    {
        signalIntensity = 3;
    }
    else if (signalValue >= -77 && signalValue < -66)
    {
        signalIntensity = 2;
    }
    else if (signalValue >= -88 && signalValue < -77)
    {
        signalIntensity = 1;
    }
    else if (signalValue < -88)
    {
        signalIntensity = 0;
    }
    
    return signalIntensity;
}

/**
  * @brief      处理获取连接状态和参数的响应
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
static void OnUnpackGetConnectStatusAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
 
    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL || 
        strstr(kBufData, sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        if (strstr(kBufData, "CIPSTATUS") != 0)
        {
        
        }
        
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

static void OnUnpackBuildTCPOrUDPAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
    }

    if (strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        JudgeNetConnectionStatus(kBufData, len);
        
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

/*
    src 源字符串的首地址(buf的地址)
    separator 指定的分割字符
    dest 接收子字符串的数组
    num 分割后子字符串的个数
*/
void split(const char *src,const char *separator,char **dest, uint8_t *num) {
     char *pNext;
     int count = 0;
     if (src == NULL || strlen(src) == 0) //如果传入的地址为空或长度为0，直接终止
        return;
     if (separator == NULL || strlen(separator) == 0) //如未指定分割的字符串，直接终止
        return;
     pNext = (char *)strtok((char *)src, (char *)separator); //必须使用(char *)进行强制类型转换(虽然不写有的编译器中不会出现指针错误)
     while(pNext != NULL) {
          *dest++ = pNext;
          ++count;
         pNext = (char *)strtok(NULL,separator);  //必须使用(char *)进行强制类型转换
    }
    *num = count;
}

static void OnUnpackTCPOrUDPDataCmdAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
    }
 
    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
    else if (strstr((char *)kBufData, "link is not valid") != 0)
    {
        ESP_DEBUG_PRINTF("TCP Data Send Fail: %d Closed\r\n", sg_tEspHandle.currNetTcpId);
        
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_FAIL, NULL, 0);
        }
        
        sg_tEspHandle.arrtNetManager[sg_tEspHandle.currNetTcpId].isConnect = 0;
    }
}

static void OnUnpackTCPOrUDPDataSendAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
    }
 
    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        sg_tEspHandle.isATAckWait = 1;
        return;
    }
    
    if (strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

static void OnUnpackStartMultiConnectAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL || strlen(sg_tEspHandle.tATCmdInfo.pszATAck) == 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
    }

    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (strstr((char *)kBufData, (char *)sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
    else if (strstr((char *)kBufData, "Link is builded") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_FAIL, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        sg_tEspHandle.isMultiConnect = sg_tEspHandle.isMultiConnectBak;
    }
}

static void OnUnpackEnterConfigSmartAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;

    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL || 
        strstr(kBufData, sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        sg_tEspHandle.isConfigSmartState = 1;
        
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

static void OnUnpackExitConfigSmartAck(const char *kBufData, uint16_t len)
{
    EspResCallFun pfnCallFun = ReadCmdTaskCallFunFromList(&sg_tEspHandle.tATCmdTask);
    
    sg_tEspHandle.isATAckWait = 0;

    if (strstr((char *)kBufData, "busy s...") != 0)
    {
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_BUSY, NULL, 0);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
        return;
    }
    
    if (sg_tEspHandle.tATCmdInfo.pszATAck == NULL || 
        strstr(kBufData, sg_tEspHandle.tATCmdInfo.pszATAck) != 0)
    {
        sg_tEspHandle.isConfigSmartState = 0;
        
        if (pfnCallFun != NULL)
        {
            pfnCallFun(ESP_ACK_SUCCESS, kBufData, len);
        }
        
        DeletCurrCmdTaskFromList(&sg_tEspHandle.tATCmdTask);
    }
}

/**
  * @brief      获取 WIFI 连接状态. 
  * @note       实时监控更新状态
  * @retval     返回的值含义如下
  *                 @arg  1: 已连接
  *                 @arg  0: 未连接
  */
uint8_t ESP_GetWifiConnectStatus(void)
{
    return sg_tEspHandle.isWifiConnected;
}

/**
  * @brief      获取指定 ID 号的 TCP/UDP 连接状态.
  * @note       实时监控更新状态, 也可以通过函数 ESP_QueryNetConnectStatus 查询更新
  * @param[in]  id          连接的 id 号, 范围 0-4 
  * @retval     返回的值含义如下
  *                 @arg  1: 已连接
  *                 @arg  0: 未连接
  */
uint8_t ESP_GetTcpConnectStatus(uint8_t id)
{
    if (id > 4)
    {
        return 0;
    }
    
    return sg_tEspHandle.arrtNetManager[id].isConnect;
}

/**
  * @brief      查询 ESP8266 的版本信息.
  * @note       回调函数中响应成功后通过函数 ESP_GetVersionInfo 解析数据得到版本信息
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_QueryVersionInfo(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_SEE_VERSION].isQuery = 1;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SEE_VERSION, CallFun);
    
    return ret;
}

/**
  * @brief      设置 ESP8266 的工作模式.
  * @param[in]  mode  工作模式, 参见枚举 @enum eEspCWMode
  * @param[in]  CallFun  设置后的回调函数
  * @retval     None
  */
int ESP_SetWorkMode(eEspCWMode mode, EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_tEspHandle.workMode = mode;
    sg_ktCmdTable[ESP_AT_CMD_LISTS_AP].isQuery = 0;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_MODE, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_RST, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_AT_TEST, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_ATE, CallFun);
    
    return ret;
}

/**
  * @brief      查询 ESP8266 的工作模式.
  * @note       回调函数中响应成功后通过函数 ESP_GetWorkMode 解析数据得到当前工作模式
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_QueryWorkMode(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_SET_MODE].isQuery = 1;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_MODE, CallFun);
    
    return ret;
}

/**
  * @brief      解析当前工作模式
  * @param[out] pMode   工作模式, 参见枚举 @enum eEspCWMode
  * @param[in]  pszRx   数据
  * @param[in]  lenth   数据长度
  * @retval     -1, 解析失败; 其他, 可用的 WIFI 列表数目
  */
int ESP_GetWorkMode(eEspCWMode *pMode, const char *pszRx, uint16_t lenth)
{
    const char *kBufData = pszRx;
    
    if ((kBufData = strstr(kBufData, "+CWMODE:")) != NULL)
    {
        kBufData += 8;
        *pMode = (eEspCWMode)atoi(kBufData);
        
        return 0;
    }
    
    return -1;
}

/**
  * @brief      查询当前可用的 WIFI 列表信息.
  * @note       回调函数中响应成功后通过函数 ESP_AnalysisWifiInfoLists 解析数据得到 Wifi 列表信息
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_QueryWifiInfoLists(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_LISTS_AP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_LISTS_AP, CallFun);
    
    return ret;
}

/**
  * @brief      设置连接的 WIFI 
  * @param[in]  pkszName    WIFI名称
  * @param[in]  pkszPassword WIFI密码
  * @param[in]  CallFun  设置后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_SetWifiConnect(const char *pkszName, const char *pkszPassword, EspResCallFun CallFun)
{
    sg_ktCmdTable[ESP_AT_CMD_JOIN_AP].isQuery = 0;
    snprintf(sg_tEspHandle.tWifiInfo.szSsid, sizeof(sg_tEspHandle.tWifiInfo.szSsid), "%s", pkszName);
    snprintf(sg_tEspHandle.tWifiInfo.szPassword, sizeof(sg_tEspHandle.tWifiInfo.szPassword), "%s", pkszPassword);
    
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_JOIN_AP, CallFun);
}

/**
  * @brief      断开连接的 WIFI 
  * @param[in]  CallFun  设置后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_SetWifiDisconnect(EspResCallFun CallFun)
{
    sg_ktCmdTable[ESP_AT_CMD_QUIT_AP].isQuery = 0;
    
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_QUIT_AP, CallFun);
}

/**
  * @brief      设置 AP 模式(WIFI热点)下的参数
  * @param[in]  pkszName    WIFI热点名称
  * @param[in]  pkszPassword WIFI热点密码, 长度大于等于8
  * @param[in]  ch          通道号
  * @param[in]  encWay 加密方式, 取值参考 @enum eEncryptionWay
  * @param[in]  CallFun  设置后的回调函数
  * @return     None
  */
int ESP_SetWiFiHotspot(const char *pkszName, const char *pkszPassword, uint8_t ch, eEncryptionWay encWay, EspResCallFun CallFun)
{
    snprintf(sg_tEspHandle.tWifiHostInfo.szSsid, sizeof(sg_tEspHandle.tWifiHostInfo.szSsid), "%s", pkszName);
    snprintf(sg_tEspHandle.tWifiHostInfo.szPassword, sizeof(sg_tEspHandle.tWifiHostInfo.szPassword), "%s", pkszPassword);
    sg_tEspHandle.tWifiHostInfo.channel = ch;
    sg_tEspHandle.tWifiHostInfo.eEncWay = encWay;
    
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_AP_PARAM, CallFun);
}

/**
  * @brief      解析当前可用的 WIFI 列表信息.
  * @param[out] pInfo   解析后的 WIFI 的列表信息
  * @param[in]  num     列表支持大小
  * @param[in]  pszRx   数据
  * @param[in]  lenth   数据长度
  * @retval     -1, 解析失败; 其他, 可用的 WIFI 列表数目
  */
int ESP_GetWifiInfoLists(ESP_WifiInfo_t *pInfo, uint8_t num, const char *pszRx, uint16_t lenth)
{
    char *pend;
    const char *pszWifiList = pszRx;
    char szWifiName[30];
    uint8_t encWay;
    uint16_t cnt = 0;
    
    if (pInfo == NULL || num == 0)
    {
        return -1;
    }

    do
    {
        pszWifiList = strstr(pszWifiList, "+CWLAP:(");
        
        if (pszWifiList)
        {
            memset(szWifiName, 0, 30);
            pszWifiList += 8;
            encWay = atoi(pszWifiList);
            pszWifiList += 3;
            pend = strstr(pszWifiList, "\"");
            memcpy(szWifiName, pszWifiList, pend - pszWifiList);
            pszWifiList = pend;
            
            pInfo[cnt].eEncWay = (eEncryptionWay)encWay;
            memcpy(pInfo[cnt].szSsid, szWifiName, strlen(szWifiName));
            pInfo[cnt].signalIntensity = ConversionSignalIntensity(atoi(pszWifiList + 2));
            cnt++;
        }
    } while (pszWifiList && cnt < num);
    
    if (cnt)
    {
        return cnt;
    }

    return -1;
}

/**
  * @brief      查询当前 WIFI 的信息.
  * @note       回调函数中响应成功后通过函数 ESP_AnalysisWifiInfo 解析数据得到 Wifi 信息
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_QueryWifiInfo(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_JOIN_AP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_JOIN_AP, CallFun);

    return ret;
}

/**
  * @brief      解析出当前 WIFI 的信息.
  * @note       无加密类型信息
  * @param[out] pInfo   解析后的 WIFI 的信息
  * @param[in]  pszRx   数据
  * @param[in]  lenth   数据长度
  * @retval     -1, 解析失败; 0, 解析成功
  */
int ESP_GetWifiInfo(ESP_WifiInfo_t *pInfo, const char *pszRx, uint16_t lenth)
{
    uint8_t num = 0;
    const char *kBufData = pszRx;
    char *pSplitPtr[8] = {0}; //存放分割后的子字符串指针 
    
    if ((kBufData = strstr(kBufData, "+CWJAP:")) != NULL)
    {
        kBufData += 8;
        split(kBufData, ",", pSplitPtr, &num);
        memcpy(pInfo->szSsid, pSplitPtr[0], strlen(pSplitPtr[0]) - 1);
        pInfo->eEncWay = ESP_ENC_WAY_OPEN;
        pInfo->channel = atoi(pSplitPtr[2]);
        pInfo->signalIntensity = ConversionSignalIntensity(atoi(pSplitPtr[3]));

        return 0;
    }
    
    return -1;
}

/**
  * @brief      WIFI 热点模式下查看已接入的设备 IP
  * @note       回调函数中响应成功后通过函数 ESP_GetCurrentConnectIP 解析数据得到已接入设备的IP信息
  * @param[in]  CallFun  查询后的回调函数
  * @retval     0, 添加任务成功; <0, 添加任务失败
  */
int ESP_QueryCurrentConnectIP(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_LISTS_JOINED_IP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_LISTS_JOINED_IP, CallFun);

    return ret;
}

/**
  * @brief      解析获取已接入设备的IP
  * @param[out] pInfo   解析后的 WIFI 的信息
  * @param[in]  pszRx   数据
  * @param[in]  lenth   数据长度
  * @retval     -1, 解析失败; 0, 解析成功
  */
int ESP_GetCurrentConnectIP(ESP_IPInfo_t *pIPListInfo, const char *pszRx, uint16_t lenth)
{
    uint8_t num = 0;
    const char *kBufData = pszRx;
    char *pSplitPtr[8] = {0}; //存放分割后的子字符串指针 
    char szTmpStr[30] = {0};
    
    memcpy(szTmpStr, pIPListInfo->szIP, strlen(pIPListInfo->szIP) + 1);
    split(kBufData, ".", pSplitPtr, &num);
    
    if (num == 4)
    {
        ESP_DEBUG_PRINTF("kBufData: %s\r\n", kBufData);
        split(kBufData, ",", pSplitPtr, &num);
        ESP_DEBUG_PRINTF("split %d[%s  %s]\r\n", num, pSplitPtr[0],  pSplitPtr[1]);
        memcpy(pIPListInfo->szIP, pSplitPtr[0], strlen(pSplitPtr[0]) - 1);
        memcpy(pIPListInfo->szMAC, pSplitPtr[1], strlen(pSplitPtr[1]));
        
        memcpy(szTmpStr, pIPListInfo->szIP, strlen(pIPListInfo->szIP) + 1);
        split(szTmpStr, ".", pSplitPtr, &num);
        pIPListInfo->IP[0] = atoi(pSplitPtr[0]);
        pIPListInfo->IP[1] = atoi(pSplitPtr[1]);
        pIPListInfo->IP[2] = atoi(pSplitPtr[2]);
        pIPListInfo->IP[3] = atoi(pSplitPtr[3]);
        
        memcpy(szTmpStr, pIPListInfo->szMAC, strlen(pIPListInfo->szMAC) + 1);
        split(szTmpStr, ":", pSplitPtr, &num);
        pIPListInfo->MAC[0] = atoi(pSplitPtr[0]);
        pIPListInfo->MAC[1] = atoi(pSplitPtr[1]);
        pIPListInfo->MAC[2] = atoi(pSplitPtr[2]);
        pIPListInfo->MAC[3] = atoi(pSplitPtr[3]);
        pIPListInfo->MAC[4] = atoi(pSplitPtr[4]);
        pIPListInfo->MAC[5] = atoi(pSplitPtr[5]);

        return 0;
    }
    
    return -1;
}

/**
  * @brief      获得TCP/UDP连接状态.
  * @note       回调函数中响应成功后通过函数 ESP_GetTcpConnectStatus 得到TCP/UDP连接状态
  * @param[in]  CallFun  查询后的回调函数
  * @retval     返回的值含义如下
  *                 @arg -1: 失败
  *                 @arg  0: 成功
  */
int ESP_QueryNetConnectStatus(EspResCallFun CallFun)
{
        sg_ktCmdTable[ESP_AT_CMD_GET_CONNECT_STATE].isQuery = 0;
        return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_GET_CONNECT_STATE, CallFun);
}

/**
  * @brief      设置 TCP 连接或注册 UDP 端口号.
  * @param[in]  id          连接的 id 号, 范围 0-4 
  * @param[in]  type        明连接类型, 参见枚举 @enum eEspIpType
  * @param[in]  pkszAddr    字符串参数，远程服务器 IP 地址
  * @param[in]  port        远程服务器端口号
  * @param[in]  CallFun     设置后的回调函数
  * @retval     返回的值含义如下
  *                 @arg -1: 失败
  *                 @arg  0: 成功
  */
int ESP_BuildNetConnect(uint8_t id, eEspIpType type, const char *pkszAddr, uint32_t port, EspResCallFun CallFun)
{
    if (id < 5)
    {
        sg_tEspHandle.currNetTcpId = id;
        sg_tEspHandle.arrtNetManager[id].eIPConnectType = type;
        snprintf(sg_tEspHandle.arrtNetManager[id].szNetAddr, sizeof(sg_tEspHandle.arrtNetManager[id].szNetAddr), "%s", pkszAddr);
        sg_tEspHandle.arrtNetManager[id].netPort = port;
        
        sg_ktCmdTable[ESP_AT_CMD_BUILD_TCP_OR_UDP].isQuery = 0;
        return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_BUILD_TCP_OR_UDP, CallFun);
    }
    
    return -1;
}

/**
  * @brief      设置 TCP 断开或注销 UDP 端口号.
  * @param[in]  id          连接的 id 号, 范围 0-4 
  * @param[in]  CallFun     设置后的回调函数
  * @retval     返回的值含义如下
  *                 @arg -1: 失败
  *                 @arg  0: 成功
  */
int ESP_CloseNetConnect(uint8_t id, EspResCallFun CallFun)
{
    if (id < 5)
    {
        sg_tEspHandle.currNetTcpId = id;
        
        sg_ktCmdTable[ESP_AT_CMD_CLOSE_TCP_OR_UDP].isQuery = 0;
        return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_CLOSE_TCP_OR_UDP, CallFun);
    }
    
    return -1;
}

/**
  * @brief      获取本地 IP 地址.
  * @param[in]  CallFun  查询后的回调函数
  * @retval     返回的值含义如下
  *                 @arg -1: 失败
  *                 @arg  0: 成功
  */
int ESP_QueryLocalIP(EspResCallFun CallFun)
{
    sg_ktCmdTable[ESP_AT_CMD_GET_LOCAL_IP].isQuery = 0;
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_GET_LOCAL_IP, CallFun);
}

static int CharToByte(uint8_t *pHex, const char pChar[2])
{
	int i;
	uint8_t hex[2] = { 0 };

	for (i = 0; i < 2; i++)
	{
		if (pChar[i] >= '0' && pChar[i] <= '9')
		{
			hex[i] = (uint8_t)(pChar[i] - '0');
		}
		else if (pChar[i] >= 'A' && pChar[i] <= 'F')
		{
			hex[i] = (uint8_t)(pChar[i] - 'A' + 10);
		}
		else if (pChar[i] >= 'a' && pChar[i] <= 'f')
		{
			hex[i] = (uint8_t)(pChar[i] - 'a' + 10);
		}
		else
		{
			return -1;
		}
	}

	*pHex = (hex[0] << 4) | hex[1];

	return 0;
}

/**
  * @brief      解析获取本地的IP
  * @param[out] pWifiIP   解析后的 WIFI IP 的信息
  * @param[out] pWifiIP   解析后的服务器 IP 信息
  * @param[in]  pszRx     接收的数据
  * @param[in]  lenth     接收的数据长度
  * @retval     -1, 解析失败; 0, 解析成功
  */
int ESP_GetLocalIP(ESP_IPInfo_t *pWifiIP, ESP_IPInfo_t *pServerIP, const char *pszRx, uint16_t lenth)
{
    uint8_t num = 0, cnt = 0;
    const char *kBufData = pszRx;
    char *pLineSplitPtr[8] = {0};
    char *pSplitPtr[8] = {0};
    char szTmpStr[30] = {0};
    
    memset(pWifiIP->szIP, 0, sizeof(pWifiIP->szIP));
    memset(pWifiIP->szMAC, 0, sizeof(pWifiIP->szMAC));
    memset(pServerIP->szIP, 0, sizeof(pServerIP->szIP));
    memset(pServerIP->szMAC, 0, sizeof(pServerIP->szMAC));
    
    split(kBufData, "\r\n", pLineSplitPtr, &num);
    
    if (num >= 4)
    {
        while  (cnt < num)
        {
            if ((kBufData = strstr(pLineSplitPtr[cnt], "APIP,")) != NULL)
            {
                kBufData += 6;
                memcpy(pWifiIP->szIP, kBufData, strlen(kBufData) - 1);
                memcpy(szTmpStr, pWifiIP->szIP, strlen(pWifiIP->szIP) + 1);
                split(szTmpStr, ".", pSplitPtr, &num);
                pWifiIP->IP[0] = atoi(pSplitPtr[0]);
                pWifiIP->IP[1] = atoi(pSplitPtr[1]);
                pWifiIP->IP[2] = atoi(pSplitPtr[2]);
                pWifiIP->IP[3] = atoi(pSplitPtr[3]);
            }
            else if ((kBufData = strstr(pLineSplitPtr[cnt], "APMAC,")) != NULL)
            {
                kBufData += 7;
                memcpy(pWifiIP->szMAC, kBufData, strlen(kBufData) - 1);
                memcpy(szTmpStr, pWifiIP->szMAC, strlen(pWifiIP->szMAC) + 1);
                split(szTmpStr, ":", pSplitPtr, &num);
                CharToByte(&pWifiIP->MAC[0], pSplitPtr[0]);
                CharToByte(&pWifiIP->MAC[1], pSplitPtr[1]);
                CharToByte(&pWifiIP->MAC[2], pSplitPtr[2]);
                CharToByte(&pWifiIP->MAC[3], pSplitPtr[3]);
                CharToByte(&pWifiIP->MAC[4], pSplitPtr[4]);
                CharToByte(&pWifiIP->MAC[5], pSplitPtr[5]);
            }
            else if ((kBufData = strstr(pLineSplitPtr[cnt], "STAIP,")) != NULL)
            {
                kBufData += 7;
                memcpy(pServerIP->szIP, kBufData, strlen(kBufData) - 1);
                memcpy(szTmpStr, pServerIP->szIP, strlen(pServerIP->szIP) + 1);
                split(szTmpStr, ".", pSplitPtr, &num);
                pServerIP->IP[0] = atoi(pSplitPtr[0]);
                pServerIP->IP[1] = atoi(pSplitPtr[1]);
                pServerIP->IP[2] = atoi(pSplitPtr[2]);
                pServerIP->IP[3] = atoi(pSplitPtr[3]);
            }
            else if ((kBufData = strstr(pLineSplitPtr[cnt], "STAMAC,")) != NULL)
            {
                kBufData += 8;
                memcpy(pServerIP->szMAC, kBufData, strlen(kBufData) - 1);
                memcpy(szTmpStr, pServerIP->szMAC, strlen(pServerIP->szMAC) + 1);
                split(szTmpStr, ":", pSplitPtr, &num);
                CharToByte(&pServerIP->MAC[0], pSplitPtr[0]);
                CharToByte(&pServerIP->MAC[1], pSplitPtr[1]);
                CharToByte(&pServerIP->MAC[2], pSplitPtr[2]);
                CharToByte(&pServerIP->MAC[3], pSplitPtr[3]);
                CharToByte(&pServerIP->MAC[4], pSplitPtr[4]);
                CharToByte(&pServerIP->MAC[5], pSplitPtr[5]);
            }
            
            cnt++;
        }
        
        return 0;
    }
    
    return -1;

}

/**
  * @brief      设置为多路连接模式
  * @note       只有当连接都断开后才能更改，如果开启过 server 需要重启模块
  * @param[in]  isEnable 多路连接模式开启/关闭
  *               @arg 0 禁止, 单路连接模式  
  *               @arg 1 使能, 多路连接模式
  * @param[in]  CallFun  设置后的回调函数
  * @return     None
  */
int ESP_SetMultiConnect(uint8_t isEnable, EspResCallFun CallFun)
{
    sg_tEspHandle.isMultiConnectBak = sg_tEspHandle.isMultiConnect;
    sg_tEspHandle.isMultiConnect = isEnable;
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_START_MULTI_CONNECT, CallFun);
}

/**
  * @brief      设置服务器参数
  *             开启 server 模式前需要设置多路连接模式, 即先调用函数 ESP_SetMultiConnect 使能
  * @param[in]  isEnable server 模式开启/关闭
  *               @arg 0 关闭 server 模式  
  *               @arg 1 开启 server 模式
  * @param[in]  port 端口号
  * @param[in]  overtime 服务器超时时间, 范围0~28800，单位为 s
  * @param[in]  CallFun  设置后的回调函数
  * @return     None
  */
int ESP_SetServerConfig(uint8_t isEnable, uint32_t port, uint16_t overtime, EspResCallFun CallFun)
{
    int ret = 0;
    
    if (isEnable && !sg_tEspHandle.isMultiConnect)
    {
        return -1;
    }
    
    if (overtime > 28800)
    {
        overtime = 28800;
    }
    
    sg_tEspHandle.tServerInfo.isEnable = isEnable;
    sg_tEspHandle.tServerInfo.port = port;
    sg_tEspHandle.tServerInfo.overtime = overtime;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_CONFIG_SERVICE, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_SERVICE_OVERTIME, CallFun);
    
    return ret;
}

/**
  * @brief      设置为透传模式
  * @note       设置透传模式前需要设置单路连接模式, 即先调用函数 ESP_SetMultiConnect 禁止
  * @param[in]  isEnable 透传模式开启/关闭
  *               @arg 0 禁止, 非透传模式
  *               @arg 1 使能, 透传模式
  * @param[in]  CallFun  设置后的回调函数
  * @return     None
  */
int ESP_SetSeriaNet(uint8_t isEnable, EspResCallFun CallFun)
{
    if (isEnable && sg_tEspHandle.isMultiConnect)
    {
        return -1;
    }
    
    sg_tEspHandle.isSeriaNet = isEnable;
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_TRANSFER_MODE, CallFun);
}

/**
  * @brief      ESP8266 模块任务函数.
  * @retval     None
  */
void ESP_RunTask(void)
{
    ProcessAckOverTime(&sg_tEspHandle);
    
    if (!sg_tEspHandle.isATAckWait)
    {
        SendCmdByTaskList(&sg_tEspHandle);
    }
}

/**
  * @brief      进入 Airkiss 一键配置网络.
  * @note       不管最后是否配置成功，都需要退出
  * @param[in]  CallFun  设置后的回调函数
  * @retval     None
  */
int ESP_EnterAirkissConfigWifi(EspResCallFun CallFun)
{
    int ret = 0;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_ENABLE_AUTO_LINK_WIFI, NULL);
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_ENTER_SMART_CONFIG, CallFun);
    
    return ret;
}

/**
  * @brief      退出 Airkiss 一键配置网络.
  * @param[in]  CallFun  设置后的回调函数
  * @retval     None
  */
int ESP_ExitAirkissConfigWifi(EspResCallFun CallFun)
{
    int ret = 0;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_EXIT_SMART_CONFIG, CallFun);
    
    return ret;
}
