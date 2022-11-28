/**
  **********************************************************************************************************************
  * @file    esp8266.h
  * @brief   该文件提供 ESP8266 驱动所有函数原型
  * @author  周鹏程    any question please send mail to const_zpc@163.com
  * @version V1.0.0
  * @date    2021-3-13
  **********************************************************************************************************************
  *
  **********************************************************************************************************************
  */

/* Define to prevent recursive inclusion -----------------------------------------------------------------------------*/
#ifndef __ESP8266_H
#define __ESP8266_H
 
/* Includes ----------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
 

#define ESP_USART_SEND(data, lenth)  FML_USART_Transmit(USART_ESP8266, data, lenth)

#define ESP_DEBUG_PRINTF(format, ...)  DEBUG_PRINTF(format, ##__VA_ARGS__)
 
 
/* Exported types ----------------------------------------------------------------------------------------------------*/
#ifndef NULL
#define NULL   ((void *)0)
#endif

/**
  * @brief 响应结果
  */
typedef enum
{
    ESP_ACK_SUCCESS = 0,            /*!< (0)响应成功 */
    ESP_ACK_FAIL,                   /*!< (1)响应失败 */
    ESP_ACK_OVERTIME,               /*!< (2)响应超时 */
    ESP_ACK_BUSY,                   /*!< (3)响应忙碌 */
} eEspAck;

typedef void (*EspResCallFun)(eEspAck, const char *, uint16_t);
typedef void (*EspRecvCallFun)(uint8_t, const void *, uint16_t);

/**
  * @brief 应用工作模式
  */
typedef enum
{
    ESP_MODE_STATION = 1,           /*!< (1)Station 模式 */
    ESP_MODE_AP,                    /*!< (2)AP 模式(WIFI热点) */
    ESP_MODE_AP_STATION,            /*!< (3)AP + Station 共存模式 */
} eEspCWMode;

/**
  * @brief AP加密类型
  */
typedef enum
{
    ESP_ENC_WAY_OPEN = 0,           /*!< (0)无密码 */
    ESP_ENC_WAY_WEP,                /*!< (1)WEP 加密方式 */
    ESP_ENC_WAY_WPA_PSK,            /*!< (2)WPA 加密方式 */
    ESP_ENC_WAY_WPA2_PSK,           /*!< (3)WPA2 PSK 加密方式 */
    ESP_ENC_WAY_WPA_WPA2_PSK        /*!< (4)WPA WPA2 PSK 加密方式 */
} eEncryptionWay;

/**
  * @brief 网络连接类型
  */
typedef enum
{
    ESP_TYPE_TCP = 0,               /*!< (0)TCP连接 */
    ESP_TYPE_UDP                    /*!< (1)UDP连接 */
} eEspIpType;

/**
  * @breif WIFI 信息结构体定义
  */
typedef struct
{
    char szSsid[32];                            /*!< WIFI 的名称 */
    
    char szPassword[64];                        /*!< WIFI 的密码 */
    
    uint8_t channel;                            /*!< 通道号 */
    
    eEncryptionWay eEncWay;                     /*!< 加密方式, 取值见 @enum eEncryptionWay */
    
    uint8_t signalIntensity;                    /*!< 信号强度, 0-4: 数值越大信号越好 */
} ESP_WifiInfo_t;

/**
  * @breif IP 信息结构体定义
  */
typedef struct
{
    char szIP[20];                              /*!< IP 地址, 字符串格式 */
    
    char szMAC[20];                             /*!< MAC 物理地址, 字符串格式 */
    
    uint8_t IP[4];                              /*!< IP 地址, 点分十进制格式 */
    
    uint8_t MAC[6];                             /*!< MAC 物理地址, 点分十进制格式 */
} ESP_IPInfo_t;


/* 核心功能函数 *******************************************************************************************************/
extern void ESP_Init(void);
extern void ESP_OnReciveParseUsartData(const void *bufData, uint16_t len);
extern void ESP_RunTask(void);

/* 状态获取函数 *******************************************************************************************************/
extern uint8_t ESP_GetWifiConnectStatus(void);
extern uint8_t ESP_GetTcpConnectStatus(uint8_t id);

/* 基础功能函数 *******************************************************************************************************/
extern int ESP_QueryVersionInfo(EspResCallFun CallFun);

/* WIFI 功能函数 ******************************************************************************************************/
extern int ESP_SetWorkMode(eEspCWMode mode, EspResCallFun CallFun);
extern int ESP_QueryWorkMode(EspResCallFun CallFun);
extern int ESP_GetWorkMode(eEspCWMode *pMode, const char *pszRx, uint16_t lenth);

extern int ESP_SetWifiConnect(const char *pkszName, const char *pkszPassword, EspResCallFun CallFun);
extern int ESP_SetWifiDisconnect(EspResCallFun CallFun);
extern int ESP_SetWiFiHotspot(const char *pkszName, const char *pkszPassword, uint8_t ch, eEncryptionWay encWay, EspResCallFun CallFun);
extern int ESP_QueryWifiInfoLists(EspResCallFun CallFun);
extern int ESP_GetWifiInfoLists(ESP_WifiInfo_t *pInfo, uint8_t num, const char *pszRx, uint16_t lenth);
extern int ESP_QueryWifiInfo(EspResCallFun CallFun);
extern int ESP_GetWifiInfo(ESP_WifiInfo_t *pInfo, const char *pszRx, uint16_t lenth);

extern int ESP_QueryCurrentConnectIP(EspResCallFun CallFun);
extern int ESP_GetCurrentConnectIP(ESP_IPInfo_t *pIPListInfo, const char *pszRx, uint16_t lenth);


/* TCP/IP 功能函数 ****************************************************************************************************/
extern int ESP_QueryNetConnectStatus(EspResCallFun CallFun);
extern int ESP_BuildNetConnect(uint8_t id, eEspIpType type, const char *pkszAddr, uint32_t port, EspResCallFun CallFun);
extern int ESP_CloseNetConnect(uint8_t id, EspResCallFun CallFun);
extern int ESP_QueryLocalIP(EspResCallFun CallFun);
extern int ESP_GetLocalIP(ESP_IPInfo_t *pWifiIP, ESP_IPInfo_t *pServerIP, const char *pszRx, uint16_t lenth);
extern int ESP_SetMultiConnect(uint8_t isEnable, EspResCallFun CallFun);
extern int ESP_SetServerConfig(uint8_t isEnable, uint32_t port, uint16_t overtime, EspResCallFun CallFun);
extern int ESP_SetSeriaNet(uint8_t isEnable, EspResCallFun CallFun);

extern int ESP_SendNetData(uint8_t id, const void *pBody, uint16_t lenth, EspResCallFun CallFun);
extern int ESP_SetNetDataRecv(uint8_t id, EspRecvCallFun CallFun);


/* Airkiss 一键配置网络 ***********************************************************************************************/
extern int ESP_EnterAirkissConfigWifi(EspResCallFun CallFun);
extern int ESP_ExitAirkissConfigWifi(EspResCallFun CallFun);
  
#endif
