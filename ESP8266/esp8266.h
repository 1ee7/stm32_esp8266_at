/**
  **********************************************************************************************************************
  * @file    esp8266.h
  * @brief   ���ļ��ṩ ESP8266 �������к���ԭ��
  * @author  ������    any question please send mail to const_zpc@163.com
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
  * @brief ��Ӧ���
  */
typedef enum
{
    ESP_ACK_SUCCESS = 0,            /*!< (0)��Ӧ�ɹ� */
    ESP_ACK_FAIL,                   /*!< (1)��Ӧʧ�� */
    ESP_ACK_OVERTIME,               /*!< (2)��Ӧ��ʱ */
    ESP_ACK_BUSY,                   /*!< (3)��Ӧæµ */
} eEspAck;

typedef void (*EspResCallFun)(eEspAck, const char *, uint16_t);
typedef void (*EspRecvCallFun)(uint8_t, const void *, uint16_t);

/**
  * @brief Ӧ�ù���ģʽ
  */
typedef enum
{
    ESP_MODE_STATION = 1,           /*!< (1)Station ģʽ */
    ESP_MODE_AP,                    /*!< (2)AP ģʽ(WIFI�ȵ�) */
    ESP_MODE_AP_STATION,            /*!< (3)AP + Station ����ģʽ */
} eEspCWMode;

/**
  * @brief AP��������
  */
typedef enum
{
    ESP_ENC_WAY_OPEN = 0,           /*!< (0)������ */
    ESP_ENC_WAY_WEP,                /*!< (1)WEP ���ܷ�ʽ */
    ESP_ENC_WAY_WPA_PSK,            /*!< (2)WPA ���ܷ�ʽ */
    ESP_ENC_WAY_WPA2_PSK,           /*!< (3)WPA2 PSK ���ܷ�ʽ */
    ESP_ENC_WAY_WPA_WPA2_PSK        /*!< (4)WPA WPA2 PSK ���ܷ�ʽ */
} eEncryptionWay;

/**
  * @brief ������������
  */
typedef enum
{
    ESP_TYPE_TCP = 0,               /*!< (0)TCP���� */
    ESP_TYPE_UDP                    /*!< (1)UDP���� */
} eEspIpType;

/**
  * @breif WIFI ��Ϣ�ṹ�嶨��
  */
typedef struct
{
    char szSsid[32];                            /*!< WIFI ������ */
    
    char szPassword[64];                        /*!< WIFI ������ */
    
    uint8_t channel;                            /*!< ͨ���� */
    
    eEncryptionWay eEncWay;                     /*!< ���ܷ�ʽ, ȡֵ�� @enum eEncryptionWay */
    
    uint8_t signalIntensity;                    /*!< �ź�ǿ��, 0-4: ��ֵԽ���ź�Խ�� */
} ESP_WifiInfo_t;

/**
  * @breif IP ��Ϣ�ṹ�嶨��
  */
typedef struct
{
    char szIP[20];                              /*!< IP ��ַ, �ַ�����ʽ */
    
    char szMAC[20];                             /*!< MAC �����ַ, �ַ�����ʽ */
    
    uint8_t IP[4];                              /*!< IP ��ַ, ���ʮ���Ƹ�ʽ */
    
    uint8_t MAC[6];                             /*!< MAC �����ַ, ���ʮ���Ƹ�ʽ */
} ESP_IPInfo_t;


/* ���Ĺ��ܺ��� *******************************************************************************************************/
extern void ESP_Init(void);
extern void ESP_OnReciveParseUsartData(const void *bufData, uint16_t len);
extern void ESP_RunTask(void);

/* ״̬��ȡ���� *******************************************************************************************************/
extern uint8_t ESP_GetWifiConnectStatus(void);
extern uint8_t ESP_GetTcpConnectStatus(uint8_t id);

/* �������ܺ��� *******************************************************************************************************/
extern int ESP_QueryVersionInfo(EspResCallFun CallFun);

/* WIFI ���ܺ��� ******************************************************************************************************/
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


/* TCP/IP ���ܺ��� ****************************************************************************************************/
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


/* Airkiss һ���������� ***********************************************************************************************/
extern int ESP_EnterAirkissConfigWifi(EspResCallFun CallFun);
extern int ESP_ExitAirkissConfigWifi(EspResCallFun CallFun);
  
#endif
