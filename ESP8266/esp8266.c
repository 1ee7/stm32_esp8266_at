/**
  **********************************************************************************************************************
  * @file    esp8266.c
  * @brief   ���ļ��ṩ ESP8266 ������ع���
  * @author  ������    any question please send mail to const_zpc@163.com
  * @version V1.0.0
  * @date    2021-3-13
  *
  * @details  ������ϸ˵����
  *           + 
  *
  **********************************************************************************************************************
  *
  * ʹ�÷�ʽ:
  *    1��ʹ��ǰ��ʼ������ ESP_Init, ֮��ʹ�� ESP_SetCWMode ���ù���ģʽ
  *    2�����ڵ��ú��� ESP_RunTask, ������ ESP8266 ģ��ͨ��
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

/* AT �����б�֧����Ŀ */
#define ESP_CMD_TASK_LIST_NUM   30

/* Private typedef ---------------------------------------------------------------------------------------------------*/
 
/**
  * @brief AT ָ��ö��
  * @note  ��Чָ��ö�ٺ� AT ָ��� sg_ktCmdTable ������Ӧ
  */
typedef enum
{
    ESP_AT_CMD_INVALID = -1,            /*!< ��Чָ�� */
    
    /* ����ָ�������ָ�� */
    ESP_AT_CMD_AT_TEST = 0,             /*!< ���� AT ���� */
    ESP_AT_CMD_RST,                     /*!< ����ģ�� */
    ESP_AT_CMD_SEE_VERSION,             /*!< �鿴�汾��Ϣ */
    ESP_AT_CMD_ATE,                     /*!< ���Կ���/�ر� */
    /* ����ָ���wifi ����ָ�� */
    ESP_AT_CMD_SET_MODE,                /*!< ѡ�� WIFI Ӧ��ģʽ */
    ESP_AT_CMD_JOIN_AP,                 /*!< ���� AP */
    ESP_AT_CMD_LISTS_AP,                /*!< �г���ǰ���� AP */
    ESP_AT_CMD_QUIT_AP,                 /*!< �˳��� AP ������ */
    ESP_AT_CMD_SET_AP_PARAM,            /*!< ���� AP ģʽ�µĲ��� */
    ESP_AT_CMD_LISTS_JOINED_IP,         /*!< �鿴�ѽ����豸�� IP */
    /* ����ָ���TCP/IP ָ�� */
    ESP_AT_CMD_GET_CONNECT_STATE,       /*!< �������״̬ */
    ESP_AT_CMD_BUILD_TCP_OR_UDP,        /*!< ���� TCP ���ӻ�ע�� UDP �˿ں� */
    ESP_AT_CMD_SEND_DATA,               /*!< ��������ָ�� */
    ESP_AT_TRANSFER_DATA,               /*!< ͸������ */
    ESP_AT_CMD_CLOSE_TCP_OR_UDP,        /*!< �ر� TCP �� UDP */
    ESP_AT_CMD_GET_LOCAL_IP,            /*!< ��ȡ���� IP ��ַ */
    ESP_AT_CMD_START_MULTI_CONNECT,     /*!< ���������� */
    ESP_AT_CMD_CONFIG_SERVICE,          /*!< ����Ϊ������ */
    ESP_AT_CMD_SET_TRANSFER_MODE,       /*!< ����ģ�鴫��ģʽ */
    ESP_AT_CMD_SET_SERVICE_OVERTIME,    /*!< ���÷�������ʱʱ�� */
    /* ���ſ�ָ�������ָ�� */
    ESP_AT_CMD_ENABLE_AUTO_LINK_WIFI,   /*!< ʹ���ϵ��Զ�����AP */
    ESP_AT_CMD_ENTER_SMART_CONFIG,      /*!< ���� ESP-Touch �� Airkiss �������� */
    ESP_AT_CMD_EXIT_SMART_CONFIG,       /*!< �˳� ESP-Touch �� Airkiss �������� */
} eATCmd;

/**
  * @breif ATָ�����Ӧ��Ϣ�ṹ�嶨��
  */
typedef struct
{
    char*    pszATCmd;                              /*!< AT ָ��ָ�� */
    
    char*    pszATAck;                              /*!< AT ָ���Ԥ�ڽ�� */
    
    uint8_t resendMaxCnt;                           /*!< �ط�������, Ϊ 0 ��ʾ����Ҫ�ط� */
    
    uint8_t sendCnt;                                /*!< �ط�����, ��ʼΪ 0 */
    
    uint8_t isQuery;                                /*!< �Ƿ��ǲ�ѯ */
    
    uint32_t maxWaitTime;                           /*!< ��Ӧ�ȴ�ʱ�� */
    
    void (*pfnPackCmd)(void *);                     /*!< ���´�� AT ָ���, Ϊ NULL ����Ĭ�Ϸ�ʽ���� */
    
    void (*pfnUnpackAck)(const char *, uint16_t);   /*!< ���½��� AT ָ����Ӧ������, Ϊ NULL ����Ĭ�Ϸ�ʽ���� */
    
    void (*pfnOverTimeHandle)(void);                /*!< AT ָ����Ӧ��ʱ������, Ϊ NULL �򲻴��� */
}ATCmdInfo_t;

/**
  * @breif ATָ�������б�ṹ�嶨��
  */
typedef struct
{
    uint8_t  state;                                 /*!< ����״̬ */
    
    uint8_t  end;                                   /*!< ѭ������β�ڱ� */
    
    uint8_t  head;                                  /*!< ѭ���������ڱ� */
    
    uint8_t  num;                                   /*!< ѭ���������ܴ洢��������� */
    
    uint8_t  isFirst;                               /*!< ѭ���������ڱ��״ζ�ȡ */
    
    eATCmd  aCmdList[ESP_CMD_TASK_LIST_NUM];        /*!< �����б� */
    
    EspResCallFun pfnResCallFun[ESP_CMD_TASK_LIST_NUM];     /*!< �����б����ص����� */
}ATCmdTask_t;

/**
  * @breif TCP ״̬��Ϣ�ṹ�嶨��
  */
typedef struct
{
    uint8_t isConnect;                              /*!< ����״̬ */
    
    eEspIpType eIPConnectType;                      /*!< IP �������� */
    
    char szNetAddr[64];                             /*!< TCP/UDP �����ַ/���� */
    
    uint32_t netPort;                               /*!< TCP/UDP ����ͨ�Ŷ˿ں� */

    EspRecvCallFun pfnRecvCallFun;                  /*!< ���ݽ��ջص������� */
}NetManager_t;

/**
  * @breif ��������Ϣ�ṹ�嶨��
  */
typedef struct
{
    uint8_t isEnable;                         /*!< �Ƿ�ʹ�� server ģʽ */
    
    uint16_t overtime;                        /*!< ��������ʱʱ��, ��ΧΪ0~28800, ��λΪ s */
    
    uint32_t port;                            /*!< �����Ķ˿ں� */
}ServerInfo_t;

/**
  * @brief  ESP8266 ģ��ATָ�����ṹ����Ϣ����.
  */
typedef struct tag_EspHandle
{
    ATCmdInfo_t  tATCmdInfo;                        /*!< ��ǰʹ���е� AT ָ����Ϣ */
    
    char szAtCmdBuf[64];                            /*!< AT ָ�����ݻ��� */

    char sendBufData[200];                         /*!< ����TCP���ݵĻ��� */
    
    uint16_t sendBufDataLenth;                      /*!< ����TCP���ݵĳ��� */
    
    eEspCWMode workMode;                            /*!< ����Ӧ��ģʽ */
    
    uint8_t currNetTcpId;                           /*!< ��ǰʹ���е� id */

    uint8_t isWifiConnected;                        /*!< WIFI �Ƿ������� */
    
    uint8_t isATAckWait;                            /*!< AT ָ���Ƿ����ڵȴ���Ӧ */
    
    uint8_t isSeriaNet;                             /*!< �Ƿ���͸��ģʽ */
    
    uint32_t ATAckWaitTic;                          /*!< AT ָ��ȴ���ʱ */
    
    uint8_t isMultiConnect;                         /*!< �Ƿ��ǵ�·����ģʽ */
    
    uint8_t isConfigSmartState;                     /*!< �Ƿ��� SMART CONFIG ģʽ */
    
    uint8_t isMultiConnectBak;                      /*!< �Ƿ��ǵ�·����ģʽ���� */
    
    ATCmdTask_t tATCmdTask;                         /*!< ATָ�������б� */

    ESP_WifiInfo_t tWifiInfo;                       /*!< ��Ҫ���ӵ� WIFI ��Ϣ */
    
    ESP_WifiInfo_t tWifiHostInfo;                   /*!< AP ģʽ��(�ȵ�)�� WIFI ��Ϣ */
    
    ServerInfo_t tServerInfo;                       /*!< ��������Ϣ */
    
    NetManager_t  arrtNetManager[5];                /*!< ����ͨ�ŵ�����״̬������, ������ģʽ��ʹ��0 */
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
  * @brief   ָ�����
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
    /* ���ſ�ָ�������ָ�� */
    {"AT+CWAUTOCONN=1",   "OK",         0, 0, 0, 1000,  NULL, NULL,  NULL},
    {"AT+CWSTARTSMART=3", "OK",         0, 0, 0, 1000,  NULL, OnUnpackEnterConfigSmartAck,  NULL},
    {"AT+CWSTOPSMART",   "OK",          0, 0, 0, 1000,  NULL, OnUnpackExitConfigSmartAck,  NULL},
};

/**
  * @brief      ��ʼ��ATָ������
  * @param[in]  cmd - AT ָ������
  * @param[in][out] info - ATCmdTask_t ��������б�
  * @retval     ���ص�ֵ��������
  *             @arg -1: д��ʧ��
  *             @arg 0: д��ɹ�
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
  * @brief      ���һ��ATָ������
  * @param[in]  cmd - AT ָ������
  * @param[in][out] info - ATCmdTask_t ��������б�
  * @param[in]  pfnCallFun - ָ���������ص�����
  * @retval     ���ص�ֵ��������
  *             @arg -1: д��ʧ��
  *             @arg 0: д��ɹ�
  */
static int AddCmdTaskToList(ATCmdTask_t* info, eATCmd cmd, EspResCallFun pfnCallFun)
{
    if (0x00 == ((info->state) & 0x02))                      /* �����򻺳���д������ */
    {
        info->aCmdList[info->end] = cmd;
        info->pfnResCallFun[info->end] = pfnCallFun;
        (info->end)++;

        if ((info->end) >= (info->num))
        {
            (info->end) = 0;
        }

        if (((info->state) & 0x01) != 0x00)                 /* �������������ݴ����ͻ��� */
        {
            (info->state) &= ~0x01;                         /* �������������ݴ����ͻ��� */
        }
        else if (((info->state) & 0x04) != 0x00)            /* ���������� */
        {
            (info->head) = (info->end);                     /* ��β�ڱ��ƶ����ڱ� */
        }
        else if ((info->end) == (info->head))
        {
            if (info->state & 0x80)                         /* �Ƿ����������� */
            {
                (info->state) |= 0x02;                      /* ��ֹ�򻺳���д������ */
            }
            else
            {
                (info->state) |= 0x04;                      /* ���������� */
            }
        }

        return 0;
    }

    return -1;
}

///**
//  * @brief      ��ȡ��ǰִ��ATָ������
//  * @param[in][out] info - ATCmdTask_t ��������б�
//  * @retval     -1: û��ATָ������; ���� ID
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
  * @brief      ��ȡ��ǰִ��ATָ�����Ļص���������
  * @param[in][out] info - ATCmdTask_t ��������б�
  * @retval     -1: û��ATָ������; ���� ID
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
  * @brief      ��ȡ��ǰִ��ATָ����Ϣ
  * @param[out] peCmd - ATָ��
  * @param[out] info - ATCmdTask_t ��������б�
  * @retval     -1: û��ATָ������;
  *              0: ���µ�ָ��
  *              1: �µ�ָ��
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
  * @brief      ɾ����ǰ��ATָ������
  * @param[in][out] info - ATCmdTask_t ��������б�
  * @retval     0
  */
static int DeletCurrCmdTaskFromList(ATCmdTask_t* info)
{
    if (((info->state) & 0x01) == 0x00)
    {
        (info->head) ++;                            /* ͷ�ڱ� */
        info->isFirst = 0;

        if ((info->head) >= (info->num))
        {
            info->head = 0;
        }

        if ((info->head) == (info->end))            /* ͷβ�ڱ��غ� */
        {
            if (((info->state) & 0x02) == 0x00)     /* �����򻺳����д������� */
            {
                (info->state) |= 0x01;              /* �������������ݵȴ�������� */
            }
        }
    }
    
    (info->state) &= ~0x02;                         /* �����򻺳����д������� */
    (info->state) &= ~0x04;                         /* ������δ�� */
    
    return 0;
}

/**
  * @brief      ���������б���ָ��.
  * @param[in]  pHandle ���
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
  * @brief      ������������.
  * @param[in]  kpszBufData ���յ�����
  * @param[in]  len ���յ����ݳ���
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
  * @brief      ������������.
  * @param[in]  kpszBufData ���յ�����
  * @param[in]  len ���յ����ݳ���
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
        if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "+IPD,")) != 0)// ��������
        {
            kpszBufDataPtr += 5;  // ����"+IPD,"
            
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
  * @brief      �ж�WIFI����״̬����.
  * @param[in]  kpszBufData ���յ�����
  * @param[in]  len ���յ����ݳ���
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
  * @brief      �ж�TCP����״̬����.
  * @param[in]  kpszBufData ���յ�����
  * @param[in]  len ���յ����ݳ���
  * @retval     None
  */
static void JudgeNetConnectionStatus(const void *kpszBufData, uint16_t len)
{
    uint8_t id;
    const char *kpszBufDataPtr = kpszBufData;
    
    if ((kpszBufDataPtr = strstr(kpszBufDataPtr, "CONNECT\r\n")) != 0 &&
        *(kpszBufDataPtr - 1) != ' ' && *(kpszBufDataPtr - 1) != 'S')  // �ų�"WIFI CONNECT" �� "WIFI DISCONNECT"
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
  * @brief      ���ղ������������ݺ���.
  * @param[in]  kpszBufData ���յ�����
  * @param[in]  len ���յ����ݳ���
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
  * @brief      ������Ӧ��ʱ����.
  * @param[in]  pHandle ���
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
  * @brief      Ӳ���ײ��ʼ��.
  * @retval     None.
  */
static void EspDriveInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    /* ʹ�ܶ˿ڸ���ʱ�� GPIOA GPIOB ʱ��*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

/**
  * @brief   ESP8266��ʼ��
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
  * @brief      ���� TCP/UDP ����ͨ�ŵ����ݽ��ջص�����.
  * @param[in]  id  ���ӵ� id ��, ��Χ 0-4 
  * @param[in]  CallFun  ���ݽ��ջص�����
  * @retval     ���ص�ֵ��������
  *             @arg -1: ʧ��
  *             @arg  0: �ɹ�
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
  * @brief      ����WIFIӦ��ģʽ��ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ������������ WIFI ��ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ����APģʽ�²�����ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ���� TCP ���ӻ�ע�� UDP �˿ںŵ�ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      TCP �� UDP �������ݵ�ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      TCP �� UDP ͸�����ݵķ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
  * @retval     None
  */
static void OnPackTCPOrUDPData(void *pBody)
{
    EspHandle_t *pHandle = (EspHandle_t *)pBody;
    
    pHandle->tATCmdInfo.pszATCmd = pHandle->sendBufData;
}

/**
  * @brief      �ر� TCP �� UDP ��ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ���������ӵ�ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ����Ϊ��������ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ѡ����ģʽ��ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ���÷�������ʱʱ���ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ����ͨ����Ӧ����.
  * @param[in]  bufData ���յ�����
  * @param[in]  len ���յ����ݳ���
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
  * @brief      ����ATָ����Կ��ص�ATָ��ͺ���.
  * @param[in]  pBody  EspHandle_t���
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
  * @brief      ת���ź�ֵ
  * @param[in]  signalValue  ԭʼ�ź�ֵ
  * @retval     ת�����ź�ֵ, ֵԽ���ź�Խ��
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
  * @brief      �����ȡ����״̬�Ͳ�������Ӧ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
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
    src Դ�ַ������׵�ַ(buf�ĵ�ַ)
    separator ָ���ķָ��ַ�
    dest �������ַ���������
    num �ָ�����ַ����ĸ���
*/
void split(const char *src,const char *separator,char **dest, uint8_t *num) {
     char *pNext;
     int count = 0;
     if (src == NULL || strlen(src) == 0) //�������ĵ�ַΪ�ջ򳤶�Ϊ0��ֱ����ֹ
        return;
     if (separator == NULL || strlen(separator) == 0) //��δָ���ָ���ַ�����ֱ����ֹ
        return;
     pNext = (char *)strtok((char *)src, (char *)separator); //����ʹ��(char *)����ǿ������ת��(��Ȼ��д�еı������в������ָ�����)
     while(pNext != NULL) {
          *dest++ = pNext;
          ++count;
         pNext = (char *)strtok(NULL,separator);  //����ʹ��(char *)����ǿ������ת��
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
  * @brief      ��ȡ WIFI ����״̬. 
  * @note       ʵʱ��ظ���״̬
  * @retval     ���ص�ֵ��������
  *                 @arg  1: ������
  *                 @arg  0: δ����
  */
uint8_t ESP_GetWifiConnectStatus(void)
{
    return sg_tEspHandle.isWifiConnected;
}

/**
  * @brief      ��ȡָ�� ID �ŵ� TCP/UDP ����״̬.
  * @note       ʵʱ��ظ���״̬, Ҳ����ͨ������ ESP_QueryNetConnectStatus ��ѯ����
  * @param[in]  id          ���ӵ� id ��, ��Χ 0-4 
  * @retval     ���ص�ֵ��������
  *                 @arg  1: ������
  *                 @arg  0: δ����
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
  * @brief      ��ѯ ESP8266 �İ汾��Ϣ.
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_GetVersionInfo �������ݵõ��汾��Ϣ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_QueryVersionInfo(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_SEE_VERSION].isQuery = 1;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SEE_VERSION, CallFun);
    
    return ret;
}

/**
  * @brief      ���� ESP8266 �Ĺ���ģʽ.
  * @param[in]  mode  ����ģʽ, �μ�ö�� @enum eEspCWMode
  * @param[in]  CallFun  ���ú�Ļص�����
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
  * @brief      ��ѯ ESP8266 �Ĺ���ģʽ.
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_GetWorkMode �������ݵõ���ǰ����ģʽ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_QueryWorkMode(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_SET_MODE].isQuery = 1;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_SET_MODE, CallFun);
    
    return ret;
}

/**
  * @brief      ������ǰ����ģʽ
  * @param[out] pMode   ����ģʽ, �μ�ö�� @enum eEspCWMode
  * @param[in]  pszRx   ����
  * @param[in]  lenth   ���ݳ���
  * @retval     -1, ����ʧ��; ����, ���õ� WIFI �б���Ŀ
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
  * @brief      ��ѯ��ǰ���õ� WIFI �б���Ϣ.
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_AnalysisWifiInfoLists �������ݵõ� Wifi �б���Ϣ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_QueryWifiInfoLists(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_LISTS_AP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_LISTS_AP, CallFun);
    
    return ret;
}

/**
  * @brief      �������ӵ� WIFI 
  * @param[in]  pkszName    WIFI����
  * @param[in]  pkszPassword WIFI����
  * @param[in]  CallFun  ���ú�Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_SetWifiConnect(const char *pkszName, const char *pkszPassword, EspResCallFun CallFun)
{
    sg_ktCmdTable[ESP_AT_CMD_JOIN_AP].isQuery = 0;
    snprintf(sg_tEspHandle.tWifiInfo.szSsid, sizeof(sg_tEspHandle.tWifiInfo.szSsid), "%s", pkszName);
    snprintf(sg_tEspHandle.tWifiInfo.szPassword, sizeof(sg_tEspHandle.tWifiInfo.szPassword), "%s", pkszPassword);
    
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_JOIN_AP, CallFun);
}

/**
  * @brief      �Ͽ����ӵ� WIFI 
  * @param[in]  CallFun  ���ú�Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_SetWifiDisconnect(EspResCallFun CallFun)
{
    sg_ktCmdTable[ESP_AT_CMD_QUIT_AP].isQuery = 0;
    
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_QUIT_AP, CallFun);
}

/**
  * @brief      ���� AP ģʽ(WIFI�ȵ�)�µĲ���
  * @param[in]  pkszName    WIFI�ȵ�����
  * @param[in]  pkszPassword WIFI�ȵ�����, ���ȴ��ڵ���8
  * @param[in]  ch          ͨ����
  * @param[in]  encWay ���ܷ�ʽ, ȡֵ�ο� @enum eEncryptionWay
  * @param[in]  CallFun  ���ú�Ļص�����
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
  * @brief      ������ǰ���õ� WIFI �б���Ϣ.
  * @param[out] pInfo   ������� WIFI ���б���Ϣ
  * @param[in]  num     �б�֧�ִ�С
  * @param[in]  pszRx   ����
  * @param[in]  lenth   ���ݳ���
  * @retval     -1, ����ʧ��; ����, ���õ� WIFI �б���Ŀ
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
  * @brief      ��ѯ��ǰ WIFI ����Ϣ.
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_AnalysisWifiInfo �������ݵõ� Wifi ��Ϣ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_QueryWifiInfo(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_JOIN_AP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_JOIN_AP, CallFun);

    return ret;
}

/**
  * @brief      ��������ǰ WIFI ����Ϣ.
  * @note       �޼���������Ϣ
  * @param[out] pInfo   ������� WIFI ����Ϣ
  * @param[in]  pszRx   ����
  * @param[in]  lenth   ���ݳ���
  * @retval     -1, ����ʧ��; 0, �����ɹ�
  */
int ESP_GetWifiInfo(ESP_WifiInfo_t *pInfo, const char *pszRx, uint16_t lenth)
{
    uint8_t num = 0;
    const char *kBufData = pszRx;
    char *pSplitPtr[8] = {0}; //��ŷָ������ַ���ָ�� 
    
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
  * @brief      WIFI �ȵ�ģʽ�²鿴�ѽ�����豸 IP
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_GetCurrentConnectIP �������ݵõ��ѽ����豸��IP��Ϣ
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     0, �������ɹ�; <0, �������ʧ��
  */
int ESP_QueryCurrentConnectIP(EspResCallFun CallFun)
{
    int ret = 0;
    
    sg_ktCmdTable[ESP_AT_CMD_LISTS_JOINED_IP].isQuery = 1;
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_LISTS_JOINED_IP, CallFun);

    return ret;
}

/**
  * @brief      ������ȡ�ѽ����豸��IP
  * @param[out] pInfo   ������� WIFI ����Ϣ
  * @param[in]  pszRx   ����
  * @param[in]  lenth   ���ݳ���
  * @retval     -1, ����ʧ��; 0, �����ɹ�
  */
int ESP_GetCurrentConnectIP(ESP_IPInfo_t *pIPListInfo, const char *pszRx, uint16_t lenth)
{
    uint8_t num = 0;
    const char *kBufData = pszRx;
    char *pSplitPtr[8] = {0}; //��ŷָ������ַ���ָ�� 
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
  * @brief      ���TCP/UDP����״̬.
  * @note       �ص���������Ӧ�ɹ���ͨ������ ESP_GetTcpConnectStatus �õ�TCP/UDP����״̬
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     ���ص�ֵ��������
  *                 @arg -1: ʧ��
  *                 @arg  0: �ɹ�
  */
int ESP_QueryNetConnectStatus(EspResCallFun CallFun)
{
        sg_ktCmdTable[ESP_AT_CMD_GET_CONNECT_STATE].isQuery = 0;
        return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_GET_CONNECT_STATE, CallFun);
}

/**
  * @brief      ���� TCP ���ӻ�ע�� UDP �˿ں�.
  * @param[in]  id          ���ӵ� id ��, ��Χ 0-4 
  * @param[in]  type        ����������, �μ�ö�� @enum eEspIpType
  * @param[in]  pkszAddr    �ַ���������Զ�̷����� IP ��ַ
  * @param[in]  port        Զ�̷������˿ں�
  * @param[in]  CallFun     ���ú�Ļص�����
  * @retval     ���ص�ֵ��������
  *                 @arg -1: ʧ��
  *                 @arg  0: �ɹ�
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
  * @brief      ���� TCP �Ͽ���ע�� UDP �˿ں�.
  * @param[in]  id          ���ӵ� id ��, ��Χ 0-4 
  * @param[in]  CallFun     ���ú�Ļص�����
  * @retval     ���ص�ֵ��������
  *                 @arg -1: ʧ��
  *                 @arg  0: �ɹ�
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
  * @brief      ��ȡ���� IP ��ַ.
  * @param[in]  CallFun  ��ѯ��Ļص�����
  * @retval     ���ص�ֵ��������
  *                 @arg -1: ʧ��
  *                 @arg  0: �ɹ�
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
  * @brief      ������ȡ���ص�IP
  * @param[out] pWifiIP   ������� WIFI IP ����Ϣ
  * @param[out] pWifiIP   ������ķ����� IP ��Ϣ
  * @param[in]  pszRx     ���յ�����
  * @param[in]  lenth     ���յ����ݳ���
  * @retval     -1, ����ʧ��; 0, �����ɹ�
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
  * @brief      ����Ϊ��·����ģʽ
  * @note       ֻ�е����Ӷ��Ͽ�����ܸ��ģ���������� server ��Ҫ����ģ��
  * @param[in]  isEnable ��·����ģʽ����/�ر�
  *               @arg 0 ��ֹ, ��·����ģʽ  
  *               @arg 1 ʹ��, ��·����ģʽ
  * @param[in]  CallFun  ���ú�Ļص�����
  * @return     None
  */
int ESP_SetMultiConnect(uint8_t isEnable, EspResCallFun CallFun)
{
    sg_tEspHandle.isMultiConnectBak = sg_tEspHandle.isMultiConnect;
    sg_tEspHandle.isMultiConnect = isEnable;
    return AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_START_MULTI_CONNECT, CallFun);
}

/**
  * @brief      ���÷���������
  *             ���� server ģʽǰ��Ҫ���ö�·����ģʽ, ���ȵ��ú��� ESP_SetMultiConnect ʹ��
  * @param[in]  isEnable server ģʽ����/�ر�
  *               @arg 0 �ر� server ģʽ  
  *               @arg 1 ���� server ģʽ
  * @param[in]  port �˿ں�
  * @param[in]  overtime ��������ʱʱ��, ��Χ0~28800����λΪ s
  * @param[in]  CallFun  ���ú�Ļص�����
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
  * @brief      ����Ϊ͸��ģʽ
  * @note       ����͸��ģʽǰ��Ҫ���õ�·����ģʽ, ���ȵ��ú��� ESP_SetMultiConnect ��ֹ
  * @param[in]  isEnable ͸��ģʽ����/�ر�
  *               @arg 0 ��ֹ, ��͸��ģʽ
  *               @arg 1 ʹ��, ͸��ģʽ
  * @param[in]  CallFun  ���ú�Ļص�����
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
  * @brief      ESP8266 ģ��������.
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
  * @brief      ���� Airkiss һ����������.
  * @note       ��������Ƿ����óɹ�������Ҫ�˳�
  * @param[in]  CallFun  ���ú�Ļص�����
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
  * @brief      �˳� Airkiss һ����������.
  * @param[in]  CallFun  ���ú�Ļص�����
  * @retval     None
  */
int ESP_ExitAirkissConfigWifi(EspResCallFun CallFun)
{
    int ret = 0;
    
    ret += AddCmdTaskToList(&sg_tEspHandle.tATCmdTask, ESP_AT_CMD_EXIT_SMART_CONFIG, CallFun);
    
    return ret;
}
