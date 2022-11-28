#include "demo_client.h"
#include "esp8266.h"
#include "usart.h"

#define _USE_irkissConfig       1

#define PLATFORM_HTTP_URL   "cn.utools.club"
#define PLATFORM_TCP_PORT   42874

#define TCP_ID                  0

uint8_t isWifiHostSuccess = 0;
uint16_t sg_sendDataFlag = 0;

static uint8_t sg_isEnterAirkissConfig = 0;

void OnSetWorkModeAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnEnterAirkissConfigWifiAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnQueryWifiInfoListsAck(eEspAck eAck, const char *pszRx, uint16_t lenth);    
void OnQueryLocalIPAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnQueryLocalIPAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnBuildNetConnetResultAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnQueryWifiInfoAck(eEspAck eAck, const char *pszRx, uint16_t lenth);
void OnSetWifiConnectAck(eEspAck eAck, const char *pszRx, uint16_t lenth);

/* 设置工作模式响应处理 */
void OnSetWorkModeAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    DEBUG_PRINTF("OnSetWorkModeAck %d\r\n", eAck);
    
    if (eAck == ESP_ACK_SUCCESS)
    {
        DEBUG_PRINTF("Set Mode Success\r\n");
#if _USE_irkissConfig        
        ESP_ExitAirkissConfigWifi(NULL);        // 不管之前是否进入, 都先退出确保进入成功
        ESP_EnterAirkissConfigWifi(OnEnterAirkissConfigWifiAck);  // 安信可配置        
#else
        ESP_QueryWifiInfoLists(OnQueryWifiInfoListsAck);
#endif
    }
}

void OnEnterAirkissConfigWifiAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    if (eAck == ESP_ACK_SUCCESS)
    {
        DEBUG_PRINTF(" Enter Airkiss Config Wifi SUCCESS\r\n");
        sg_isEnterAirkissConfig = 1;
    }
    else
    {
        DEBUG_PRINTF(" Enter Airkiss Config Wifi Fail\r\n");
    }
}

/* 查询可用WIFI列表响应处理 */
void OnQueryWifiInfoListsAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    int i, num = 0;
    ESP_WifiInfo_t wifiList[15];
    
    if (eAck == ESP_ACK_SUCCESS && (num = ESP_GetWifiInfoLists(wifiList, 15, pszRx, lenth)) > 0)
    {
        for (i = 0; i < num; i++)
        {
            DEBUG_PRINTF("%02d.SSID[%d][%d]: %s\r\n", i + 1, 
                            wifiList[i].eEncWay, wifiList[i].signalIntensity, wifiList[i].szSsid);
        }
        
        ESP_SetWifiConnect("test", "88888888", OnSetWifiConnectAck);
    }
}

void OnQueryLocalIPAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    ESP_IPInfo_t wifiInfo;
    ESP_IPInfo_t serverInfo;
    
    if (eAck == ESP_ACK_SUCCESS && ESP_GetLocalIP(&wifiInfo, &serverInfo, pszRx, lenth) == 0)
    {
        DEBUG_PRINTF("Query Local IP Success\r\n");
        
        DEBUG_PRINTF("WIFI IP: %s [%d.%d.%d.%d]\r\n", wifiInfo.szIP, wifiInfo.IP[0], wifiInfo.IP[1], wifiInfo.IP[2], wifiInfo.IP[3]);
        DEBUG_PRINTF("WIFI MAC: %s [%02x:%02x:%02x:%02x:%02x:%02x]\r\n", wifiInfo.szMAC, wifiInfo.MAC[0],
                        wifiInfo.MAC[1], wifiInfo.MAC[2], wifiInfo.MAC[3], wifiInfo.MAC[4], wifiInfo.MAC[5]);
        
        DEBUG_PRINTF("SERVER IP: %s [%d.%d.%d.%d]\r\n", serverInfo.szIP, serverInfo.IP[0], serverInfo.IP[1], serverInfo.IP[2], serverInfo.IP[3]);
        DEBUG_PRINTF("SERVER MAC: %s [%02x:%02x:%02x:%02x:%02x:%02x]\r\n", serverInfo.szMAC, serverInfo.MAC[0],
                        serverInfo.MAC[1], serverInfo.MAC[2], serverInfo.MAC[3], serverInfo.MAC[4], serverInfo.MAC[5]);
    }
}

void OnBuildNetConnetResultAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    if (eAck == ESP_ACK_SUCCESS)
    {
        DEBUG_PRINTF("Build Net Connet Success\r\n");
        isWifiHostSuccess = 1;
        sg_sendDataFlag = 1;
    }
}

/* 查询当前连接的 WIFI 响应 */
void OnQueryWifiInfoAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    ESP_WifiInfo_t wifiList;
    
    if (eAck == ESP_ACK_SUCCESS && ESP_GetWifiInfo(&wifiList, pszRx, lenth) == 0)
    {
        DEBUG_PRINTF("SSID[%d][%d]: %s\r\n", 
                        wifiList.channel, wifiList.signalIntensity, wifiList.szSsid);

        
    }
    
    ESP_BuildNetConnect(TCP_ID, ESP_TYPE_TCP, PLATFORM_HTTP_URL, PLATFORM_TCP_PORT, OnBuildNetConnetResultAck);
}

/* 设置连接WIFI响应处理 */
void OnSetWifiConnectAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    if (eAck == ESP_ACK_SUCCESS)
    {
        DEBUG_PRINTF("Set WIFI Connect Param Success\r\n");
    }
}


/* 发送TCP数据响应处理 */
void OnSendNetDataAck(eEspAck eAck, const char *pszRx, uint16_t lenth)
{
    if (eAck == ESP_ACK_SUCCESS)
    {
        DEBUG_PRINTF("TCP Send Data Success\r\n");
        sg_sendDataFlag = 1;
    }
}

/* 接收TCP数据处理 */
void OnRecvNetData(uint8_t id, const void *pszRx, uint16_t lenth)
{
    DEBUG_PRINTF("TCP Recv Data Success[%d] %s\r\n", id, (char *)pszRx);
    sg_sendDataFlag = 1;
}

void DemoClientInit(void)
{
    ESP_SetWorkMode(ESP_MODE_STATION, OnSetWorkModeAck);
    ESP_SetNetDataRecv(TCP_ID, OnRecvNetData);
}

void DemoClientTask(void)
{
    

#if _USE_irkissConfig  
    if (ESP_GetWifiConnectStatus() && sg_isEnterAirkissConfig)
    {
        sg_isEnterAirkissConfig = 0;
        ESP_ExitAirkissConfigWifi(NULL);
    }
#else 
    static uint8_t s_isQueryWifiInfo = 1;
    
    if (ESP_GetWifiConnectStatus() && s_isQueryWifiInfo)
    {
        s_isQueryWifiInfo = 0;
        ESP_QueryWifiInfo(OnQueryWifiInfoAck);
    }
#endif    
    if (ESP_GetTcpConnectStatus(TCP_ID) && sg_sendDataFlag == 1)
    {
        sg_sendDataFlag = 0;
        ESP_SendNetData(TCP_ID, "TEST\r\r", 6, OnSendNetDataAck);
    }
}

