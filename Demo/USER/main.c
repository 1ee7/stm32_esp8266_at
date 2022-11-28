
#include "demo_client.h"
#include "esp8266.h"
#include "usart.h"
#include "timer.h"
#include "sys.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int main(void)
{
    FML_USART_Init();
    FML_TIME_Init();

    FML_TIME_Register(FML_USART_MillisecondTimer, 1);
    FML_USART_Register(USART_ESP8266, ESP_OnReciveParseUsartData);

    ESP_Init();
    DemoClientInit();

    while (1)
    {
        if (g_tTimeSign.bTic1msSign) /* 1ms */
        {
            g_tTimeSign.bTic1msSign = FALSE;
        }

        if (g_tTimeSign.bTic10msSign) /* 10ms */
        {
            ESP_RunTask();
            g_tTimeSign.bTic10msSign = FALSE;
        }

        if (g_tTimeSign.bTic20msSign) /* 20ms */
        {
            FML_USART_RecvTask();
            g_tTimeSign.bTic20msSign = FALSE;
        }

        if (g_tTimeSign.bTic100msSign) /* 100ms */
        {
            g_tTimeSign.bTic100msSign = FALSE;
        }

        if (g_tTimeSign.bTic500msSign) /* 500ms */
        {
            g_tTimeSign.bTic500msSign = FALSE;
        }

        if (g_tTimeSign.bTic1secSign) /* 1sec */
        {
            DemoClientTask();
            g_tTimeSign.bTic1secSign = FALSE;
        }
    }
}
