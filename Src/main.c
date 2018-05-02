
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

#include "dhserver.h"
#include "dnserver.h"
#include <stdlib.h>
#include <stdio.h>
#include "usbd_cdc.h"
#include "netif/etharp.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/icmp.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/tcp_impl.h"
#include "lwip/tcp.h"
//#include "time.h"
#include "httpd.h"

RTC_HandleTypeDef hrtc;

int64_t usAddition = 0;

static uint8_t hwaddr[6]  = {0x20,0x89,0x84,0x6A,0x96,00};
static uint8_t ipaddr[4]  = {192, 168, 7, 1};
static uint8_t netmask[4] = {255, 255, 255, 0};
static uint8_t gateway[4] = {0, 0, 0, 0};


#define NUM_DHCP_ENTRY 3

static dhcp_entry_t entries[NUM_DHCP_ENTRY] =
{
    /* mac    ip address        subnet mask        lease time */
    { {0}, {192, 168, 7, 2}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 7, 3}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 7, 4}, {255, 255, 255, 0}, 24 * 60 * 60 }
};

static dhcp_config_t dhcp_config =
{
    {192, 168, 7, 1}, 67, /* server address, port */
    {192, 168, 7, 1},     /* dns server */
    "stm",                /* dns suffix */
    NUM_DHCP_ENTRY,       /* num entry */
    entries               /* entries */
};

struct netif netif_data;

typedef struct stmr stmr_t;

typedef void (*stmr_cb_t)(stmr_t *tmr);

#define STMR_ACTIVE 1

struct stmr
{
	uint32_t  period; /* timer period, us. */
	uint32_t  event;  /* the last event, us */
	uint32_t  flags;  /* STMR_XXX */
	void     *data;   /* user data */
	stmr_cb_t proc;   /* timer proc */
	stmr_t   *next;   /* don't touch it */
};

static stmr_t *stmrs = NULL;

int64_t mtime(void);
int64_t utime(void);
void    usleep(int us);                  /* sleep to n us */
#define msleep(ms) usleep((ms) * 1000)   /* sleep to n ms */
void rtctime(int *h, int *m, int *s);
void stmr(void);             /* call it periodically */
void stmr_init(stmr_t *tmr); /* init timer and adds to list */
void stmr_add(stmr_t *tmr);  /* adds timer to a timers list */
void stmr_free(stmr_t *tmr); /* remove timer from the list */
void stmr_stop(stmr_t *tmr); /* deactivate timer */
void stmr_run(stmr_t *tmr);  /* activate timer */

#define TIMER_PROC(name, period, active, data) \
void name##_proc(stmr_t *tmr); \
static stmr_t name = \
{ \
	period, 0, active, data, \
	name##_proc, NULL \
}; \
void name##_proc(stmr_t *tmr)


uint32_t sys_now()
{
    return (uint32_t)mtime();
}

TIMER_PROC(link_led_off, 50 * 1000, 1, NULL)
{
    //STM_EVAL_LEDOff(LINK_LED);
}

TIMER_PROC(tcp_timer, TCP_TMR_INTERVAL * 1000, 1, NULL)
{
    tcp_tmr();
}

static uint8_t received[RNDIS_MTU + 14];
static int recvSize = 0;

void on_packet(const char *data, int size)
{
    memcpy(received, data, size);
    recvSize = size;
}

void usb_polling()
{
    struct pbuf *frame;
    __disable_irq();
    if (recvSize == 0)
    {
        __enable_irq();
        return;
    }
    frame = pbuf_alloc(PBUF_RAW, recvSize, PBUF_POOL);
    if (frame == NULL)
    {
        __enable_irq();
        return;
    }
    memcpy(frame->payload, received, recvSize);
    frame->len = recvSize;
    recvSize = 0;
    __enable_irq();
    ethernet_input(frame, &netif_data);
    pbuf_free(frame);
}

static int outputs = 0;

err_t output_fn(struct netif *netif, struct pbuf *p, ip_addr_t *ipaddr)
{
    return etharp_output(netif, p, ipaddr);
}

err_t linkoutput_fn(struct netif *netif, struct pbuf *p)
{
    int i;
    struct pbuf *q;
    static char data[RNDIS_MTU + 14 + 4];
    int size = 0;
    for (i = 0; i < 200; i++)
    {
        if (rndis_can_send()) break;
        msleep(1);
    }
    for(q = p; q != NULL; q = q->next)
    {
        if (size + q->len > RNDIS_MTU + 14)
            return ERR_ARG;
        memcpy(data + size, (char *)q->payload, q->len);
        size += q->len;
    }
    if (!rndis_can_send())
        return ERR_USE;
    rndis_send(data, size);
    outputs++;
    return ERR_OK;
}


err_t netif_init_cb(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));
    netif->mtu = RNDIS_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
    netif->state = NULL;
    netif->name[0] = 'E';
    netif->name[1] = 'X';
    netif->linkoutput = linkoutput_fn;
    netif->output = output_fn;
    return ERR_OK;
}

#define PADDR(ptr) ((ip_addr_t *)ptr)


void init_lwip()
{
    struct netif  *netif = &netif_data;

    lwip_init();
    netif->hwaddr_len = 6;
    memcpy(netif->hwaddr, hwaddr, 6);

    netif = netif_add(netif, PADDR(ipaddr), PADDR(netmask), PADDR(gateway), NULL, netif_init_cb, ip_input);
    netif_set_default(netif);

    stmr_add(&tcp_timer);
}

bool dns_query_proc(const char *name, ip_addr_t *addr)
{
    if (strcmp(name, "run.stm") == 0 || strcmp(name, "www.run.stm") == 0)
    {
        addr->addr = *(uint32_t *)ipaddr;
        return true;
    }
    return false;
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();

  rndis_rxproc = on_packet;

  init_lwip();

	while (!netif_is_up(&netif_data)) ;

	while (dhserv_init(&dhcp_config) != ERR_OK) ;

	while (dnserv_init(PADDR(ipaddr), 53, dns_query_proc) != ERR_OK) ;

    //httpd_init();

    while (1)
    {
  	  usb_polling();     /* usb device polling */
  	  stmr();            /* call software timers */
    }
    while (1){

    }
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  usAddition += 1000;
  /* USER CODE END SysTick_IRQn 1 */
}

int64_t utime(void)
{
    uint32_t ctrl;
    static int64_t res;
    uint32_t ticks;

    ctrl = SysTick->CTRL;

read:
    ticks = SysTick->VAL;
    res = usAddition;
    ctrl = SysTick->CTRL;
    if (ctrl & SysTick_CTRL_COUNTFLAG_Msk)
        goto read;

    #define ticksPerUs (HAL_RCC_GetHCLKFreq()/1000000)
    res += 1000 - ticks / ticksPerUs;
    #undef usecPerTick

    return res;
}

void rtctime(int *h, int *m, int *s)
{
    RTC_TimeTypeDef time;
    //RTC_GetTime(RTC_Format_BIN, &time);
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    if (h != NULL) *h = time.Hours;
    if (m != NULL) *m = time.Minutes;
    if (s != NULL) *s = time.Seconds;
}

int64_t mtime(void)
{
    return utime() / 1000;
}

void usleep(int us)
{
    uint64_t t = utime();
    while (true)
    {
        uint64_t t1 = utime();
        if (t1 - t >= us) break;
        if (t1 < t) break; /* overflow */
    }
}

void stmr(void)
{
    stmr_t *tmr;
    uint32_t time;
    time = utime();
    tmr = stmrs;
    while (tmr != NULL)
    {
        stmr_t *t;
        uint32_t elapsed;
        t = tmr;
        tmr = tmr->next;
        if ((t->flags & STMR_ACTIVE) == 0)
            continue;
        elapsed = time;
        elapsed -= t->event;
        if (elapsed < t->period)
            continue;
        t->proc(t);
        t->event = utime();
    }
}


void stmr_init(stmr_t *tmr)
{
    tmr->period = 0;
    tmr->event = 0;
    tmr->flags = 0;
    tmr->data = NULL;
    tmr->proc = NULL;
    tmr->next = stmrs;
    stmrs = tmr;
}

void stmr_add(stmr_t *tmr)
{
    tmr->next = stmrs;
    stmrs = tmr;
}

void stmr_free(stmr_t *tmr)
{
    stmr_t *t;

    if (stmrs == NULL)
        return;

    if (tmr == stmrs)
    {
        stmrs = tmr->next;
        tmr->next = NULL;
        return;
    }

    t = stmrs;
    while (t->next != NULL)
    {
        if (t->next == tmr)
        {
            t->next = tmr->next;
            tmr->next = NULL;
            return;
        }
        t = t->next;
    }
}

void stmr_stop(stmr_t *tmr)
{
    tmr->flags &= ~(uint32_t)STMR_ACTIVE;
}

void stmr_run(stmr_t *tmr)
{
    tmr->flags |= STMR_ACTIVE;
    tmr->event = utime();
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
