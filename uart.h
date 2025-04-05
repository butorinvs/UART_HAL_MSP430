#pragma once

// Подключение основных хедеров
#ifdef __MSP430F2619__
#include "msp430f2619.h"
#endif
#ifdef __MSP430F2618__
#include "msp430f2618.h"
#endif

#include "stdint.h"
#include "spi.h"
#include "interrupts.h"

#define MAX_TIME_OUT_TX     50                           // макс. время передачи пакета мсек
#define MAX_TIME_OUT_RX     300                          // макс. время приема пакета, мсек

#define NO_INIT             0
// Частота кварца
#define FREQ_HIGH           16000000                        // Частота кварца, Гц
#define FREQ_ACLK           (!(BCSCTL1 & DIVA_0) ? FREQ_HIGH : \
                            !(BCSCTL1  & DIVA_1) ? FREQ_HIGH>>2 : \
                            !(BCSCTL1  & DIVA_2) ? FREQ_HIGH>>3 : \
                            !(BCSCTL1  & DIVA_3) ? FREQ_HIGH>>4 : FREQ_HIGH>>5)\

// Коды ошибок
enum {
    //PTR_LINK_ERR,                                      // 0 - указатель не инициализирован
    //OK,                                                // 1 - коды ошибок, все хорошо
    //TIME_OUT_ERR,
    TX_UART_ERR,
    RX_UART_ERR,
    BUSY_UART,
    TX_UART_OK,
    RX_UART_OK
};

typedef struct {
    uint32_t            baudrate;                          // скорость обмена
    uint32_t            freq_quartz;                       // Частота Работы кварца
    uint32_t            freq_aclk;                         // Частота Работы ACLK
    //PORT_TypeDef        *UART_PORT;                        // Используемый порт для работы
    Hard_TypeDef        TX_EN;                             // Инициализация порта для управления передачей
    Hard_TypeDef        RX_EN;                             // Инициализация порта для управления приемом
    char                *NAME;                             // название протокола
} UART_Init_TypeDef;

typedef struct {
    uint32_t            tx;                                // таймаут на передачу
    uint32_t            rx;                                // таймаут на прием
} TIMEOUT_TypeDef;

struct __UART_HandleTypeDef;
typedef struct __UART_HandleTypeDef
{
    UART_Init_TypeDef          Init;                                     // Начальная инициализация UART
    USCI_TypeDef               *Instance;                                // используемвый модуль USCI для работы с UART
    volatile uint32_t          ErrorCode;                                // Error code Может быть задан пользователем
    TIMEOUT_TypeDef            time_out;                                 // максимальное время для работы с устройством, при истечении происходит сброс
    DMA_TypeDef                DMA;                                      // структура для работы с DMA
    uint32_t    (*set_baudrate_callback)   (struct __UART_HandleTypeDef*);         // Функции установки скорости UART
    uint32_t    (*set_init_ins_callback)   (struct __UART_HandleTypeDef*);         // Функция инициализации управляющих регистров
    uint32_t    (*uart_txen_delay_callback)(struct __UART_HandleTypeDef*);         // Функция задержки
    uint8_t     (*mask_isr_UART_tx_pending_callback)(struct __UART_HandleTypeDef*);// Функция Возвращает маску используемого флага по прерыванию при приеме
    DMA_TypeDef (*spi_dma_init_callback)   (DMA_TypeDef, USCI_TypeDef*);           // инициаилазция DMA
}UART_Handle_TypeDef;


//TODO Переделать параметры функции set_bauderate необходимо вводить боды в сек, вместо UART_Handle_TypeDef*
UART_Handle_TypeDef uart_config              (UART_Handle_TypeDef*);
static uint32_t     set_init_ins             (UART_Handle_TypeDef*);
static uint32_t     set_baudrate             (UART_Handle_TypeDef*);
static uint32_t     uart_txen_delay          (UART_Handle_TypeDef*);
PORT_TypeDef       *usci_port_uart_use       (USCI_TypeDef*);
uint8_t            *isr_USCI_UART            (USCI_TypeDef*);
uint8_t             mask_isr_UART_rx_enable  (USCI_TypeDef*);
uint8_t             mask_isr_UART_tx_enable  (USCI_TypeDef*);
uint8_t             CALLBACK_ISR_RX          (ISR_TypeDef*);

