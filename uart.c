/*
 *  Драйвер реализации работы по UART
 *  Author: Butorin
 *  Copyright (C) 2019 - 2024, Butorin, all right reserved.
 *
 *  Версия: 1.0 для MSP430F2618
 *Модуль предназначен для связи по UART с контроллерами MSP430F261x.
* Для использования необходимо инициализировать структуру handle в зависимости
* от используемого USCI. Необходимо переинициализировать используемые порты  вернуть их состояние в обратно в структуру
* Пример инициализации handle -  hal_uart_B1:

 UART_Handle_TypeDef   hal_uart_A0 = {
                                          .Init = {
                                                   .baudrate       = 115200,                 // скорость по умолчанию бод/сек
                                                   .freq_quartz    = 16000000,               // Частота запаянного кварца, Гц
                                                   .freq_aclk      = NO_INIT,                // Инициализация частотой ACLK переферия, инициализируется в функции
                                                   .TX_EN          = {
                                                                      (PORT_TypeDef*)&PORT1, // Порт используемый для аппаратного приема данных, если slave
                                                                      BIT2},                 // нога для аппаратного управления передачей
                                                   .RX_EN          = {
                                                                     NULL,                   // Порт используемый для CS
                                                                     NULL},                  // нога для аппаратного управления приемом

                                                   .NAME           = "RUS120"},              // Название протокола для работы по UART
                                          .Instance = (USCI_TypeDef*)&USCIA0,                // Используемый модуль  USCI
                                          .ErrorCode = OK,                                   // Код ошибки при работе с SPI

                                          .time_out  = {
                                                        .tx  = MAX_TIME_OUT_TX,              // тайм аут на передачу
                                                        .rx  = MAX_TIME_OUT_RX},             // тайм аут на прием
                                          .DMA  = {
                                                   .REG = (DMA_CTL_TypeDef*)&DMA_CTL,        // Управляющий регистр DMA
                                                   .CH_TX  = (DMA_CH_TypeDef*)&DMA_CH_2,     // Используемый модуль DMA для передачи
                                                   .CH_RX  = NULL}                           // не используется
};
 */

 //*******************************************************************************
// Секция include
//*******************************************************************************
//#include "main.h"
#include "uart.h"
#include "interrupts.h"

UART_Handle_TypeDef uart_config(UART_Handle_TypeDef *handle){

    uint32_t result = 0;

    handle->Init.freq_aclk           =  FREQ_ACLK;                             // Расчет частоты ACLK
    // инициализация call_back функций
    usci_port_uart_use(handle->Instance);                                      // Инициализация ног испольуземого USCI
    handle->set_init_ins_callback    = set_init_ins;                           // Инициализация используемого USCI
    handle->set_baudrate_callback    = set_baudrate;                           // Инициализация скокрости обмена baudrate
    handle->uart_txen_delay_callback = uart_txen_delay;                        // Инициализация задержки
    handle->spi_dma_init_callback    = spi_dma_init;                           // Инициализация DMA
    //handle->mask_isr_UART_tx_pending_callback = mask_isr_UART_tx_pending;      // Возвращает маску исполь
    result = handle->set_init_ins_callback(handle);
    result = handle->set_baudrate_callback(handle);
    handle->DMA = handle->spi_dma_init_callback(handle->DMA, handle->Instance);
    return *handle;
};

static uint32_t set_init_ins(UART_Handle_TypeDef *handle){
    uint8_t mask;
    uint8_t *ptr_isr;                                                          // Указатель на используемое прерывание
    if(!handle->Instance)
        return handle->ErrorCode = PTR_NULL_ERR;                               // USCI не инициализирован
    ptr_isr = isr_USCI_UART(handle->Instance);                                 // возвращает указатель на используемое прерывание по UART
    mask = mask_isr_UART_rx_enable(handle->Instance);                          // Возврщает используемое прерывание по USCI
    *ptr_isr &= mask;                                                          // запретить прерывание от USCI по приему RX
    handle->Instance->USCI_CTL1 |= UCSWRST;                                    // перевод всех автоматов UART в сброс
    handle->Instance->USCI_CTL0  = 0;                                          // нет бита четности, первым идет LSB, 8-битные данные, 1 стоп-бит
    handle->Instance->USCI_CTL1 |= UCSSEL_1;                                   // источник тактовой частоты ACLK
    handle->Instance->USCI_STAT &= ~UCLISTEN;                                  // отключаем эхо данные
    handle->Instance->USCI_CTL1 &= ~UCSWRST;                                   // снять состояние сброса UART
    *ptr_isr |= mask;                                                          // разрешить прерывание от USCI по приему RX
    return handle->ErrorCode;
}
// TODO Для универсального кода добавить проверку оверсэмплинга
/**
 * @Function
       uint32_t set_baudrate(UART_Handle_TypeDef *handle)
 * @param
 *     *handle    - структкра для работы по UART)
 * @Summary
 *     Возвращает статус установки скорости работы по UART
*/
static uint32_t set_baudrate(UART_Handle_TypeDef *handle){
    #define BAUDRATE_DEF (uint32_t)115200
    float temp, BRF;
    uint16_t INT;
    if(!handle->Init.baudrate) handle->Init.baudrate  = BAUDRATE_DEF;          // Скорость по умолчанию
    temp = (float)handle->Init.freq_aclk / (float)handle->Init.baudrate;
    INT = (uint16_t)temp;
    handle->Instance->USCI_BR1 = (uint8_t)(INT >> 8);                                // Вычисление делителей для работы UART
    handle->Instance->USCI_BR0 = (uint8_t)(INT & 0xFF);
    BRF = (temp - (float)INT) * 8;
    handle->Instance->USCI_MCTL = (uint8_t)(BRF) << 1;
    return handle->ErrorCode = OK;
}
/**
 * @Function
       uint8_t isr_USCI_UART (USART_TypeDef *ins)
 * @param
 *     *ins    - используемый  USCI для UART (могу быть использованы только USCI A0, A1)
 * @Summary
 *     Возвращает используемое прерывание для работы с USCI
*/
static uint32_t uart_txen_delay(UART_Handle_TypeDef *handle){
    if(!handle->Init.baudrate)
        return  handle->ErrorCode = PTR_NULL_ERR;                              // Скорость не поддерживается
    switch(handle->Init.baudrate)
        {
           case 1200:  __delay_cycles(140000); break;
           case 2400:  __delay_cycles(70000);  break;
           case 4800:  __delay_cycles(35000);  break;
           case 9600:  __delay_cycles(17500);  break;
           case 19200: __delay_cycles(8750);   break;
           case 38400: __delay_cycles(4375);   break;
           case 57600: __delay_cycles(2917);   break;
           case 115200:__delay_cycles(1458);   break;
           case 230400:__delay_cycles(729);    break;
           case 460800:__delay_cycles(365);    break;
           case 921600:__delay_cycles(187);    break;
           default:                            break;
        }
    return  handle->ErrorCode = OK;
}


PORT_TypeDef *usci_port_uart_use (USCI_TypeDef *usci){

    PORT_TypeDef *uart_port = (PORT_TypeDef*)&PORT3;
          if (!usci)  return uart_port;                                    // порт не проинициализирован
    else  if (usci == (USCI_TypeDef*)&UCA0CTL0)
              uart_port->SEL_P |=  BIT4|BIT5;                         // выбрать ноги для выводов USCI_A0 SIMO/SOMI/CLK
    else if (usci == (USCI_TypeDef*)&UCA1CTL0)
             uart_port->SEL_P  |=  BIT6|BIT7;                              // выбрать ноги для выводов USCI_A1 SIMO/SOMI
    else if (usci == (USCI_TypeDef*)&UCB0CTL0 || usci == (USCI_TypeDef*)&UCB1CTL0)
            return uart_port;                                    // порт не проинициализирован
    return uart_port;
}

/**
 * @Function
       uint8_t isr_USCI_UART (USART_TypeDef *ins)
 * @param
 *     *ins    - используемый  USCI для UART (могу быть использованы только USCI A0, A1)
 * @Summary
 *     Возвращает используемое прерывание для работы с USCI
*/
 uint8_t *isr_USCI_UART (USCI_TypeDef *ins){
         if (ins  == (USCI_TypeDef*)&UCA0CTL0) return  (uint8_t*)&IE2;
    else if (ins  == (USCI_TypeDef*)&UCA1CTL0) return  (uint8_t*)&UC1IE;
    return (uint8_t*)0xFF;
}
 /**
  * @Function
        uint8_t isr_USCI_UART_flag (USART_TypeDef *ins)
  * @param
  *     *ins    - используемый  USCI для UART (могу быть использованы только USCI A0, A1)
  * @Summary
  *     Возвращает используемое флаги прерывания для работы с USCI
 */
  uint8_t *isr_USCI_UART_flag (USCI_TypeDef *ins){
          if (ins  == (USCI_TypeDef*)&UCA0CTL0) return  (uint8_t*)&IFG2;
     else if (ins  == (USCI_TypeDef*)&UCA1CTL0) return  (uint8_t*)&UC1IFG;
     return (uint8_t*)0xFF;
 }

 /**
 * @Function
       uint8_t mask_isr_UART_rx_enable (USART_TypeDef *ins)
 * @param
 *     *ins    - используемый  USCI для UART
 * @Summary
 *     Возвращает маску  включения прерывания приема
*/
 uint8_t mask_isr_UART_rx_enable (USCI_TypeDef *ins){
         if (ins  == (USCI_TypeDef*)&UCA0CTL0) return  UCA0RXIE;
    else if (ins  == (USCI_TypeDef*)&UCA1CTL0) return  UCA1RXIE;
    return 0xFF;
}
/**
 * @Function
       uint8_t mask_isr_UART_tx_enable (USART_TypeDef *ins)
 * @param
 *     *ins    - используемый  USCI для UART
 * @Summary
 *     Возвращает маску  включения прерывания передачи
*/
 uint8_t mask_isr_UART_tx_enable (USCI_TypeDef *ins){
         if (ins  == (USCI_TypeDef*)&UCA0CTL0) return  UCA0TXIE;
    else if (ins  == (USCI_TypeDef*)&UCA1CTL0) return  UCA1TXIE;
    return 0xFF;
}

 /**
  * @Function
        uint8_t mask_isr_UART_tx_enable (USART_TypeDef *ins)
  * @param
  *     *ins    - используемый  USCI для UART
  * @Summary
  *     Возвращает маску используемого флага прерывания
 */
uint8_t mask_isr_UART_tx_pending (USCI_TypeDef *ins){
          if (ins  == (USCI_TypeDef*)&UCA0CTL0) return  UCA0TXIFG;
     else if (ins  == (USCI_TypeDef*)&UCA1CTL0) return  UCA1TXIFG;
     return 0xFF;
}
/**
* @Function
      uint8_t CALLBACK_ISR_RX(ISR_TypeDef *isr)
* @param
*     *isr    - вектор прерывания
* @Summary
*     Возвращает принятое зачения из аппартаного буфера
*/
uint8_t CALLBACK_ISR_RX(ISR_TypeDef *isr){
         if (isr->ISR_IFG2 & UCA0RXIFG) return UCA0RXBUF;
    else if (isr->ISR_IFG2 & UCB0RXIFG) return UCB0RXBUF;
    else if (isr->ISR_IFG1 & UCA1RXIFG) return UCA1RXBUF;
    else if (isr->ISR_IFG1 & UCB1RXIFG) return UCB1RXBUF;
    else return 0xFF;
}

