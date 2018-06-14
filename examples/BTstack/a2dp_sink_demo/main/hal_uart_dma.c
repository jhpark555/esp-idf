/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/** 
 * @file  hal_bt.c
 ***************************************************************************/
#include <stdint.h>

//#include "hal_compat.h"

#include "hal_uart_dma.h"

extern void hal_cpu_set_uart_needed_during_sleep(uint8_t enabled);

// debugging only
// #include <stdio.h>


void dummy_handler(void){};

// rx state
static uint16_t  bytes_to_read = 0;
static uint8_t * rx_buffer_ptr = 0;

// tx state
static uint16_t  bytes_to_write = 0;
static uint8_t * tx_buffer_ptr = 0;

// handlers
static void (*rx_done_handler)(void) = dummy_handler;
static void (*tx_done_handler)(void) = dummy_handler;
static void (*cts_irq_handler)(void) = dummy_handler;

/**
 * @brief  Initializes the serial communications peripheral and GPIO ports 
 *         to communicate with the PAN BT .. assuming 16 Mhz CPU
 * 
 * @param  none
 * 
 * @return none
 */
void hal_uart_dma_init(void)
{
    bytes_to_write = 0;
    bytes_to_read = 0;
}


int hal_uart_dma_set_baud(uint32_t baud)
{

    int result = 0;

    return result;


}

void hal_uart_dma_set_block_received( void (*the_block_handler)(void))
{
    rx_done_handler = the_block_handler;
}

void hal_uart_dma_set_block_sent( void (*the_block_handler)(void))
{
    tx_done_handler = the_block_handler;
}

void hal_uart_dma_set_csr_irq_handler( void (*the_irq_handler)(void))
{

}

/**********************************************************************/
/**
 * @brief  Disables the serial communications peripheral and clears the GPIO
 *         settings used to communicate with the BT.
 * 
 * @param  none
 * 
 * @return none
 **************************************************************************/
void hal_uart_dma_shutdown(void)
{

}

void hal_uart_dma_send_block(const uint8_t * data, uint16_t len)
{
    tx_buffer_ptr = (uint8_t *) data;
    bytes_to_write = len;
}

static inline void hal_uart_dma_enable_rx(void)
{

}

static inline void hal_uart_dma_disable_rx(void)
{

}

// int used to indicate a request for more new data
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len)
{
    rx_buffer_ptr = buffer;
    bytes_to_read = len;
}

void hal_uart_dma_set_sleep(uint8_t sleep)
{

}


