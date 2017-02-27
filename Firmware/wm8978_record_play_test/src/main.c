/*
****************************************************
    WM8978 Record&Play Demo
    Requirements:
        MT7687 HDK
	WM8978 Module
    Author:Michael DU
****************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* hal includes */
#include "hal.h"
#include "system_mt7687.h"
#include "top.h"

#include "wm8978.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define time_last      4000   //Record&Play last time,4000ms
#define AUDIO_BUFFER_LENGTH    time_last*16000/1000  //16K Sample Rate
#define I2S_TX_BUFFER_LENGTH   1024
#define I2S_RX_BUFFER_LENGTH   1024

/* Private variables ---------------------------------------------------------*/
static uint32_t I2S_TxBuf[I2S_TX_BUFFER_LENGTH];
static uint32_t I2S_RxBuf[I2S_RX_BUFFER_LENGTH];
static uint16_t audio_buffer[AUDIO_BUFFER_LENGTH];
uint16_t audio_play_callback_count = 0;
uint16_t audio_buffer_index = 0;

/* Private functions ---------------------------------------------------------*/

#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
    /* Place your implementation of fputc here */
    /* E.g. write a character to the HAL_UART_0 one at a time */
    hal_uart_put_char(HAL_UART_0, ch);
    return ch;
}
static void i2s_read(void);
static void i2s_write(void);
/**
*@brief Set pinmux to UART and initialize UART hardware initialization for logging.
*@param None.
*@return None.
*/
static void plain_log_uart_init(void)
{
    hal_uart_config_t uart_config;
    /* Set Pinmux to UART */
    hal_pinmux_set_function(HAL_GPIO_0, HAL_GPIO_0_UART1_RTS_CM4);
    hal_pinmux_set_function(HAL_GPIO_1, HAL_GPIO_1_UART1_CTS_CM4);
    hal_pinmux_set_function(HAL_GPIO_2, HAL_GPIO_2_UART1_RX_CM4);
    hal_pinmux_set_function(HAL_GPIO_3, HAL_GPIO_3_UART1_TX_CM4);

    /* COM port settings */
    uart_config.baudrate = HAL_UART_BAUDRATE_115200;
    uart_config.word_length = HAL_UART_WORD_LENGTH_8;
    uart_config.stop_bit = HAL_UART_STOP_BIT_1;
    uart_config.parity = HAL_UART_PARITY_NONE;
    hal_uart_init(HAL_UART_0, &uart_config);
}

/**
*@brief Configure and initialize the systerm clock.
*@param None.
*@return None.
*/
static void SystemClock_Config(void)
{
    top_xtal_init();
}

/**
*@brief  Initialize the periperal driver in this function. In this example, we initialize UART drivers.
*@param None.
*@return None.
*/
static void prvSetupHardware(void)
{
    /* Peripherals initialization */
    plain_log_uart_init();
}


/**
*@brief  In this function, we send datum to EEPROM and read them back to verify the success of i2c communication with EEPROM.
*@param None.
*@return None.
*/


/* Example of i2s_configure */
static int8_t i2s_configure(void)
{
    hal_i2s_config_t i2s_config;
    hal_i2s_status_t result = HAL_I2S_STATUS_OK;
    
    hal_gpio_init(HAL_GPIO_29);
    hal_gpio_init(HAL_GPIO_35);
    //hal_gpio_init(HAL_GPIO_26);
    hal_gpio_init(HAL_GPIO_31);
    hal_gpio_init(HAL_GPIO_32);
    hal_gpio_init(HAL_GPIO_30);

    hal_pinmux_set_function(HAL_GPIO_29, HAL_GPIO_29_I2S_MCLK);
    hal_pinmux_set_function(HAL_GPIO_35, HAL_GPIO_35_I2S_TX);
    //hal_pinmux_set_function(HAL_GPIO_26, HAL_GPIO_26_I2S_TX);
    hal_pinmux_set_function(HAL_GPIO_31, HAL_GPIO_31_I2S_RX);
    hal_pinmux_set_function(HAL_GPIO_32, HAL_GPIO_32_I2S_BCLK);//SCLK
    hal_pinmux_set_function(HAL_GPIO_30, HAL_GPIO_30_I2S_FS);//LRCK
    
    /* Set I2S as internal loopback mode */
    result = hal_i2s_init(HAL_I2S_TYPE_EXTERNAL_MODE);
    if (HAL_I2S_STATUS_OK != result) {
        log_hal_info("\r\n ---i2s:    hal_i2s_init failed---\r\n");
        return -1;
    }

    /* Configure I2S  */
    i2s_config.clock_mode = HAL_I2S_SLAVE;
    i2s_config.rx_down_rate = HAL_I2S_RX_DOWN_RATE_DISABLE;
    i2s_config.tx_mode = HAL_I2S_TX_MONO_DUPLICATE_DISABLE;
    i2s_config.i2s_out.channel_number = HAL_I2S_MONO;

    i2s_config.i2s_out.sample_rate = HAL_I2S_SAMPLE_RATE_16K;
    i2s_config.i2s_in.sample_rate = HAL_I2S_SAMPLE_RATE_16K;
    i2s_config.i2s_in.msb_offset = 0;
    i2s_config.i2s_out.msb_offset = 0;
    i2s_config.i2s_in.word_select_inverse = 0;
    i2s_config.i2s_out.word_select_inverse = 0;
    i2s_config.i2s_in.lr_swap = 0;
    i2s_config.i2s_out.lr_swap = 0;

    result = hal_i2s_set_config(&i2s_config);
    if (HAL_I2S_STATUS_OK != result) {
        log_hal_info("\r\n ---i2s:    hal_i2s_set_config failed---\r\n");
        return -1;
    }

    result = hal_i2s_setup_tx_vfifo(I2S_TxBuf, I2S_TX_BUFFER_LENGTH / 2, I2S_TX_BUFFER_LENGTH);
    if (HAL_I2S_STATUS_OK != result) {
        log_hal_info("\r\n ---i2s:    hal_i2s_setup_tx_vfifo failed---\r\n");
        return -1;
    }

    result = hal_i2s_setup_rx_vfifo(I2S_RxBuf, I2S_RX_BUFFER_LENGTH / 2, I2S_RX_BUFFER_LENGTH);
    if (HAL_I2S_STATUS_OK != result) {
        log_hal_info("\r\n ---i2s:    hal_i2s_setup_rx_vfifo failed---\r\n");
        return -1;
    }

    return 1;
}

static void user_i2s_tx_callback(hal_i2s_event_t event, void *user_data)
{
    // always send I2S TX events to user task
    // audio_procedure_call_in_media_task(src_module_id, user_i2s_event_handler, event, user_data);
    i2s_write();
}

static void user_i2s_rx_callback(hal_i2s_event_t event, void *user_data)
{
    // always send I2S RX events to user task
    // audio_procedure_call_in_media_task(src_module_id, user_i2s_event_handler, event, user_data);
    i2s_read();
}

void user_i2s_event_handler(hal_i2s_event_t event, void *user_data)
{
    switch(event) {
        case HAL_I2S_EVENT_DATA_REQUEST:
          // write data to the TX VFIFO
          
          break;
        case HAL_I2S_EVENT_DATA_NOTIFICATION:
          // read data from the RX VFIFO
          break;
     }
}

static void i2s_start_write(void)
{
      hal_i2s_register_tx_vfifo_callback(user_i2s_tx_callback, NULL);
      hal_i2s_enable_tx();
      hal_i2s_enable_audio_top();
      hal_i2s_enable_tx_dma_interrupt();
}

static void i2s_start_read(void)
{
    hal_i2s_register_rx_vfifo_callback(user_i2s_rx_callback, NULL);
    hal_i2s_enable_rx();
    hal_i2s_enable_audio_top();
    hal_i2s_enable_rx_dma_interrupt();
}

static void i2s_write(void)
{
    uint32_t write_sample_count = 0;
    uint32_t i = 0;
    hal_i2s_get_tx_sample_count(&write_sample_count);
    
    if (write_sample_count >= I2S_TX_BUFFER_LENGTH) {
        for (i = 0; i < I2S_TX_BUFFER_LENGTH; i++) {
            hal_i2s_tx_write(audio_buffer[audio_play_callback_count*I2S_TX_BUFFER_LENGTH + i]);
        }
            audio_play_callback_count++;
    }
    
    if(audio_play_callback_count > (AUDIO_BUFFER_LENGTH/I2S_TX_BUFFER_LENGTH)){
            hal_i2s_disable_tx_dma_interrupt();
      hal_i2s_disable_rx_dma_interrupt();
    
      hal_i2s_disable_tx();
      hal_i2s_disable_rx();
      hal_i2s_disable_audio_top();
      hal_i2s_deinit();
      hal_i2s_stop_tx_vfifo();
      hal_i2s_stop_rx_vfifo();
      audio_play_callback_count = 0;
    }
}

/* Example of i2s_read */
static void i2s_read(void)
{
    uint32_t read_sample_count = 0;
    uint32_t rx_data_temp = 0; 
    hal_i2s_get_rx_sample_count(&read_sample_count);
    while (read_sample_count != 0) {
      
        hal_i2s_rx_read(&rx_data_temp);
        if (audio_buffer_index < AUDIO_BUFFER_LENGTH) {
            audio_buffer[audio_buffer_index] = (uint16_t)rx_data_temp;           
            audio_buffer_index++;
        } else {
            hal_i2s_disable_rx_dma_interrupt();
            hal_i2s_disable_rx();
            break;
        }
        hal_i2s_get_rx_sample_count(&read_sample_count);
    }

}


/* Example of i2s_close */
static void i2s_close(void)
{
    // disable the interrupt
    hal_i2s_disable_tx_dma_interrupt();
    hal_i2s_disable_rx_dma_interrupt();
    
    hal_i2s_disable_tx();
    hal_i2s_disable_rx();
    hal_i2s_disable_audio_top();
    hal_i2s_deinit();
    hal_i2s_stop_tx_vfifo();
    hal_i2s_stop_rx_vfifo();
}

int main(void)
{
    /* Configure system clock */
    SystemClock_Config();

    /* Configure the hardware */
    prvSetupHardware();

    /* Enable I,F bits */
    __enable_irq();
    __enable_fault_irq();

    log_hal_info("Hello,Let's test:)\r\n");

    while(1){ 
        wm8978_play_record_start();
        i2s_configure();
        log_hal_info("Record Start\r\n");
        i2s_start_read();
        hal_gpt_delay_ms(time_last + 1500);
        log_hal_info("Record Done\r\n");
        audio_buffer_index = 0;
        log_hal_info("Play Start\r\n");
        i2s_start_write();
        hal_gpt_delay_ms(time_last + 1500);
        log_hal_info("Play Done\r\n");
  }
}

