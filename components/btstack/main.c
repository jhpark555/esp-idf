/*
 * Copyright (C) 2016 BlueKitchen GmbH
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

#define __BTSTACK_FILE__ "main.c"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack_config.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_freertos.h"
#include "btstack_ring_buffer.h"
#include "btstack_tlv.h"
#include "btstack_tlv_esp32.h"
#include "ble/le_device_db_tlv.h"
#include "classic/btstack_link_key_db.h"
#include "classic/btstack_link_key_db_tlv.h"
#include "hci.h"
#include "hci_dump.h"
#include "esp_bt.h"
#include "btstack_debug.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s.h" 
#include "driver/i2c.h"
#include "driver/gpio.h"

uint32_t esp_log_timestamp();

uint32_t hal_time_ms(void) {
    // super hacky way to get ms
    return esp_log_timestamp();
}

#ifdef CONFIG_BT_ENABLED

// assert pre-buffer for packet type is available
#if !defined(HCI_OUTGOING_PRE_BUFFER_SIZE) || (HCI_OUTGOING_PRE_BUFFER_SIZE < 1)
#error HCI_OUTGOING_PRE_BUFFER_SIZE not defined or smaller than 1. Please update hci.h
#endif

static void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size);

// ring buffer for incoming HCI packets. Each packet has 2 byte len tag + H4 packet type + packet itself
#define MAX_NR_HOST_EVENT_PACKETS 4
static uint8_t hci_ringbuffer_storage[HCI_HOST_ACL_PACKET_NUM   * (2 + 1 + HCI_ACL_HEADER_SIZE + HCI_HOST_ACL_PACKET_LEN) +
                                      HCI_HOST_SCO_PACKET_NUM   * (2 + 1 + HCI_SCO_HEADER_SIZE + HCI_HOST_SCO_PACKET_LEN) +
                                      MAX_NR_HOST_EVENT_PACKETS * (2 + 1 + HCI_EVENT_BUFFER_SIZE)];

static btstack_ring_buffer_t hci_ringbuffer;
static uint8_t hci_receive_buffer[1 + HCI_PACKET_BUFFER_SIZE];
static SemaphoreHandle_t ring_buffer_mutex;

// data source for integration with BTstack Runloop
static btstack_data_source_t transport_data_source;
static int                   transport_signal_sent;
static int                   transport_packets_to_deliver;

// TODO: remove once stable 
void report_recv_called_from_isr(void){
     printf("host_recv_pkt_cb called from ISR!\n");
}

void report_sent_called_from_isr(void){
     printf("host_send_pkt_available_cb called from ISR!\n");
}

// VHCI callbacks, run from VHCI Task "BT Controller"

static void host_send_pkt_available_cb(void){

    if (xPortInIsrContext()){
        report_sent_called_from_isr();
        return;
    }

    // set flag and trigger polling of transport data source on main thread
    transport_signal_sent = 1;
    btstack_run_loop_freertos_trigger();
}

static int host_recv_pkt_cb(uint8_t *data, uint16_t len){

    if (xPortInIsrContext()){
        report_recv_called_from_isr();
        return 0;
    }

    xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);

    // check space
    uint16_t space = btstack_ring_buffer_bytes_free(&hci_ringbuffer);
    if (space < len){
        xSemaphoreGive(ring_buffer_mutex);
        log_error("transport_recv_pkt_cb packet %u, space %u -> dropping packet", len, space);
        return 0;
    }

    // store size in ringbuffer
    uint8_t len_tag[2];
    little_endian_store_16(len_tag, 0, len);
    btstack_ring_buffer_write(&hci_ringbuffer, len_tag, sizeof(len_tag));

    // store in ringbuffer
    btstack_ring_buffer_write(&hci_ringbuffer, data, len);

    xSemaphoreGive(ring_buffer_mutex);

    // set flag and trigger delivery of packets on main thread
    transport_packets_to_deliver = 1;
    btstack_run_loop_freertos_trigger();
    return 0;
}

static const esp_vhci_host_callback_t vhci_host_cb = {
    .notify_host_send_available = host_send_pkt_available_cb,
    .notify_host_recv = host_recv_pkt_cb,
};

// run from main thread

static void transport_notify_packet_send(void){
    // notify upper stack that it might be possible to send again
    uint8_t event[] = { HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
}

static void transport_deliver_packets(void){
    xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);
    while (btstack_ring_buffer_bytes_available(&hci_ringbuffer)){
        uint32_t number_read;
        uint8_t len_tag[2];
        btstack_ring_buffer_read(&hci_ringbuffer, len_tag, 2, &number_read);
        uint32_t len = little_endian_read_16(len_tag, 0);
        btstack_ring_buffer_read(&hci_ringbuffer, hci_receive_buffer, len, &number_read);
        xSemaphoreGive(ring_buffer_mutex);
        transport_packet_handler(hci_receive_buffer[0], &hci_receive_buffer[1], len-1);
        xSemaphoreTake(ring_buffer_mutex, portMAX_DELAY);
    }
    xSemaphoreGive(ring_buffer_mutex);
}


static void transport_process(btstack_data_source_t *ds, btstack_data_source_callback_type_t callback_type) {
    switch (callback_type){
        case DATA_SOURCE_CALLBACK_POLL:
            if (transport_signal_sent){
                transport_signal_sent = 0;
                transport_notify_packet_send();
            }
            if (transport_packets_to_deliver){
                transport_packets_to_deliver = 0;
                transport_deliver_packets();
            }
            break;
        default:
            break;
    }
}

/**
 * init transport
 * @param transport_config
 */
static void transport_init(const void *transport_config){
    log_info("transport_init");
    ring_buffer_mutex = xSemaphoreCreateMutex();

    // set up polling data_source
    btstack_run_loop_set_data_source_handler(&transport_data_source, &transport_process);
    btstack_run_loop_enable_data_source_callbacks(&transport_data_source, DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&transport_data_source);
}

/**
 * open transport connection
 */
static int bt_controller_initialized;
static int transport_open(void){
    esp_err_t ret;

    log_info("transport_open");

    btstack_ring_buffer_init(&hci_ringbuffer, hci_ringbuffer_storage, sizeof(hci_ringbuffer_storage));

    // http://esp-idf.readthedocs.io/en/latest/api-reference/bluetooth/controller_vhci.html (2017104)
    // - "esp_bt_controller_init: ... This function should be called only once, before any other BT functions are called."
    // - "esp_bt_controller_deinit" .. This function should be called only once, after any other BT functions are called. 
    //    This function is not whole completed, esp_bt_controller_init cannot called after this function."
    // -> esp_bt_controller_init can only be called once after boot
    if (!bt_controller_initialized){
        bt_controller_initialized = 1;

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            log_error("transport: esp_bt_controller_init failed");
            return -1;
        }

        esp_vhci_host_register_callback(&vhci_host_cb);
    }

    // enable dual mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        log_error("transport: esp_bt_controller_enable failed");
        return -1;
    }

    return 0;
}

/**
 * close transport connection
 */
static int transport_close(void){
    log_info("transport_close");

    // disable controller
    esp_bt_controller_disable();
    return 0;
}

/**
 * register packet handler for HCI packets: ACL, SCO, and Events
 */
static void transport_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size)){
    log_info("transport_register_packet_handler");
    transport_packet_handler = handler;
}

static int transport_can_send_packet_now(uint8_t packet_type) {
    return esp_vhci_host_check_send_available();
}

static int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size){
    // store packet type before actual data and increase size
    size++;
    packet--;
    *packet = packet_type;

    // send packet
    esp_vhci_host_send_packet(packet, size);
    return 0;
}

static const hci_transport_t transport = {
    "esp32-vhci",
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_packet,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
};

#else

// this port requires the ESP32 Bluetooth to be enabled in the sdkconfig
// try to tell the user

#include "esp_log.h"
static void transport_init(const void *transport_config){
    while (1){
        ESP_LOGE("BTstack", "ESP32 Transport Init called, but Bluetooth support not enabled in sdkconfig.");
        ESP_LOGE("BTstack", "Please enable CONFIG_BT_ENABLED with 'make menuconfig and compile again.");
        ESP_LOGE("BTstack", "");
    }
}

static const hci_transport_t transport = {
    "none",
    &transport_init,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
};
#endif

#if 1 

static btstack_packet_callback_registration_t hci_event_callback_registration;
static bd_addr_t cmdline_addr = { };
static btstack_timer_source_t warm_boot_timer;
static int cmdline_addr_found;
static const char * prog_name;

static uint8_t csr_set_bd_addr[] = {
    // 0x0001: Set Bluetooth address 
    0x00, 0xFC, 0x19, 0xc2, 0x02, 0x00, 0x0A, 0x00, 0x03, 0x00, 0x03, 0x70, 0x00, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0xf3, 0x00, 0xf5, 0xf4, 0xf2, 0x00, 0xf2, 0xf1,
};

static uint8_t csr_warm_start[] = {
    //  WarmReset
    0x00, 0xFC, 0x13, 0xc2, 0x02, 0x00, 0x09, 0x00, 0x03, 0x0e, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void usage(void){
    fprintf(stderr, "\nUsage: %s aa:bb:cc:dd:ee:ff\n", prog_name);
    exit(0);
}
/* @section Bluetooth Logic 
 *
 * @text The Bluetooth logic is implemented as a state machine within the packet
 * handler. In this example, the following states are passed sequentially:
 * INIT, and ACTIVE.
 */ 

static void local_version_information_handler(uint8_t * packet){
    uint16_t hci_version    = packet[6];
    uint16_t hci_revision   = little_endian_read_16(packet, 7);
    uint16_t lmp_version    = packet[9];
    uint16_t manufacturer   = little_endian_read_16(packet, 10);
    uint16_t lmp_subversion = little_endian_read_16(packet, 12);
    switch (manufacturer){
        case 0xA:
            printf("Cambridge Silicon Radio - CSR chipset.\n");
            break;
        default:
            printf("Local version information:\n");
            printf("- HCI Version    0x%04x\n", hci_version);
            printf("- HCI Revision   0x%04x\n", hci_revision);
            printf("- LMP Version    0x%04x\n", lmp_version);
            printf("- LMP Subversion 0x%04x\n", lmp_subversion);
            printf("- Manufacturer   0x%04x\n", manufacturer);
            printf("Not a CSR chipset, exit\n");
            usage();
            break;
    }
}

static int istate = 0;

static void warm_boot_handler(struct btstack_timer_source *ts){
    UNUSED(ts);

    if (istate != 4) return;
    printf("Done\n");
    exit(0);
} 

#endif




static const hci_transport_t * transport_get_instance(void){
    return &transport;
}

static btstack_packet_callback_registration_t hci_event_callback_registration;

#if 1  
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;

	printf("===>event_packet=%x  event_state=%x\n",hci_event_packet_get_type(packet),btstack_event_state_get_state(packet)); 
	
    switch(hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            printf("BTstack: up and running.\n");
            break;
        case HCI_EVENT_COMMAND_COMPLETE:
            if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_read_local_version_information)){
                // @TODO
            }
            break;
        default:
            break;
    }
}

#else
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
printf("===>event_packet=%x  event_state=%x\n",hci_event_packet_get_type(packet),btstack_event_state_get_state(packet)); 
    switch(hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                if (!cmdline_addr_found){
                    usage();
                    break;
                }
                printf("Setting BD ADDR to %s\n", bd_addr_to_str(cmdline_addr));
                istate = 1;                
            break;
        case HCI_EVENT_COMMAND_COMPLETE:
            if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_read_local_version_information)){
                local_version_information_handler(packet);
            }
            break;
        case HCI_EVENT_VENDOR_SPECIFIC:
            // Vendor event
            istate++;
            break;
        default:
            break;
    }

    if (!hci_can_send_command_packet_now()) return;
    switch (istate){
        case 0:
        case 2:
            break;
        case 1:
            istate++;
            hci_send_cmd_packet(csr_set_bd_addr, sizeof(csr_set_bd_addr));
            break;
        case 3:
            istate++;
            hci_send_cmd_packet(csr_warm_start, sizeof(csr_warm_start));
            // set timer
            warm_boot_timer.process = &warm_boot_handler;
            btstack_run_loop_set_timer(&warm_boot_timer, 1000);
            btstack_run_loop_add_timer(&warm_boot_timer);
            break;
    }
}


#endif


extern int btstack_main(int argc, const char * argv[]);


#include "driver/uart.h" 
#include "soc/uart_struct.h" 

#define BUF_SIZE (1024)  
static QueueHandle_t uart0_queue; 

hci_transport_config_uart_t uconfig = {
    HCI_TRANSPORT_CONFIG_UART,
    115200,
    1000000,  // main baudrate
    1,  // flow control
    NULL,
};


#define EX_UART_NUM UART_NUM_1
#define BUF_SIZE (1024)  
static QueueHandle_t uart0_queue; 

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
	
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
           printf("uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    printf( "data, len: %d; buffered len: %d", event.size, buffered_size);					
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                   printf( "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                   printf( "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(EX_UART_NUM);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    printf( "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    printf( "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    printf( "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    printf( "uart pattern detected\n");
                    break;
                //Others
                default:
                   printf( "uart event type: %d\n", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
	
}


#if 1 
#define SDA_PIN GPIO_NUM_18
#define SCL_PIN GPIO_NUM_19

void i2c_master_init()
{
		i2c_config_t i2c_config = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = SDA_PIN,
			.scl_io_num = SCL_PIN,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = 1000000
		};
		
		i2c_param_config(I2C_NUM_0, &i2c_config);
		i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

}
#define bt_addr  0xf0
#define bt_control_cmd 0x00

/*
void BT_init()
{
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (bt_addr << 1) | I2C_MASTER_WRITE, 0);
	i2c_master_write_byte(cmd, bt_control_cmd, 1);

	i2c_master_write_byte(cmd, 0x31, true);
	i2c_master_write_byte(cmd, 0x2f, 0);

	i2c_master_write_byte(cmd, 0x36, 0); 
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "OLED configured successfully");
	} else {
		ESP_LOGE(tag, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);   

}
*/

#endif

extern const btstack_chipset_t * btstack_chipset_csr_instance(void);

static QueueHandle_t i2s_event_queue;

// main
int app_main(void){

    printf("BTstack: setup\n");

    // enable packet logger
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX         
        .sample_rate = 44100,
        .bits_per_sample = 16,                                              
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 32,                                                       //number of buffers, 128 max
        .dma_buf_len = 64,                                                      // size of each buffer
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };

    i2s_driver_install(0, &i2s_config, 0, NULL);
	
    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_26,
        .ws_io_num = GPIO_NUM_25,
        .data_out_num = GPIO_NUM_22,
        .data_in_num = I2S_PIN_NO_CHANGE                                           //Not used
    };

    i2s_set_pin(0, &pin_config);

#if 1
	uart_config_t uart_config = {
	   .baud_rate =115200,
	   .data_bits = UART_DATA_8_BITS,
	   .parity = UART_PARITY_DISABLE,
	   .stop_bits = UART_STOP_BITS_1,
	   .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	}; 

	//Set UART parameters
	uart_param_config(EX_UART_NUM, &uart_config);
	//Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
	//Set UART pins (using UART1 default pins ie no changes.)  
	//uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   // tx 10, rx 9, 6,11 
	uart_set_pin(EX_UART_NUM, 4, 5, 10, 9);	 
	//Set uart pattern detect function.
	uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);
	uart_pattern_queue_reset(EX_UART_NUM, 20);
	//Create a task to handler UART event from ISR
	xTaskCreatePinnedToCore(&uart_event_task, "uart_event_task", 2048, NULL, 12, NULL,0);
#endif
#if 1 // i2c initial
    gpio_set_direction(0,GPIO_MODE_OUTPUT);
	gpio_set_level(0,0); 
for(int i=0;i<0xfff;i++)
	for(int j=0;j<0xfff;j++);
    gpio_set_level(0,1); 
	i2c_master_init();
#endif


    /// GET STARTED with BTstack ///
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());

#if 1 //   for esp32 vhci
    // init HCI
    hci_init(transport_get_instance(), NULL);

	// setup TLV ESP32 implementation and register with system
	const btstack_tlv_t * btstack_tlv_impl = btstack_tlv_esp32_get_instance();
	btstack_tlv_set_instance(btstack_tlv_impl, NULL);

	// setup Link Key DB using TLV
	const btstack_link_key_db_t * btstack_link_key_db = btstack_link_key_db_tlv_get_instance(btstack_tlv_impl, NULL);
	hci_set_link_key_db(btstack_link_key_db);

	// setup LE Device DB using TLV
	le_device_db_tlv_configure(btstack_tlv_impl, NULL);

	
    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
	
#else  // csr chipset
	const hci_transport_t * transport = hci_transport_h4_instance(btstack_uart_block_freertos_instance());
	hci_init(transport, &uconfig);	
	hci_set_link_key_db(btstack_link_key_db_memory_instance());
	hci_set_chipset(btstack_chipset_csr_instance());
	
	    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

	if (cmdline_addr_found){
	// prepare set bd addr command
	csr_set_bd_addr[20] = cmdline_addr[3];
	csr_set_bd_addr[22] = cmdline_addr[5];
	csr_set_bd_addr[23] = cmdline_addr[4];
	csr_set_bd_addr[24] = cmdline_addr[2];
	csr_set_bd_addr[26] = cmdline_addr[1];
	csr_set_bd_addr[27] = cmdline_addr[0];
	}
	
#endif


    btstack_main(0, NULL);

#if 0 //
	esp_err_t espRc;
	extern const uint8_t init_script[];
	
		printf("	 0	1  2  3  4	5  6  7  8	9  a  b  c	d  e  f\n");
		printf("00: 		");

	for (int i=3; i< 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);		
		//i2c_master_write(cmd, init_script, 140, 1);  // test for mcs 
		i2c_master_stop(cmd);

		espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (i%16 == 0) {
			printf("\n%.2x:", i);
		}
		if (espRc == 0) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
		//ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
		i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	
#endif




    printf("BTstack: execute run loop\n");
    btstack_run_loop_execute();
    return 0;
}

