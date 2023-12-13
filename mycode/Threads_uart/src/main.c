#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/kernel.h>



#define PRIORITY 7
/* size of stack area used by each thread */
#define STACKSIZE 1024

K_MUTEX_DEFINE(mutex);

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)


#define MSG_SIZE 100

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;
	

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	
	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';
			
			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
			print_uart("Hello! I'm your reciver thread .\r\n");
		}
		/* else: characters beyond buffer size are dropped */
	}
	
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}


void thread_RX(){

		
		if (!device_is_ready(uart_dev)) {
			printk("UART device not found!");
			return;
		}

		/* configure interrupt and callback to receive data */
		k_mutex_lock(&mutex, K_FOREVER);
		int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
		

		if (ret < 0) {
			if (ret == -ENOTSUP) {
				printk("Interrupt-driven UART API support not enabled\n");
			} else if (ret == -ENOSYS) {
				printk("UART device does not support interrupt-driven API\n");
			} else {
				printk("Error setting UART callback: %d\n", ret);
			}
			return;
		}

	    k_mutex_unlock(&mutex);
		uart_irq_rx_enable(uart_dev);
	
		
		print_uart("Tell me something and press enter:\r\n");
		
}

void thread_TX(){

	
	k_mutex_lock(&mutex, K_FOREVER);

	char tx_buf[MSG_SIZE];

	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		printk("Hello! I'm your Trasnmitter thread .\r\n");
		print_uart(tx_buf);
		print_uart("\r\n");
	}
	k_mutex_unlock(&mutex);
	
    
}

K_THREAD_DEFINE(thread_TX_id, STACKSIZE, thread_TX, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(thread_RX_id, STACKSIZE, thread_RX, NULL, NULL, NULL,
		PRIORITY, 0, 0);
