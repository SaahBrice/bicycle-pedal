#include "communication/communication.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define PEDAL_UART_NODE DT_CHOSEN(zephyr_console)
#define PEDAL_UART_TX_CAPACITY 2048u // [bytes] Queued output capacity.
#define PEDAL_UART_TX_MASK (PEDAL_UART_TX_CAPACITY - 1u) // [1] Circular index mask.

#if (PEDAL_UART_TX_CAPACITY & PEDAL_UART_TX_MASK) != 0u
#error "UART TX capacity must be a power of two"
#endif

static const struct device *const pedal_uart = DEVICE_DT_GET(PEDAL_UART_NODE);
static uint8_t tx_queue[PEDAL_UART_TX_CAPACITY];
static volatile size_t tx_head;
static volatile size_t tx_tail;
static bool uart_initialized;

/* ------------------------------------------------------------------ */
/* Creates: static void pedal_uart_isr(const struct device *device,   */
/*                                     void *user_data)                */
/*                                                                    */
/* Description:                                                       */
/*   Moves queued bytes to the UART hardware whenever its transmit    */
/*   FIFO can accept data. Disables TX interrupts when the queue is   */
/*   empty. The driver copies bytes to DMA-safe RAM before returning. */
/*                                                                    */
/* Input:                                                             */
/*   device - UART device that triggered the interrupt                */
/*   user_data - unused                                               */
/*                                                                    */
/* Output: void - updates the queue tail                              */
/* ------------------------------------------------------------------ */
static void pedal_uart_isr(const struct device *device, void *user_data)
{
    (void)user_data;
    if (!uart_irq_update(device)) {
        return;
    }
    while (uart_irq_tx_ready(device)) {
        if (tx_tail == tx_head) {
            uart_irq_tx_disable(device);
            break;
        }
        const size_t available = (tx_head - tx_tail) & PEDAL_UART_TX_MASK;
        const size_t contiguous = PEDAL_UART_TX_CAPACITY - tx_tail;
        const size_t length = available < contiguous ? available : contiguous;
        const int sent = uart_fifo_fill(device, &tx_queue[tx_tail], (int)length);
        if (sent <= 0) {
            break;
        }
        tx_tail = (tx_tail + (size_t)sent) & PEDAL_UART_TX_MASK;
    }
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_communication_init(void)                        */
/*                                                                    */
/* Description:                                                       */
/*   Checks the chosen UART and registers the interrupt callback.     */
/*   This module owns the UART TX interrupt; no other component       */
/*   should install a callback on the same device.                    */
/*                                                                    */
/* Input: none                                                         */
/*                                                                    */
/* Output: int - 0 on success or negative errno from the driver      */
/* ------------------------------------------------------------------ */
int pedal_communication_init(void)
{
    if (!device_is_ready(pedal_uart)) {
        return -ENODEV;
    }
    if (uart_initialized) {
        return 0;
    }
    const int rc = uart_irq_callback_user_data_set(pedal_uart, pedal_uart_isr,
                                                   NULL);
    if (rc < 0) {
        return rc;
    }
    tx_head = 0u;
    tx_tail = 0u;
    uart_initialized = true;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Creates: int pedal_communication_send(PedalMessageLevel level,     */
/*                                      const char *message)           */
/*                                                                    */
/* Description:                                                       */
/*   Queues a state prefix, message, and CRLF atomically. Returns     */
/*   -ENOSPC if the whole line cannot fit; it never waits for UART    */
/*   transmission. A successful return means queued, not delivered.  */
/*                                                                    */
/* Input:                                                             */
/*   level - plain, success, warning, or error                         */
/*   message - null-terminated text                                    */
/*                                                                    */
/* Output: int - 0 or -EINVAL, -ENODEV, or -ENOSPC                     */
/* ------------------------------------------------------------------ */
int pedal_communication_send(PedalMessageLevel level, const char *message)
{
    static const char *const prefixes[] = {"", "SUCCESS: ", "WARNING: ", "ERROR: "};
    if (message == NULL || level < PEDAL_MESSAGE_PLAIN ||
        level > PEDAL_MESSAGE_ERROR) {
        return -EINVAL;
    }
    if (!uart_initialized) {
        return -ENODEV;
    }
    const char *prefix = prefixes[level];
    const size_t prefix_length = strlen(prefix);
    const size_t message_length = strlen(message);
    const size_t length = prefix_length + message_length + 2u;
    if (length >= PEDAL_UART_TX_CAPACITY) {
        return -ENOSPC;
    }

    const unsigned int key = irq_lock();
    const size_t used = (tx_head - tx_tail) & PEDAL_UART_TX_MASK;
    if (length > PEDAL_UART_TX_CAPACITY - 1u - used) {
        irq_unlock(key);
        return -ENOSPC;
    }
    for (size_t i = 0u; i < prefix_length; ++i) {
        tx_queue[tx_head] = (uint8_t)prefix[i];
        tx_head = (tx_head + 1u) & PEDAL_UART_TX_MASK;
    }
    for (size_t i = 0u; i < message_length; ++i) {
        tx_queue[tx_head] = (uint8_t)message[i];
        tx_head = (tx_head + 1u) & PEDAL_UART_TX_MASK;
    }
    tx_queue[tx_head] = '\r';
    tx_head = (tx_head + 1u) & PEDAL_UART_TX_MASK;
    tx_queue[tx_head] = '\n';
    tx_head = (tx_head + 1u) & PEDAL_UART_TX_MASK;
    uart_irq_tx_enable(pedal_uart);
    irq_unlock(key);
    return 0;
}
