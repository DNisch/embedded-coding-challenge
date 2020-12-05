#include <FreeRTOS.h>
#include <task.h>
#include <message_buffer.h>
#include <string.h>
#include <challenge.h>
#include "console.h"

// oh hi, you found the emulator code.
// please don't use this as a guideline for your implementation, this is neither pretty nor robust.

static uint8_t message_buffer_data[128];
static MessageBufferHandle_t message_buffer;

// called by the challenge code
void send(uint8_t* message, uint32_t length) {
    xMessageBufferSend(message_buffer, message, length, 0);
}

static void execute(
        uint8_t* request, uint32_t request_length,
        uint8_t* response, uint32_t response_length,
        uint32_t response_time) {

    for (uint32_t i = 0; i < request_length; i++) {
        receive_ISR(request[i]);
        if (i < request_length - 1) vTaskDelay(1);
    }
    if (response == NULL) return;
    uint32_t start = xTaskGetTickCount();
    uint8_t rx[16];
    size_t length = xMessageBufferReceive(message_buffer, rx, sizeof(rx), 5000);
    if (length == 0) {
        console_print("ERROR: response timeout\n");
        return;
    }
    uint32_t elapsed = xTaskGetTickCount() - start;
//    console_print("response time: %u\n", elapsed);
    if (response_time > 0 && response_time != elapsed) {
        console_print("ERROR: response time %u ms (expected %u ms)\n", elapsed, response_time);
    }
    if (length != response_length || memcmp(rx, response, length) != 0) {
        console_print("ERROR: invalid response\n");
    }
}

static void send_empty() {
    uint8_t empty[] = { 0x10 };
    execute(empty, sizeof(empty), empty, sizeof(empty), 0);
}

static void send_add() {
    uint8_t add[] = {
            0x24,
            (uint8_t) (xTaskGetTickCount() >> 3),
            (uint8_t) (xTaskGetTickCount() >> 21),
            (uint8_t) (xTaskGetTickCount() >> 5),
            (uint8_t) (xTaskGetTickCount() >> 13),
    };
    uint16_t r = (add[1] + add[3]) * 256 + add[2] + add[4];
    uint8_t result[] = { 0x32, r >> 8, r };
    execute(add, sizeof(add), result, sizeof(result), 0);
}

static void send_delay() {
    uint8_t delay[] = {
            0x43,
            (uint8_t) (xTaskGetTickCount() >> 3) & 0x4,
            (uint8_t) (xTaskGetTickCount() >> 1),
            (uint8_t) (xTaskGetTickCount() >> 5)
    };
    uint8_t result[] = { 0x51, delay[3] };
    execute(delay, sizeof(delay), result, sizeof(result), delay[1] * 256 + delay[2]);
}

static void send_log() {
    static uint8_t i = 0;
    uint8_t hello[] = { 0x65 + i, 'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd' };
    execute(hello, 6 + i, NULL, 0, 0);
    i = (i + 2) % (sizeof(hello) - 5);
}

static void emulator(void* argument) {
    while (1) {
        send_empty();
        vTaskDelay(1000);
        send_add();
        send_add();
        vTaskDelay(1000);
        send_delay();
        send_delay();
        vTaskDelay(1000);
        send_log();
        send_log();
        send_log();
        vTaskDelay(1000);
    }
}

static void runner(void* argument) {
    challenge_run();
    console_print("ERROR: main function returned\n");
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

int main() {
    console_init();
    console_print("starting challenge\n");
    message_buffer = xMessageBufferCreate(sizeof(message_buffer_data));
    challenge_init();
    xTaskCreate(emulator, "emulator", configMINIMAL_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(runner, "challenge", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    vTaskStartScheduler();
    return 0;
}
