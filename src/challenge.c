#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <timers.h>
#include <console.h>
#include <stdlib.h>
#include <stdbool.h>

#define PRIORITY_0 0

#define EMPTY 1
#define ADD 2
#define DELAY 4
#define LOG 6

#define EMPTY_MSG_LENGTH 1
#define EMPTY_MSG_DEFINITION 0x10

#define RESULT_MSG_LENGTH 3
#define RESULT_MSG_DEFINITION 0x32

#define TIMEOUT_MSG_LENGTH 2
#define TIMEOUT_MSG_DEFINITION 0x51

typedef enum { WAIT_FOR_MSG, CHECK_MSG, WAIT_FOR_PAYLOAD, PROCESS_MSG, NUM_OF_STATES } state_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t* receivedData;
    uint8_t bytesReceived;
} data_t;

typedef state_t state_func_t(data_t* data);

state_t doStateWaitForMessage(data_t *data);
state_t doStateCheckMsg(data_t *data);
state_t doStateWaitForPayload(data_t *data);
state_t doStateProcessMsg(data_t *data);

state_func_t* const stateTable[NUM_OF_STATES] = {
    doStateWaitForMessage,
    doStateCheckMsg,
    doStateWaitForPayload,
    doStateProcessMsg
};

QueueHandle_t dataQueue;

/**
 * Call this to "send" data over the (simulated) serial interface.
 * @param message message buffer
 * @param length length of the message
 */
void send(uint8_t* message, uint32_t length);

/**
 * This will get called for each byte of data received.
 * @param data received byte
 */
void receive_ISR(uint8_t data) {
    if (xQueueSend(dataQueue, &data, 0) != pdPASS)
        console_print("ERROR: failed to add data to queue => data has been lost\n");
    if (uxQueueMessagesWaiting(dataQueue) > 1)
        console_print("WARN: data was received faster then it could be proccesed\n");
}

/**
 * Initialize challenge. This is called once on startup, before any interrupts can occur.
 */
void challenge_init() {
    dataQueue = xQueueCreate(2, 1);
    if (!dataQueue)
        console_print("ERROR: failed to create data queue\n");
}

/**
 * Main function for the coding challenge. Must not return.
 *
 * Please use the following API functions if necessary:
 *
 * print string to stdout
 * console_print("format %d", 123);
 *
 * millisecond delay
 * vTaskDelay(123);
 *
 * get elapsed milliseconds
 * TickType_t ms = xTaskGetTickCount();
 */
void challenge_run() {
    state_t cur_state = WAIT_FOR_MSG;
    data_t data;
    while (1) {
        cur_state = stateTable[cur_state](&data);
    }
}

void initializeDataForMsg(data_t* data) {
    uint8_t newData;
    while (xQueueReceive(dataQueue, &newData, portMAX_DELAY) != pdPASS);
    data->type = newData >> 4;
    data->length = newData & 0xF;
    if (!data->receivedData) {
        free(data->receivedData);
    }
    data->receivedData = malloc(data->length);
    data->bytesReceived = 0;
}

state_t doStateWaitForMessage(data_t* data) {
    initializeDataForMsg(data);
    return CHECK_MSG;
}

bool isValidMsgDefinition(data_t* data) {
    return (data->type == EMPTY && data->length == 0) 
        || (data->type == ADD && data->length == 4)
        || (data->type == DELAY && data->length == 3)
        || (data->type == LOG && data->length > 0);
}

state_t doStateCheckMsg(data_t* data) {
    if (isValidMsgDefinition(data)) {
        console_print("INFO: Receiving message of type: ");
        if (data->type == EMPTY) console_print("EMPTY\n");
        else if (data->type == ADD) console_print("ADD\n");
        else if (data->type == DELAY) console_print("DELAY\n");
        else if (data->type == LOG) console_print("LOG\n");
        return WAIT_FOR_PAYLOAD;
    } else {
        console_print("ERROR: received message definition which will not be handled: type=%d, length=%d\n",
                      data->type, data->length);
        return WAIT_FOR_MSG;
    }
}

state_t doStateWaitForPayload(data_t* data) {
    if (data->bytesReceived >= data->length) {
        return PROCESS_MSG;
    }
    uint8_t newData;
    while (xQueueReceive(dataQueue, &newData, 2) != pdPASS)
      console_print("WARN: waited too long for the payload. The current state might not be correct\n");
    *(data->receivedData + data->bytesReceived) = newData;
    ++data->bytesReceived;
    console_print("INFO: payload received: %d/%d\n", data->bytesReceived, data->length);
    return WAIT_FOR_PAYLOAD;
}

void sendEmpty() {
    uint8_t msg = EMPTY_MSG_DEFINITION;
    console_print("INFO: send message EMPTY\n");
    send(&msg, EMPTY_MSG_LENGTH);
}

void sendResult(data_t* data) {
    uint8_t* receivedData = data->receivedData;
    uint16_t result = (receivedData[0] + receivedData[2] << 8) + receivedData[1] + receivedData[3];
    uint8_t msg[] = { RESULT_MSG_DEFINITION, result >> 8, result & 0xF };
    console_print("INFO: send message RESULT\n");
    send(&msg[0], RESULT_MSG_LENGTH);
}

void vTimeoutCallback(TimerHandle_t xTimer) {
    uint8_t msg[] = { TIMEOUT_MSG_DEFINITION, *((uint8_t*) pvTimerGetTimerID(xTimer)) };
    console_print("INFO: send message TIMEOUT\n");
    send(&msg[0], TIMEOUT_MSG_LENGTH);
}

void sendTimeoutMsgAfterDelay(data_t* data) {
    uint16_t delay = (data->receivedData[0] << 8) + data->receivedData[1];
    console_print("INFO: delay for %dms\n", delay);
    TimerHandle_t xTimer = xTimerCreate("sendTimeoutTimer", pdMS_TO_TICKS(delay), pdFALSE, 
                                        &data->receivedData[2], vTimeoutCallback);
    if(xTimer != NULL)
        xTimerStart(xTimer, portMAX_DELAY);
}

void printLogMsg(data_t* data) {
    console_print("INFO: Received log message: %s\n", data->receivedData);
}

state_t doStateProcessMsg(data_t* data) {
    switch (data->type) {
       case EMPTY:
            sendEmpty();
            break;
        case ADD:
            sendResult(data);
            break;
        case DELAY:
            sendTimeoutMsgAfterDelay(data);
            break;
        case LOG:
            printLogMsg(data);
            break;
        default:
            console_print("ERROR: reached processMsg with unprocessable msg type\n");
    }
    return WAIT_FOR_MSG;
}
