/*
 * Copyright (C) 2019-21 Intel Corporation and others.  All rights reserved.
 * SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wasm_export.h"
#include "bh_platform.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define IWASM_MAIN_STACK_SIZE 5120
#else
#define IWASM_MAIN_STACK_SIZE 8192
#endif

#define LOG_TAG "wamr"
#define UART_TAG "uart"

/**
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_TXD 1 //(CONFIG_CONSOLE_UART_NUM)
#define ECHO_TEST_RXD 3 //(CONFIG_CONSOLE_UART_NUM)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (CONFIG_CONSOLE_UART_NUM)
#define ECHO_UART_BAUD_RATE (CONFIG_CONSOLE_UART_BAUDRATE)
#define UART_TASK_STACK_SIZE (CONFIG_MAIN_TASK_STACK_SIZE)

#define WASM_ENABLE_INTERP 1

int STX = 2; // start of text

unsigned char wasm_data[1024];
int wasm_size = 0;
bool wasm_loaded = false;

#define BUF_SIZE (1024)

/******************************************************/

static void *
uart_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_REF_TICK,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0,
                                        NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD,
                                 ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1),
                                  20 / portTICK_PERIOD_MS);

        if (len) {
            data[len] = '\0';
            ESP_LOGI(UART_TAG, "Recieved data: %s", (char *)data);

            int data_index = 0;
            bool load = false;
            int wasm_index = 0;

            while (wasm_loaded == false) {
                if (data_index == len) {
                    if (load) {
                        wasm_loaded = true;
                        wasm_size = wasm_index;
                    }

                    break;
                }
                if (data[data_index] == STX) {
                    if (data[data_index + 1] == 'w'
                        && data[data_index + 2] == 'a'
                        && data[data_index + 3] == 's'
                        && data[data_index + 4] == 'm'
                        && data[data_index + 5] == 0x1F) {
                        load = true;
                        data_index = data_index + 6;
                        continue;
                    }
                }

                if (load) {
                    wasm_data[wasm_index] = data[data_index];
                    wasm_index++;
                }

                data_index++;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/******************************************************/

static void *
app_instance_main(wasm_module_inst_t module_inst)
{
    const char *exception;

    wasm_application_execute_main(module_inst, 0, NULL);
    if ((exception = wasm_runtime_get_exception(module_inst)))
        printf("%s\n", exception);
    return NULL;
}

/******************************************************/

void *
iwasm_main(void *arg)
{
    while (1) {
        /* setup variables for instantiating and running the wasm module */
        uint8_t *wasm_file_buf = NULL;
        unsigned wasm_file_buf_size = 0;
        wasm_module_t wasm_module = NULL;
        wasm_module_inst_t wasm_module_inst = NULL;
        char error_buf[128];
        void *ret;
        RuntimeInitArgs init_args;

        /* configure memory allocation */
        memset(&init_args, 0, sizeof(RuntimeInitArgs));
#if WASM_ENABLE_GLOBAL_HEAP_POOL == 0
        init_args.mem_alloc_type = Alloc_With_Allocator;
        init_args.mem_alloc_option.allocator.malloc_func = (void *)os_malloc;
        init_args.mem_alloc_option.allocator.realloc_func = (void *)os_realloc;
        init_args.mem_alloc_option.allocator.free_func = (void *)os_free;
#else
#error The usage of a global heap pool is not implemented yet for esp-idf.
#endif

        ESP_LOGI(LOG_TAG, "Initialize WASM runtime");
        /* initialize runtime environment */
        if (!wasm_runtime_full_init(&init_args)) {
            ESP_LOGE(LOG_TAG, "Init runtime failed.");
            return NULL;
        }

        ESP_LOGI(LOG_TAG, "Run wamr with interpreter");

        if (wasm_loaded) {
            ESP_LOGI(LOG_TAG, "Loading wasm from UART data");

            wasm_file_buf = wasm_data;
            wasm_file_buf_size = wasm_size;
            wasm_loaded = false;
        }
        else {
            ESP_LOGI(LOG_TAG, "Waiting for wasm binary");
        }

        /* load WASM module */
        if (!(wasm_module = wasm_runtime_load(wasm_file_buf, wasm_file_buf_size,
                                              error_buf, sizeof(error_buf)))) {
            ESP_LOGE(LOG_TAG, "Error in wasm_runtime_load: %s", error_buf);
            goto fail1interp;
        }

        ESP_LOGI(LOG_TAG, "Instantiate WASM runtime");
        if (!(wasm_module_inst =
                  wasm_runtime_instantiate(wasm_module, 1 * 1024, // stack size
                                           1 * 1024,              // heap size
                                           error_buf, sizeof(error_buf)))) {
            ESP_LOGE(LOG_TAG, "Error while instantiating: %s", error_buf);
            goto fail2interp;
        }

        ESP_LOGI(LOG_TAG, "run main() of the application in a loop");

        while (wasm_loaded == false) {
            ret = app_instance_main(wasm_module_inst);
            assert(!ret);
            for (int i = 0; i < 5; i++) {
                if (wasm_loaded) {
                    break;
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        /* destroy the module instance */
        ESP_LOGI(LOG_TAG, "Deinstantiate WASM runtime");
        wasm_runtime_deinstantiate(wasm_module_inst);

    fail2interp:
        /* unload the module */
        ESP_LOGI(LOG_TAG, "Unload WASM module");
        wasm_runtime_unload(wasm_module);

    fail1interp:

        /* destroy runtime environment */
        ESP_LOGI(LOG_TAG, "Destroy WASM runtime");
        wasm_runtime_destroy();
    }

    return NULL;
}

/******************************************************/

void
app_main(void)
{
    pthread_t wasm_exec_thread, uart_thread;
    pthread_attr_t tattr;
    int res;

    // wait for the scheduler to start, works without it, but first ESP_LOGI
    // about received data is broken
    vTaskDelay(100);

    // start uart task
    pthread_attr_init(&tattr);
    pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&tattr, UART_TASK_STACK_SIZE);
    res = pthread_create(&uart_thread, &tattr, uart_task, (void *)NULL);
    assert(res == 0);

    // wait for the first wasm binary to be loaded
    while (!wasm_loaded) {
        vTaskDelay(100);
    }

    // start wasm execution task
    pthread_attr_init(&tattr);
    pthread_attr_setdetachstate(&tattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setstacksize(&tattr, IWASM_MAIN_STACK_SIZE);
    res = pthread_create(&wasm_exec_thread, &tattr, iwasm_main, (void *)NULL);
    assert(res == 0);

    while (1) {
        vTaskDelay(100);
    }

    ESP_LOGI(LOG_TAG, "Exiting...");
}
