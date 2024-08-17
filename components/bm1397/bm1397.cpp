#include <string.h>
#include <stdint.h>
#include <math.h>

#include "bm13xx.h"
#include "esp_log.h"
#include "serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const char* TAG="BM1397";

BM1397::BM1397() : BM13xx() {
    this->prev_nonce = 0;
}


uint8_t BM1397::init(uint64_t frequency, uint16_t asic_count) {
    ESP_LOGI(TAG, "Initializing BM1397");

    memset(asic_response_buffer, 0, sizeof(asic_response_buffer));

    //esp_rom_gpio_pad_select_gpio(BM1397_RST_PIN);
    gpio_pad_select_gpio(BM13XX_RST_PIN);
    gpio_set_direction(BM13XX_RST_PIN, GPIO_MODE_OUTPUT);

    // reset the bm1397
    this->reset();

    return this->send_init(frequency, asic_count);
}


int BM1397::set_max_baud() {
    // divider of 0 for 3,125,000
    ESP_LOGI(TAG, "Setting max baud of 3125000");
    uint8_t baudrate[9] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01100000, 0b00110001};
    ; // baudrate - misc_control
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, BM13xx_SERIALTX_DEBUG);
    return 3125000;
}

void BM1397::send_work(void * pvParameters, bm_job * next_bm_job) {

    bm1397_job_packet job;
    // max job number is 128
    // there is still some really weird logic with the job id bits for the asic to sort out
    // so we have it limited to 128 and it has to increment by 4
    id = (id + 4) % 128;

    job.job_id = id;
    job.num_midstates = next_bm_job->num_midstates;
    memcpy(&job.starting_nonce, &next_bm_job->starting_nonce, 4);
    memcpy(&job.nbits, &next_bm_job->target, 4);
    memcpy(&job.ntime, &next_bm_job->ntime, 4);
    memcpy(&job.merkle4, next_bm_job->merkle_root + 28, 4);
    memcpy(job.midstate, next_bm_job->midstate, 32);

    if (job.num_midstates == 4)
    {
        memcpy(job.midstate1, next_bm_job->midstate1, 32);
        memcpy(job.midstate2, next_bm_job->midstate2, 32);
        memcpy(job.midstate3, next_bm_job->midstate3, 32);
    }

    if (this->active_jobs[job.job_id] != NULL)
    {
        free_bm_job(this->active_jobs[job.job_id]);
    }

    this->active_jobs[job.job_id] = next_bm_job;

    this->valid_jobs[job.job_id] = 1;
    // ESP_LOGI(TAG, "Added Job: %i", job.job_id);

    this->send((TYPE_JOB | GROUP_SINGLE | CMD_WRITE), (uint8_t*) &job, sizeof(bm1397_job_packet), BM13xx_DEBUG_WORK);
}

uint8_t BM1397::send_init(uint64_t frequency, uint16_t asic_count) {
    // send the init command
    this->send_read_address();

    int chip_counter = 0;
    while (true) {
        if (SERIAL_rx(asic_response_buffer, 11, 1000) > 0) {
            chip_counter++;
        } else {
            break;
        }
    }
    ESP_LOGI(TAG, "%i chip(s) detected on the chain, expected %i", chip_counter, asic_count);

    // send serial data
    vTaskDelay(SLEEP_TIME / portTICK_PERIOD_MS);
    this->send_chain_inactive();

    // split the chip address space evenly
    for (uint8_t i = 0; i < asic_count; i++) {
        this->set_chip_address(i * (256 / asic_count));
    }

    uint8_t init[6] = {0x00, CLOCK_ORDER_CONTROL_0, 0x00, 0x00, 0x00, 0x00}; // init1 - clock_order_control0
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init, 6, BM13xx_SERIALTX_DEBUG);

    uint8_t init2[6] = {0x00, CLOCK_ORDER_CONTROL_1, 0x00, 0x00, 0x00, 0x00}; // init2 - clock_order_control1
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init2, 6, BM13xx_SERIALTX_DEBUG);

    uint8_t init3[9] = {0x00, ORDERED_CLOCK_ENABLE, 0x00, 0x00, 0x00, 0x01}; // init3 - ordered_clock_enable
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init3, 6, BM13xx_SERIALTX_DEBUG);

    uint8_t init4[9] = {0x00, CORE_REGISTER_CONTROL, 0x80, 0x00, 0x80, 0x74}; // init4 - init_4_?
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init4, 6, BM13xx_SERIALTX_DEBUG);

    this->set_job_difficulty_mask(256);

    uint8_t init5[9] = {0x00, PLL3_PARAMETER, 0xC0, 0x70, 0x01, 0x11}; // init5 - pll3_parameter
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init5, 6, BM13xx_SERIALTX_DEBUG);

    uint8_t init6[9] = {0x00, FAST_UART_CONFIGURATION, 0x06, 0x00, 0x00, 0x0F}; // init6 - fast_uart_configuration
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), init6, 6, BM13xx_SERIALTX_DEBUG);

    this->set_default_baud();

    this->send_hash_frequency(frequency);

    return chip_counter;
}

bm1397_asic_result *BM1397::receive_work(void)
{

    // wait for a response, wait time is pretty arbitrary
    int received = SERIAL_rx(asic_response_buffer, 9, 60000);

    if (received < 0)
    {
        ESP_LOGI(TAG, "Error in serial RX");
        return NULL;
    }
    else if (received == 0)
    {
        // Didn't find a solution, restart and try again
        return NULL;
    }

    if (received != 9 || asic_response_buffer[0] != 0xAA || asic_response_buffer[1] != 0x55)
    {
        ESP_LOGI(TAG, "Serial RX invalid %i", received);
        ESP_LOG_BUFFER_HEX(TAG, asic_response_buffer, received);
        return NULL;
    }

    return (bm1397_asic_result*) asic_response_buffer;
}

task_result *BM1397::process_work(void *pvParameters) {
    bm1397_asic_result *asic_result = this->receive_work();

    if (asic_result == NULL)
    {
        ESP_LOGI(TAG, "return null");
        return NULL;
    }

    uint8_t nonce_found = 0;
    uint32_t first_nonce = 0;

    uint8_t rx_job_id = asic_result->job_id & 0xfc;
    uint8_t rx_midstate_index = asic_result->job_id & 0x03;

    if (this->valid_jobs[rx_job_id] == 0)
    {
        ESP_LOGI(TAG, "Invalid job nonce found, id=%d", rx_job_id);
        return NULL;
    }

    uint32_t rolled_version = this->active_jobs[rx_job_id]->version;
    for (int i = 0; i < rx_midstate_index; i++)
    {
        rolled_version = increment_bitmask(rolled_version, this->active_jobs[rx_job_id]->version_mask);
    }

    // ASIC may return the same nonce multiple times
    // or one that was already found
    // most of the time it behavies however
    if (nonce_found == 0)
    {
        first_nonce = asic_result->nonce;
        nonce_found = 1;
    }
    else if (asic_result->nonce == first_nonce)
    {
        // stop if we've already seen this nonce
        return NULL;
    }

    if (asic_result->nonce == this->prev_nonce)
    {
        return NULL;
    }
    else
    {
        prev_nonce = asic_result->nonce;
    }

    this->result.job_id = rx_job_id;
    this->result.nonce = asic_result->nonce;
    this->result.rolled_version = rolled_version;

    return &this->result;
}
