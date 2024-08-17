#include "bm1366.h"

#include "crc.h"
#include "global_state.h"
#include "serial.h"
#include "utils.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bm13xx.h"

const char *TAG = "BM13xx";

BM13xx::BM13xx() {
    id = 0;
    memset(&this->result, 0, sizeof(task_result));
    memset(this->active_jobs, 0, sizeof(this->active_jobs));
    memset(this->valid_jobs, 0, sizeof(this->valid_jobs));
}

uint16_t BM13xx::reverse_uint16(uint16_t num)
{
    return (num >> 8) | (num << 8);
}

uint32_t BM13xx::reverse_uint32(uint32_t val)
{
    return ((val >> 24) & 0xff) |      // Move byte 3 to byte 0
           ((val << 8) & 0xff0000) |   // Move byte 1 to byte 2
           ((val >> 8) & 0xff00) |     // Move byte 2 to byte 1
           ((val << 24) & 0xff000000); // Move byte 0 to byte 3
}

void BM13xx::send(uint8_t header, uint8_t * data, uint8_t data_len, bool debug) {
packet_type_t packet_type = (header & TYPE_JOB) ? JOB_PACKET : CMD_PACKET;
    uint8_t total_length = (packet_type == JOB_PACKET) ? (data_len + 6) : (data_len + 5);

    // allocate memory for buffer
    uint8_t *buf = (uint8_t*) malloc(total_length);

    // add the preamble
    buf[0] = 0x55;
    buf[1] = 0xAA;

    // add the header field
    buf[2] = header;

    // add the length field
    buf[3] = (packet_type == JOB_PACKET) ? (data_len + 4) : (data_len + 3);

    // add the data
    memcpy(buf + 4, data, data_len);

    // add the correct crc type
    if (packet_type == JOB_PACKET) {
        uint16_t crc16_total = crc16_false(buf + 2, data_len + 2);
        buf[4 + data_len] = (crc16_total >> 8) & 0xFF;
        buf[5 + data_len] = crc16_total & 0xFF;
    } else {
        buf[4 + data_len] = crc5(buf + 2, data_len + 2);
    }

    // send serial data
    SERIAL_send(buf, total_length, debug);

    free(buf);
}

void BM13xx::send_simple(uint8_t * data, uint8_t total_length)
{
    uint8_t *buf = (uint8_t*) malloc(total_length);
    memcpy(buf, data, total_length);
    SERIAL_send(buf, total_length, BM13xx_SERIALTX_DEBUG);
    free(buf);
}

void BM13xx::send_read_address(void)
{
    uint8_t read_address[2] = {0x00, 0x00};
    // send serial data
    this->send((TYPE_CMD | GROUP_ALL | CMD_READ), read_address, 2, BM13xx_SERIALTX_DEBUG);
}

// Baud formula = 25M/((denominator+1)*8)
// The denominator is 5 bits found in the misc_control (bits 9-13)
int BM13xx::set_default_baud(void)
{
    // default divider of 26 (11010) for 115,749
    uint8_t baudrate[9] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001}; // baudrate - misc_control
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, BM1937_SERIALTX_DEBUG);
    return 115749;
}

void BM13xx::set_chip_address(uint8_t chipAddr)
{

    uint8_t read_address[2] = {chipAddr, 0x00};
    // send serial data
    this->send((TYPE_CMD | GROUP_SINGLE | CMD_SETADDRESS), read_address, 2, BM13xx_SERIALTX_DEBUG);
}

void BM13xx::send_chain_inactive(void)
{
    uint8_t read_address[2] = {0x00, 0x00};
    // send serial data
    this->send((TYPE_CMD | GROUP_ALL | CMD_INACTIVE), read_address, 2, BM1937_SERIALTX_DEBUG);
}

void BM13xx::set_job_difficulty_mask(int difficulty)
{

    // Default mask of 256 diff
    uint8_t job_difficulty_mask[9] = {0x00, TICKET_MASK, 0b00000000, 0b00000000, 0b00000000, 0b11111111};

    // The mask must be a power of 2 so there are no holes
    // Correct:  {0b00000000, 0b00000000, 0b11111111, 0b11111111}
    // Incorrect: {0b00000000, 0b00000000, 0b11100111, 0b11111111}
    difficulty = _largest_power_of_two(difficulty) - 1; // (difficulty - 1) if it is a pow 2 then step down to second largest for more hashrate sampling

    // convert difficulty into char array
    // Ex: 256 = {0b00000000, 0b00000000, 0b00000000, 0b11111111}, {0x00, 0x00, 0x00, 0xff}
    // Ex: 512 = {0b00000000, 0b00000000, 0b00000001, 0b11111111}, {0x00, 0x00, 0x01, 0xff}
    for (int i = 0; i < 4; i++)
    {
        char value = (difficulty >> (8 * i)) & 0xFF;
        // The char is read in backwards to the register so we need to reverse them
        // So a mask of 512 looks like 0b00000000 00000000 00000001 1111111
        // and not 0b00000000 00000000 10000000 1111111

        job_difficulty_mask[5 - i] = _reverse_bits(value);
    }

    ESP_LOGI(TAG, "Setting job ASIC mask to %d", difficulty);

    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), job_difficulty_mask, 6, BM1937_SERIALTX_DEBUG);
}

void BM13xx::reset(void)
{
    gpio_set_level(BM13XX_RST_PIN, 0);

    // delay for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // set the gpio pin high
    gpio_set_level(BM13XX_RST_PIN, 1);

    // delay for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

}
