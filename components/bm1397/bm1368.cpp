#include <string.h>
#include <stdint.h>

#include "bm13xx.h"
#include "esp_log.h"
#include "serial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


const char* TAG="BM1368";

static const uint8_t chip_id[6] = {0xaa, 0x55, 0x13, 0x68, 0x00, 0x00};

BM1368::BM1368() : BM1366() {
}


uint8_t BM1368::get_job_id_from_result(uint8_t job_id) {
    return (job_id & 0xf0) >> 1;
}

uint8_t BM1368::next_job_id() {
    this->id = (this->id + 24) % 128;
    return this->id;
}

uint8_t BM1368::send_init(uint64_t frequency, uint16_t asic_count)
{

    //enable and set version rolling mask to 0xFFFF
    uint8_t init0[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA4, 0x90, 0x00, 0xFF, 0xFF, 0x1C};
    this->send_simple(init0, 11);

    //enable and set version rolling mask to 0xFFFF (again)
    uint8_t init1[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA4, 0x90, 0x00, 0xFF, 0xFF, 0x1C};
    this->send_simple(init1, 11);

    //enable and set version rolling mask to 0xFFFF (again)
    uint8_t init2[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA4, 0x90, 0x00, 0xFF, 0xFF, 0x1C};
    this->send_simple(init2, 11);

    //read register 00 on all chips (should respond AA 55 13 68 00 00 00 00 00 00 0F)
    uint8_t init3[7] = {0x55, 0xAA, 0x52, 0x05, 0x00, 0x00, 0x0A};
    this->send_simple(init3, 7);

    int chip_counter = 0;
    while (true) {
        if (SERIAL_rx(asic_response_buffer, 11, 1000) > 0) {
            if (!strncmp((char*) chip_id, (char*) asic_response_buffer, sizeof(chip_id))) {
                chip_counter++;
                ESP_LOGI(TAG, "found asic #%d", chip_counter);
            } else {
                ESP_LOGE(TAG, "unexpected response ... ignoring ...");
                ESP_LOG_BUFFER_HEX(TAG, asic_response_buffer, 11);
            }
        } else {
            break;
        }
    }
    ESP_LOGI(TAG, "%i chip(s) detected on the chain, expected %i", chip_counter, asic_count);

    //enable and set version rolling mask to 0xFFFF (again)
    uint8_t init4[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA4, 0x90, 0x00, 0xFF, 0xFF, 0x1C};
    this->send_simple(init4, 11);

    //Reg_A8
    uint8_t init5[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA8, 0x00, 0x07, 0x00, 0x00, 0x03};
    this->send_simple(init5, 11);

    //Misc Control
    uint8_t init6[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x18, 0xFF, 0x0F, 0xC1, 0x00, 0x00};
    this->send_simple(init6, 11);

    //chain inactive
    this->send_chain_inactive();
    // uint8_t init7[7] = {0x55, 0xAA, 0x53, 0x05, 0x00, 0x00, 0x03};
    // this->send_simple(init7, 7);


    uint8_t address_interval = 2;
    for (uint8_t i = 0; i < chip_counter; i++) {
        this->set_chip_address((uint8_t) (i * address_interval));
        // uint8_t init8[7] = {0x55, 0xAA, 0x40, 0x05, 0x00, 0x00, 0x1C};
        // this->send_simple(init8, 7);
    }

    //Core Register Control
    uint8_t init9[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x3C, 0x80, 0x00, 0x8B, 0x00, 0x12};
    this->send_simple(init9, 11);

    //Core Register Control
    uint8_t init10[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x3C, 0x80, 0x00, 0x80, 0x18, 0x1F};
    this->send_simple(init10, 11);

    //set ticket mask
    // uint8_t init11[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x14, 0x00, 0x00, 0x00, 0xFF, 0x08};
    // this->send_simple(init11, 11);
    this->set_job_difficulty_mask(BM136x_INITIAL_DIFFICULTY);

    //Analog Mux Control
    uint8_t init12[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x54, 0x00, 0x00, 0x00, 0x03, 0x1D};
    this->send_simple(init12, 11);

    //Set the IO Driver Strength on chip 00
    uint8_t init13[11] = {0x55, 0xAA, 0x51, 0x09, 0x00, 0x58, 0x02, 0x11, 0x11, 0x11, 0x06};
    this->send_simple(init13, 11);

    for (uint8_t i = 0; i < chip_counter; i++) {
        //Reg_A8
        uint8_t set_a8_register[6] = {(uint8_t) (i * address_interval), 0xA8, 0x00, 0x07, 0x01, 0xF0};
        this->send((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_a8_register, 6, BM13xx_SERIALTX_DEBUG);
        //Misc Control
        uint8_t set_18_register[6] = {(uint8_t) (i * address_interval), 0x18, 0xF0, 0x00, 0xC1, 0x00};
        this->send((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_18_register, 6, BM13xx_SERIALTX_DEBUG);
        //Core Register Control
        uint8_t set_3c_register_first[6] = {(uint8_t) (i * address_interval), 0x3C, 0x80, 0x00, 0x8B, 0x00};
        this->send((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_first, 6, BM13xx_SERIALTX_DEBUG);
        //Core Register Control
        uint8_t set_3c_register_second[6] = {(uint8_t) (i * address_interval), 0x3C, 0x80, 0x00, 0x80, 0x18};
        this->send((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_second, 6, BM13xx_SERIALTX_DEBUG);
        //Core Register Control
        uint8_t set_3c_register_third[6] = {(uint8_t) (i * address_interval), 0x3C, 0x80, 0x00, 0x82, 0xAA};
        this->send((TYPE_CMD | GROUP_SINGLE | CMD_WRITE), set_3c_register_third, 6, BM13xx_SERIALTX_DEBUG);
    }

    this->do_frequency_ramp_up();

    this->send_hash_frequency(frequency);

    //register 10 is still a bit of a mystery. discussion: https://github.com/skot/ESP-Miner/pull/167

    // uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x11, 0x5A}; //S19k Pro Default
    // uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x14, 0x46}; //S19XP-Luxos Default
    // uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x15, 0x1C}; //S19XP-Stock Default
    uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x00, 0x15, 0xA4}; //S21-Stock Default
    // uint8_t set_10_hash_counting[6] = {0x00, 0x10, 0x00, 0x0F, 0x00, 0x00}; //supposedly the "full" 32bit nonce range
    this->send((TYPE_CMD | GROUP_ALL | CMD_WRITE), set_10_hash_counting, 6, BM13xx_SERIALTX_DEBUG);

    return chip_counter;
}