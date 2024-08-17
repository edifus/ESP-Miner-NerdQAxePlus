#pragma once

#include <stdint.h>
#include "common.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "mining.h"

#define BM13xx_SERIALTX_DEBUG true
#define BM13xx_DEBUG_WORK true
#define BM136x_INITIAL_DIFFICULTY 512

#define BM13XX_RST_PIN GPIO_NUM_1

#define TYPE_JOB 0x20
#define TYPE_CMD 0x40

#define GROUP_SINGLE 0x00
#define GROUP_ALL 0x10

#define CMD_JOB 0x01

#define CMD_SETADDRESS 0x00
#define CMD_WRITE 0x01
#define CMD_READ 0x02
#define CMD_INACTIVE 0x03

#define RESPONSE_CMD 0x00
#define RESPONSE_JOB 0x80

#define SLEEP_TIME 20
#define FREQ_MULT 25.0

#define CLOCK_ORDER_CONTROL_0 0x80
#define CLOCK_ORDER_CONTROL_1 0x84
#define ORDERED_CLOCK_ENABLE 0x20
#define CORE_REGISTER_CONTROL 0x3C
#define PLL3_PARAMETER 0x68
#define FAST_UART_CONFIGURATION 0x28
#define TICKET_MASK 0x14
#define MISC_CONTROL 0x18

#define CHUNK_SIZE 20


/*struct asic_result {
    virtual ~asic_result() = default;
};*/
struct asic_result {

};

class BM13xx {
protected:
    uint8_t asic_response_buffer[CHUNK_SIZE];
    task_result result;
    uint8_t id;
    bm_job *active_jobs[256];
    uint8_t valid_jobs[256];

public:
    BM13xx();
    virtual uint8_t init(uint64_t frequency, uint16_t asic_count) = 0;
    virtual task_result *process_work(void *pvParameters) = 0;
    virtual int set_max_baud() = 0;
    virtual void send_work(void * pvParameters, bm_job * next_bm_job) = 0;
    void set_job_difficulty_mask(int difficulty);

protected:
    virtual uint8_t send_init(uint64_t frequency, uint16_t asic_count) = 0;
    virtual void do_frequency_ramp_up() = 0;
    virtual void send_hash_frequency(float frequency) = 0;
    virtual void reset();
    virtual asic_result *receive_work();

    void send(uint8_t header, uint8_t * data, uint8_t data_len, bool debug);
    void send_simple(uint8_t * data, uint8_t total_length);
    void send_read_address(void);
    void send_chain_inactive();
    void set_chip_address(uint8_t chipAddr);
    int set_default_baud(void);
    uint16_t reverse_uint16(uint16_t num);
    uint32_t reverse_uint32(uint32_t val);
};

typedef struct __attribute__((__packed__)) : public asic_result
{
    uint8_t preamble[2];
    uint32_t nonce;
    uint8_t midstate_num;
    uint8_t job_id;
    uint8_t crc;
} bm1397_asic_result;

typedef struct __attribute__((__packed__))
{
    uint8_t job_id;
    uint8_t num_midstates;
    uint8_t starting_nonce[4];
    uint8_t nbits[4];
    uint8_t ntime[4];
    uint8_t merkle4[4];
    uint8_t midstate[32];
    uint8_t midstate1[32];
    uint8_t midstate2[32];
    uint8_t midstate3[32];
} bm1397_job_packet;

class BM1397: public BM13xx {
public:
    BM1397();
    uint8_t init(uint64_t frequency, uint16_t asic_count);
    task_result *process_work(void *pvParameters);
    int set_max_baud();
    void send_work(void * pvParameters, bm_job * next_bm_job);

protected:
    uint8_t send_init(uint64_t frequency, uint16_t asic_count);
    void do_frequency_ramp_up();
    void send_hash_frequency(float frequency);
    void reset();
    bm1397_asic_result *receive_work();

private:
    uint32_t prev_nonce;
};

typedef struct __attribute__((__packed__)) : public asic_result
{
    uint8_t preamble[2];
    uint32_t nonce;
    uint8_t midstate_num;
    uint8_t job_id;
    uint16_t version;
    uint8_t crc;
} bm136x_asic_result;

typedef struct __attribute__((__packed__))
{
    uint8_t job_id;
    uint8_t num_midstates;
    uint8_t starting_nonce[4];
    uint8_t nbits[4];
    uint8_t ntime[4];
    uint8_t merkle_root[32];
    uint8_t prev_block_hash[32];
    uint8_t version[4];
} BM136x_job;

class BM1366: public BM13xx {
public:
    BM1366();
    uint8_t init(uint64_t frequency, uint16_t asic_count);
    task_result *process_work(void *pvParameters);
    int set_max_baud();
    void send_work(void * pvParameters, bm_job * next_bm_job);

protected:
    uint8_t send_init(uint64_t frequency, uint16_t asic_count);
    void do_frequency_ramp_up();
    void send_hash_frequency(float frequency);
    void reset();
    bm136x_asic_result *receive_work();

    virtual uint8_t get_job_id_from_result(uint8_t job_id);
    virtual uint8_t next_job_id();

};

class BM1368: public BM1366 {
public:
    BM1368();
protected:
    uint8_t send_init(uint64_t frequency, uint16_t asic_count);
    bm136x_asic_result *receive_work();

    virtual uint8_t get_job_id_from_result(uint8_t job_id);
    virtual uint8_t next_job_id();
};