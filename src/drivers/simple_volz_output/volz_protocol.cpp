#include "volz_protocol.h"

uint8_t high_byte(uint16_t val)
{
    return (val >> 8) & 0xff;
}

uint8_t low_byte(uint16_t val)
{
    return val & 0xff;
}

void add_crc(uint8_t* cmd)
{
    uint16_t crc = 0xFFFF; // init value of result
    uint8_t command[4]={cmd[0], cmd[1], cmd[2], cmd[3]}; // command code, ID, argument 1, argument 2
    int x, y;

    for(x = 0; x < 4; x++) {
        crc= ((command[x] << 8) ^ crc);
        for (y = 0; y < 8; y++)	{
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }

    cmd[4] = high_byte(crc);
    cmd[5] = low_byte(crc);
}

bool valid_crc(uint8_t* resp) {
    uint8_t resp_high_crc = resp[4];
    uint8_t resp_low_crc = resp[5];

    // create deep copy
    uint8_t resp_copy[DATA_FRAME_SIZE];
    resp_copy[0] = resp[0];
    resp_copy[1] = resp[1];
    resp_copy[2] = resp[2];
    resp_copy[3] = resp[3];

    add_crc(resp_copy);
    return resp_copy[4] == resp_high_crc && resp_copy[5] == resp_low_crc;
}

void set_extended_pos(uint8_t id, uint16_t pos, uint8_t* cmd)
{
    cmd[0] = SET_EXTENDED_POS_CMD_CODE;
    cmd[1] = id;
    cmd[2] = high_byte(pos);
    cmd[3] = low_byte(pos);

    add_crc(cmd);
}

void set_id(uint8_t old_id, uint8_t new_id, uint8_t* cmd)
{
    cmd[0] = SET_ID_CMD_CODE;
    cmd[1] = old_id;
    cmd[2] = new_id;
    cmd[3] = new_id;

    add_crc(cmd);
}

void set_failsafe_pos(uint8_t id, uint16_t pos, uint8_t* cmd)
{
    cmd[0] = SET_FAILSAFE_POS_CMD_CODE;
    cmd[1] = id;
    cmd[2] = high_byte(pos);
    cmd[3] = low_byte(pos);

    add_crc(cmd);
}

void set_failsafe_time(uint8_t id, uint8_t time, uint8_t* cmd)
{
    cmd[0] = SET_FAILSAFE_TIME_CMD_CODE;
    cmd[1] = id;
    cmd[2] = time;
    cmd[3] = time;

    add_crc(cmd);
}

void set_current_pos_as_failsafe(uint8_t id, uint8_t* cmd)
{
    cmd[0] = SET_CURRENT_POS_AS_FAILSAFE_CMD_CODE;
    cmd[1] = id;
    cmd[2] = SET_CURRENT_POS_AS_FAILSAFE_CMD_ARGS;
    cmd[3] = SET_CURRENT_POS_AS_FAILSAFE_CMD_ARGS;

    add_crc(cmd);
}

void set_current_pos_as_zero(uint8_t id, uint8_t* cmd)
{
    cmd[0] = SET_CURRENT_POS_AS_ZERO_CMD_CODE;
    cmd[1] = id;
    cmd[2] = SET_CURRENT_POS_AS_ZERO_CMD_ARGS;
    cmd[3] = SET_CURRENT_POS_AS_ZERO_CMD_ARGS;

    add_crc(cmd);
}

void restore_defaults(uint8_t id, uint8_t* cmd)
{
    cmd[0] = RESTORE_DEFAULTS_CMD_CODE;
    cmd[1] = id;
    cmd[2] = RESTORE_DEFAULTS_ARG1;
    cmd[3] = RESTORE_DEFAULTS_ARG2;

    add_crc(cmd);
}

bool valid_resp_set_extended_pos(uint8_t* resp, uint8_t* cmd) {
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_EXTENDED_POS_RESP_CODE && id_valid && resp[2] == cmd[2] && resp[3] == cmd[3] && valid_crc(resp);
}

bool valid_resp_set_id(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_ID_RESP_CODE && id_valid && resp[2] == cmd[2] && resp[3] == cmd[3] && valid_crc(resp);
}

bool valid_resp_set_failsafe_pos(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_FAILSAFE_POS_RESP_CODE && id_valid && resp[2] == cmd[2] && resp[3] == cmd[3] && valid_crc(resp);
}

bool valid_resp_set_failsafe_time(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_FAILSAFE_TIME_RESP_CODE && id_valid && resp[2] == cmd[2] && resp[3] == cmd[3] && valid_crc(resp);
}

bool valid_resp_set_current_pos_as_failsafe(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_CURRENT_POS_AS_FAILSAFE_RESP_CODE && id_valid && valid_crc(resp);
}

bool valid_resp_set_current_pos_as_zero(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == SET_CURRENT_POS_AS_ZERO_RESP_CODE && id_valid && valid_crc(resp);
}

bool valid_resp_restore_defaults(uint8_t* resp, uint8_t* cmd)
{
    bool id_valid = resp[1] == cmd[1] || cmd[1] == ID_BROADCAST;
    return resp[0] == RESTORE_DEFAULTS_RESP_CODE && id_valid && resp[2] == cmd[2] && resp[3] == cmd[3] && valid_crc(resp);
}
