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

void set_zero_pos(uint8_t id, uint8_t* cmd)
{
    cmd[0] = SET_ZERO_POS_CMD_CODE;
    cmd[1] = id;
    cmd[2] = SET_ZERO_POS_CMD_ARGS;
    cmd[3] = SET_ZERO_POS_CMD_ARGS;

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

void get_amps(uint8_t id, uint8_t* cmd)
{

}

void get_volts(uint8_t id, uint8_t* cmd)
{

}

void get_temp(uint8_t, uint8_t* cmd)
{

}

