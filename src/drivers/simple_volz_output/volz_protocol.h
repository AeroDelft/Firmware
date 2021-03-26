#pragma once

constexpr uint8_t DATA_FRAME_SIZE = 6;

constexpr uint8_t SET_EXTENDED_POS_CMD_CODE = 0xDC;
constexpr uint8_t SET_EXTENDED_POS_RESP_CODE = 0x2C;
constexpr uint16_t EXTENDED_POS_MIN = 0x0060;
constexpr uint16_t EXTENDED_POS_MAX = 0x1F9F;

constexpr uint8_t SET_ID_CMD_CODE = 0xAA;
constexpr uint8_t SET_ID_RESP_CODE = 0x55;
constexpr uint8_t ID_MIN = 0x01;
constexpr uint8_t ID_MAX = 0x1E;
constexpr uint8_t ID_BROADCAST = 0x1F;

constexpr uint8_t SET_FAILSAFE_POS_CMD_CODE = 0xBB;
constexpr uint8_t SET_FAILSAFE_POS_RESP_CODE = 0x5D;

constexpr uint8_t SET_FAILSAFE_TIME_CMD_CODE = 0xCC;
constexpr uint8_t SET_FAILSAFE_TIME_RESP_CODE = 0x66;
constexpr uint8_t FAILSAFE_TIME_NEVER = 0x00;
constexpr uint8_t FAILSAFE_TIME_MIN = 0x01;
constexpr uint8_t FAILSAFE_TIME_MAX = 0x7F;
constexpr float FAILSAFE_TIME_FACTOR = 0.1;  // converts value to s

constexpr uint8_t SET_ZERO_POS_CMD_CODE = 0x99;
constexpr uint8_t SET_ZERO_POS_CMD_ARGS = 0x00;
constexpr uint8_t SET_ZERO_POS_RESP_CODE = 0x4C;

constexpr uint8_t RESTORE_DEFAULTS_CMD_CODE = 0xB4;
constexpr uint8_t RESTORE_DEFAULTS_RESP_CODE = 0x5A;
constexpr uint8_t RESTORE_DEFAULTS_ARG1 = 0x41;
constexpr uint8_t RESTORE_DEFAULTS_ARG2 = 0x53;

constexpr uint8_t GET_AMPS_CMD_CODE = 0xB0;
constexpr uint8_t GET_AMPS_CMD_ARGS = 0x00;
constexpr uint8_t GET_AMPS_RESP_CODE = 0x30;
constexpr float GET_AMPS_FACTOR = 0.008;  // converts value to A

constexpr uint8_t GET_VOLTS_CMD_CODE = 0xB1;
constexpr uint8_t GET_VOLTS_CMD_ARGS = 0x00;
constexpr uint8_t GET_VOLTS_RESP_CODE = 0x31;
constexpr float GET_VOLTS_FACTOR = 0.165;  // converts value to V

constexpr uint8_t GET_TEMP_CMD_CODE = 0xB1;
constexpr uint8_t GET_TEMP_CMD_ARGS = 0x00;
constexpr uint8_t GET_TEMP_RESP_CODE = 0x31;
constexpr float GET_TEMP_FACTOR = 0.634;  // converts value to °C
constexpr float GET_TEMP_OFFSET = -50;

uint8_t high_byte(uint16_t val);
uint8_t low_byte(uint16_t val);
void add_crc(uint8_t* cmd);

void set_extended_pos(uint8_t id, uint16_t pos, uint8_t* cmd);
void set_id(uint8_t old_id, uint8_t new_id, uint8_t* cmd);
void set_failsafe_pos(uint8_t id, uint16_t pos, uint8_t* cmd);
void set_failsafe_time(uint8_t id, uint8_t time, uint8_t* cmd);
void set_zero_pos(uint8_t id, uint8_t* cmd);
void restore_defaults(uint8_t id, uint8_t* cmd);
void get_amps(uint8_t id, uint8_t* cmd);
void get_volts(uint8_t id, uint8_t* cmd);
void get_temp(uint8_t, uint8_t* cmd);
