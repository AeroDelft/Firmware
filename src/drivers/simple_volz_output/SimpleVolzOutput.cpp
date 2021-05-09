/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "SimpleVolzOutput.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

SimpleVolzOutput::SimpleVolzOutput() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _volz_connected_pub{ORB_ID(volz_connected), ORB_PRIO_HIGH},
    _volz_error_pub{ORB_ID(volz_error), ORB_PRIO_HIGH},
    _volz_outputs_pub{ORB_ID(volz_outputs), ORB_PRIO_HIGH}
{
    _volz_connected_pub.advertise();
    _volz_error_pub.advertise();
    _volz_outputs_pub.advertise();
}

SimpleVolzOutput::~SimpleVolzOutput()
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }

    _volz_connected_pub.unadvertise();
    _volz_error_pub.unadvertise();
    _volz_outputs_pub.unadvertise();

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SimpleVolzOutput::init()
{
	ScheduleOnInterval(loop_interval);

	return true;
}

void SimpleVolzOutput::init_fd()
{
    do { // create a scope to handle exit conditions using break

        _fd = ::open(_port, O_RDWR | O_NOCTTY);

        if (_fd < 0) {
            PX4_ERR("Error opening fd");
        }

        // baudrate 115200, 8 bits, no parity, 1 stop bit
        unsigned speed = B115200;
        termios uart_config{};
        int termios_state{};

        tcgetattr(_fd, &uart_config);

        // clear ONLCR flag (which appends a CR for every LF)
        uart_config.c_oflag &= ~ONLCR;

        // set baud rate
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d ISPD", termios_state);
            break;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d OSPD\n", termios_state);
            break;
        }

        if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
            PX4_ERR("baud %d ATTR", termios_state);
            break;
        }

        uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
        uart_config.c_cflag &= ~CSIZE;
        uart_config.c_cflag |= CS8;			// 8-bit characters
        uart_config.c_cflag &= ~PARENB;			// no parity bit
        uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
        uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

        // setup for non-canonical mode
        uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        uart_config.c_oflag &= ~OPOST;

        // fetch bytes as they become available
        uart_config.c_cc[VMIN] = 1;
        uart_config.c_cc[VTIME] = 1;

        if (_fd < 0) {
            PX4_ERR("FAIL: fd");
            break;
        }
    } while (0);
}

void SimpleVolzOutput::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

    // fds initialized?
    if (_fd < 0) {
        init_fd();
    }

    if (!waiting_for_resp) {  // if we are not currently waiting for a response, commands can be sent
        if (_armed.load()) {  // send the next control output if the driver is armed
            send_next_ctrl_cmd();

        } else {  // if the driver is not armed, handle CLI-related commands
            if (_checking_connected_servos.load()) {  // if a check for connected servos is being run, proceed
                uint8_t cmd[DATA_FRAME_SIZE];
                uint8_t id = current_id.load();
                set_id(id, id, cmd);  // send command to set ID to itself (does nothing, but initiates response)
                write_cmd(cmd);
                current_id.store(id + 1);

            } else {  // check for regular CLI commands
                uint8_t *cli_cmd = _cli_cmd.load();

                if (cli_cmd) { // if CLI commands are available, send them
                    write_cmd(cli_cmd);
                    _cli_cmd.store(nullptr);  // delete the command after sending
                }
            }
        }
    } else {  // if we are waiting for a response, check if one arrived
        check_for_resp();
    }

    // if we have been waiting for too long, move on and send an error report
    if (waiting_for_resp && hrt_elapsed_time(&last_cmd_time) > resp_timeout) {
        if (_armed.load() && last_cmd[0] == SET_EXTENDED_POS_CMD_CODE) {
            if (last_cmd[1] == 1) {
                _first_timestamp = hrt_absolute_time();
            }

            _outputs[last_cmd[1] - 1] = volz_error_s::TIMEOUT;

            if (last_cmd[1] >= NUM_SERVOS) {
                send_output_msg();
            }
        } else if (!_armed.load() && _checking_connected_servos.load() && last_cmd[0] == SET_ID_CMD_CODE) {
            if (last_cmd[1] == 1) {
                _first_timestamp = hrt_absolute_time();
            }

            _connected[last_cmd[1] - 1] = false;

            if (last_cmd[1] >= ID_MAX) {
                send_connected_msg();
            }
        }

        send_timeout_msg();
    }

    if (_armed.load() && !waiting_for_resp && last_cmd[1] >= NUM_SERVOS) {
        send_output_msg();
    } else if (!_armed.load() && !waiting_for_resp && last_cmd[1] >= ID_MAX && _checking_connected_servos.load()) {
        send_connected_msg();
    }

	perf_end(_loop_perf);
}

void SimpleVolzOutput::send_timeout_msg() {
    volz_error_s &report = _volz_error_pub.get();

    report.timestamp = hrt_absolute_time();
    report.type = volz_error_s::TIMEOUT;

    report.cmd_code = last_cmd[0];
    report.cmd_id = last_cmd[1];
    report.cmd_arg1 = last_cmd[2];
    report.cmd_arg2 = last_cmd[3];

    report.resp_code = 0;
    report.resp_id = 0;
    report.resp_arg1 = 0;
    report.resp_arg2 = 0;
    report.resp_data = 0;

    _volz_error_pub.update();

    _n_timeout++;

    waiting_for_resp = false;
}

// TODO: are the signs for the ailerons and elevators reversed in the right way?
void SimpleVolzOutput::mix(const float* control, uint16_t* pos)
{
    uint16_t volz_range = EXTENDED_POS_MAX - EXTENDED_POS_MIN;

    float aileron_offset = 18 / 90;  // TODO: check if this is applied correctly

    // aileron outputs
    float aileron_1_ctrl = apply_ctrl_offset(control[actuator_controls_s::INDEX_ROLL], aileron_offset);
    float aileron_2_ctrl = apply_ctrl_offset(-control[actuator_controls_s::INDEX_ROLL], -aileron_offset);
    pos[0] = (uint16_t)(EXTENDED_POS_MIN + (aileron_1_ctrl + 1) / 2 * volz_range);
    pos[1] = (uint16_t)(EXTENDED_POS_MIN + (1 + aileron_2_ctrl) / 2 * volz_range);

    // elevator outputs
    pos[2] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_PITCH] + 1) / 2 * volz_range);
    pos[3] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_PITCH] + 1) / 2 * volz_range);

    // rudder output
    pos[4] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_YAW] + 1) / 2 * volz_range);

    // wheel brake output
    pos[5] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_LANDING_GEAR] + 1) / 2 * volz_range);
}

float SimpleVolzOutput::apply_ctrl_offset(float control_val, float center_offset)
{
    if (control_val > 0) {
        return center_offset + control_val * (1 - center_offset);
    } else {
        return center_offset + control_val * (1 + center_offset);
    }
}

void SimpleVolzOutput::write_cmd(uint8_t* cmd)
{
    ::write(_fd, cmd, DATA_FRAME_SIZE);
    waiting_for_resp = true;
    last_cmd_time = hrt_absolute_time();

    last_cmd[0] = cmd[0];
    last_cmd[1] = cmd[1];
    last_cmd[2] = cmd[2];
    last_cmd[3] = cmd[3];
    last_cmd[4] = cmd[4];
    last_cmd[5] = cmd[5];
}

void SimpleVolzOutput::send_next_ctrl_cmd()
{
    // retrieve actuator controls
    _actuator_controls_sub.update();
    const actuator_controls_s &ctrl = _actuator_controls_sub.get();

    // mix the actuator controls
    uint16_t pos[NUM_SERVOS];
    mix(ctrl.control, pos);

    // send actuator command to the current servo
    uint8_t cmd[DATA_FRAME_SIZE];
    uint8_t id = current_id.load();
    set_extended_pos(id, pos[id - 1], cmd);
    write_cmd(cmd);

    current_id.store(id + 1);
}

void SimpleVolzOutput::send_output_msg() {
    volz_outputs_s &report = _volz_outputs_pub.get();

    report.first_timestamp = _first_timestamp;
    report.timestamp = hrt_absolute_time();  // TODO: maybe this timestamp should be set when the response is received

    report.n_outputs = NUM_SERVOS;

    for (int i = 0; i < NUM_SERVOS; i++) {
        report.outputs[i] = _outputs[i];
    }

    _volz_outputs_pub.update();

    current_id.store(1);
}

void SimpleVolzOutput::send_connected_msg() {
    volz_connected_s &report = _volz_connected_pub.get();

    report.first_timestamp = _first_timestamp;
    report.timestamp = hrt_absolute_time();  // TODO: maybe this timestamp should be set when the response is received

    for (int i = 0; i < ID_MAX; i++) {
        report.connected[i] = _connected[i];
    }

    _volz_connected_pub.update();

    _checking_connected_servos.store(false);
    current_id.store(1);
}

void SimpleVolzOutput::send_invalid_resp_msg(uint8_t *readbuf, uint16_t data) {
    volz_error_s &report = _volz_error_pub.get();

    report.timestamp = hrt_absolute_time();
    report.type = volz_error_s::INVALID_RESP;

    report.cmd_code = last_cmd[0];
    report.cmd_id = last_cmd[1];
    report.cmd_arg1 = last_cmd[2];
    report.cmd_arg2 = last_cmd[3];

    report.resp_code = readbuf[0];
    report.resp_id = readbuf[1];
    report.resp_arg1 = readbuf[2];
    report.resp_arg2 = readbuf[3];
    report.resp_data = data;

    _volz_error_pub.update();

    _n_invalid_resp++;
}

void SimpleVolzOutput::check_for_resp()
{
    // check how many bytes are available and return if there are less than 6
    int bytes_available = 0;
    ::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
    if (bytes_available < DATA_FRAME_SIZE) {
        return;
    }

    waiting_for_resp = false;  // we have received a response, so we are no longer waiting

    // read the response
    uint8_t readbuf[DATA_FRAME_SIZE];
    int ret = ::read(_fd, &readbuf[0], DATA_FRAME_SIZE);
    if (ret < 0) {
        PX4_ERR("read error: %d", ret);
        return;
    }

    // analyse the response
    bool valid = false;
    uint16_t data = 0;

    switch (readbuf[0]) {
        case SET_EXTENDED_POS_RESP_CODE:
            valid = valid_resp_set_extended_pos(readbuf, last_cmd);
            data = (readbuf[4] >> 8) + readbuf[5];

            // if this is a valid response and we are in armed mode, store the output for publication
            if (_armed.load()) {
                if (valid) {
                    _outputs[readbuf[1] - 1] = data;
                } else {
                    _outputs[readbuf[1] - 1] = volz_error_s::INVALID_RESP;
                }

                if (readbuf[1] == 1) {  // if this is the first servo in a series
                    _first_timestamp = hrt_absolute_time();
                }
            }

            break;

        case SET_ID_RESP_CODE:
            valid = valid_resp_set_id(readbuf, last_cmd);
            data = readbuf[4];

            if (_checking_connected_servos.load()) {  // if the response corresponds to a check for connection
                _connected[readbuf[1] - 1] = valid;

                if (readbuf[1] == 1) {  // if this is the first servo in a series
                    _first_timestamp = hrt_absolute_time();
                }
            }
            break;

        case SET_FAILSAFE_POS_RESP_CODE:
            valid = valid_resp_set_failsafe_pos(readbuf, last_cmd);
            data = (readbuf[4] >> 8) + readbuf[5];
            break;

        case SET_FAILSAFE_TIME_RESP_CODE:
            valid = valid_resp_set_failsafe_time(readbuf, last_cmd);
            data = readbuf[4];
            break;

        case SET_CURRENT_POS_AS_FAILSAFE_RESP_CODE:
            valid = valid_resp_set_current_pos_as_failsafe(readbuf, last_cmd);
            data = (readbuf[4] >> 8) + readbuf[5];
            break;

        case SET_CURRENT_POS_AS_ZERO_RESP_CODE:
            valid = valid_resp_set_current_pos_as_zero(readbuf, last_cmd);
            data = (readbuf[4] >> 8) + readbuf[5];
            break;

        case RESTORE_DEFAULTS_RESP_CODE:
            valid = valid_resp_restore_defaults(readbuf, last_cmd);
            data = 1;
            break;
    }

    if (!valid) {  // send out uORB message if an invalid response was received
        send_invalid_resp_msg(readbuf, data);
    }
}

void SimpleVolzOutput::send_cmd_threadsafe(uint8_t *cmd)
{
    _cli_cmd.store(cmd);
}

void SimpleVolzOutput::arm(bool armed) {
    if (!armed && _armed.load()) {  // delete old CLI commands before disarming
        send_cmd_threadsafe(nullptr);

    } else if (armed && !_armed.load()) {  // set current ID to 1 before arming
        current_id.store(1);
    }

    _armed.store(armed);
}

void SimpleVolzOutput::check_connected_servos() {
    if (!_armed.load()) {  // only allow for checking on servos if the driver is disarmed
        current_id.store(1);  // reset current ID to 1 to start at the beginning
        _checking_connected_servos.store(true);
    }
};

int SimpleVolzOutput::task_spawn(int argc, char *argv[])
{
    SimpleVolzOutput *instance = new SimpleVolzOutput();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SimpleVolzOutput::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	// TODO: maybe print the number of errors
	return 0;
}

// TODO: add support for more CLI commands
int SimpleVolzOutput::custom_command(int argc, char *argv[])
{
    const char *verb = argv[0];

    if (strcmp(verb, "set_pos") == 0) {
        if (argc < 2) {
            PX4_WARN("enter a position between %d and %d", EXTENDED_POS_MIN, EXTENDED_POS_MAX);
            return print_usage();
        }
        uint16_t pos = strtol(argv[1], 0, 0);
        if (pos < EXTENDED_POS_MIN || EXTENDED_POS_MAX < pos) {
            PX4_WARN("enter a position between %d and %d", EXTENDED_POS_MIN, EXTENDED_POS_MAX);
            return print_usage();
        }

        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 3) {
            id = strtol(argv[2], 0, 0);
        }

        set_extended_pos(id, pos, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "set_id") == 0) {
        if (argc < 2) {
            PX4_WARN("enter an ID between %d and %d", ID_MIN, ID_MAX);
            return print_usage();
        }
        uint8_t new_id = strtol(argv[1], 0, 0);
        if (new_id < ID_MIN || ID_MAX < new_id) {
            PX4_WARN("enter an ID between %d and %d", ID_MIN, ID_MAX);
            return print_usage();
        }

        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t old_id = ID_BROADCAST;

        if (argc >= 3) {
            old_id = strtol(argv[2], 0, 0);
        }

        set_id(old_id, new_id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "set_failsafe_pos") == 0) {
        if (argc < 2) {
            PX4_WARN("enter a position between %d and %d", EXTENDED_POS_MIN, EXTENDED_POS_MAX);
            return print_usage();
        }
        uint16_t pos = strtol(argv[1], 0, 0);
        if (pos < EXTENDED_POS_MIN || EXTENDED_POS_MAX < pos) {
            PX4_WARN("enter a position between %d and %d", EXTENDED_POS_MIN, EXTENDED_POS_MAX);
            return print_usage();
        }

        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 3) {
            id = strtol(argv[2], 0, 0);
        }

        set_failsafe_pos(id, pos, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "set_failsafe_time") == 0) {
        if (argc < 2) {
            PX4_WARN("enter a time between %d and %d", FAILSAFE_TIME_MIN, FAILSAFE_TIME_MAX);
            return print_usage();
        }
        uint8_t time = strtol(argv[1], 0, 0);
        if (time < FAILSAFE_TIME_MIN || FAILSAFE_TIME_MAX < time) {
            PX4_WARN("enter a time between %d and %d", FAILSAFE_TIME_MIN, FAILSAFE_TIME_MAX);
            return print_usage();
        }

        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 3) {
            id = strtol(argv[2], 0, 0);
        }

        set_failsafe_time(id, time, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "set_current_pos_as_failsafe") == 0) {
        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 2) {
            id = strtol(argv[1], 0, 0);
        }

        set_current_pos_as_failsafe(id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "set_current_pos_as_zero") == 0) {
        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 2) {
            id = strtol(argv[1], 0, 0);
        }

        set_current_pos_as_zero(id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "restore_defaults") == 0) {
        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 2) {
            id = strtol(argv[1], 0, 0);
        }

        restore_defaults(id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "arm") == 0) {
        if (get_instance()->_armed.load()) {
            PX4_INFO("already armed");
        }
        get_instance()->arm(true);

    } else if (strcmp(verb, "disarm") == 0) {
        if (!get_instance()->_armed.load()) {
            PX4_INFO("already disarmed");
        }
        get_instance()->arm(false);

    } else if (strcmp(verb, "check_connected") == 0) {
        get_instance()->check_connected_servos();
    } else {
        return print_usage("unknown command");
    }
    return 0;
}

// TODO: describe newly added CLI commands
int SimpleVolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
    // TODO: check the description of left vs right aileron given here
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver to send position commands for Volz DA22 actuators via the TELEM3 (UART/I2C) port.
The mixer is hard-coded to match the servo layout of AeroDelft's Phoenix aircraft.
The different control surfaces are identified by the ID of the corresponding servos. The assignment is as follows:
0x01 - left aileron
0x02 - right aileron
0x03 - elevator 1
0x04 - elevator 2
0x05 - rudder
0x06 - wheel brake

The actuator ID is often an optional argument for commands. If no ID is provided, the command is broadcasted to all servos.

)DESCR_STR"); // TODO: instead of broadcasting commands, should they be sent sequentially?

	PRINT_MODULE_USAGE_NAME("simple_volz_output", "template");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_pos", "Command actuator to a given position");
	PRINT_MODULE_USAGE_ARG("<pos>", "New commanded position", false);
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_id", "Set a new ID to an actuator");
	PRINT_MODULE_USAGE_ARG("<new_id>", "New actuator ID", false);
	PRINT_MODULE_USAGE_ARG("<old_id>", "Actuator ID", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("set_failsafe_pos", "Set actuator failsafe position");
    PRINT_MODULE_USAGE_ARG("<pos>", "Actuator's new failsafe position", false);
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_failsafe_time", "Set actuator failsafe timeout");
    PRINT_MODULE_USAGE_ARG("<time>", "Actuator's new failsafe timeout", false);
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_current_pos_as_failsafe", "Set an actuator's current position as its new failsafe position");
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("set_current_pos_as_zero", "Set an actuator's current position as its new mid-position (zero)");
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("restore_defaults", "Reset settings of an actuator to factory default values");
    PRINT_MODULE_USAGE_ARG("<id>", "Actuator ID", true);

    PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Activates position commands from flight controller and disables CLI commands");
    PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Deactivates position commands from flight controller and enables CLI commands");
    PRINT_MODULE_USAGE_COMMAND_DESCR("check_connected", "Sends a volz_connected message indicating which servos are connected");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int simple_volz_output_main(int argc, char *argv[])
{
	return SimpleVolzOutput::main(argc, argv);
}
