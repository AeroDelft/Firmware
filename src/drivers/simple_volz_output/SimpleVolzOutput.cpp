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
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

SimpleVolzOutput::~SimpleVolzOutput()
{
    if (_fd != -1) {
        ::close(_fd);
        _fd = -1;
    }
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SimpleVolzOutput::init()
{

	ScheduleOnInterval(1000_us); // 1000 us interval, 1000 Hz rate

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
            PX4_ERR("FAIL: laser fd");
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

    update_outputs();
    update_telemetry();

	perf_end(_loop_perf);
}

void SimpleVolzOutput::mix(const float* control, uint16_t* pos)
{
    uint16_t volz_range = EXTENDED_POS_MAX - EXTENDED_POS_MIN;

    // aileron outputs
    pos[0] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_ROLL] + 1) / 2 * volz_range);
    pos[1] = (uint16_t)(EXTENDED_POS_MIN + (1 - control[actuator_controls_s::INDEX_ROLL]) / 2 * volz_range);

    // elevator outputs
    pos[2] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_PITCH] + 1) / 2 * volz_range);
    pos[3] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_PITCH] + 1) / 2 * volz_range);

    // rudder output
    pos[4] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_YAW] + 1) / 2 * volz_range);

    // wheel brake output
    pos[5] = (uint16_t)(EXTENDED_POS_MIN + (control[actuator_controls_s::INDEX_LANDING_GEAR] + 1) / 2 * volz_range);
}

void SimpleVolzOutput::update_outputs()
{
    uint8_t *cli_cmd = _cli_cmd.load();

    // if CLI commands are available, send them
    if (cli_cmd) {
        ::write(_fd, cli_cmd, DATA_FRAME_SIZE);
        _cli_cmd.store(nullptr);  // delete previous command
    }

    // if there are no CLI commands, send position commands from flight controller
    else {
        // retrieve actuator controls
        _actuator_controls_sub.update();
        const actuator_controls_s &ctrl = _actuator_controls_sub.get();

        // mix the actuator controls
        uint16_t pos[NUM_SERVOS];
        mix(ctrl.control, pos);

        // send all actuator commands to the respective servos
        for (int i = 0; i < NUM_SERVOS; i++) {
            uint8_t cmd[DATA_FRAME_SIZE];
            set_extended_pos(i + 1, pos[i], cmd);
            ::write(_fd, cmd, DATA_FRAME_SIZE);
        }
    }
}

void SimpleVolzOutput::update_telemetry()
{
    int bytes_available = 0;
    ::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);
    if (!bytes_available) {
        return;
    }

    do {
        uint8_t readbuf[DATA_FRAME_SIZE];
        int ret = ::read(_fd, &readbuf[0], DATA_FRAME_SIZE);
        if (ret < 0) {
            PX4_ERR("read error: %d", ret);
            break;
        }

        PX4_INFO("received telemetry");


    } while (bytes_available > 0);
}

void SimpleVolzOutput::send_cmd_threadsafe(uint8_t *cmd)
{
    _cli_cmd.store(cmd);

    while (get_instance()->_cli_cmd.load()) {
        px4_usleep(1000);
    }
}

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
	return 0;
}

int SimpleVolzOutput::custom_command(int argc, char *argv[])
{
    const char *verb = argv[0];
    PX4_INFO("argc: %d", argc);

    if (strcmp(verb, "set_id") == 0) {
        if (argc < 2) {
            PX4_WARN("enter an ID between %d and %d", ID_MIN, ID_MAX);
            return print_usage();
        }

        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t old_id = ID_BROADCAST;
        uint8_t new_id = strtol(argv[1], 0, 0);

        PX4_INFO("new id: %d", new_id);

        if (new_id < ID_MIN || ID_MAX < new_id) {
            PX4_WARN("enter an ID between %d and %d", ID_MIN, ID_MAX);
            return print_usage();
        }
        if (argc >= 3) {
            old_id = strtol(argv[2], 0, 0);
        }

        set_id(old_id, new_id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else if (strcmp(verb, "reset") == 0) {
        uint8_t cmd[DATA_FRAME_SIZE];
        uint8_t id = ID_BROADCAST;

        if (argc >= 2) {
            id = strtol(argv[1], 0, 0);
        }

        restore_defaults(id, cmd);
        get_instance()->send_cmd_threadsafe(cmd);

    } else {
        return print_usage("unknown command");
    }
    return 0;
}

int SimpleVolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver to send position commands for Volz DA22 actuators via the TELEM3 (UART/I2C) port.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simple_volz_output", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_id", "Set a new ID");
	PRINT_MODULE_USAGE_ARG("<new_id>", "New actuator ID", false);
	PRINT_MODULE_USAGE_ARG("<old_id>", "ID of actuator to be set", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset settings to factory default values");
    PRINT_MODULE_USAGE_ARG("<id>", "ID of actuator to be reset", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int simple_volz_output_main(int argc, char *argv[])
{
	return SimpleVolzOutput::main(argc, argv);
}
