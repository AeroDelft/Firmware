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

    // close the fd
    ::close(_fd);
    _fd = -1;

	ScheduleOnInterval(1000_us); // 1000 us interval, 1000 Hz rate

	return true;
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
        // open fd
        _fd = ::open(_port, O_RDWR | O_NOCTTY);
    }

    // send test command to the TELEM2 port
    uint8_t cmd[VOLZ_CMD_LEN];
    pos_cmd(MIN_VALUE + 3 / 4 * (MAX_VALUE - MIN_VALUE), cmd);
    ::write(_fd, cmd, VOLZ_CMD_LEN);

	// Example
	// grab latest accelerometer data
	_sensor_accel_sub.update();
	const sensor_accel_s &accel = _sensor_accel_sub.get();


	// Example
	// publish some data
	orb_test_s data{};
	data.timestamp = hrt_absolute_time();
	data.val = accel.device_id;
	data.val = cmd[0];
	_orb_test_pub.publish(data);


	perf_end(_loop_perf);
}

int SimpleVolzOutput::highbyte(int value) {
    return (value >> 8) & 0xff;
}

int SimpleVolzOutput::lowbyte(int value) {
    return value & 0xff;
}

int SimpleVolzOutput::generate_crc(int cmd, int actuator_id, int arg_1, int arg_2)	{
    unsigned short int crc=0xFFFF; // init value of result
    int command[4]={cmd,actuator_id,arg_1,arg_2}; // command, ID, argument1, argument 2
    int x,y;
    for(x=0; x<4; x++)	{
        crc= ( ( command[x] <<8 ) ^ crc);

        for ( y=0; y<8; y++ )	{

            if ( crc & 0x8000 )
                crc = (crc << 1) ^ 0x8005;

            else
                crc = crc << 1;

        }
    }

    return crc;
}

void SimpleVolzOutput::pos_cmd(int pos, uint8_t* cmd) {
    int arg = VOLZ_POS_MIN + 2 * (VOLZ_POS_CENTER - VOLZ_POS_MIN) * pos / (MAX_VALUE - MIN_VALUE);
    uint8_t arg_1 = highbyte(arg);
    uint8_t arg_2 = lowbyte(arg);
    int crc_cmd = generate_crc(POS_CMD, VOLZ_ID_UNKNOWN, arg_1, arg_2);

    cmd[0] = POS_CMD;
    cmd[1] = VOLZ_ID_UNKNOWN;
    cmd[2] = arg_1;
    cmd[3] = arg_2;
    cmd[4] = highbyte(crc_cmd);
    cmd[5] = lowbyte(crc_cmd);
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
	return print_usage("unknown command");
}

int SimpleVolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue. Modified to send position commands for a Volz servo via the TELEM2 port.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simple_volz_output", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int simple_volz_output_main(int argc, char *argv[])
{
	return SimpleVolzOutput::main(argc, argv);
}
