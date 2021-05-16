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

#pragma once

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/getopt.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/volz_connected.h>
#include <uORB/topics/volz_error.h>
#include <uORB/topics/volz_outputs.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <termios.h>

#include "volz_protocol.h"

// TODO: delete the other volz module and rename this to simply volz. Also make sure it is started automatically

class SimpleVolzOutput : public ModuleBase<SimpleVolzOutput>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    SimpleVolzOutput();
	~SimpleVolzOutput() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	static const int NUM_SERVOS = 6;
	static const int SERVO_UPDATE_FREQ = 50; // Hz

private:
	void Run() override;

    uORB::PublicationMultiData<volz_connected_s> _volz_connected_pub{ORB_ID(volz_connected)};
    uORB::PublicationMultiData<volz_error_s> _volz_error_pub{ORB_ID(volz_error)};
    uORB::PublicationMultiData<volz_outputs_s> _volz_outputs_pub{ORB_ID(volz_outputs)};

    uORB::SubscriptionData<actuator_controls_s> _actuator_controls_sub{ORB_ID(actuator_controls_0)};

	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	int _fd{-1};
	const char *_port = "/dev/ttyS3";  // TODO: don't forget to switch back to TELEM2
	void init_fd();

	void mix(const float* control, uint16_t* pos);
	float apply_ctrl_offset(float control_val, float center_offset);
	void send_next_ctrl_cmd();
	void write_cmd(uint8_t* cmd);
	void check_for_resp();

	px4::atomic<uint8_t> current_id{1};
    bool waiting_for_resp{false};
    uint8_t last_cmd[DATA_FRAME_SIZE];
    hrt_abstime last_cmd_time{0};

    uint64_t loop_interval = 1000000/SERVO_UPDATE_FREQ/NUM_SERVOS;  // microseconds
    uint64_t resp_timeout = loop_interval * 2;  // microseconds

    px4::atomic<uint8_t*> _cli_cmd{nullptr};
    void send_cmd_threadsafe(uint8_t* cmd);

    px4::atomic<bool> _armed{false};
    void arm(bool armed);

    bool _connected[ID_MAX];
    px4::atomic<bool> _checking_connected_servos{false};
    void check_connected_servos();
    void send_connected_msg();

    // TODO: double-check if we are at risk of an overflow error here
    px4::atomic<uint32_t> _n_valid_resp{0};
    px4::atomic<uint32_t> _n_invalid_resp{0};
    px4::atomic<uint32_t> _n_timeout{0};

    uint64_t _first_timestamp{0};
    uint16_t _outputs[NUM_SERVOS];
    void send_output_msg();

    void send_timeout_msg();
    void send_invalid_resp_msg(uint8_t *readbuf, uint16_t data);
};
