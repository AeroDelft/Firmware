/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_arch/dshot.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/esc_status.h>


using namespace time_literals; // TODO: what does this do?

#if !defined(BOARD_HAS_PWM) // TODO: what does this do?
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif


class VolzOutput : public cdev::CDev, public ModuleBase<VolzOutput>, public OutputModuleInterface
{
public:
	VolzOutput();
	virtual ~VolzOutput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();


	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	void mixerChanged() override;

	typedef enum{
        POS_CMD = 0xDD,
        SET_ACTUATOR_ID = 0xAA,
        SET_FAILSAFE_TIMEOUT = 0xCC,
        SET_CURRENT_POS_AS_FAILSAFE_POS = 0xBC,
        SET_CURRENT_POS_AS_ZERO = 0x99
	} volz_command_t;

	/**
	 * Send a volz command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param actuator_id id or 0x1F for all
	 * @return 0 on success, <0 error otherwise
	 */
	int sendCommandThreadSafe(volz_command_t command, int num_repetitions, int actuator_id);

private:

	void Run() override;

	static constexpr uint16_t DISARMED_VALUE = 0;
    static constexpr uint16_t MIN_VALUE = -1;
    static constexpr uint16_t MAX_VALUE = 1;

	struct Command {
		volz_command_t command;
		int num_repetitions{0};
		uint8_t motor_mask{0xff}; // TODO: what does this do?

		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

	MixingOutput _mixing_output{DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	uORB::Subscription _param_sub{ORB_ID(parameter_update)};

	Command _current_command;
	px4::atomic<Command *> _new_command{nullptr};

	unsigned	_num_outputs{0};
	int		_class_instance{-1}; // TODO: what does this do?

    // TODO: what does this do?
	bool		_outputs_on{false};
	uint32_t	_output_mask{0};
	bool		_outputs_initialized{false};

	perf_counter_t	_cycle_perf;

	void		update_params();

    int			pwm_ioctl(file *filp, int cmd, unsigned long arg);
    int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	VolzOutput(const VolzOutput &) = delete;
	VolzOutput operator=(const VolzOutput &) = delete;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VOLZ_CONFIG>) _param_volz_config // TODO: replace by actual parameters
	)
};

DShotOutput::DShotOutput() :
	CDev("/dev/volz"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllDisarmedValues(DISARMED_VALUE);
	_mixing_output.setAllMinValues(MIN_VALUE);
	_mixing_output.setAllMaxValues(MAX_VALUE);

}

DShotOutput::~DShotOutput()
{
	/* make sure outputs are off */
	// TODO: Disarm all servos

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance); // TODO: what does this do?

	perf_free(_cycle_perf);
}

int
DShotOutput::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH); // TODO: what does this do?

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return 0;
}

int
VolzOutput::task_spawn(int argc, char *argv[])
{
	VolzOutput *instance = new VolzOutput();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int VolzOutput::sendCommandThreadSafe(volz_command_t command, int num_repetitions, int actuator_id)
{
	Command cmd;
	cmd.command = command;

	if (actuator_id == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << _mixing_output.reorderedMotorIndex(actuator_id);
	}

	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	// wait until main thread processed it
	while (_new_command.load()) {
		px4_usleep(1000);
	}

	return 0;
}

void VolzOutput::mixerChanged()
{
	// This shouldn't happen. Do nothing
}

bool VolzOutput::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
    // TODO: Rewrite all this
	if (!_outputs_on) {
		return false;
	}

	int requested_telemetry_index = -1;

	if (_telemetry) {
		// check for an ESC info request. We only process it when we're not expecting other telemetry data
		if (_request_esc_info.load() != nullptr && !_waiting_for_esc_info && stop_motors
		    && !_telemetry->handler.expectingData() && !_current_command.valid()) {
			requested_telemetry_index = requestESCInfo();

		} else {
			requested_telemetry_index = _mixing_output.reorderedMotorIndex(_telemetry->handler.getRequestMotorIndex());
		}
	}

	if (stop_motors) {

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);
			}
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;
		}

	} else {
		for (int i = 0; i < (int)num_outputs; i++) {
			if (outputs[i] == DISARMED_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);

			} else {
				up_dshot_motor_data_set(i, math::min(outputs[i], (uint16_t)DSHOT_MAX_THROTTLE), i == requested_telemetry_index);
			}
		}

		// clear commands when motors are running
		_current_command.clear();
	}

	if (stop_motors || num_control_groups_updated > 0) {
		up_dshot_trigger();
	}

	return true;
}

void
VolzOutput::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	/* update output status if armed or if mixer is loaded */
	bool outputs_on = _mixing_output.armed().armed || _mixing_output.mixers();

	if (_param_sub.updated()) {
		update_params();
	}

	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void VolzOutput::update_params()
{
    // TODO: Make this function actually do something useful, i.e. come up with parameters for this driver
	parameter_update_s pupdate;
	_param_sub.update(&pupdate);

	updateParams();
}


int
VolzOutput::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	/* try it as a Capture ioctl next */
	ret = capture_ioctl(filp, cmd, arg);
    if (ret != -ENOTTY) {
        return ret;
    }

    ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int
VolzOutput::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("volz ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case PWM_SERVO_GET_COUNT:
        *(unsigned *)arg = 5;
		break;

	case PWM_SERVO_SET_COUNT: {
			/* change the number of outputs that are enabled for
			 * PWM. This is used to change the split between GPIO
			 * and PWM under control of the flight config
			 * parameters.
			 * TODO: Implement this if you want
			 */
			break;
		}

	case PWM_SERVO_SET_MODE: {
            // TODO: Implement this if you want
			break;
		}

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int
VolzOutput::capture_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = -ENOTTY;
	// TODO: Find out what this method is good for in dshot
	return ret;
}

int VolzOutput::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 10) - 1;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	struct Command {
		const char *name;
		volz_command_t command;
		int num_repetitions;
	};

	constexpr Command commands[] = {
		{"pos_cmd", volz_command_t::POS_CMD, 10},
		{"set_id", volz_command_t::SET_ACTUATOR_ID, 10},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			return get_instance()->sendCommandThreadSafe(commands[i].command, commands[i].num_repetitions, motor_index);
		}
	}

	if (!is_running()) {
		int ret = VolzOutput::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int VolzOutput::print_status()
{
	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}

int VolzOutput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the Volz output driver. It is similar to the DShot output driver, from which it borrows heavily.

It supports:
- sending Volz commands via CLI
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("volz", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_STRING('p', "/dev/ttyS2", "<device>", "UART device", true);
    // TODO: Add more command descriptions
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int volz_main(int argc, char *argv[])
{
	return VolzOutput::main(argc, argv);
}
