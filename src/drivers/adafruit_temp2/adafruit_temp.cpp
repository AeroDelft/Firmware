/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file adafruit_temp.cpp
 *
 * @author Jakob Schoser <jakob.schoser@aerodelft.nl>
 *
 * Driver for the Adafruit temperature sensor.
 * Default I2C address 0x66 is used.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <uORB/topics/adafruit_temp.h>
#include <uORB/PublicationMulti.hpp>

/* Configuration Constants */
#define AdafruitTemp_BASEADDR		0b11000
#define AdafruitTemp_READ_CMD       0x05

// TODO: Change DRV_DIST_DEVTYPE_SF1XX

class AdafruitTemp : public device::I2C, public I2CSPIDriver<AdafruitTemp>
{
public:
	AdafruitTemp(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = AdafruitTemp_BASEADDR);

	~AdafruitTemp() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_status() override;

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void RunImpl();

private:
	int probe() override; // TODO: This is public in MPL3115A2

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address The I2C bus address to probe.
	* @return True if the device is present.
	*/
	int probe_address(uint8_t address); // TODO: This line is not in MPL3115A2

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	int measure();
	int collect();

	bool _sensor_ok{false}; // TODO: This line is not in MPL3115A2

	int _conversion_interval{-1}; // TODO: This line is not in MPL3115A2
	int _measure_interval{0}; // TODO: This line is not in MPL3115A2

    uORB::Publication<adafruit_temp_s> _adafruit_temp_pub{ORB_ID(adafruit_temp)}; // TODO: This line is not in MPL3115A2

    // TODO: MPL3115A2 doesn't pass arguments to these, and has one for _measure_perf
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};

	// TODO: MPL3115A2 has PX4Barometer and _collect_phase
};

AdafruitTemp::AdafruitTemp(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(DRV_DIST_DEVTYPE_SF1XX, MODULE_NAME, bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)
	// TODO: MPL3115A2 has the perf counters are _px4_barometers here
{
}

AdafruitTemp::~AdafruitTemp()
{
    _adafruit_temp_pub.unadvertise(); // TODO: This line is not in MPL3115A2

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	// TODO: MPL3115A2 frees _measure_perf here as well
}

int AdafruitTemp::init()
{
    PX4_INFO("init");
	int ret = PX4_ERROR;
	// param_get(param_find("SENS_EN_SF1XX"), &hw_model); // TODO: Fix parameter

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	// Select altitude register TODO: These lines are not in MPL3115A2
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
	}

    // TODO: MPL3115A2 calls start from here

	return ret;
}

int AdafruitTemp::probe()
{
    PX4_INFO("probe");
	return measure();
}

int AdafruitTemp::measure()
{
    PX4_INFO("measure");
	/*
	 * Send the command '0' -- read altitude
	 */
	uint8_t cmd = AdafruitTemp_READ_CMD;
    uint8_t val[2] = {0, 0};
	int ret = transfer(&cmd, 1, &val, 2);
    PX4_INFO("first byte received %d", val[0]);
    PX4_INFO("second byte %d", val[1]);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	ret = OK;

	return ret;
}

int AdafruitTemp::collect()
{
    PX4_INFO("collect");
	/* read from the sensor */
	perf_begin(_sample_perf);
	uint8_t val[2] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (transfer(nullptr, 0, &val[0], 2) < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

    uint16_t raw = val[0] << 8 | val[1];
    float temp = raw & 0x0FFF;
    temp = (float) (temp / (float) 16.0);
    if (raw & 0x1000) temp -= 256;

    adafruit_temp_s report{};
    report.timestamp = timestamp_sample;
    report.temp = temp;  // temporary thing just to test

    /* publish it, if we are the primary */
    _adafruit_temp_pub.publish(report);

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

	return PX4_OK;
}

void AdafruitTemp::start()
{
    PX4_INFO("start");
	if (_measure_interval == 0) {
		_measure_interval = _conversion_interval;
	}

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	ScheduleDelayed(_conversion_interval);
}

void AdafruitTemp::RunImpl()
{
    PX4_INFO("run");
	/* Collect results */
	if (OK != collect()) {
		PX4_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_conversion_interval);
}

void AdafruitTemp::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void AdafruitTemp::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Adafruit temperature sensor.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("adafruit_temp", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("temperature_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0b11000);
    PRINT_MODULE_USAGE_COMMAND("set_address");
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0b11000);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *AdafruitTemp::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
    PX4_INFO("instantiate");
	AdafruitTemp* instance = new AdafruitTemp(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start();  // TODO: This line not in MPL3115A2
	return instance;
}

extern "C" __EXPORT int adafruit_temp_main(int argc, char *argv[])
{
    PX4_INFO("main");
	using ThisDriver = AdafruitTemp;
	BusCLIArguments cli{true, false};
	cli.i2c_address = AdafruitTemp_BASEADDR;  // TODO: This line not in MPL3115A2
	cli.default_i2c_frequency = 400000;

    const char *verb = cli.parseDefaultArguments(argc, argv);

    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_SF1XX);

	if (!strcmp(verb, "start")) {
	    PX4_INFO("start called");
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
	    PX4_INFO("stop called");
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
	    PX4_INFO("status called");
		return ThisDriver::module_status(iterator);
	}

	PX4_INFO("failed main");
	ThisDriver::print_usage();
	return -1;
}
