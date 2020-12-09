/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file MCP9808.hpp
 *
 * Driver for the MCP9808 temperature sensor connected via I2C. Inspired by the
 * MPL3115A2 barometer driver.
 * Note: the driver can currently be started more than once on the same I2C
 * address. This will simply create duplicate publications
 */

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_temp.h>

#define MCP9808_BASE_ADDRESS       0x18

#define MCP9808_REG_AMB_TEMP       0x05
#define MCP9808_REG_DEV_ID         0x07
#define MCP9808_DEV_ID             0x04

#define TEMP_BASE_DEVICE_PATH      "/dev/temp"

#define MCP9808_CONVERSION_INTERVAL	10000	/* microseconds */


class MCP9808 : public device::I2C, public I2CSPIDriver<MCP9808>
{
public:
    MCP9808(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = MCP9808_BASE_ADDRESS);
    ~MCP9808() override;

    static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance);
    static void print_usage();


    int init() override;
    int probe() override;

    void print_status();

    void RunImpl();
private:

    void start();

    int measure();
    int collect();

    uORB::PublicationMultiData<sensor_temp_s>	_sensor_temp_pub;

    int			_class_device_instance{-1};

    bool _collect_phase{false};

    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;
};

MCP9808::MCP9808(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
        I2C(DRV_TEMP_DEVTYPE_MCP9808, MODULE_NAME, bus, address, bus_frequency),
        I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
        _sensor_temp_pub{ORB_ID(sensor_temp), ORB_PRIO_DEFAULT},
        _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
        _measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
        _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
    _class_device_instance = register_class_devname(TEMP_BASE_DEVICE_PATH);
    _sensor_temp_pub.advertise();

    _sensor_temp_pub.get().device_id = get_device_id();
}

MCP9808::~MCP9808()
{
    if (_class_device_instance != -1) {
        unregister_class_devname(TEMP_BASE_DEVICE_PATH, _class_device_instance);
    }

    _sensor_temp_pub.unadvertise();

    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);
}

int MCP9808::init()
{
    int ret = I2C::init();

    if (ret != PX4_OK) {
        PX4_ERR("init failed");
        return PX4_ERROR;
    }

    start();

    return PX4_OK;
}

int MCP9808::probe()
{
    _retries = 10;
    uint8_t val[2] {};

    // Read the device ID of the connected sensor, and make sure it matches with the MCP9808
    uint8_t reg = MCP9808_REG_DEV_ID;
    int ret = transfer(&reg, 1, &val[0], 2);

    if ((ret == PX4_OK) && (val[0] == MCP9808_DEV_ID)) {
        /*
         * Disable retries; we may enable them selectively in some cases,
         * but the device gets confused if we retry some of the commands.
         */
        _retries = 0;
        return PX4_OK;
    }

    return -EIO;
}

void MCP9808::start()
{
    /* reset the report ring and state machine */
    _collect_phase = false;

    /* schedule a cycle to start things */
    ScheduleNow();
}

void MCP9808::RunImpl()
{
    int ret = PX4_ERROR;

    /* collection phase? */
    if (_collect_phase) {

        /* perform collection */
        ret = collect();

        if (ret == -EIO) {  // TODO: Find a way to deal with this
            return;
        }

        if (ret == -EAGAIN) {
            /* Ready read it on next cycle */
            ScheduleDelayed(MCP9808_CONVERSION_INTERVAL);

            return;
        }

        /* next phase is measurement */
        _collect_phase = false;
    }

    /* Look for a ready condition */
    ret = measure();

    if (ret == -EIO) {  // TODO: Find a way to deal with this
        return;
    }

    /* next phase is measurement */
    _collect_phase = true;

    /* schedule a fresh cycle call when the measurement is done */
    ScheduleDelayed(MCP9808_CONVERSION_INTERVAL);
}

int MCP9808::measure()
{
    perf_begin(_measure_perf);
    /*
     * Disable retries on this command; we can't know whether failure
     * means the device did or did not see the command.
     */
    _retries = 0;

    // Select the temperature register so subsequent read commands do not
    // have to specify it
    uint8_t val[2] = {0, 0};
    uint8_t reg = MCP9808_REG_AMB_TEMP;
    int ret = transfer(&reg, 1, &val[0], 2);

    if (ret == -EIO) {
        perf_count(_comms_errors);
    }

    perf_end(_measure_perf);

    return PX4_OK;
}

int MCP9808::collect()
{
    perf_begin(_sample_perf);

    /* read the most recent measurement
     * 2 bytes for temperature
     */
    uint8_t	val[2] {};
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    int ret = transfer(nullptr, 0, &val[0], 2);

    if (ret == -EIO) {
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return ret;
    }

    uint16_t raw = val[0] << 8 | val[1];
    float temp = raw & 0x0FFF;
    temp = (float) (temp / (float) 16.0);
    if (raw & 0x1000) temp -= 256;

    sensor_temp_s &report = _sensor_temp_pub.get();

    report.timestamp_sample = timestamp_sample;
    report.temperature = temp;
    report.timestamp = hrt_absolute_time();
    _sensor_temp_pub.update();

    perf_end(_sample_perf);

    return PX4_OK;
}

void MCP9808::print_status()
{
    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}

void
MCP9808::print_usage()
{
    PRINT_MODULE_USAGE_NAME("mcp9808", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("temp");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(MCP9808_BASE_ADDRESS);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *MCP9808::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance)
{
    MCP9808 *dev = new MCP9808(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

    if (dev == nullptr) {
        PX4_ERR("alloc failed");
        return nullptr;
    }

    if (OK != dev->init()) {
        delete dev;
        return nullptr;
    }

    return dev;
}

extern "C" int mcp9808_main(int argc, char *argv[])
{
    using ThisDriver = MCP9808;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 400000;
    cli.i2c_address = MCP9808_BASE_ADDRESS; // This is not in MPL3115A2

    const char *verb = cli.parseDefaultArguments(argc, argv);

    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BARO_DEVTYPE_MPL3115A2);

    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();
    return -1;
}
