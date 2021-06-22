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
 * @file XEN5320.hpp
 * @author Tim Rotmans
 *
 * Driver for the XEN5320 hydrogen leakage sensors. The sensor will be read out by an Arduino, the Arduino will be connected to PX4 via I2C.
 * Inspired by Jakob's MCP9808 driver.
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
#include <uORB/topics/sensor_hydrogen.h>

#define XEN5320_BASE_ADDRESS       0x27 // The arduino joins the bus with specified adress: 0x27

#define MCP9808_REG_AMB_TEMP       0x05 // Why 4 addresses?
#define MCP9808_REG_DEV_ID         0x07 //
#define MCP9808_DEV_ID             0x04 //

#define TEMP_BASE_DEVICE_PATH      "/dev/temp" //

#define XEN5320_CONVERSION_INTERVAL	10000	/* microseconds */


class XEN5320 : public device::I2C, public I2CSPIDriver<MCP9808> //
{
public:
    XEN5320(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = XEN5320_BASE_ADDRESS); // declare XEN5320_BASE_ADDRESS as default?
    ~XEN5320() override; // why do we override the destructor?

    static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance);
    static void print_usage();


    int init() override; // wat doet deze?
    int probe() override; // en deze?

    void print_status();

    void RunImpl();
private:

    void start();

    int measure();
    int collect();

    uORB::PublicationMultiData<sensor_hydrogen_s>	_sensor_hydrogen_pub;

    int			_class_device_instance{-1};

    bool _collect_phase{false};

    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;
};

XEN5320::XEN5320(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
        I2C(DRV_TEMP_DEVTYPE_MCP9808, MODULE_NAME, bus, address, bus_frequency),
        I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
        _sensor_hydrogen_pub{ORB_ID(sensor_hydrogen), ORB_PRIO_DEFAULT}, // snap niet helemaal wat hier gebeurt. is dit onderdeel v constructor definition?
        _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
        _measure_perf(perf_alloc(PC_ELAPSED, MO1DULE_NAME": measure")),
        _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
    _class_device_instance = register_class_devname(TEMP_BASE_DEVICE_PATH); // moet ik nog aanpassen, maar wat is dit path precies?
    _sensor_hydrogen_pub.advertise();

    _sensor_hydrogen_pub.get().device_id = get_device_id();
}

XEN5320::~XEN5320()
{
    if (_class_device_instance != -1) {
        unregister_class_devname(TEMP_BASE_DEVICE_PATH, _class_device_instance); // same here, what exactly is this path?
    }

    _sensor_hydrogen_pub.unadvertise();

    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);
}

int XEN5320::init() // only error message?
{
    int ret = I2C::init();

    if (ret != PX4_OK) {
        PX4_ERR("init failed");
        return PX4_ERROR;
    }

    start();

    return PX4_OK;
}

int XEN5320::probe()
{
    _retries = 10;
    uint8_t val[2] {};

    // Read the device ID of the connected sensor, and make sure it matches with the MCP9808
    uint8_t reg = MCP9808_REG_DEV_ID; // CHANGE!
    int ret = transfer(&reg, 1, &val[0], 2);

    if ((ret == PX4_OK) && (val[0] == MCP9808_DEV_ID)) { // CHANGE!
        /*
         * Disable retries; we may enable them selectively in some cases,
         * but the device gets confused if we retry some of the commands.
         */
        _retries = 0;
        return PX4_OK;
    }

    return -EIO;
}

void XEN5320::start()
{
    /* reset the report ring and state machine */
    _collect_phase = false;

    /* schedule a cycle to start things */
    ScheduleNow();
}

void XEN5320::RunImpl()
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
            ScheduleDelayed(XEN5320_CONVERSION_INTERVAL);

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
    ScheduleDelayed(XEN5320_CONVERSION_INTERVAL); // leave it at 10000?
}

int XEN5320::measure()
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
    uint8_t reg = MCP9808_REG_AMB_TEMP; // CHANGE!
    int ret = transfer(&reg, 1, &val[0], 2);

    if (ret == -EIO) {
        perf_count(_comms_errors);
    }

    perf_end(_measure_perf);

    return PX4_OK;
}

int XEN5320::collect()
{
    perf_begin(_sample_perf);

    /* read the most recent measurement
     * 2 bytes for temperature
     */

    // not sure yet for the h2 sensors, probably 2x2 bytes (2 sensors). Collect them in buffer?
    // uint8_t	val[2] {};
    // const hrt_abstime timestamp_sample = hrt_absolute_time();
    // int ret = transfer(nullptr, 0, &val[0], 2);

    // if (ret == -EIO) {
    //     perf_count(_comms_errors);
    //     perf_end(_sample_perf);
    //     return ret;
    // }

    // // change this part for the H2 sensors:
    // uint16_t raw = val[0] << 8 | val[1];
    // float temp = raw & 0x0FFF; // why masking the highest 4 bits?
    // temp = (float) (temp / (float) 16.0);
    // if (raw & 0x1000) temp -= 256;


    // try out code for h2 sensors:
    uint8_t	val[4] {};
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    int ret = transfer(nullptr, 0, &val[0], 4);  // does this work? Should receive 4 bytes (first 2 sensor1, last 2 sensor2)

    if (ret == -EIO) {
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return ret;
    }

    uint16_t raw_s1 = val[0] << 8 | val[1];
    uint16_t raw_s2 = val[2] << 8 | val[3];

    float Vref = 3.3;
    float sensor_output_1 = (float raw_s1) * (Vref/ 1024.0);
    float sensor_output_2 = (float raw_s2) * (Vref/ 1024.0);

    float perc(sensor_value)
    {
        percentage =  (sensor_value - 0.5) * 50;
        if (percentage < 0.0)
        {
            percentage = 0.0;
        }
        return percentage;
    }

    float hydrogen_value_1 = perc(sensor_output_1);
    float hydrogen_value_2 = perc(sensor_output_2);


    sensor_hydrogen_s &report = _sensor_hydrogen_pub.get();

    report.timestamp_sample = timestamp_sample;
    report.hydrogen_percentage = hydrogen_value_1;
    //report.temperature = temp;
    report.timestamp = hrt_absolute_time();
    _sensor_hydrogen_pub.update();

    perf_end(_sample_perf);

    return PX4_OK;
}

void XEN5320::print_status()
{
    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}

void
XEN5320::print_usage()
{
    PRINT_MODULE_USAGE_NAME("XEN5320", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("hydrogen percentage");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(XEN5320_BASE_ADDRESS);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *XEN5320::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
                                         int runtime_instance)
{
    XEN5320 *dev = new XEN5320(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

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

extern "C" int xen5320_main(int argc, char *argv[])
{
    using ThisDriver = XEN5320;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 400000;
    cli.i2c_address = XEN5320_BASE_ADDRESS; // This is not in MPL3115A2

    const char *verb = cli.parseDefaultArguments(argc, argv);

    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BARO_DEVTYPE_MPL3115A2); //CHANGE!

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
