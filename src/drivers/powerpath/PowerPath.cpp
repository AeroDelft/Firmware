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
  * @file PowerPath.cpp
  * @author Vincent Steenhuizen
  *
  * Driver for the Phoenix PT powerpath, connected via I2C. Inspired by the
  * MCP9808 Tempertature Sensor driver.
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
#include <uORB/topics/powerpath.h>


#define POWERPATH_BASE_ADDRESS       0xc8           //set by Aleks

#define POWERPATH_REG_AMB_TEMP       0x05
#define POWERPATH_REG_DEV_ID         0x07
#define POWERPATH_DEV_ID             0x04

#define POW_BASE_DEVICE_PATH      "/dev/pow"         //???

#define POWERPATH_CONVERSION_INTERVAL	10000	/* microseconds */

class POWERPATH : public device::I2C, public I2CSPIDriver<POWERPATH>
{
public:
    POWERPATH(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = POWERPATH_BASE_ADDRESS);
    ~POWERPATH() override;

    static I2CSPIDriverBase* instantiate(const BusCLIArguments& cli, const BusInstanceIterator& iterator,
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

    uORB::PublicationMultiData<powerpath_s>	_powerpath_pub{ORB_ID(powerpath)};

    int			_class_device_instance{ -1 };

    bool _collect_phase{ false };

    perf_counter_t _sample_perf;
    perf_counter_t _measure_perf;
    perf_counter_t _comms_errors;
};

POWERPATH::POWERPATH(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) : 
    I2C(DRV_POW_DEVTYPE_POWERPATH, MODULE_NAME, bus, address, bus_frequency), 
    I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address), 
    _powerpath_pub{ ORB_ID(powerpath), ORB_PRIO_DEFAULT },
    _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
    _measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
    _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
    _class_device_instance = register_class_devname(POW_BASE_DEVICE_PATH);
    _powerpath_pub.advertise();

    _powerpath_pub.get().device_id = get_device_id();
}

POWERPATH::~POWERPATH()
{
    if (_class_device_instance != -1)
    {
        unregister_class_devname(POW_BASE_DEVICE_PATH, _class_device_instance);
    }

    _powerpath_pub.unadvertise();

    perf_free(_sample_perf);
    perf_free(_measure_perf);
    perf_free(_comms_errors);
}

int POWERPATH::init()
{
    int ret = I2C::init();

    if (ret != PX4_OK)
    {
        PX4_ERR("init failed");
        return PX4_ERROR;
    }

    start();

    return PX4_OK;
}

int POWERPATH::probe()
{
    _retries = 10;
    uint8_t val[2]{};           //Decide on how many bytes a single message is (currently 2)

    uint8_t reg = POWERPATH_REG_DEV_ID;
    int ret = transfer(&reg, 1, &val[0], 2);        //Again decide on number of bytes

    if ((ret == PX4_OK) && (val[0] == POWERPATH_DEV_ID))
    {
        _retries = 0;
        return PX4_OK;
    }

    return -EIO;
}

void POWERPATH::start()
{
    _collect_phase = false;

    ScheduleNow();
}

void POWERPATH::RunImpl()
{
    int ret = PX4_ERROR;

    if (_collect_phase)
    {
        ret = collect();

        if (ret == -EIO)
        {
            return;
        }

        if (ret == -EAGAIN)
        {
            ScheduleDelayed(POWERPATH_CONVERSION_INTERVAL);

            return;
        }

        _collect_phase = false;
    }

    ret = measure();

    if (ret == -EIO)
    {
        return;
    }

    _collect_phase = true;

    ScheduleDelayed(POWERPATH_CONVERSION_INTERVAL);

}

int POWERPATH::measure()
{
    perf_begin(_measure_perf);

    _retries = 0; 

    uint8_t val[2] = { 0, 0 };                  //check size of val
    uint8_t reg = POWERPATH_REG_AMB_TEMP;
    int ret = transfer(&reg, 1, &val[0], 2);    //check size of val

    if (ret == -EIO)
    {
        perf_count(_comms_errors);
    }

    perf_end(_measure_perf);

    return PX4_OK;
}

int POWERPATH::collect()
{
    perf_begin(_sample_perf);

    uint8_t val[19]{};                          //check size of val
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    int ret = transfer(nullptr, 0, &val[0], 18); //check size of val
    
    if (ret == -EIO)
    {
        perf_count(_comms_errors);
        perf_end(_sample_perf);
        return ret;
    }

    /* IMPLEMENT DATA READOUT*/
    // [2] motor rpm            (raw, RPM)
    // [2] power motor          (raw, Watts)
    // [2] voltage takeoff batt (10bit readout * 10)  Volts * FACTOR
    // [2] voltage fc           (10bit readout * 10) Volts * FACTOR
    // [2] temp mosfet bat      (KELVIN*100)
    // [2] temp mosfet fc       (KELVIN*100)
    // [2] temp internal        (KELVIN*100)
    // [1] status byte:         [][][TOReady: 1 ready for takeoff, 0 not ready for takeoff][GotSig: 1 when signal from pixhawk, 0 when no signal][motorSpins: 0 off, 1 on][escActive: 0 off, 1 on][chargerStatus: 0 off, 1 on][powerSource: 0 battery, 1 FC]
    // [1] errors byte:         [][][][][3-0th bits: error codes, upto 16]
    // [1] % takeoff batt       (experimantal, %*2)
    // [1] % fc                 (experimental, %*2)

    uint16_t raw_motor_rpm = val[0] << 8 | val[1];

    uint16_t raw_motor_pow = val[2] << 8 | val[3];

    uint16_t raw_bat_volt = val[4] << 8 | val[5];

    uint16_t raw_fc_volt = val[6] << 8 | val[7];

    uint16_t raw_temp_mosfet_bat = val[8] << 8 | val[9];
    float temp_mosfet_bat = raw_temp_mosfet_bat / 100;

    uint16_t raw_temp_mosfet_fc = val[10] << 8 | val[11];
    float temp_mosfet_fc = raw_temp_mosfet_fc / 100;

    uint16_t raw_temp_internal = val[12] << 8 | val[13];
    float temp_internal = raw_temp_internal / 100;

    uint8_t raw_status = val[14];

    //const char* powersource;
    //const char* chargerstatus;
    //const char* ESCActive;
    //const char* motorspins;

    bool powersource;
    bool chargerstatus;
    bool ESCActive;
    bool motorspins;
    bool gotsig;
    bool toready;

    switch ((raw_status & (1 << 0)) != 0)   
    {
        case 0:
            //powersource = "battery"; 
            powersource = false;
            break;
        case 1:
            //powersource = "fuel-cell"; 
            powersource = true;
            break;
    }

    switch ((raw_status & (1 << 1)) != 0)
    {
        case 0:
            //chargerstatus = "off"; 
            chargerstatus = false;
            break;
        case 1:
            //chargerstatus = "on"; 
            chargerstatus = true;
            break;
    }

    switch ((raw_status & (1 << 2)) != 0)
    {
        case 0:
            //ESCActive = "off"; 
            ESCActive = false;
            break;
        case 1:
            //ESCActive = "on"; 
            ESCActive = true;
            break;
    }

    switch ((raw_status & (1 << 3)) != 0)
    {
        case 0:
            //motorspins = "off"; 
            motorspins = false;
            break;
        case 1:
            //motorspins = "on"; 
            motorspins = true;
            break;
    }

    switch ((raw_status & (1 << 4)) != 0)
    {
        case 0:
            //gotsig = "off"; 
            gotsig = false;
            break;
        case 1:
            //gotsig = "on"; 
            gotsig = true;
            break;
    }

    switch ((raw_status & (1 << 5)) != 0)
    {
        case 0:
            //toready = "off"; 
            toready = false;
            break;
        case 1:
            //toready = "on"; 
            toready = true;
            break;
    }

    //uint16_t raw_errors = val[15] << 8 | val[16];

    uint8_t raw_bat_percentage = val[17];
    float bat_percentage = raw_bat_percentage / 2;

    uint8_t raw_fc_percentage = val[18];
    float fc_percentage = raw_fc_percentage / 2;

    /* UPDATE TOPICS*/
    powerpath_s& report = _powerpath_pub.get();

    report.timestamp_sample = timestamp_sample;
    report.timestamp = hrt_absolute_time();
    report.motor_rpm = raw_motor_rpm;
    report.motor_power = raw_motor_pow;
    report.battery_voltage = raw_bat_volt;
    report.fc_voltage = raw_fc_volt;
    report.temperature_mosfet_battery = temp_mosfet_bat;
    report.temperature_mosfet_fc = temp_mosfet_fc;
    report.temperature_internal = temp_internal;
    report.status_powersource = powersource;
    report.status_chargerstatus = chargerstatus;
    report.status_escactive = ESCActive;
    report.status_motorspins = motorspins;
    report.status_gotsig = gotsig;
    report.status_toready = toready; 
    report.battery_charge_level = bat_percentage;
    report.fc_charge_level = fc_percentage;

    _powerpath_pub.update();

    perf_end(_sample_perf);

    return PX4_OK;
}

void POWERPATH::print_status()
{
    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}

void POWERPATH::print_usage()
{
    PRINT_MODULE_USAGE_NAME("powerpath", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("pow");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(POWERPATH_BASE_ADDRESS);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase* POWERPATH::instantiate(const BusCLIArguments &cli, const BusInstanceIterator& iterator, int runtime_instance)
{
    POWERPATH* dev = new POWERPATH(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

    if (dev == nullptr)
    {
        PX4_ERR("alloc failed");
        return nullptr;
    }

    if (OK != dev->init())
    {
        delete dev; 
        return nullptr;
    }

    return dev;
}

extern "C" int powerpath_main(int argc, char* argv[])
{
    using ThisDriver = POWERPATH;
    BusCLIArguments cli{ true, false };
    cli.default_i2c_frequency = 400000;
    cli.i2c_address = POWERPATH_BASE_ADDRESS;

    const char* verb = cli.parseDefaultArguments(argc, argv);

    if (!verb)
    {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BARO_DEVTYPE_MPL3115A2); //why this driver and not our own? DRV_POW_DEVTYPE_POWERPATH

    if (!strcmp(verb, "start"))
    {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop"))
    {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status"))
    {
        return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();

    return -1;
}