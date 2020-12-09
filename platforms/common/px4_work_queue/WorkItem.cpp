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

#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkItem::WorkItem(const char *name, const wq_config_t &config) :
	_item_name(name)
{
    // PX4_INFO("expected WorkItem constructor called");
	if (!Init(config)) {
        // PX4_INFO("WorkItem init failed");
		PX4_ERR("init failed");
	}
}

WorkItem::WorkItem(const char *name, const WorkItem &work_item) :
	_item_name(name)
{
	px4::WorkQueue *wq = work_item._wq;

	if ((wq != nullptr) && wq->Attach(this)) {
		_wq = wq;
	}
}

WorkItem::~WorkItem()
{
	Deinit();
}

bool WorkItem::Init(const wq_config_t &config)
{
    PX4_INFO("WorkItem Init called");
	// clear any existing first
	Deinit();

	px4::WorkQueue *wq = WorkQueueFindOrCreate(config);

	// PX4_INFO("found or created work queue with following name: %s", wq->get_name());
    PX4_INFO("found or created work queue with following name");

	if ((wq != nullptr) && wq->Attach(this)) {
		_wq = wq;
		_time_first_run = 0;
        PX4_INFO("able to attach item to work queue");
		// PX4_INFO("able to attach item to work queue %s", wq->get_name());
		return true;
	}

    PX4_INFO("%s not available", config.name);
	PX4_ERR("%s not available", config.name);
	return false;
}

void WorkItem::Deinit()
{
    PX4_INFO("WorkItem Deinit called");
	// remove any currently queued work
	if (_wq != nullptr) {
	    PX4_INFO("Deinit -> work queue is not null");
		// prevent additional insertions
		px4::WorkQueue *wq_temp = _wq;
		_wq = nullptr;

		// remove any queued work
		wq_temp->Remove(this);

		wq_temp->Detach(this);
	}
    PX4_INFO("Deinit -> work queue is null");
}

void WorkItem::ScheduleClear()
{
	if (_wq != nullptr) {
		_wq->Remove(this);
	}
}

float WorkItem::elapsed_time() const
{
	return hrt_elapsed_time(&_time_first_run) / 1e6f;
}

float WorkItem::average_rate() const
{
	const float rate = _run_count / elapsed_time();

	if ((_run_count > 1) && PX4_ISFINITE(rate)) {
		return rate;
	}

	return 0.f;
}

float WorkItem::average_interval() const
{
	const float rate = average_rate();
	const float interval = 1e6f / rate;

	if ((rate > FLT_EPSILON) && PX4_ISFINITE(interval)) {
		return roundf(interval);
	}

	return 0.f;
}

void WorkItem::print_run_status() const
{
	PX4_INFO_RAW("%-26s %8.1f Hz %12.0f us\n", _item_name, (double)average_rate(), (double)average_interval());
}

} // namespace px4
