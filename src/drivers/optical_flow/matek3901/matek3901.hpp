/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file matek3901.hpp
 * @author Peter Guglielmino <peter.guglielmino1@gmail.com>
 *
 * Driver for the Matek 3901 Optical flow and Lidar sensor
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/uORB.h>

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include "matek3901_parser.hpp"

class Matek3901 : public px4::ScheduledWorkItem
{
public:
	Matek3901(const char *port, int baudrate);
	~Matek3901() override;

	int 						init();
	void						print_info();

private:

	void						start();
	void						stop();
	void						Run() override;
	int						measure();
	int						collect();


	char 						_port[20] {};
	int         		    		  	_interval{30000};
	int						_fd{-1};

	Rotation	          			_rotation;
	char						_linebuf[18] {};
	unsigned					_linebuf_index{0};

	unsigned					_consecutive_fail_count;

	hrt_abstime					_last_read{0};

	optical_flow_s  			        _of_report;
	distance_sensor_s           			_lidar_report;

	uORB::PublicationMulti<optical_flow_s>		_optical_flow_topic{ORB_ID(optical_flow)};
	uORB::PublicationMulti<distance_sensor_s>	_distance_sensor_topic{ORB_ID(distance_sensor)};

	enum MATEK_PARSE_STATE				_parse_state {MATEK_PARSE_STATE_HEADER1};
	enum MATEK_SENSOR_STATE				_sensor_state {SENSOR_STATE_NO_SENSOR};

	bool						_of_update{false};
	bool						_lidar_update{false};

	hrt_abstime					_last_of_time;
	hrt_abstime					_last_lidar_time;

	perf_counter_t					_sample_perf;
	perf_counter_t					_comms_errors;

	param_t						_x_param{PARAM_INVALID};
	param_t 					_y_param{PARAM_INVALID};

	float 						_x_factor;
	float 						_y_factor;
};
