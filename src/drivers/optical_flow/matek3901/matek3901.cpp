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

#include "matek3901.hpp"

#include <fcntl.h>
#include <termios.h>

/* Configuration Constants */
// #define LW_TAKE_RANGE_REG		'd'

Matek3901::Matek3901(const char *port, int baudrate) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_of_report.sensor_id = DRV_DEVTYPE_UNUSED;

	_lidar_report.device_id = device_id.devid;
	_lidar_report.type = DRV_DEVTYPE_UNUSED;

}

Matek3901::~Matek3901()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Matek3901::init()
{
	/* get yaw rotation from sensor frame to body frame */
	param_t rot = param_find("SENS_FLOW_ROT");

	if (rot != PARAM_INVALID) {
		int32_t val = 0;
		param_get(rot, &val);

		_rotation = Rotation(val);
	}

	/* Initialise report structure for optical flow sensor */
	/* No gyro on this board */
	_of_report.gyro_x_rate_integral = NAN;
	_of_report.gyro_y_rate_integral = NAN;
	_of_report.gyro_z_rate_integral = NAN;

	/* Conservative specs according to datasheet */
	_of_report.max_flow_rate = 5.0f;           // Datasheet: 7.4 rad/s
	_of_report.min_ground_distance = 0.1f;     // Datasheet: 80mm
	_of_report.max_ground_distance = 30.0f;    // Datasheet: infinity

	_of_report.frame_count_since_last_readout = 1;
	_of_report.integration_timespan = 15151;	// microseconds

	/* ------------------------------------------------ */
	/* Initialise report structure for distance sensor */
	_lidar_report.min_distance = 0;
	_lidar_report.max_distance = 2;
	_lidar_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	_lidar_report.h_fov = 0.436332;
	_lidar_report.v_fov = 0.436332;
	_lidar_report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;


	start();
	return PX4_OK;
}

void Matek3901::stop()
{
	ScheduleClear();
}

void Matek3901::start()
{
	/* schedule a cycle to start things */
	ScheduleNow();
}

void Matek3901::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_INFO("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		uart_config.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */

		uart_config.c_cflag &= ~CSIZE;

		uart_config.c_cflag |= CS8;         /* 8-bit characters */

		uart_config.c_cflag &= ~PARENB;     /* no parity bit */

		uart_config.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */

		uart_config.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

		/* setup for non-canonical mode */
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

		uart_config.c_oflag &= ~OPOST;

		/* fetch bytes as they become available */
		uart_config.c_cc[VMIN] = 1;

		uart_config.c_cc[VTIME] = 1;

		unsigned speed;

		speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}
	}

	/* perform collection */

	if (collect() == -EAGAIN) {
		/* Reschedule earlier to grab the missing bits */
		ScheduleDelayed(520 * 20);
		return;
	}

	ScheduleDelayed(_interval);
}

int Matek3901::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is the packet size */
	char readbuf[34];
	unsigned readlen = sizeof(readbuf)  - 1;

	int bytes_available = 0;
	::ioctl(_fd, FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		return -EAGAIN;
	}


	bool valid{false};
	int ret{0};

	do {
		/* Read from UART buffer) */
		ret = ::read(_fd, &readbuf[0], readlen);

		if (ret < 0) {
			PX4_INFO("read err: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);

			/* only throw an error if we time out */
			if (read_elapsed > (_interval * 2)) {
				/* flush anything in RX buffer */
				tcflush(_fd, TCIFLUSH);
				return ret;

			} else {
				return -EAGAIN;
			}
		}

		_last_read = hrt_absolute_time();

		// /* Parse each byte of read buffer */
		for (int i = 0; i < ret; i++) {
			valid |= matek3901_parse(readbuf[i], _linebuf, &_linebuf_index, &_parse_state, &_sensor_state, &_of_report,
						 &_lidar_report, &_of_update, &_lidar_update);
		}

		/* Publish most recent valid measurement */
		if (valid) {
			hrt_abstime current_time = hrt_absolute_time();

			if (_lidar_update) {
				_lidar_report.timestamp = current_time;

				_distance_sensor_topic.publish(_lidar_report);
			}

			if (_of_update) {
				_of_report.timestamp = current_time;
				/* Rotate measurements from sensor frame to body frame */
				float zeroval = 0.0f;
				rotate_3f(_rotation, _of_report.pixel_flow_x_integral, _of_report.pixel_flow_y_integral, zeroval);

				_of_report.time_since_last_sonar_update = current_time - _last_of_time;
				_last_of_time = current_time;

				_optical_flow_topic.publish(_of_report);
			}
		}

		/* Bytes left to parse */
		bytes_available -= ret;

	} while (bytes_available > 0);

	perf_end(_sample_perf);
	return PX4_OK;

}

void Matek3901::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
