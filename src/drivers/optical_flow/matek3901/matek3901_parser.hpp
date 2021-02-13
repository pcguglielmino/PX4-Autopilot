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
#pragma once

#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

// Data Format for MATEKSYS Optical Flow and Lidar Sensor 3901-L0X (MSPv2 Protocol, Little Endian)
// ===============================
// 14 bytes per LIDAR message:
// 0-2) Header (0x24, 0x58, 0x3C) "$X<"
// 3) Flag (0x00)
// 4-5) Function (0x1F01)
// 6-7) Payload size (0x0005)
// 8) Quality (uint8)
// 9-12) Distance (int32)
// 13) Checksum (byte3+...+byte11)

// ===============================
// 18 bytes per OF message:
// 0-2) Header (0x24, 0x58, 0x3C) "$X<"
// 3) Flag (0x00)
// 4-5) Function (0x1F02)
// 6-7) Payload size (0x0009)
// 8) Quality (uint8)
// 9-12) Motion_X (int32)
// 13-16) Motion_Y (int32)
// 17) Checksum (byte3+...+byte15)

enum MATEK_PARSE_STATE {
	MATEK_PARSE_STATE_HEADER1,
	MATEK_PARSE_STATE_HEADER2,
	MATEK_PARSE_STATE_HEADER3,
	MATEK_PARSE_STATE_FLAG,
	MATEK_PARSE_STATE_FUNCTION1,
	MATEK_PARSE_STATE_FUNCTION2,
	MATEK_PARSE_STATE_PAYLOAD_SIZE1,
	MATEK_PARSE_STATE_PAYLOAD_SIZE2,
	MATEK_PARSE_STATE_PAYLOAD1,
	MATEK_PARSE_STATE_PAYLOAD2,
	MATEK_PARSE_STATE_PAYLOAD3,
	MATEK_PARSE_STATE_PAYLOAD4,
	MATEK_PARSE_STATE_PAYLOAD5,
	MATEK_PARSE_STATE_PAYLOAD6,
	MATEK_PARSE_STATE_PAYLOAD7,
	MATEK_PARSE_STATE_PAYLOAD8,
	MATEK_PARSE_STATE_PAYLOAD9,
	MATEK_PARSE_STATE_CHECKSUM
};

enum MATEK_SENSOR_STATE {
	SENSOR_STATE_NO_SENSOR,
	SENSOR_STATE_OF,
	SENSOR_STATE_DISTANCE
};

bool matek3901_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum MATEK_PARSE_STATE *p_state,
		     enum MATEK_SENSOR_STATE *s_state, optical_flow_s *of_report, distance_sensor_s *lidar_report, bool *of_update,
		     bool *lidar_update);

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a); // checksum function
