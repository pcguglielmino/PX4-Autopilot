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

/**
 * @author Peter Guglielmino <peter.guglielmino1@gmail.com>
 *
 * Declarations of parser for the MATEKSYS Optical Flow and Lidar Sensor 3901-L0X
 */

#include "matek3901_parser.hpp"
#include <string.h>
#include <stdlib.h>

#include <stdio.h>
// #define MATEK_DEBUG

#ifdef MATEK_DEBUG
// #include <stdio.h>

bool matek_report_data{false};
int matek_count{0};

double x_flow_debug;
double y_flow_debug;

// const char *parser_state[] = {
// 	"HEADER1",
// 	"HEADER2",
// 	"HEADER3",
// 	"FLAG",
// 	"FUNCTION1",
// 	"FUNCTION2",
// 	"PAYLOAD_SIZE1",
// 	"PAYLOAD_SIZE2",
// 	"PAYLOAD1",
// 	"PAYLOAD2",
// 	"PAYLOAD3",
// 	"PAYLOAD4",
// 	"PAYLOAD5",
// 	"PAYLOAD6",
// 	"PAYLOAD7",
// 	"PAYLOAD8",
// 	"PAYLOAD9",
// 	"CHECKSUM"
// };
#endif

bool matek3901_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum MATEK_PARSE_STATE *p_state,
		     enum MATEK_SENSOR_STATE *s_state, optical_flow_s *of_report, distance_sensor_s *lidar_report, bool *of_update,
		     bool *lidar_update)
{
	bool parsed_packet = false;

	switch (*p_state) {

	case MATEK_PARSE_STATE_HEADER1:
		if (c == 0x24) {
			*p_state = MATEK_PARSE_STATE_HEADER2;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
		}

		break;

	case MATEK_PARSE_STATE_HEADER2:

		if (c == 0x58) {
			*p_state = MATEK_PARSE_STATE_HEADER3;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
		}

		break;

	case MATEK_PARSE_STATE_HEADER3:
		if (c == 0x3C) {
			*p_state = MATEK_PARSE_STATE_FLAG;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
		}

		break;

	case MATEK_PARSE_STATE_FLAG:
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		if (c == 0x00) {
			*p_state = MATEK_PARSE_STATE_FUNCTION1;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
			*parserbuf_index = 0;
		}

		break;

	case MATEK_PARSE_STATE_FUNCTION1:

		// Function LSB
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		if (c == 0x01) {
			*p_state = MATEK_PARSE_STATE_FUNCTION2;
			*s_state = SENSOR_STATE_DISTANCE;

		} else if (c == 0x02) {
			*p_state = MATEK_PARSE_STATE_FUNCTION2;
			*s_state = SENSOR_STATE_OF;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
			*s_state = SENSOR_STATE_NO_SENSOR;
			*parserbuf_index = 0;
		}

		break;

	case MATEK_PARSE_STATE_FUNCTION2:

		// Function MSB
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		if (c == 0x1F) {
			*p_state = MATEK_PARSE_STATE_PAYLOAD_SIZE1;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
			*s_state = SENSOR_STATE_NO_SENSOR;
			*parserbuf_index = 0;
		}

		break;

	case MATEK_PARSE_STATE_PAYLOAD_SIZE1:

		// Payload LSB
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		if ((c == 0x05) && (*s_state == SENSOR_STATE_DISTANCE)) {
			*p_state = MATEK_PARSE_STATE_PAYLOAD_SIZE2;

		} else if ((c == 0x09) && (*s_state == SENSOR_STATE_OF)) {
			*p_state = MATEK_PARSE_STATE_PAYLOAD_SIZE2;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
			*s_state = SENSOR_STATE_NO_SENSOR;
			*parserbuf_index = 0;
		}

		break;

	case MATEK_PARSE_STATE_PAYLOAD_SIZE2:

		// Payload MSB
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		if (c == 0x00) {
			*p_state = MATEK_PARSE_STATE_PAYLOAD1;

		} else {
			*p_state = MATEK_PARSE_STATE_HEADER1;
			*s_state = SENSOR_STATE_NO_SENSOR;
			*parserbuf_index = 0;
		}

		break;

	case MATEK_PARSE_STATE_PAYLOAD1:

		// Quality
		*p_state = MATEK_PARSE_STATE_PAYLOAD2;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD2:

		// Distance LSB
		// X Motion LSB
		*p_state = MATEK_PARSE_STATE_PAYLOAD3;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD3:

		// Distance
		// X Motion
		*p_state = MATEK_PARSE_STATE_PAYLOAD4;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD4:

		// Distance
		// X Motion
		*p_state = MATEK_PARSE_STATE_PAYLOAD5;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD5:

		// Distance MSB
		// X Motion MSB
		if (*s_state == SENSOR_STATE_OF) {
			*p_state = MATEK_PARSE_STATE_PAYLOAD6;

		} else {
			*p_state = MATEK_PARSE_STATE_CHECKSUM;
		}

		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD6:

		// Y Motion LSB
		*p_state = MATEK_PARSE_STATE_PAYLOAD7;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD7:

		// Y Motion
		*p_state = MATEK_PARSE_STATE_PAYLOAD8;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD8:

		// Y Motion
		*p_state = MATEK_PARSE_STATE_PAYLOAD9;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_PAYLOAD9:

		// Y Motion MSB
		*p_state = MATEK_PARSE_STATE_CHECKSUM;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		break;

	case MATEK_PARSE_STATE_CHECKSUM: {

			// Checksum
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
			unsigned char cksm = 0;

			if (*s_state == SENSOR_STATE_DISTANCE) {

				for (int i = 0; i < 10; i++) {
					cksm = crc8_dvb_s2(cksm, parserbuf[i]);
				}

				if (c == cksm) {
					uint8_t quality = parserbuf[5];
					int32_t distance = uint32_t(parserbuf[9]) << (8 + 8 + 8) | uint32_t(parserbuf[8]) << (8 + 8) |
							   uint32_t(parserbuf[7]) << 8 | uint32_t(parserbuf[6]);

					lidar_report->signal_quality = quality;
					lidar_report->current_distance = static_cast<float>(distance) / 1000.0f;
					parsed_packet = true;
					*lidar_update = true;
					*p_state = MATEK_PARSE_STATE_HEADER1;

				} else {
					*p_state = MATEK_PARSE_STATE_HEADER1;
					*s_state = SENSOR_STATE_NO_SENSOR;
				}

			}

			if (*s_state == SENSOR_STATE_OF) {

				printf("of received \n");

				for (int i = 0; i < 14; i++) {
					printf("%#0x \n", parserbuf[i]);
					cksm = crc8_dvb_s2(cksm, parserbuf[i]);
				}

				if (c == cksm) {

					uint8_t quality = parserbuf[5];
					// Little endian
					int32_t x_flow = uint32_t(parserbuf[9]) << (8 + 8 + 8) | uint32_t(parserbuf[8]) << (8 + 8) |
							 uint32_t(parserbuf[7]) << 8 | uint32_t(parserbuf[6]);
					int32_t y_flow = uint32_t(parserbuf[13]) << (8 + 8 + 8) | uint32_t(parserbuf[12]) << (8 + 8) |
							 uint32_t(parserbuf[11]) << 8 | uint32_t(parserbuf[10]);

					of_report->quality = quality;

					// of_report->pixel_flow_x_integral = static_cast<float>(x_flow);
					// of_report->pixel_flow_y_integral = static_cast<float>(y_flow);

					of_report->pixel_flow_x_integral += static_cast<float>(x_flow);
					of_report->pixel_flow_y_integral += static_cast<float>(y_flow);

					parsed_packet = true;
					*of_update = true;
					*p_state = MATEK_PARSE_STATE_HEADER1;

				} else {
					*p_state = MATEK_PARSE_STATE_HEADER1;
					*s_state = SENSOR_STATE_NO_SENSOR;
				}

			}

			*parserbuf_index = 0;

		}

		break;

	}

// #ifdef MATEK_DEBUG
// 	printf("state: MATEK_PARSE_STATE%s, got char: %#02x\n", parser_state[*p_state], c);
// #endif

	return parsed_packet;
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
	// From the MSP-V2 GitHub page: https://github.com/iNavFlight/inav/wiki/MSP-V2
	crc ^= a;

	for (int ii = 0; ii < 8; ++ii) {
		if (crc & 0x80) {
			crc = (crc << 1) ^ 0xD5;

		} else {
			crc = crc << 1;
		}
	}

	return crc;
}
