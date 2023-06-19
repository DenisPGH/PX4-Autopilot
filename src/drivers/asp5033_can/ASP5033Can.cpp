/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file ASP5033Can.cpp
 * @author Denislav Petrov <denislavamitoba@gmail.com>
 *
 * Driver for the ASP5033 sensor connected over CAN.
 *
 *
 */

#include "ASP5033Can.hpp"
//#include "stm32_can.h"

extern orb_advert_t mavlink_log_pub;

ASP5033Can::ASP5033Can() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

ASP5033Can::~ASP5033Can()
{
}

int ASP5033Can::init()
{
	// struct can_dev_s *can = stm32_caninitialize(1);

	// if (can == nullptr) {
	// 	PX4_ERR("Failed to get CAN interface");

	// } else {
	// 	/* Register the CAN driver at "/dev/can0" */
	// 	int ret = can_register("/dev/can0", can);

	// 	if (ret < 0) {
	// 		PX4_ERR("can_register failed: %d", ret);

	// 	} else {
	// 		_fd = ::open("/dev/can0", O_RDWR | O_NONBLOCK);
	// 	}
	// }

	return 0;
}

void ASP5033Can::Run()
{

	perf_begin(_perf_elapsed);

	if (!_initialized) {

		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {
			PX4_INFO("FAILED TO OPEN /dev/can0");
			return;
		}

		_initialized = true;
	}

	//uint8_t data[64] {};
	//CanFrame received_frame{};
	//received_frame.payload = &data;

	ASP5033Message asp5033_message = {};

	// while (receive(&received_frame) > 0) {

	// 	// Find the start of a transferr
	// 	if ((received_frame.payload_size == 8) && ((uint8_t *)received_frame.payload)[7] == TAIL_BYTE_START_OF_TRANSFER) {
	// 	} else {
	// 		continue;
	// 	}

	// 	// We have the start of a transfer
	// 	size_t offset = 5;
	// 	memcpy(&asp5033_message, &(((uint8_t *)received_frame.payload)[2]), offset);

	// 	while (receive(&received_frame) > 0) {

	// 		size_t payload_size = received_frame.payload_size - 1;
	// 		// TODO: add check to prevent buffer overflow from a corrupt 'payload_size' value
	// 		// TODO: AND look for TAIL_BYTE_START_OF_TRANSFER to indicate end of transfer. Untested...
	// 		memcpy(((char *)&asp5033_message) + offset, received_frame.payload, payload_size);
	// 		offset += payload_size;
	// 	}


	// 	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// 	// publish values
	// 	differential_pressure_s differential_pressure{};
	// 	differential_pressure.timestamp_sample = timestamp_sample;
	// 	differential_pressure.device_id = static_cast<uint8_t>(asp5033_message.sku);
	// 	differential_pressure.differential_pressure_pa = 0.12; //static_cast<float>(asp5033_message.pressure);;
	// 	differential_pressure.temperature = 23; // static_cast<float>(asp5033_message.temperature); ;
	// 	differential_pressure.error_count = perf_event_count(_comms_errors);
	// 	differential_pressure.timestamp = timestamp_sample;
	// 	_differential_pressure_pub.publish(differential_pressure);
	// }
	// publish values
		const hrt_abstime timestamp_sample = hrt_absolute_time();
		differential_pressure_s differential_pressure{};
		differential_pressure.timestamp_sample = timestamp_sample;
		differential_pressure.device_id = static_cast<uint8_t>(asp5033_message.sku);
		differential_pressure.differential_pressure_pa = 0.12f; //static_cast<float>(asp5033_message.pressure);;
		differential_pressure.temperature = 23.56f; // static_cast<float>(asp5033_message.temperature); ;
		differential_pressure.error_count = perf_event_count(_comms_errors);
		differential_pressure.timestamp = timestamp_sample;
		_differential_pressure_pub.publish(differential_pressure);

	perf_end(_perf_elapsed);
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}
}

int16_t ASP5033Can::receive(CanFrame *received_frame)
{
	if ((_fd < 0) || (received_frame == nullptr)) {
		PX4_INFO("fd < 0");
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events = POLLIN;

	// Jake: this doesn't block even in blocking mode, dunno if we need it
	::poll(&fds, 1, 0);

	// // Only execute this part if can0 is changed.
	// if (fds.revents & POLLIN) {

	// 	// Try to read.
	// 	struct can_msg_s receive_msg;
	// 	const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

	// 	if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
	// 		PX4_INFO("error");
	// 		return -1;

	// 	} else {
	// 		received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
	// 		received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
	// 		memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
	// 		return nbytes;
	// 	}
	// }

	return 0;
}

int16_t ASP5033Can::transmit()
{
	// if (_fd < 0) {
	// 	return -1;
	// }

	// struct pollfd fds {};

	// fds.fd = _fd;

	// fds.events |= POLLOUT;

	// const int poll_result = poll(&fds, 1, timeout_ms);

	// if (poll_result < 0) {
	// 	return -1;
	// }

	// if (poll_result == 0) {
	// 	return 0;
	// }

	// if ((fds.revents & POLLOUT) == 0) {
	// 	return -1;
	// }

	// struct can_msg_s transmit_msg {};

	// transmit_msg.cm_hdr.ch_id = txf.frame.extended_can_id;

	// transmit_msg.cm_hdr.ch_dlc = txf.frame.payload_size;

	// transmit_msg.cm_hdr.ch_extid = 1;

	// memcpy(transmit_msg.cm_data, txf.frame.payload, txf.frame.payload_size);

	// const size_t msg_len = CAN_MSGLEN(transmit_msg.cm_hdr.ch_dlc);

	// const ssize_t nbytes = ::write(_fd, &transmit_msg, msg_len);

	// if (nbytes < 0 || (size_t)nbytes != msg_len) {
	// 	return -1;
	// }

	return 1;
}

int ASP5033Can::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// /dev/can0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

int ASP5033Can::task_spawn(int argc, char *argv[])
{
	ASP5033Can *instance = new ASP5033Can();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int ASP5033Can::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for reading data from the ASP5033 differential pressure sensor.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("asp5033_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ASP5033Can::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int asp5033_can_main(int argc, char *argv[])
{
	return ASP5033Can::main(argc, argv);
}
