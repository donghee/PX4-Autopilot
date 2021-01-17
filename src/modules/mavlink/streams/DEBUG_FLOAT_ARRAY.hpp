/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef DEBUG_FLOAT_ARRAY_HPP
#define DEBUG_FLOAT_ARRAY_HPP

#include <uORB/topics/debug_array.h>
#include <uORB/topics/encapsulated_data.h>

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDebugFloatArray(mavlink); }

	static constexpr const char *get_name_static() { return "DEBUG_FLOAT_ARRAY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _encapsulated_data_sub.advertised() ? MAVLINK_MSG_ID_ENCAPSULATED_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	// uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};
	uORB::Subscription _encapsulated_data_sub{ORB_ID(encapsulated_data)};

	bool send() override
	{
		encapsulated_data_s data;

		if (_encapsulated_data_sub.update(&data)) {
			mavlink_encapsulated_data_t msg{};


			msg.seqnr = 0;
			// memcpy(msg.data, debug.data, 58*4);

			for (size_t i = 0; i < 128; i++) {
				msg.data[i] = data.data[i];
			}

			mavlink_msg_encapsulated_data_send_struct(_mavlink->get_channel(), &msg);

			msg.seqnr = 1;

			for (size_t i = 128; i < 256; i++) {
				msg.data[i - 128] = data.data[i];
			}

			mavlink_msg_encapsulated_data_send_struct(_mavlink->get_channel(), &msg);

			msg.seqnr = 2;

			for (size_t i = 0; i < (16 + 128 + 16); i++) {
				msg.data[i] = data.data[256 + i];
			}

			mavlink_msg_encapsulated_data_send_struct(_mavlink->get_channel(), &msg);


			return true;
		}


		return false;
	}
};

#endif // DEBUG_FLOAT_ARRAY_HPP
