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
 * @file SubscriptionCallback.hpp
 *
 */

#pragma once

#include <uORB/SubscriptionInterval.hpp>
#include <containers/List.hpp>

namespace uORB
{

// Subscription wrapper class with callbacks on new publications
class SubscriptionCallback : public SubscriptionInterval, public ListNode<SubscriptionCallback *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval_ms The requested maximum update interval in milliseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionCallback(const orb_metadata *meta, uint8_t interval_ms = 0, uint8_t instance = 0) :
		SubscriptionInterval(meta, interval_ms, instance)
	{
	}

	virtual ~SubscriptionCallback()
	{
		unregisterCallback();
	};

	bool registerCallback()
	{
		bool ret = false;

		if (_subscription.get_node() && _subscription.get_node()->register_callback(this)) {
			// registered
			ret = true;

		} else {
			// force topic creation by subscribing with old API
			int fd = orb_subscribe_multi(_subscription.get_topic(), _subscription.get_instance());

			// try to register callback again
			if (_subscription.subscribe()) {
				if (_subscription.get_node() && _subscription.get_node()->register_callback(this)) {
					ret = true;
				}
			}

			orb_unsubscribe(fd);
		}


		return ret;
	}

	void unregisterCallback()
	{
		if (_subscription.get_node()) {
			_subscription.get_node()->unregister_callback(this);
		}
	}

	virtual void call() = 0;

};

} // namespace uORB
