/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "uORBTest_UnitTest.hpp"
#include "base/orb_log.h"
#include "base/orb_posix.h"
#include "uORBCommon.hpp"
#include <math.h>
#include <poll.h>
#include <stdio.h>

ORB_DEFINE(orb_test, struct orb_test, sizeof(orb_test), "ORB_TEST:int val;hrt_abstime time;");
ORB_DEFINE(orb_multitest, struct orb_test, sizeof(orb_test), "ORB_MULTITEST:int val;hrt_abstime time;");

ORB_DEFINE(orb_test_medium, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_multi, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_queue, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_queue_poll, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");

ORB_DEFINE(orb_test_large, struct orb_test_large, sizeof(orb_test_large),
	   "ORB_TEST_LARGE:int val;hrt_abstime time;char[512] junk;");

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main()
{
	/* poll on sample topic and output latency */
	float latency_integral = 0.0f;

	/* wakeup source(s) */
	orb_pollfd_struct_t fds[3];

	int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
	int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
	int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

	struct orb_test_large t;

	/* clear all ready flags */
	orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
	orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
	orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

	fds[0].fd = test_multi_sub;
	fds[0].events = POLLIN;
	fds[1].fd = test_multi_sub_medium;
	fds[1].events = POLLIN;
	fds[2].fd = test_multi_sub_large;
	fds[2].events = POLLIN;

	const unsigned maxruns = 1000;
	unsigned timingsgroup = 0;
	int current_value = t.val;
	int num_missed = 0;

	// timings has to be on the heap to keep frame size below 2048 bytes
	unsigned *timings = new unsigned[maxruns];
	unsigned timing_min = 9999999, timing_max = 0;

	for (unsigned i = 0; i < maxruns; i++) {
		/* wait for up to 500ms for data */
		int pret =  orb_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
			timingsgroup = 0;

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
			timingsgroup = 1;

		} else if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
			timingsgroup = 2;
		}

		if (pret < 0) {
			ORB_ERR("poll error %d, %d", pret, orb_errno);
			continue;
		}

		num_missed += t.val - current_value - 1;
		current_value = t.val;

		auto elt = hrt_elapsed_time(&t.time);
		latency_integral += elt;
		timings[i] = elt;

		if (elt > timing_max) {
			timing_max = elt;
		}

		if (elt < timing_min) {
			timing_min = elt;
		}
	}

	orb_unsubscribe(test_multi_sub);
	orb_unsubscribe(test_multi_sub_medium);
	orb_unsubscribe(test_multi_sub_large);

	if (pubsubtest_print) {
		char fname[32];
		sprintf(fname, "./uorb_timings%u.txt", timingsgroup);
		FILE *f = fopen(fname, "w");

		if (f == nullptr) {
			ORB_ERR("Error opening file!");
			delete[] timings;
			return ORB_ERROR;
		}

		for (unsigned i = 0; i < maxruns; i++) {
			fprintf(f, "%u\n", timings[i]);
		}

		fclose(f);
	}


	float std_dev = 0.f;
	float mean = latency_integral / maxruns;

	for (unsigned i = 0; i < maxruns; i++) {
		float diff = (float)timings[i] - mean;
		std_dev += diff * diff;
	}

	delete[] timings;

	ORB_INFO("mean:    %8.4f us", static_cast<double>(mean));
	ORB_INFO("std dev: %8.4f us", static_cast<double>(sqrtf(std_dev / (maxruns - 1))));
	ORB_INFO("min:     %3i us", timing_min);
	ORB_INFO("max:     %3i us", timing_max);
	ORB_INFO("missed topic updates: %i", num_missed);

	pubsubtest_passed = true;

	if (static_cast<float>(latency_integral / maxruns) > 100.0f) {
		pubsubtest_res = ORB_ERROR;

	} else {
		pubsubtest_res = ORB_OK;
	}

	return pubsubtest_res;
}

int uORBTest::UnitTest::test()
{
	int ret = test_single();

	if (ret != ORB_OK) {
		return ret;
	}

	ret = test_multi();

	if (ret != ORB_OK) {
		return ret;
	}

	ret = test_multi_reversed();

	if (ret != ORB_OK) {
		return ret;
	}

	ret = test_unadvertise();

	if (ret != ORB_OK) {
		return ret;
	}

	ret = test_multi2();

	if (ret != ORB_OK) {
		return ret;
	}

	ret = test_queue();

	if (ret != ORB_OK) {
		return ret;
	}

	return test_queue_poll_notify();
}

int uORBTest::UnitTest::test_unadvertise()
{
	ORB_INFO("Testing unadvertise");

	//we still have the advertisements from the previous test_multi calls.
	for (auto & i : _pfd) {
		int ret = orb_unadvertise(i);

		if (ret != ORB_OK) {
			ORB_ERR("orb_unadvertise failed (%i)", ret);
			return ORB_ERROR;
		}
	}

	//try to advertise and see whether we get the right instance
	int instance_test[4];
	struct orb_test t{};

	for (int i = 0; i < 4; ++i) {
		_pfd[i] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance_test[i], ORB_PRIO_MAX);

		if (instance_test[i] != i) {
			ORB_ERR("got wrong instance (should be %i, is %i)", i,
                                 instance_test[i]);
			return ORB_ERROR;
		}
	}

	for (auto & i : _pfd) {
		orb_unadvertise(i);
	}

	ORB_INFO("PASS unadvertise");

	return ORB_OK;
}

int uORBTest::UnitTest::test_single()
{
	ORB_INFO("try single-topic support");

	struct orb_test t{}, u{};
	int sfd;
	orb_advert_t ptopic;
	bool updated;

	t.val = 0;
	ptopic = orb_advertise(ORB_ID(orb_test), &t);

	if (ptopic == nullptr) {
		ORB_ERR("advertise failed: %d", orb_errno);
		return ORB_ERROR;
        }

	ORB_INFO("publish handle %p", ptopic);
	sfd = orb_subscribe(ORB_ID(orb_test));

	if (sfd < 0) {
		ORB_ERR("subscribe failed: %d", orb_errno);
		return ORB_ERROR;
	}

	ORB_INFO("subscribe fd %d", sfd);
	u.val = 1;

	if (ORB_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		ORB_ERR("copy(1) failed: %d", orb_errno);
		return ORB_ERROR;
	}

	if (u.val != t.val) {
		ORB_ERR("copy(1) mismatch: %d expected %d", u.val, t.val);
		return ORB_ERROR;
	}

	if (ORB_OK != orb_check(sfd, &updated)) {
		ORB_ERR("check(1) failed");
		return ORB_ERROR;
	}

	if (updated) {
		ORB_ERR("spurious updated flag");
		return ORB_ERROR;
	}

	t.val = 2;
	ORB_INFO("try publish");

	if (ORB_OK != orb_publish(ORB_ID(orb_test), ptopic, &t)) {
		ORB_ERR("publish failed");
		return ORB_ERROR;
	}

	if (ORB_OK != orb_check(sfd, &updated)) {
		ORB_ERR("check(2) failed");
		return ORB_ERROR;
	}

	if (!updated) {
		ORB_ERR("missing updated flag");
		return ORB_ERROR;
	}

	if (ORB_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		ORB_ERR("copy(2) failed: %d", orb_errno);
		return ORB_ERROR;
	}

	if (u.val != t.val) {
		ORB_ERR("copy(2) mismatch: %d expected %d", u.val, t.val);
		return ORB_ERROR;
	}

	orb_unsubscribe(sfd);

	int ret = orb_unadvertise(ptopic);

	if (ret != ORB_OK) {
		ORB_ERR("orb_unadvertise failed: %i", ret);
          return ORB_ERROR;
	}

	ORB_INFO("PASS single-topic sample");

	return ORB_OK;
}

int uORBTest::UnitTest::test_multi()
{
	/* this routine tests the multi-topic support */
	ORB_INFO("try multi-topic support");

	struct orb_test t {}, u {};
	t.val = 0;
	int instance0;
	_pfd[0] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0, ORB_PRIO_MAX);

	ORB_INFO("advertised");

	int instance1;
	_pfd[1] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1, ORB_PRIO_MIN);

	if (instance0 != 0) {
		ORB_ERR("mult. id0: %d", instance0);
          return ORB_ERROR;
	}

	if (instance1 != 1) {
		ORB_ERR("mult. id1: %d", instance1);
          return ORB_ERROR;
	}

	t.val = 103;

	if (ORB_OK != orb_publish(ORB_ID(orb_multitest), _pfd[0], &t)) {
		ORB_ERR("mult. pub0 fail");
          return ORB_ERROR;
	}

	ORB_INFO("published");

	t.val = 203;

	if (ORB_OK != orb_publish(ORB_ID(orb_multitest), _pfd[1], &t)) {
		ORB_ERR("mult. pub1 fail");
          return ORB_ERROR;
	}

	/* subscribe to both topics and ensure valid data is received */
	int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);

	if (ORB_OK != orb_copy(ORB_ID(orb_multitest), sfd0, &u)) {
		ORB_ERR("sub #0 copy failed: %d", orb_errno);
          return ORB_ERROR;
	}

	if (u.val != 103) {
		ORB_ERR("sub #0 val. mismatch: %d", u.val);
          return ORB_ERROR;
	}

	int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1);

	if (ORB_OK != orb_copy(ORB_ID(orb_multitest), sfd1, &u)) {
		ORB_ERR("sub #1 copy failed: %d", orb_errno);
          return ORB_ERROR;
	}

	if (u.val != 203) {
		ORB_ERR("sub #1 val. mismatch: %d", u.val);
          return ORB_ERROR;
	}

	/* sample priorities */
	int prio;

	if (ORB_OK != orb_priority(sfd0, &prio)) {
		ORB_ERR("prio #0");
          return ORB_ERROR;
	}

	if (prio != ORB_PRIO_MAX) {
		ORB_ERR("prio: %d", prio);
          return ORB_ERROR;
	}

	if (ORB_OK != orb_priority(sfd1, &prio)) {
		ORB_ERR("prio #1");
          return ORB_ERROR;
	}

	if (prio != ORB_PRIO_MIN) {
		ORB_ERR("prio: %d", prio);
          return ORB_ERROR;
	}

	if (ORB_OK != latency_test<struct orb_test>(ORB_ID(orb_test), false)) {
		ORB_ERR("latency sample failed");
          return ORB_ERROR;
	}

	orb_unsubscribe(sfd0);
	orb_unsubscribe(sfd1);

	ORB_INFO("PASS multi-topic sample");

	return ORB_OK;
}



int uORBTest::UnitTest::pub_test_multi2_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_multi2_main();
}

int uORBTest::UnitTest::pub_test_multi2_main()
{
	int data_next_idx = 0;
	const int num_instances = 3;
	orb_advert_t orb_pub[num_instances];
	struct orb_test_medium data_topic{};

	for (int i = 0; i < num_instances; ++i) {
		orb_advert_t &pub = orb_pub[i];
		int idx = i;
//		PX4_WARN("advertise %i, t=%" PRIu64, i, hrt_absolute_time());
		pub = orb_advertise_multi(ORB_ID(orb_test_medium_multi), &data_topic, &idx, ORB_PRIO_DEFAULT);

		if (idx != i) {
			_thread_should_exit = true;
			ORB_ERR("Got wrong instance! should be: %i, but is %i", i, idx);
			return -1;
		}
	}

	usleep(100 * 1000);

	int message_counter = 0, num_messages = 50 * num_instances;

	while (message_counter++ < num_messages) {
		usleep(2); //make sure the timestamps are different
		orb_advert_t &pub = orb_pub[data_next_idx];

		data_topic.time = hrt_absolute_time();
		data_topic.val = data_next_idx;

		orb_publish(ORB_ID(orb_test_medium_multi), pub, &data_topic);
//		ORB_WARN("publishing msg (idx=%i, t=%" PRIu64 ")", data_next_idx, data_topic.time);

		data_next_idx = (data_next_idx + 1) % num_instances;

		if (data_next_idx == 0) {
			usleep(50 * 1000);
		}
	}

	usleep(100 * 1000);
	_thread_should_exit = true;

	for (auto & i : orb_pub) {
		orb_unadvertise(i);
	}

	return 0;
}

int uORBTest::UnitTest::test_multi2()
{
	ORB_INFO("Testing multi-topic 2 sample (queue simulation)");
	//sample: first subscribe, then advertise

	_thread_should_exit = false;
	const int num_instances = 3;
	int orb_data_fd[num_instances];
	int orb_data_next = 0;

	for (int i = 0; i < num_instances; ++i) {
//		ORB_WARN("subscribe %i, t=%" PRIu64, i, hrt_absolute_time());
		orb_data_fd[i] = orb_subscribe_multi(ORB_ID(orb_test_medium_multi), i);
	}

	char *const args[1] = { nullptr };
	pthread_t pubsub_task = px4_task_spawn_cmd("uorb_test_multi",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 5,
					     2000,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_multi2_entry,
					     args);

	if (pubsub_task < 0) {
		ORB_ERR("failed launching task");
          return ORB_ERROR;
	}

	hrt_abstime last_time = 0;

	while (!_thread_should_exit) {

		bool updated = false;
		int orb_data_cur_fd = orb_data_fd[orb_data_next];
		orb_check(orb_data_cur_fd, &updated);

		if (updated) {
			struct orb_test_medium msg{};
			orb_copy(ORB_ID(orb_test_medium_multi), orb_data_cur_fd, &msg);

// Relax timing requirement for Darwin CI system
#ifdef __PX4_DARWIN
			usleep(10000);
#else
			usleep(1000);
#endif

			if (last_time >= msg.time && last_time != 0) {
				ORB_ERR("Timestamp not increasing! (%" PRIu64
                                         " >= %" PRIu64 ")",
                                         last_time, msg.time);
				return ORB_ERROR;
			}

			last_time = msg.time;

//			ORB_WARN("      got message (val=%i, idx=%i, t=%" PRIu64 ")", msg.val, orb_data_next, msg.time);
			orb_data_next = (orb_data_next + 1) % num_instances;
		}
	}

	for (int i : orb_data_fd) {
		orb_unsubscribe(i);
	}

	ORB_INFO("PASS multi-topic 2 sample (queue simulation)");

	return ORB_OK;
}

int uORBTest::UnitTest::test_multi_reversed()
{
	ORB_INFO("try multi-topic support subscribing before publishing");

	/* For these tests 0 and 1 instances are taken from before, therefore continue with 2 and 3. */

	/* Subscribe first and advertise afterwards. */
	int sfd2 = orb_subscribe_multi(ORB_ID(orb_multitest), 2);

	if (sfd2 < 0) {
		ORB_ERR("sub. id2: ret: %d", sfd2);
		return ORB_ERROR;
	}

	struct orb_test t {}, u {};

	t.val = 0;

	int instance2;

	_pfd[2] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance2, ORB_PRIO_MAX);

	int instance3;

	_pfd[3] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance3, ORB_PRIO_MIN);

	ORB_INFO("advertised");

	if (instance2 != 2) {
		ORB_ERR("mult. id2: %d", instance2);
		return ORB_ERROR;
	}

	if (instance3 != 3) {
		ORB_ERR("mult. id3: %d", instance3);
		return ORB_ERROR;
	}

	t.val = 204;

	if (ORB_OK != orb_publish(ORB_ID(orb_multitest), _pfd[2], &t)) {
		ORB_ERR("mult. pub0 fail");
		return ORB_ERROR;
	}


	t.val = 304;

	if (ORB_OK != orb_publish(ORB_ID(orb_multitest), _pfd[3], &t)) {
		ORB_ERR("mult. pub1 fail");
		return ORB_ERROR;
	}

	ORB_INFO("published");

	if (ORB_OK != orb_copy(ORB_ID(orb_multitest), sfd2, &u)) {
		ORB_ERR("sub #2 copy failed: %d", orb_errno);
		return ORB_ERROR;
	}

	if (u.val != 204) {
		ORB_ERR("sub #3 val. mismatch: %d", u.val);
		return ORB_ERROR;
	}

	int sfd3 = orb_subscribe_multi(ORB_ID(orb_multitest), 3);

	if (ORB_OK != orb_copy(ORB_ID(orb_multitest), sfd3, &u)) {
		ORB_ERR("sub #3 copy failed: %d", orb_errno);
		return ORB_ERROR;
	}

	if (u.val != 304) {
		ORB_ERR("sub #3 val. mismatch: %d", u.val);
		return ORB_ERROR;
	}

	ORB_INFO("PASS multi-topic reversed");

	return ORB_OK;
}

int uORBTest::UnitTest::test_queue()
{
	ORB_INFO("Testing orb queuing");

	struct orb_test_medium t{}, u{};
	int sfd;
	orb_advert_t ptopic;
	bool updated;

	sfd = orb_subscribe(ORB_ID(orb_test_medium_queue));

	if (sfd < 0) {
		ORB_ERR("subscribe failed: %d", orb_errno);
		return ORB_ERROR;
	}

	const int queue_size = 11;
	t.val = 0;
	ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue), &t, queue_size);

	if (ptopic == nullptr) {
		ORB_ERR("advertise failed: %d", orb_errno);
		return ORB_ERROR;
	}

	orb_check(sfd, &updated);

	if (!updated) {
		ORB_ERR("update flag not set");
		return ORB_ERROR;
	}

	if (ORB_OK != orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u)) {
		ORB_ERR("copy(1) failed: %d", orb_errno);
		return ORB_ERROR;
	}

	if (u.val != t.val) {
		ORB_ERR("copy(1) mismatch: %d expected %d", u.val, t.val);
		return ORB_ERROR;
	}

	orb_check(sfd, &updated);

	if (updated) {
		ORB_ERR("spurious updated flag");
		return ORB_ERROR;
	}

#define CHECK_UPDATED(element) \
	orb_check(sfd, &updated); \
	if (!updated) { \
		ORB_ERR("update flag not set, element %i", element); \
		return ORB_ERROR; \
	}
#define CHECK_NOT_UPDATED(element) \
	orb_check(sfd, &updated); \
	if (updated) { \
		ORB_ERR("update flag set, element %i", element); \
		return ORB_ERROR; \
	}
#define CHECK_COPY(i_got, i_correct) \
	orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u); \
	if (i_got != i_correct) { \
		ORB_ERR("got wrong element from the queue (got %i, should be %i)", i_got, i_correct); \
		return ORB_ERROR; \
	}

	//no messages in the queue anymore

	ORB_INFO("  Testing to write some elements...");

	for (int i = 0; i < queue_size - 2; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size - 2; ++i) {
          CHECK_UPDATED(i);
		CHECK_COPY(u.val, i);
	}

        CHECK_NOT_UPDATED(queue_size);

	ORB_INFO("  Testing overflow...");
	int overflow_by = 3;

	for (int i = 0; i < queue_size + overflow_by; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size; ++i) {
          CHECK_UPDATED(i);
		CHECK_COPY(u.val, i + overflow_by);
	}

        CHECK_NOT_UPDATED(queue_size);

	ORB_INFO("  Testing underflow...");

	for (int i = 0; i < queue_size; ++i) {
          CHECK_NOT_UPDATED(i);
		CHECK_COPY(u.val, queue_size + overflow_by - 1);
	}

	t.val = 943;
	orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
        CHECK_UPDATED(-1);
	CHECK_COPY(u.val, t.val);

#undef CHECK_COPY
#undef CHECK_UPDATED
#undef CHECK_NOT_UPDATED

	orb_unadvertise(ptopic);

	ORB_INFO("PASS orb queuing");

	return ORB_OK;
}


int uORBTest::UnitTest::pub_test_queue_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_queue_main();
}

int uORBTest::UnitTest::pub_test_queue_main()
{
	struct orb_test_medium t{};
	orb_advert_t ptopic;
	const int queue_size = 50;
	t.val = 0;

	if ((ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue_poll), &t, queue_size)) == nullptr) {
		_thread_should_exit = true;
		ORB_ERR("advertise failed: %d", orb_errno);
		return ORB_ERROR;
	}

	int message_counter = 0, num_messages = 20 * queue_size;
	++t.val;

	while (message_counter < num_messages) {

		//simulate burst
		int burst_counter = 0;

		while (burst_counter++ < queue_size / 2 + 7) { //make interval non-boundary aligned
			orb_publish(ORB_ID(orb_test_medium_queue_poll), ptopic, &t);
			++t.val;
		}

		message_counter += burst_counter;
		usleep(20 * 1000); //give subscriber a chance to catch up
	}

	_num_messages_sent = t.val;
	usleep(100 * 1000);
	_thread_should_exit = true;
	orb_unadvertise(ptopic);

	return 0;
}

int uORBTest::UnitTest::test_queue_poll_notify()
{
	ORB_INFO("Testing orb queuing (poll & notify)");

	struct orb_test_medium t{};
	int sfd;

	if ((sfd = orb_subscribe(ORB_ID(orb_test_medium_queue_poll))) < 0) {
		ORB_ERR("subscribe failed: %d", orb_errno);
		return ORB_ERROR;
	}

	_thread_should_exit = false;

	char *const args[1] = { nullptr };
	pthread_t pubsub_task = px4_task_spawn_cmd("uorb_test_queue",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MIN + 5,
					     1500,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_queue_entry,
					     args);

	if (pubsub_task < 0) {
		ORB_ERR("failed launching task");
		return ORB_ERROR;
	}

	int next_expected_val = 0;
	 orb_pollfd_struct_t fds[1];
	fds[0].fd = sfd;
	fds[0].events = POLLIN;

	while (!_thread_should_exit) {

		int poll_ret =  orb_poll(fds, 1, 500);

		if (poll_ret == 0) {
			if (_thread_should_exit) {
				break;
			}

			ORB_ERR("poll timeout");
		return ORB_ERROR;

		} else if (poll_ret < 0) {
			ORB_ERR("poll error (%d, %d)", poll_ret, orb_errno);
		return ORB_ERROR;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium_queue_poll), sfd, &t);

			if (next_expected_val != t.val) {
				ORB_ERR("copy mismatch: %d expected %d", t.val,
                                         next_expected_val);
				return ORB_ERROR;
			}

			++next_expected_val;
		}
	}

	if (_num_messages_sent != next_expected_val) {
		ORB_ERR("number of sent and received messages mismatch (sent: %i, received: %i)",
                         _num_messages_sent, next_expected_val);
		return ORB_ERROR;
	}

	ORB_INFO("PASS orb queuing (poll & notify), got %i messages", next_expected_val);

	return ORB_OK;
}

int uORBTest::UnitTest::pubsubtest_threadEntry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pubsublatency_main();
}
