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

#ifndef _uORBManager_hpp_
#define _uORBManager_hpp_

#include <stdint.h>

#include "base/orb_mutex.hpp"
#include "uORBCommon.hpp"
#include "uORBDeviceMaster.hpp"

namespace uORB {
class Manager;
}

/**
 * This is implemented as a singleton.  This class manages creating the
 * uORB nodes for each uORB topics and also implements the behavor of the
 * uORB Api's.
 */
class uORB::Manager
{
 public:
  // public interfaces for this class.

  /**
   * Method to get the singleton instance for the uORB::Manager.
   * @return uORB::Manager*
   */
  static inline uORB::Manager &get_instance() {
    return Instance;
  }

  /**
   * Get the DeviceMaster. If it does not exist,
   * it will be created and initialized.
   * Note: the first call to this is not thread-safe.
   * @return nullptr if initialization failed (and errno will be set)
   */
  uORB::DeviceMaster *get_device_master();

  // ==== uORB interface methods ====
  /**
   * Advertise as the publisher of a topic.
   *
   * This performs the initial advertisement of a topic; it creates the topic
   * node in /obj if required and publishes the initial data.
   *
   * Any number of advertisers may publish to a topic; publications are atomic
   * but co-ordination between publishers is not provided by the ORB.
   *
   * Internally this will call orb_advertise_multi with an instance of 0 and
   * default priority.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @param data    A pointer to the initial data to be published.
   *      For topics updated by interrupt handlers, the advertisement
   *      must be performed from non-interrupt context.
   * @param queue_size  Maximum number of buffered elements. If this is 1, no
   * queuing is used.
   * @return    nullptr on error, otherwise returns an object pointer
   *      that can be used to publish to the topic.
   *      If the topic in question is not known (due to an
   *      ORB_DEFINE with no corresponding ORB_DECLARE)
   *      this function will return nullptr and set errno to ENOENT.
   */
  orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data,
                             unsigned int queue_size = 1) {
    return orb_advertise_multi(meta, data, nullptr, ORB_PRIO_DEFAULT,
                               queue_size);
  }

  /**
   * Advertise as the publisher of a topic.
   *
   * This performs the initial advertisement of a topic; it creates the topic
   * node in /obj if required and publishes the initial data.
   *
   * Any number of advertisers may publish to a topic; publications are atomic
   * but co-ordination between publishers is not provided by the ORB.
   *
   * The multi can be used to create multiple independent instances of the same
   * topic (each instance has its own buffer). This is useful for multiple
   * publishers who publish the same topic. The subscriber then subscribes to
   * all instances and chooses which source he wants to use.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @param data    A pointer to the initial data to be published.
   *      For topics updated by interrupt handlers, the advertisement
   *      must be performed from non-interrupt context.
   * @param instance  Pointer to an integer which will yield the instance ID
   * (0-based) of the publication. This is an output parameter and will be set
   * to the newly created instance, ie. 0 for the first advertiser, 1 for the
   * next and so on.
   * @param priority  The priority of the instance. If a subscriber subscribes
   * multiple instances, the priority allows the subscriber to prioritize the
   * best data source as long as its available. The subscriber is responsible to
   * check and handle different priorities (@see orb_priority()).
   * @param queue_size  Maximum number of buffered elements. If this is 1, no
   * queuing is used.
   * @return    ORB_ERROR on error, otherwise returns a handle
   *      that can be used to publish to the topic.
   *      If the topic in question is not known (due to an
   *      ORB_DEFINE with no corresponding ORB_DECLARE)
   *      this function will return -1 and set errno to ENOENT.
   */
  orb_advert_t orb_advertise_multi(const struct orb_metadata *meta,
                                   const void *data, int *instance,
                                   ORB_PRIO priority, unsigned int queue_size = 1);

  /**
   * Unadvertise a topic.
   *
   * @param handle  handle returned by orb_advertise or orb_advertise_multi.
   * @return 0 on success
   */
  static int orb_unadvertise(orb_advert_t handle);

  /**
   * Publish new data to a topic.
   *
   * The data is atomically published to the topic and any waiting subscribers
   * will be notified.  Subscribers that are not waiting can check the topic
   * for updates using orb_check.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @handle    The handle returned from orb_advertise.
   * @param data    A pointer to the data to be published.
   * @return    ORB_OK on success, ORB_ERROR otherwise with errno set
   * accordingly.
   */
  static int orb_publish(const struct orb_metadata *meta, orb_advert_t handle,
                         const void *data);

  /**
   * Subscribe to a topic.
   *
   * The returned value is a file descriptor that can be passed to poll()
   * in order to wait for updates to a topic, as well as topic_read,
   * orb_check.
   *
   * If there were any publications of the topic prior to the subscription,
   * an orb_check right after orb_subscribe will return true.
   *
   * Subscription will succeed even if the topic has not been advertised;
   * in this case the topic will have a timestamp of zero, it will never
   * signal a poll() event, checking will always return false and it cannot
   * be copied. When the topic is subsequently advertised, poll, check,
   * stat and copy calls will react to the initial publication that is
   * performed as part of the advertisement.
   *
   * Subscription will fail if the topic is not known to the system, i.e.
   * there is nothing in the system that has declared the topic and thus it
   * can never be published.
   *
   * Internally this will call orb_subscribe_multi with instance 0.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @return    ORB_ERROR on error, otherwise returns a handle
   *      that can be used to read and update the topic.
   */
  int orb_subscribe(const struct orb_metadata *meta);

  /**
   * Subscribe to a multi-instance of a topic.
   *
   * The returned value is a file descriptor that can be passed to poll()
   * in order to wait for updates to a topic, as well as topic_read,
   * orb_check.
   *
   * If there were any publications of the topic prior to the subscription,
   * an orb_check right after orb_subscribe_multi will return true.
   *
   * Subscription will succeed even if the topic has not been advertised;
   * in this case the topic will have a timestamp of zero, it will never
   * signal a poll() event, checking will always return false and it cannot
   * be copied. When the topic is subsequently advertised, poll, check,
   * stat and copy calls will react to the initial publication that is
   * performed as part of the advertisement.
   *
   * Subscription will fail if the topic is not known to the system, i.e.
   * there is nothing in the system that has declared the topic and thus it
   * can never be published.
   *
   * If a publisher publishes multiple instances the subscriber should
   * subscribe to each instance with orb_subscribe_multi
   * (@see orb_advertise_multi()).
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @param instance  The instance of the topic. Instance 0 matches the
   *      topic of the orb_subscribe() call, higher indices
   *      are for topics created with orb_advertise_multi().
   * @return    ORB_ERROR on error, otherwise returns a handle
   *      that can be used to read and update the topic.
   *      If the topic in question is not known (due to an
   *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
   *      this function will return -1 and set errno to ENOENT.
   */
  int orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance);

  /**
   * Unsubscribe from a topic.
   *
   * @param handle  A handle returned from orb_subscribe.
   * @return    ORB_OK on success, ORB_ERROR otherwise with errno set
   * accordingly.
   */
  static int orb_unsubscribe(int handle);

  /**
   * Fetch data from a topic.
   *
   * This is the only operation that will reset the internal marker that
   * indicates that a topic has been updated for a subscriber. Once poll
   * or check return indicating that an updaet is available, this call
   * must be used to update the subscription.
   *
   * @param meta    The uORB metadata (usually from the ORB_ID() macro)
   *      for the topic.
   * @param handle  A handle returned from orb_subscribe.
   * @param buffer  Pointer to the buffer receiving the data, or NULL
   *      if the caller wants to clear the updated flag without
   *      using the data.
   * @return    ORB_OK on success, ORB_ERROR otherwise with errno set
   * accordingly.
   */
  static int orb_copy(const struct orb_metadata *meta, int handle,
                      void *buffer);

  /**
   * Check whether a topic has been published to since the last orb_copy.
   *
   * This check can be used to determine whether to copy the topic when
   * not using poll(), or to avoid the overhead of calling poll() when the
   * topic is likely to have updated.
   *
   * Updates are tracked on a per-handle basis; this call will continue to
   * return true until orb_copy is called using the same handle.
   *
   * @param handle  A handle returned from orb_subscribe.
   * @param updated Set to true if the topic has been updated since the
   *      last time it was copied using this handle.
   * @return    OK if the check was successful, PX4_ERROR otherwise with
   *      errno set accordingly.
   */
  int  orb_check(int handle, bool *updated);

  /**
   * Check if a topic has already been created and published (advertised)
   *
   * @param meta    ORB topic metadata.
   * @param instance  ORB instance
   * @return    ORB_OK if the topic exists, ORB_ERROR otherwise.
   */
  int orb_exists(const struct orb_metadata *meta, int instance);

  /**
   * Return the priority of the topic
   *
   * @param handle  A handle returned from orb_subscribe.
   * @param priority  Returns the priority of this topic. This is only relevant
   * for topics which are published by multiple publishers (e.g. mag0, mag1,
   * etc.) and allows a subscriber to pick the topic with the highest priority,
   *      independent of the startup order of the associated publishers.
   * @return    ORB_OK on success, ORB_ERROR otherwise with errno set
   * accordingly.
   */
  static int orb_priority(int handle, ORB_PRIO *priority);

  /**
   * Set the minimum interval between which updates are seen for a subscription.
   *
   * If this interval is set, the subscriber will not see more than one update
   * within the period.
   *
   * Specifically, the first time an update is reported to the subscriber a
   * timer is started. The update will continue to be reported via poll and
   * orb_check, but once fetched via orb_copy another update will not be
   * reported until the timer expires.
   *
   * This feature can be used to pace a subscriber that is watching a topic that
   * would otherwise update too quickly.
   *
   * @param handle  A handle returned from orb_subscribe.
   * @param interval  An interval period in milliseconds.
   * @return    ORB_OK on success, ORB_ERROR otherwise with ERRNO set
   * accordingly.
   */
  static int orb_set_interval(int handle, unsigned interval);

  /**
   * Get the minimum interval between which updates are seen for a subscription.
   *
   * @see orb_set_interval()
   *
   * @param handle  A handle returned from orb_subscribe.
   * @param interval  The returned interval period in milliseconds.
   * @return    ORB_OK on success, ORB_ERROR otherwise with ERRNO set
   * accordingly.
   */
  static int orb_get_interval(int handle, unsigned *interval);

 private:  // class methods
  /**
   * Common implementation for orb_advertise and orb_subscribe.
   *
   * Handles creation of the object and the initial publication for
   * advertisers.
   */
  int node_open(const struct orb_metadata *meta, bool advertiser,
                int *instance = nullptr, ORB_PRIO priority = ORB_PRIO_DEFAULT);

 private:  // data members
  static Manager Instance;

  DeviceMaster _device_master{};

 private:  // class methods
  Manager();
  virtual ~Manager();
};

#endif /* _uORBManager_hpp_ */
