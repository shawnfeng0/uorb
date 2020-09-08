/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#include "uorb/base/condition_variable.h"
#include "uorb/base/intrusive_list.h"
#include "uorb/base/mutex.h"
#include "uorb/base/rw_mutex.h"
#include "uorb/uorb.h"

namespace uorb {
class DeviceMaster;

/**
 * Per-object device instance.
 */
class DeviceNode : public ListNode<DeviceNode *> {
  friend DeviceMaster;

 public:
  class Callback : public ListNode<Callback *> {
   public:
    void operator()() { call(); }

   protected:
    virtual void call() = 0;
    virtual ~Callback() = default;
  };

  class SemaphoreCallback : public base::SimpleSemaphore<CLOCK_MONOTONIC>,
                            public Callback {
   public:
    explicit SemaphoreCallback() : SimpleSemaphore(0) {}
    void call() override { release(); }
  };

  /* do not allow copying this class */
  // no copy, assignment, move, move assignment
  DeviceNode(const DeviceNode &) = delete;
  DeviceNode &operator=(const DeviceNode &) = delete;
  DeviceNode(DeviceNode &&) = delete;
  DeviceNode &operator=(DeviceNode &&) = delete;

  /**
   * Method to publish a data to this node.
   */
  bool Publish(const void *data);

  /**
   * Add the subscriber to the node's list of subscriber.  If there is
   * remote proxy to which this subscription needs to be sent, it will
   * done via uORBCommunicator::IChannel interface.
   * @param sd
   *   the subscriber to be added.
   */
  void IncreaseSubscriberCount();

  /**
   * Removes the subscriber from the list.  Also notifies the remote
   * if there a uORBCommunicator::IChannel instance.
   * @param sd
   *   the Subscriber to be removed.
   */
  void ReduceSubscriberCount();

  // Whether meta and instance are the same as the current one
  bool IsSameWith(const orb_metadata &meta, uint8_t instance) const;
  bool IsSameWith(const orb_metadata &meta) const;

  // add item to list of work items to schedule on node update
  bool RegisterCallback(Callback *callback);

  // remove item from list of work items
  void UnregisterCallback(Callback *callback);

  /**
   * Return true if this topic has been advertised.
   *
   * This is used in the case of multi_pub/sub to check if it's valid to
   * advertise and publish to this node or if another node should be tried. */
  bool is_advertised() const { return advertised_; }

  /*
   * We are cheating a bit here. First, with the current implementation, we can
   * only have multiple publishers for instance 0. In this case the caller will
   * have instance=nullptr and _published has no effect at all. Thus no
   * unadvertise is necessary. In case of multiple instances, we have at most 1
   * publisher per instance and we can signal an instance as 'free' by setting
   * _published to false. We never really free the DeviceNode, for this we would
   * need reference counting of subscribers and publishers. But we also do not
   * have a leak since future publishers reuse the same DeviceNode object.
   */
  void mark_as_unadvertised() { advertised_ = false; }
  void mark_as_advertised() { advertised_ = true; }

  unsigned oldest_data_index() const {
    return (queue_size_ < generation_) ? generation_ - queue_size_ : 0;
  }

  uint16_t queue_size() const { return queue_size_; }
  bool set_queue_size(uint16_t queue_size) {
    base::WriterLockGuard lg(lock_);

    if (data_ || queue_size_ > queue_size) {
      return false;
    }

    queue_size_ = RoundUpPowOfTwo(queue_size);
    return true;
  }

  const char *name() const { return meta_.o_name; }
  uint8_t instance() const { return instance_; }

  /**
   * Copies data and the corresponding generation
   * from a node to the buffer provided.
   *
   * @param dst
   *   The buffer into which the data is copied.
   * @param sub_generation
   *   The generation that was copied.
   * @return bool
   *   Returns true if the data was copied.
   */
  bool Copy(void *dst, unsigned &sub_generation);

  /**
   * Check if there is newer data than sub_generation
   * @param sub_generation
   * @return
   *   Returns true if have new data.
   */
  bool CheckUpdate(const unsigned &sub_generation) const;

 private:
  const orb_metadata &meta_; /**< object metadata information */
  const uint8_t instance_;   /**< orb multi instance identifier */

  uint8_t *data_{nullptr}; /**< allocated object buffer */
  uint16_t queue_size_;    /**< maximum number of elements in the queue */
  unsigned generation_{0}; /**< object generation count */
  bool queue_is_full_{false};

  base::RwMutex lock_; /**< lock to protect access to all class members
  (also for derived classes) */

  uint8_t subscriber_count_{0};
  List<Callback *> callbacks_;

  bool advertised_{false}; /**< has ever been advertised (not necessarily
                              published data yet) */

  DeviceNode(const struct orb_metadata &meta, uint8_t instance,
             uint16_t queue_size = 1);
  ~DeviceNode();

  // round up to nearest power of two
  static uint64_t RoundUpPowOfTwo(uint64_t n);
};
}  // namespace uorb
