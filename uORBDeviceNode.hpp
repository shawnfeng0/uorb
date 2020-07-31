/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include "base/List.hpp"
#include "base/cdev_platform.hpp"
#include "base/orb_atomic.hpp"
#include "uORBCommon.hpp"
#include "uORBDeviceMaster.hpp"

namespace uORB {
class DeviceNode;
class DeviceMaster;
class Manager;
}  // namespace uORB

/**
 * Per-object device instance.
 */
class uORB::DeviceNode : public ListNode<uORB::DeviceNode *> {
 public:
  DeviceNode(const struct orb_metadata *meta, uint8_t instance,
             const char *path, ORB_PRIO priority, uint8_t queue_size = 1);
  ~DeviceNode();

  /* do not allow copying this class */
  // no copy, assignment, move, move assignment
  DeviceNode(const DeviceNode &) = delete;
  DeviceNode &operator=(const DeviceNode &) = delete;
  DeviceNode(DeviceNode &&) = delete;
  DeviceNode &operator=(DeviceNode &&) = delete;

  int init();

  /**
   * Method to create a subscriber instance and return the struct
   * pointing to the subscriber as a file pointer.
   */
  int open(cdev::file_t *filp);

  /**
   * Method to close a subscriber for this topic.
   */
  int close(cdev::file_t *filp);

  /**
   * reads data from a subscriber node to the buffer provided.
   * @param filp
   *   The subscriber from which the data needs to be read from.
   * @param buffer
   *   The buffer into which the data is read into.
   * @param buflen
   *   the length of the buffer
   * @return
   *   ssize_t the number of bytes read.
   */
  ssize_t read(cdev::file_t *filp, char *buffer, size_t buflen);

  /**
   * writes the published data to the internal buffer to be read by
   * subscribers later.
   * @param buffer
   *   The buffer for the input data
   * @param buflen
   *   the length of the buffer.
   * @return ssize_t
   *   The number of bytes that are written
   */
  ssize_t write(const char *buffer, size_t buflen);

  /**
   * IOCTL control for the subscriber.
   * Perform an ioctl operation on the device.
   *
   * The default implementation handles DIOC_GETPRIV, and otherwise
   * returns -ENOTTY. Subclasses should call the default implementation
   * for any command they do not handle themselves.
   *
   * @param filep		Pointer to the NuttX file structure.
   * @param cmd		The ioctl command value.
   * @param arg		The ioctl argument value.
   * @return		ORB_OK on success, or -errno otherwise.
   */
  int ioctl(cdev::file_t *filp, int cmd, unsigned long arg);

  /**
   * Perform a poll setup/teardown operation.
   *
   * This is handled internally and should not normally be overridden.
   *
   * @param filep		Pointer to the internal file structure.
   * @param fds		Poll descriptor being waited on.
   * @param setup		True if this is establishing a request, false if
   *			it is being torn down.
   * @return		ORB_OK on success, or -errno otherwise.
   */
  int poll(cdev::file_t *filep, orb_pollfd_t *fds, bool setup);

  /**
   * Get the device name.
   *
   * @return the file system string of the device handle
   */
  const char *get_devname() const { return _devname; }

  /**
   * Method to publish a data to this node.
   */
  static ssize_t publish(const orb_metadata *meta, orb_advert_t handle,
                         const void *data);

  static int unadvertise(orb_advert_t handle);

  /**
   * Add the subscriber to the node's list of subscriber.  If there is
   * remote proxy to which this subscription needs to be sent, it will
   * done via uORBCommunicator::IChannel interface.
   * @param sd
   *   the subscriber to be added.
   */
  void add_internal_subscriber();

  /**
   * Removes the subscriber from the list.  Also notifies the remote
   * if there a uORBCommunicator::IChannel instance.
   * @param sd
   *   the Subscriber to be removed.
   */
  void remove_internal_subscriber();

  /**
   * Return true if this topic has been advertised.
   *
   * This is used in the case of multi_pub/sub to check if it's valid to
   * advertise and publish to this node or if another node should be tried. */
  bool is_advertised() const { return _advertised; }

  void mark_as_advertised() { _advertised = true; }

  /**
   * Try to change the size of the queue. This can only be done as long as
   * nobody published yet. This is the case, for example when orb_subscribe was
   * called before an orb_advertise. The queue size can only be increased.
   * @param queue_size new size of the queue
   * @return ORB_OK if queue size successfully set
   */
  int update_queue_size(unsigned int queue_size);

  /**
   * Print statistics (nr of lost messages)
   * @param reset if true, reset statistics afterwards
   * @return true if printed something, false otherwise (if no lost messages)
   */
  bool print_statistics(bool reset);

  uint8_t get_queue_size() const { return _queue_size; }

  int8_t subscriber_count() const { return _subscriber_count; }

  uint32_t lost_message_count() const { return _lost_messages; }

  unsigned published_message_count() const { return _generation.load(); }

  const orb_metadata *get_meta() const { return _meta; }

  ORB_ID id() const { return static_cast<ORB_ID>(_meta->o_id); }

  const char *get_name() const { return _meta->o_name; }

  uint8_t get_instance() const { return _instance; }

  ORB_PRIO get_priority() const { return _priority; }
  void set_priority(ORB_PRIO priority) { _priority = priority; }

  /**
   * Copies data and the corresponding generation
   * from a node to the buffer provided.
   *
   * @param dst
   *   The buffer into which the data is copied.
   * @param generation
   *   The generation that was copied.
   * @return bool
   *   Returns true if the data was copied.
   */
  bool copy(void *dst, unsigned &generation);

 protected:
  /**
   * Check the current state of the device for poll events from the
   * perspective of the file.
   *
   * This function is called by the default poll() implementation when
   * a poll is set up to determine whether the poll should return
   * immediately.
   *
   * The default implementation returns no events.
   *
   * @param filep		The file that's interested.
   * @return		The current set of poll events.
   */
  pollevent_t poll_state(cdev::file_t *filp);

  /**
   * Report new poll events.
   *
   * This function should be called anytime the state of the device changes
   * in a fashion that might be interesting to a poll waiter.
   *
   * @param events	The new event(s) being announced.
   */
  void poll_notify(pollevent_t events);

  /**
   * Internal implementation of poll_notify.
   *
   * @param fds		A poll waiter to notify.
   * @param events	The event(s) to send to the waiter.
   */
  void poll_notify_one(orb_pollfd_t *fds, pollevent_t events);

 private:
  /**
   * Copies data and the corresponding generation
   * from a node to the buffer provided. Caller handles locking.
   *
   * @param dst
   *   The buffer into which the data is copied.
   * @param generation
   *   The generation that was copied.
   * @return bool
   *   Returns true if the data was copied.
   */
  bool copy_locked(void *dst, unsigned &generation);

  struct UpdateIntervalData {
    uint64_t last_update{0}; /**< time at which the last update was provided,
                                used when update_interval is nonzero */
    unsigned interval{0};    /**< if nonzero minimum interval between updates */
  };

  struct SubscriberData {
    ~SubscriberData() {
      if (update_interval) {
        delete (update_interval);
      }
    }

    unsigned generation{0}; /**< last generation the subscriber has seen */
    UpdateIntervalData *update_interval{
        nullptr}; /**< if null, no update interval */
  };

  const orb_metadata *_meta;   /**< object metadata information */
  uint8_t *_data{nullptr};     /**< allocated object buffer */
  uORB::base::atomic<unsigned> _generation{0}; /**< object generation count */

  // statistics
  uint32_t _lost_messages =
      0; /**< nr of lost messages for all subscribers. If two subscribers lose
            the same message, it is counted as two. */

  uORB::base::Mutex _lock; /**< lock to protect access to all class members
                      (also for derived classes) */

  const char *_devname{nullptr}; /**< device node name */

  orb_pollfd_t **_pollset{nullptr};

  bool _registered{false}; /**< true if device name was registered */

  uint8_t _max_pollwaiters{0}; /**< size of the _pollset array */

  ORB_PRIO _priority;      /**< priority of the topic */
  const uint8_t _instance; /**< orb multi instance identifier */
  bool _advertised{false}; /**< has ever been advertised (not necessarily
                              published data yet) */
  uint8_t _queue_size;     /**< maximum number of elements in the queue */
  int8_t _subscriber_count{0};

  /**
   * Check whether a topic appears updated to a subscriber.
   *
   * Lock must already be held when calling this.
   *
   * @param sd    The subscriber for whom to check.
   * @return    True if the topic should appear updated to the subscriber
   */
  bool      appears_updated(cdev::file_t *filp);

  /**
   * Store a pollwaiter in a slot where we can find it later.
   *
   * Expands the pollset as required.  Must be called with the driver locked.
   *
   * @return		ORB_OK, or -errno on error.
   */
  inline int store_poll_waiter(orb_pollfd_t *fds);

  /**
   * Remove a poll waiter.
   *
   * @return		ORB_OK, or -errno on error.
   */
  inline int remove_poll_waiter(orb_pollfd_t *fds);

  /**
   * First, unregisters the driver. Next, free the memory for the devname,
   * in case it was expected to have ownership. Sets devname to nullptr.
   *
   * This is only needed if the ownership of the devname was passed to the CDev,
   * otherwise ~CDev handles it.
   *
   * @return  ORB_OK on success, -ENODEV if the devname is already nullptr
   */
  int unregister_driver_and_memory();
};
