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

#include "uORBDeviceNode.hpp"

#include <base/orb_defines.h>

#include "base/orb_errno.h"
#include "base/orb_log.h"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

uORB::DeviceNode::SubscriberData *uORB::DeviceNode::filp_to_sd(
    cdev::file_t *filp) {
#ifndef __PX4_NUTTX

  if (!filp) {
    return nullptr;
  }

#endif
  return (SubscriberData *)(filp->f_priv);
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta,
                             uint8_t instance, const char *path,
                             uint8_t priority, uint8_t queue_size)
    : _devname(path),
      _meta(meta),
      _instance(instance),
      _priority(priority),
      _queue_size(queue_size) {
  ORB_DEBUG("CDev::CDev");
}

uORB::DeviceNode::~DeviceNode() {
  if (_data != nullptr) {
    delete[] _data;
  }
  unregister_driver_and_memory();

  ORB_DEBUG("CDev::~CDev");

  if (_registered) {
    unregister_driver(_devname);
  }

  if (_pollset) {
    delete[](_pollset);
  }
}

int uORB::DeviceNode::open(cdev::file_t *filp) {
  /* is this a publisher? */
  if (filp->f_oflags == PX4_F_WRONLY) {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);
    mark_as_advertised();

    /* now complete the open */
    return ORB_OK;
  }

  /* is this a new subscriber? */
  if (filp->f_oflags == PX4_F_RDONLY) {
    /* allocate subscriber data */
    auto *sd = new SubscriberData{};

    if (nullptr == sd) {
      return -ENOMEM;
    }

    /* If there were any previous publications, allow the subscriber to read
     * them */
    const unsigned gen = published_message_count();
    sd->generation =
        gen - (_queue_size < gen
                   ? _queue_size
                   : gen);  // = (_queue_size < gen ? gen - _queue_size : 0);

    filp->f_priv = (void *)sd;

    add_internal_subscriber();

    return ORB_OK;
  }

  /* can only be pub or sub, not both */
  return -EINVAL;
}

int uORB::DeviceNode::close(cdev::file_t *filp) {
  if (filp->f_oflags == PX4_F_RDONLY) { /* subscriber */
    SubscriberData *sd = filp_to_sd(filp);

    if (sd != nullptr) {
      remove_internal_subscriber();
      delete sd;
    }
  }

  return ORB_OK;
}

bool uORB::DeviceNode::copy_locked(void *dst, unsigned &generation) {
  bool updated = false;

  if ((dst != nullptr) && (_data != nullptr)) {
    unsigned current_generation = _generation.load();

    if (current_generation > generation + _queue_size) {
      // Reader is too far behind: some messages are lost
      _lost_messages += current_generation - (generation + _queue_size);
      generation = current_generation - _queue_size;
    }

    if ((current_generation == generation) && (generation > 0)) {
      /* The subscriber already read the latest message, but nothing new was
       * published yet. Return the previous message
       */
      --generation;
    }

    memcpy(dst, _data + (_meta->o_size * (generation % _queue_size)),
           _meta->o_size);

    if (generation < current_generation) {
      ++generation;
    }

    updated = true;
  }

  return updated;
}

bool uORB::DeviceNode::copy(void *dst, unsigned &generation) {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);

  bool updated = copy_locked(dst, generation);

  return updated;
}

uint64_t uORB::DeviceNode::copy_and_get_timestamp(void *dst,
                                                  unsigned &generation) {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);

  const hrt_abstime update_time = _last_update;
  copy_locked(dst, generation);

  return update_time;
}

ssize_t uORB::DeviceNode::read(cdev::file_t *filp, char *buffer,
                               size_t buflen) {
  /* if the object has not been written yet, return zero */
  if (_data == nullptr) {
    return 0;
  }

  /* if the caller's buffer is the wrong size, that's an error */
  if (buflen != _meta->o_size) {
    return -EIO;
  }

  auto *sd = (SubscriberData *)filp_to_sd(filp);

  /*
   * Perform an atomic copy & state update
   */
  {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);
    copy_locked(buffer, sd->generation);

    // if subscriber has an interval track the last update time
    if (sd->update_interval) {
      sd->update_interval->last_update = _last_update;
    }
  }

  return _meta->o_size;
}

ssize_t uORB::DeviceNode::write(const char *buffer, size_t buflen) {
  /*
   * Writes are legal from interrupt context as long as the
   * object has already been initialised from thread context.
   *
   * Writes outside interrupt context will allocate the object
   * if it has not yet been allocated.
   *
   * Note that filp will usually be NULL.
   */
  if (nullptr == _data) {
#ifdef __PX4_NUTTX

    if (!up_interrupt_context()) {
#endif /* __PX4_NUTTX */

      {
        // Automatic mutex guarding
        uORB::base::MutexGuard lg(_lock);

        /* re-check size */
        if (nullptr == _data) {
          _data = new uint8_t[_meta->o_size * _queue_size];
        }
      }

#ifdef __PX4_NUTTX
    }

#endif /* __PX4_NUTTX */

    /* failed or could not allocate */
    if (nullptr == _data) {
      return -ENOMEM;
    }
  }

  /* If write size does not match, that is an error */
  if (_meta->o_size != buflen) {
    return -EIO;
  }

  /* Perform an atomic copy. */
  {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);

    /* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
    unsigned generation = _generation.fetch_add(1);

    memcpy(_data + (_meta->o_size * (generation % _queue_size)), buffer,
           _meta->o_size);

    /* update the timestamp and generation count */
    _last_update = hrt_absolute_time();

  }

  /* notify any poll waiters */
  poll_notify(POLLIN);

  return _meta->o_size;
}

int uORB::DeviceNode::ioctl(cdev::file_t *filp, int cmd, unsigned long arg) {
  SubscriberData *sd = filp_to_sd(filp);

  switch (cmd) {
    case ORBIOCLASTUPDATE: {
      // Automatic mutex guarding
      uORB::base::MutexGuard lg(_lock);
      *(hrt_abstime *)arg = _last_update;
      return ORB_OK;
    }

    case ORBIOCUPDATED: {
      // Automatic mutex guarding
      uORB::base::MutexGuard lg(_lock);
      *(bool *)arg = appears_updated(sd);
      return ORB_OK;
    }

    case ORBIOCSETINTERVAL: {
      int ret = ORB_OK;
      // Automatic mutex guarding
      uORB::base::MutexGuard lg(_lock);

      if (arg == 0) {
        if (sd->update_interval) {
          delete (sd->update_interval);
          sd->update_interval = nullptr;
        }

      } else {
        if (sd->update_interval) {
          sd->update_interval->interval = arg;

        } else {
          sd->update_interval = new UpdateIntervalData();

          if (sd->update_interval) {
            sd->update_interval->interval = arg;

          } else {
            ret = -ENOMEM;
          }
        }
      }
      return ret;
    }

    case ORBIOCGADVERTISER:
      *(uintptr_t *)arg = (uintptr_t)this;
      return ORB_OK;

    case ORBIOCGPRIORITY:
      *(int *)arg = get_priority();
      return ORB_OK;

    case ORBIOCSETQUEUESIZE: {
      // Automatic mutex guarding
      uORB::base::MutexGuard lg(_lock);
      int ret = update_queue_size(arg);
      return ret;
    }

    case ORBIOCGETINTERVAL:
      if (sd->update_interval) {
        *(unsigned *)arg = sd->update_interval->interval;

      } else {
        *(unsigned *)arg = 0;
      }

      return ORB_OK;

    case ORBIOCISADVERTISED:
      *(unsigned long *)arg = _advertised;

      return ORB_OK;

    default:
      /* Unknown operation*/
      return ORB_ERROR;
  }
}

ssize_t uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle,
                                  const void *data) {
  auto *devnode = (uORB::DeviceNode *)handle;
  int ret;

  /* check if the device handle is initialized and data is valid */
  if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
    orb_errno = EFAULT;
    return ORB_ERROR;
  }

  /* check if the orb meta data matches the publication */
  if (devnode->_meta != meta) {
    orb_errno = EINVAL;
    return ORB_ERROR;
  }

  /* call the devnode write method with no file pointer */
  ret = devnode->write((const char *)data, meta->o_size);

  if (ret < 0) {
    orb_errno = -ret;
    return ORB_ERROR;
  }

  if (ret != (int)meta->o_size) {
    orb_errno = EIO;
    return ORB_ERROR;
  }

#ifdef ORB_COMMUNICATOR
  /*
   * if the write is successful, send the data over the Multi-ORB link
   */
  uORBCommunicator::IChannel *ch =
      uORB::Manager::get_instance()->get_uorb_communicator();

  if (ch != nullptr) {
    if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
      PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
      return ORB_ERROR;
    }
  }

#endif /* ORB_COMMUNICATOR */

  return ORB_OK;
}

int uORB::DeviceNode::unadvertise(orb_advert_t handle) {
  if (handle == nullptr) {
    return -EINVAL;
  }

  auto *devnode = (uORB::DeviceNode *)handle;

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
  devnode->_advertised = false;

  return ORB_OK;
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta,
                                           int priority) {
  uORBCommunicator::IChannel *ch =
      uORB::Manager::get_instance()->get_uorb_communicator();

  if (ch != nullptr && meta != nullptr) {
    return ch->topic_advertised(meta->o_name);
  }

  return -1;
}

/*
//TODO: Check if we need this since we only unadvertise when things all shutdown
and it doesn't actually remove the device int16_t
uORB::DeviceNode::topic_unadvertised(const orb_metadata *meta, int priority)
{
        uORBCommunicator::IChannel *ch =
uORB::Manager::get_instance()->get_uorb_communicator(); if (ch != nullptr &&
meta != nullptr) { return ch->topic_unadvertised(meta->o_name);
        }
        return -1;
}
*/
#endif /* ORB_COMMUNICATOR */

pollevent_t uORB::DeviceNode::poll_state(cdev::file_t *filp) {
  SubscriberData *sd = filp_to_sd(filp);

  /*
   * If the topic appears updated to the subscriber, say so.
   */
  if (appears_updated(sd)) {
    return POLLIN;
  }

  return 0;
}

void uORB::DeviceNode::poll_notify_one(orb_pollfd_t *fds,
                                       pollevent_t events) {
  SubscriberData *sd = filp_to_sd((cdev::file_t *)fds->priv);

  /*
   * If the topic looks updated to the subscriber, go ahead and notify them.
   */
  if (appears_updated(sd)) {
    ORB_DEBUG("CDev::poll_notify_one");

    /* update the reported event set */
    fds->revents |= fds->events & events;

    ORB_DEBUG(" Events fds=%p %0x %0x %0x", fds, fds->revents, fds->events,
              events);

    if (fds->revents != 0) {
      fds->sem->release();
    }
  }
}

bool uORB::DeviceNode::appears_updated(SubscriberData *sd) {
  // check if this topic has been published yet, if not bail out
  if (_data == nullptr) {
    return false;
  }

  // if subscriber has interval check time since last update
  if (sd->update_interval != nullptr) {
    if (hrt_elapsed_time(&sd->update_interval->last_update) <
        sd->update_interval->interval) {
      return false;
    }
  }

  // finally, compare the generation
  return (sd->generation != published_message_count());
}

bool uORB::DeviceNode::print_statistics(bool reset) {
  if (!_lost_messages) {
    return false;
  }
  uint32_t lost_messages;
  {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);

    // This can be wrong: if a reader never reads, _lost_messages will not be
    // increased either
    lost_messages = _lost_messages;

    if (reset) {
      _lost_messages = 0;
    }
  }

  ORB_INFO("%s: %i", _meta->o_name, lost_messages);
  return true;
}

void uORB::DeviceNode::add_internal_subscriber() {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);
  _subscriber_count++;

#ifdef ORB_COMMUNICATOR
  uORBCommunicator::IChannel *ch =
      uORB::Manager::get_instance()->get_uorb_communicator();

  if (ch != nullptr && _subscriber_count > 0) {
    unlock();  // make sure we cannot deadlock if add_subscription calls back
               // into DeviceNode
    ch->add_subscription(_meta->o_name, 1);

  } else
#endif /* ORB_COMMUNICATOR */
}

void uORB::DeviceNode::remove_internal_subscriber() {
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);
  _subscriber_count--;

#ifdef ORB_COMMUNICATOR
  uORBCommunicator::IChannel *ch =
      uORB::Manager::get_instance()->get_uorb_communicator();

  if (ch != nullptr && _subscriber_count == 0) {
    unlock();  // make sure we cannot deadlock if remove_subscription calls back
               // into DeviceNode
    ch->remove_subscription(_meta->o_name);

  } else
#endif /* ORB_COMMUNICATOR */
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz) {
  // if there is already data in the node, send this out to
  // the remote entity.
  // send the data to the remote entity.
  uORBCommunicator::IChannel *ch =
      uORB::Manager::get_instance()->get_uorb_communicator();

  if (_data != nullptr &&
      ch != nullptr) {  // _data will not be null if there is a publisher.
    ch->send_message(_meta->o_name, _meta->o_size, _data);
  }

  return ORB_OK;
}

int16_t uORB::DeviceNode::process_remove_subscription() { return ORB_OK; }

int16_t uORB::DeviceNode::process_received_message(int32_t length,
                                                   uint8_t *data) {
  int16_t ret = -1;

  if (length != (int32_t)(_meta->o_size)) {
    ORB_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]",
            _meta->o_name, (int)length, (int)_meta->o_size);
    return ORB_ERROR;
  }

  /* call the devnode write method with no file pointer */
  ret = write(nullptr, (const char *)data, _meta->o_size);

  if (ret < 0) {
    return ORB_ERROR;
  }

  if (ret != (int)_meta->o_size) {
    orb_errno = EIO;
    return ORB_ERROR;
  }

  return ORB_OK;
}
#endif /* ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size) {
  if (_queue_size == queue_size) {
    return ORB_OK;
  }

  // queue size is limited to 255 for the single reason that we use uint8 to
  // store it
  if (_data || _queue_size > queue_size || queue_size > 255) {
    return ORB_ERROR;
  }

  _queue_size = queue_size;
  return ORB_OK;
}

void uORB::DeviceNode::poll_notify(pollevent_t events) {
  ORB_DEBUG("CDev::poll_notify events = %0x", events);

  /* lock against poll() as well as other wakeups */
  // Automatic mutex guarding
  uORB::base::MutexGuard lg(_lock);

  for (unsigned i = 0; i < _max_pollwaiters; i++) {
    if (nullptr != _pollset[i]) {
      poll_notify_one(_pollset[i], events);
    }
  }
}

int uORB::DeviceNode::store_poll_waiter(orb_pollfd_t *fds) {
  // Look for a free slot.
  ORB_DEBUG("CDev::store_poll_waiter");

  for (unsigned i = 0; i < _max_pollwaiters; i++) {
    if (nullptr == _pollset[i]) {
      /* save the pollfd */
      _pollset[i] = fds;

      return ORB_OK;
    }
  }

  return -ENFILE;
}

int uORB::DeviceNode::remove_poll_waiter(orb_pollfd_t *fds) {
  ORB_DEBUG("CDev::remove_poll_waiter");

  for (unsigned i = 0; i < _max_pollwaiters; i++) {
    if (fds == _pollset[i]) {
      _pollset[i] = nullptr;
      return ORB_OK;
    }
  }

  ORB_DEBUG("poll: bad fd state");
  return -EINVAL;
}

int uORB::DeviceNode::unregister_driver_and_memory() {
  int retval = ORB_OK;

  if (_registered) {
    unregister_driver(_devname);
    _registered = false;

  } else {
    retval = -ENODEV;
  }

  if (_devname != nullptr) {
    delete[] _devname;
    _devname = nullptr;

  } else {
    retval = -ENODEV;
  }

  return retval;
}

/*
 * Default implementations of the character device interface
 */
int uORB::DeviceNode::poll(cdev::file_t *filep, orb_pollfd_t *fds,
                           bool setup) {
  ORB_DEBUG("CDev::Poll %s", setup ? "setup" : "teardown");
  int ret;

  if (setup) {
    /*
     * Save the file pointer in the pollfd for the subclass'
     * benefit.
     */
    fds->priv = (void *)filep;
    ORB_DEBUG("CDev::poll: fds->priv = %p", filep);

    /*
     * Lock against poll_notify() and possibly other callers (protect _pollset).
     */
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);

    /*
     * Try to store the fds for later use and handle array resizing.
     */
    while ((ret = store_poll_waiter(fds)) == -ENFILE) {
      // No free slot found. Resize the pollset. This is expensive, but it's
      // only needed initially.

      if (_max_pollwaiters >= 256 / 2) {  //_max_pollwaiters is uint8_t
        ret = -ENOMEM;
        break;
      }

      const uint8_t new_count = _max_pollwaiters > 0 ? _max_pollwaiters * 2 : 1;
      orb_pollfd_t **prev_pollset = _pollset;

#ifdef __PX4_NUTTX
      // malloc uses a semaphore, we need to call it enabled IRQ's
      orb_leave_critical_section(flags);
#endif
      auto **new_pollset = new orb_pollfd_t *[new_count];

#ifdef __PX4_NUTTX
      flags = orb_enter_critical_section();
#endif

      if (prev_pollset == _pollset) { // Feng: Delete this line,
        // no one else updated the _pollset meanwhile, so we're good to go
        if (!new_pollset) {
          ret = -ENOMEM;
          break;
        }

        if (_max_pollwaiters > 0) {
          memset(
              new_pollset + _max_pollwaiters, 0,
              sizeof(orb_pollfd_t *) * (new_count - _max_pollwaiters));
          memcpy(new_pollset, _pollset,
                 sizeof(orb_pollfd_t *) * _max_pollwaiters);
        }

        _pollset = new_pollset;
        _pollset[_max_pollwaiters] = fds;
        _max_pollwaiters = new_count;

        // free the previous _pollset (we need to unlock here which is fine
        // because we don't access _pollset anymore)
#ifdef __PX4_NUTTX
        orb_leave_critical_section(flags);
#endif

        if (prev_pollset) {
          delete[](prev_pollset);
        }

#ifdef __PX4_NUTTX
        flags = orb_enter_critical_section();
#endif

        // Success
        ret = ORB_OK;
        break;
      }

#ifdef __PX4_NUTTX
      orb_leave_critical_section(flags);
#endif
      // We have to retry
      delete[] new_pollset;
#ifdef __PX4_NUTTX
      flags = orb_enter_critical_section();
#endif
    }

    if (ret == ORB_OK) {
      /*
       * Check to see whether we should send a poll notification
       * immediately.
       */
      fds->revents |= fds->events & poll_state(filep);

      /* yes? post the notification */
      if (fds->revents != 0) {
        fds->sem->release();
      }
    }

  } else {
    // Automatic mutex guarding
    uORB::base::MutexGuard lg(_lock);

    /*
     * Handle a teardown request.
     */
    ret = remove_poll_waiter(fds);
  }

  return ret;
}

int uORB::DeviceNode::init() {
  ORB_DEBUG("CDev::init");

  int ret = ORB_OK;

  // now register the driver
  if (_devname != nullptr) {
    ret = register_driver(_devname, (void *)this);

    if (ret == ORB_OK) {
      _registered = true;
    }
  }

  return ret;
}
