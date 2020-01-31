/****************************************************************************
 *
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
 * Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "cdev_platform.hpp"

#include <string.h>

#include <map>
#include <string>

#include "../uORBDeviceNode.hpp"
#include "orb_defines.h"
#include "orb_log.h"
#include "orb_mutex.hpp"
#include "orb_posix.h"

using namespace std;

static uORB::Mutex devmutex;
static uORB::Mutex filemutex;

#define PX4_MAX_FD 350
static map<string, void *> devmap;
static cdev::file_t filemap[PX4_MAX_FD] = {};

extern "C" {

static uORB::DeviceNode *getDev(const char *path) {
  ORB_DEBUG("CDev::getDev");

  uORB::MutexGuard guard(devmutex);

  auto item = devmap.find(path);

  if (item != devmap.end()) {
    return (uORB::DeviceNode *)item->second;
  }

  return nullptr;
}

static uORB::DeviceNode *get_vdev(int fd) {
  uORB::MutexGuard guard(filemutex);

  bool valid = (fd < PX4_MAX_FD && fd >= 0 && filemap[fd].vdev);
  uORB::DeviceNode *dev;

  if (valid) {
    dev = (uORB::DeviceNode *)(filemap[fd].vdev);

  } else {
    dev = nullptr;
  }

  return dev;
}

int register_driver(const char *name, void *data) {
  ORB_DEBUG("CDev::register_driver %s", name);
  int ret = 0;

  if (name == nullptr || data == nullptr) {
    return -EINVAL;
  }

  uORB::MutexGuard grard(devmutex);

  // Make sure the device does not already exist
  auto item = devmap.find(name);

  if (item != devmap.end()) {
    return -EEXIST;
  }

  devmap[name] = data;
  ORB_DEBUG("Registered DEV %s", name);

  return ret;
}

int unregister_driver(const char *name) {
  ORB_DEBUG("CDev::unregister_driver %s", name);
  int ret = -EINVAL;

  if (name == nullptr) {
    return -EINVAL;
  }

  uORB::MutexGuard guard(devmutex);

  if (devmap.erase(name) > 0) {
    ORB_DEBUG("Unregistered DEV %s", name);
    ret = 0;
  }

  return ret;
}

int orb_open(const char *path, int flags, ...) {
  ORB_DEBUG("orb_open");
  uORB::DeviceNode *dev = getDev(path);
  int ret = 0;
  int i = 0;

  if (dev) {
    {
      uORB::MutexGuard guard(filemutex);
      for (i = 0; i < PX4_MAX_FD; ++i) {
        if (filemap[i].vdev == nullptr) {
          filemap[i] = cdev::file_t(flags, dev);
          break;
        }
      }
    }

    if (i < PX4_MAX_FD) {
      ret = dev->open(&filemap[i]);

    } else {
#if defined(__unix__)
      const unsigned NAMELEN = 32;
      char thread_name[NAMELEN] = {};

      int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

      if (nret || thread_name[0] == 0) {
        ORB_WARN("failed getting thread name");
      }
#else
      char thread_name[] = {"\"unknow name\""};
#endif
      ORB_WARN("%s: exceeded maximum number of file descriptors, accessing %s",
               thread_name, path);

      ret = -ENOENT;
    }

  } else {
    ret = -EINVAL;
  }

  if (ret < 0) {
    return -1;
  }

  ORB_DEBUG("orb_open fd = %d", i);
  return i;
}

int orb_close(int fd) {
  int ret;

  uORB::DeviceNode *dev = get_vdev(fd);

  if (dev) {
    uORB::MutexGuard guard(filemutex);

    ret = dev->close(&filemap[fd]);
    filemap[fd].vdev = nullptr;
    ORB_DEBUG("orb_close fd = %d", fd);

  } else {
    ret = -EINVAL;
  }

  if (ret < 0) {
    ret = ORB_ERROR;
  }

  return ret;
}

ssize_t orb_read(int fd, void *buffer, size_t buflen) {
  int ret;

  uORB::DeviceNode *dev = get_vdev(fd);

  if (dev) {
    ORB_DEBUG("orb_read fd = %d", fd);
    ret = dev->read(&filemap[fd], (char *)buffer, buflen);

  } else {
    ret = -EINVAL;
  }

  if (ret < 0) {
    ret = ORB_ERROR;
  }

  return ret;
}

int orb_ioctl(int fd, int cmd, unsigned long arg) {
  ORB_DEBUG("orb_ioctl fd = %d", fd);
  int ret = 0;

  uORB::DeviceNode *dev = get_vdev(fd);

  if (dev) {
    ret = dev->ioctl(&filemap[fd], cmd, arg);

  } else {
    ret = -EINVAL;
  }

  if (ret < 0) {
  }

  return ret;
}

int orb_poll(orb_pollfd_struct_t *fds, nfds_t nfds, int timeout_ms) {
  if (nfds == 0) {
    ORB_WARN("orb_poll with no fds");
    return -1;
  }

  int count = 0;
  int ret = -1;
  unsigned int i;

#if defined(__unix__)
  const unsigned NAMELEN = 32;
  char thread_name[NAMELEN] = {};

  int nret = pthread_getname_np(pthread_self(), thread_name, NAMELEN);

  if (nret || thread_name[0] == 0) {
    ORB_WARN("failed getting thread name");
  }
#else
  char thread_name[] = {"\"unknow name\""};
#endif

  ORB_DEBUG("Called orb_poll timeout = %d", timeout_ms);

  // Go through all fds and check them for a pollable state
  bool fd_pollable = false;

  uORB::Semaphore sem(0);
  for (i = 0; i < nfds; ++i) {
    fds[i].sem = &sem;
    fds[i].revents = 0;
    fds[i].priv = nullptr;

    uORB::DeviceNode *dev = get_vdev(fds[i].fd);

    // If fd is valid
    if (dev) {
      ORB_DEBUG("%s: orb_poll: CDev->poll(setup) %d", thread_name, fds[i].fd);
      ret = dev->poll(&filemap[fds[i].fd], &fds[i], true);

      if (ret < 0) {
        ORB_WARN("%s: orb_poll() error: %s", thread_name, strerror(orb_errno));
        break;
      }

      if (ret >= 0) {
        fd_pollable = true;
      }
    }
  }

  // If any FD can be polled, lock the semaphore and
  // check for new data
  if (fd_pollable) {
    if (timeout_ms > 0) {
      sem.try_acquire_for(timeout_ms);
    } else if (timeout_ms < 0) {
      sem.acquire();
    }

    // We have waited now (or not, depending on timeout),
    // go through all fds and count how many have data
    for (i = 0; i < nfds; ++i) {
      uORB::DeviceNode *dev = get_vdev(fds[i].fd);

      // If fd is valid
      if (dev) {
        ORB_DEBUG("%s: orb_poll: CDev->poll(teardown) %d", thread_name,
                  fds[i].fd);
        ret = dev->poll(&filemap[fds[i].fd], &fds[i], false);

        if (ret < 0) {
          ORB_WARN("%s: orb_poll() 2nd poll fail", thread_name);
          break;
        }

        if (fds[i].revents) {
          count += 1;
        }
      }
    }
  }

  // Return the positive count if present,
  // return the negative error number if failed
  return (count) ? count : ret;
}

int orb_access(const char *pathname, int mode) {
  if (mode != F_OK) {
    orb_errno = EINVAL;
    return -1;
  }

  uORB::DeviceNode *dev = getDev(pathname);
  return (dev != nullptr) ? 0 : -1;
}

void orb_show_topics() {
  ORB_INFO("Devices:");

  uORB::MutexGuard guard(devmutex);

  for (const auto &dev : devmap) {
    if (strncmp(dev.first.c_str(), "/obj/", 5) == 0) {
      ORB_INFO("   %s", dev.first.c_str());
    }
  }
}

}  // extern "C"
