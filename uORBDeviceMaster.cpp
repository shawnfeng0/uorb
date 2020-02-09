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

#include "uORBDeviceMaster.hpp"

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

#include "base/orb_defines.h"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

#include "base/orb_log.h"

/* Duplicate S, returning an identical string assigned using "new". */
static inline char *strdup_with_new(const char *__s) {
  const char *__old = (__s);
  size_t __len = strlen(__old) + 1;
  char *__new = new char[__len];
  // Possibility of allocation failure in microcontroller
  if (__new == nullptr) return __new;
  return (char *)memcpy(__new, __old, __len);
}

int uORB::DeviceMaster::advertise(const struct orb_metadata *meta,
                                  bool is_advertiser, int *instance,
                                  int priority) {
  int ret;
  char nodepath[orb_maxpath];

  /* construct a path to the node - this also checks the node name */
  ret = uORB::Utils::node_mkpath(nodepath, meta, instance);

  if (ret != ORB_OK) {
    return ret;
  }

  /* try for topic groups */
  const unsigned max_group_tries =
      (instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1;
  unsigned group_tries = 0;

  if (instance) {
    /* for an advertiser, this will be 0, but a for subscriber that requests a
     * certain instance, we do not want to start with 0, but with the instance
     * the subscriber actually requests.
     */
    group_tries = *instance;

    if (group_tries >= max_group_tries) {
      return -ENOMEM;
    }
  }

  uORB::base::MutexGuard lg(_lock);

  do {
    /* if path is modifyable change try index */
    if (instance != nullptr) {
      /* replace the number at the end of the string */
      nodepath[strlen(nodepath) - 1] = '0' + group_tries;
      *instance = group_tries;
    }

    /* driver wants a permanent copy of the path, so make one here */
    char *devpath = strdup_with_new(nodepath);

    if (devpath == nullptr) {
      return -ENOMEM;
    }

    /* construct the new node, passing the ownership of path to it */
    auto *node = new uORB::DeviceNode(meta, group_tries, devpath, priority);

    /* if we didn't get a device, that's bad, free the path too */
    if (node == nullptr) {
      delete[] devpath;
      return -ENOMEM;
    }

    /* initialise the node - this may fail if e.g. a node with this name already
     * exists */
    ret = node->init();

    /* if init failed, discard the node and its name */
    if (ret != ORB_OK) {
      delete node;

      if (ret == -EEXIST) {
        /* if the node exists already, get the existing one and check if it's
         * advertised. */
        uORB::DeviceNode *existing_node =
            getDeviceNodeLocked(meta, group_tries);

        /*
         * We can claim an existing node in these cases:
         * - The node is not advertised (yet). It means there is already one or
         * more subscribers or it was unadvertised.
         * - We are a single-instance advertiser requesting the first instance.
         *   (Usually we don't end up here, but we might in case of a race
         * condition between 2 advertisers).
         * - We are a subscriber requesting a certain instance.
         *   (Also we usually don't end up in that case, but we might in case of
         * a race condtion between an advertiser and subscriber).
         */
        bool is_single_instance_advertiser = is_advertiser && !instance;

        if (existing_node != nullptr &&
            (!existing_node->is_advertised() || is_single_instance_advertiser ||
             !is_advertiser)) {
          if (is_advertiser) {
            existing_node->set_priority(priority);
            /* Set as advertised to avoid race conditions (otherwise 2
             * multi-instance advertisers could get the same instance).
             */
            existing_node->mark_as_advertised();
          }

          ret = ORB_OK;

        } else {
          /* otherwise: already advertised, keep looking */
        }
      }

    } else {
      if (is_advertiser) {
        node->mark_as_advertised();
      }

      // add to the node map.
      _node_list.add(node);
    }

    group_tries++;

  } while (ret != ORB_OK && (group_tries < max_group_tries));

  if (ret != ORB_OK && group_tries >= max_group_tries) {
    ret = -ENOMEM;
  }

  return ret;
}

uORB::DeviceNode *uORB::DeviceMaster::getDeviceNode(
    const struct orb_metadata *meta, uint8_t instance) {
  if (meta == nullptr) {
    return nullptr;
  }

  uORB::base::MutexGuard lg(_lock);
  uORB::DeviceNode *node = getDeviceNodeLocked(meta, instance);

  // We can safely return the node that can be used by any thread, because
  // a DeviceNode never gets deleted.
  return node;
}

uORB::DeviceNode *uORB::DeviceMaster::getDeviceNodeLocked(
    const struct orb_metadata *meta, uint8_t instance) {
  for (uORB::DeviceNode *node : _node_list) {
    if ((strcmp(node->get_name(), meta->o_name) == 0) &&
        (node->get_instance() == instance)) {
      return node;
    }
  }

  return nullptr;
}
