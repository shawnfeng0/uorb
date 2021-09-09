#pragma once

#include <cstdint>
#include <list>

#include "base/intrusive_list.h"
#include "base/mutex.h"
#include "base/rw_mutex.h"
#include "uorb/uorb.h"

namespace uorb {
class DeviceNode;
class DeviceMaster;
}  // namespace uorb

/**
 * Master control device for uorb message device node.
 */
class uorb::DeviceMaster {
 public:
  /**
   * Method to get the singleton instance for the uorb::DeviceMaster.
   * @return DeviceMaster instance reference
   */
  static inline DeviceMaster &get_instance() { return instance_; }

  /**
   * Advertise as the publisher of a topic
   * @param meta The uORB metadata (usually from the *ORB_ID() macro) for the
   * topic.
   * @param instance  Pointer to an integer which will yield the instance ID
   * (0-based) of the publication. This is an output parameter and will be set
   * to the newly created instance, ie. 0 for the first advertiser, 1 for the
   * next and so on. If it is nullptr, it will only be created at 0.
   * @param queue_size Maximum number of buffered elements. If this is 1, no
   * queuing is used.
   * @return nullptr on error, and set errno to orb_errno. Otherwise returns a
   * DeviceNode that can be used to publish to the topic.
   */
  DeviceNode *CreateAdvertiser(const orb_metadata &meta, unsigned int *instance,
                               unsigned int queue_size);

  DeviceNode *OpenDeviceNode(const orb_metadata &meta, unsigned int instance);

  /**
   * Public interface for GetDeviceNodeLocked(). Takes care of synchronization.
   * @return node if exists, nullptr otherwise
   */
  DeviceNode *GetDeviceNode(const orb_metadata &meta, uint8_t instance) const;

 private:
  /**
   * Find a node give its name.
   * lock_ must already be held when calling this.
   * @return node if exists, nullptr otherwise
   */
  DeviceNode *GetDeviceNodeLocked(const orb_metadata &meta,
                                  uint8_t instance) const;

  // Private constructor, uorb::Manager takes care of its creation
  DeviceMaster() = default;
  ~DeviceMaster() = default;

  static DeviceMaster instance_;

  List<DeviceNode *> node_list_{};
  mutable base::RwMutex lock_{};
};
