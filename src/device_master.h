#pragma once

#include "base/intrusive_list/forward_list.h"
#include "base/mutex.h"
#include "device_node.h"
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
   * @return nullptr on error, and set errno to orb_errno. Otherwise returns a
   * DeviceNode that can be used to publish to the topic.
   */
  DeviceNode *CreateAdvertiser(const orb_metadata &meta, unsigned int *instance);

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
  DeviceNode *GetDeviceNodeLocked(const orb_metadata &meta, uint8_t instance) const;

  // Private constructor, uorb::Manager takes care of its creation
  DeviceMaster() = default;
  ~DeviceMaster() = default;

  static DeviceMaster instance_;

  intrusive_list::forward_list<DeviceNode, &DeviceNode::device_list_node_> node_list_;
  mutable base::Mutex lock_{};
};
