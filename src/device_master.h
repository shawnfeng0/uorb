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
   * @param out Output parameter for the DeviceNode handle on success.
   * @return ORB_OK on success, error code otherwise.
   */
  orb_err CreateAdvertiser(const orb_metadata &meta, unsigned int *instance, DeviceNode **out);

  orb_err OpenDeviceNode(const orb_metadata &meta, unsigned int instance, DeviceNode **out);
  bool TopicExists(const orb_metadata &meta, uint8_t instance) const;
  bool GetTopicStatus(const orb_metadata &meta, uint8_t instance, orb_status *status) const;
  size_t ForEachTopic(orb_topic_visitor_fn visitor, void *ctx) const;

  /**
   * Public interface for GetDeviceNodeLocked(). Takes care of synchronization.
   * @return node if exists, nullptr otherwise
   */
  DeviceNode *GetDeviceNode(const orb_metadata &meta, uint8_t instance) const;

 private:
  DeviceNode *CreateNodeLocked(const orb_metadata &meta, uint8_t instance,
                               bool add_initial_publisher = false);

  DeviceMaster() = default;
  ~DeviceMaster() = default;

  static DeviceMaster instance_;

  intrusive_list::forward_list<DeviceNode, &DeviceNode::device_list_node_> node_list_;
  // Protects DeviceNode creation and node_list_ insertion (lock-free fast path
  // reads meta.instances[] directly). Neither Copy() nor user publish callbacks
  // acquire DeviceMaster::lock_.
  mutable base::Mutex lock_{};
};
