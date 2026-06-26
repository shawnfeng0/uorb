#include "device_master.h"

#include <new>

#include "device_node.h"

uorb::DeviceMaster uorb::DeviceMaster::instance_;

orb_err uorb::DeviceMaster::CreateAdvertiser(const orb_metadata &meta, unsigned int *instance, DeviceNode **out) {
  const bool is_single_instance = !instance;
  const unsigned max_group_tries = is_single_instance ? 1 : ORB_MULTI_MAX_INSTANCES;

  DeviceNode *device_node = nullptr;
  unsigned group_tries = 0;

  base::LockGuard<base::Mutex> lg(lock_);

  // Find the following devices that can advertise:
  // - Unadvertised device
  // - Single instance device
  // - Unregistered device
  do {
    device_node = GetDeviceNodeLocked(meta, group_tries);
    if (device_node && (!device_node->publisher_count() || is_single_instance)) {
      device_node->add_publisher();
      break;  // Find a unadvertised device or single instance device
    }

    if (!device_node) {
      device_node = new (std::nothrow) DeviceNode(meta, group_tries);
      if (!device_node) {
        return ORB_ERR_NO_MEM;
      }
      device_node->add_publisher();
      node_list_.push_front(*device_node);
      break;  // Create new device
    }
    group_tries++;
  } while (group_tries < max_group_tries);

  // All instances already exist
  if (group_tries >= max_group_tries) {
    return ORB_ERR_EXIST;
  }

  if (instance) *instance = group_tries;
  *out = device_node;
  return ORB_OK;
}

uorb::DeviceNode *uorb::DeviceMaster::GetDeviceNode(const orb_metadata &meta, uint8_t instance) const {
  base::LockGuard<base::Mutex> lg(lock_);
  return GetDeviceNodeLocked(meta, instance);
}

bool uorb::DeviceMaster::TopicExists(const orb_metadata &meta, uint8_t instance) const {
  base::LockGuard<base::Mutex> lg(lock_);
  auto *device_node = GetDeviceNodeLocked(meta, instance);
  return device_node && device_node->publisher_count() > 0;
}

bool uorb::DeviceMaster::GetTopicStatus(const orb_metadata &meta, uint8_t instance, orb_status *status) const {
  base::LockGuard<base::Mutex> lg(lock_);
  auto *device_node = GetDeviceNodeLocked(meta, instance);
  if (!device_node) {
    return false;
  }

  if (status) {
    device_node->FillStatus(status);
  }

  return true;
}

uorb::DeviceNode *uorb::DeviceMaster::GetDeviceNodeLocked(const orb_metadata &meta, uint8_t instance) const {
  for (auto &node : node_list_) {
    if (node.IsSameWith(meta, instance)) return &node;
  }
  return nullptr;
}

orb_err uorb::DeviceMaster::OpenDeviceNode(const orb_metadata &meta, unsigned int instance, DeviceNode **out) {
  if (instance >= ORB_MULTI_MAX_INSTANCES) {
    return ORB_ERR_INVALID;
  }

  base::LockGuard<base::Mutex> lg(lock_);

  DeviceNode *device_node = GetDeviceNodeLocked(meta, instance);
  if (device_node) {
    *out = device_node;
    return ORB_OK;
  }

  device_node = new (std::nothrow) DeviceNode(meta, instance);
  if (!device_node) {
    return ORB_ERR_NO_MEM;
  }

  node_list_.push_front(*device_node);
  *out = device_node;
  return ORB_OK;
}
