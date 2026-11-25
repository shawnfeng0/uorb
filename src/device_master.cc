#include "device_master.h"

#include <atomic>
#include <new>

#include "base/allocator.h"
#include "device_node.h"

static_assert(sizeof(std::atomic<void *>) == sizeof(void *),
              "std::atomic<void*> must match void* size for C/C++ ABI compatibility");
static_assert(alignof(std::atomic<void *>) == alignof(void *),
              "std::atomic<void*> must match void* alignment for C/C++ ABI compatibility");

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
    device_node = GetDeviceNodeLocked(meta, static_cast<uint8_t>(group_tries));
    if (device_node && (!device_node->publisher_count() || is_single_instance)) {
      device_node->add_publisher();
      break;  // Find a unadvertised device or single instance device
    }

    if (!device_node) {
      void *raw = base::Allocator::Alloc(sizeof(DeviceNode));
      if (!raw) {
        return ORB_ERR_NO_MEM;
      }
      device_node = new (raw) DeviceNode(meta, static_cast<uint8_t>(group_tries));
      device_node->add_publisher();
      node_list_.push_front(*device_node);
      meta.instances[group_tries].store(device_node, std::memory_order_release);
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
  if (instance >= ORB_MULTI_MAX_INSTANCES) {
    return nullptr;
  }
  return static_cast<DeviceNode *>(meta.instances[instance].load(std::memory_order_acquire));
}

bool uorb::DeviceMaster::TopicExists(const orb_metadata &meta, uint8_t instance) const {
  auto *device_node = GetDeviceNode(meta, instance);
  return device_node && device_node->publisher_count() > 0;
}

bool uorb::DeviceMaster::GetTopicStatus(const orb_metadata &meta, uint8_t instance, orb_status *status) const {
  auto *device_node = GetDeviceNode(meta, instance);
  if (!device_node) {
    return false;
  }

  if (status) {
    device_node->FillStatus(status);
  }

  return true;
}

size_t uorb::DeviceMaster::ForEachTopic(orb_topic_visitor_fn visitor, void *ctx) const {
  if (!visitor) {
    return 0;
  }

  // Snapshot the head of node_list_ under lock_. Because DeviceNodes are persistent
  // and only ever prepended via push_front(), existing nodes' `next` pointers are
  // immutable once linked — so we can safely walk outside lock_ with zero allocations
  // and zero risk of deadlocking if the visitor calls back into uORB APIs.
  intrusive_list::forward_list<DeviceNode, &DeviceNode::device_list_node_>::Iterator it(nullptr);
  {
    base::LockGuard<base::Mutex> lg(lock_);
    it = node_list_.begin();
  }

  size_t visited = 0;
  const auto end = node_list_.end();
  for (; it != end; ++it) {
    const DeviceNode &node = *it;
    orb_status status{};
    node.FillStatus(&status);
    ++visited;
    if (!visitor(&node.meta(), node.instance(), &status, ctx)) {
      break;
    }
  }
  return visited;
}

uorb::DeviceNode *uorb::DeviceMaster::GetDeviceNodeLocked(const orb_metadata &meta, uint8_t instance) const {
  if (instance >= ORB_MULTI_MAX_INSTANCES) {
    return nullptr;
  }
  return static_cast<DeviceNode *>(meta.instances[instance].load(std::memory_order_relaxed));
}

orb_err uorb::DeviceMaster::OpenDeviceNode(const orb_metadata &meta, unsigned int instance, DeviceNode **out) {
  if (instance >= ORB_MULTI_MAX_INSTANCES) {
    return ORB_ERR_INVALID;
  }

  // Lock-free O(1) fast path: DeviceNodes are persistent once created.
  auto *device_node = static_cast<DeviceNode *>(meta.instances[instance].load(std::memory_order_acquire));
  if (device_node) {
    *out = device_node;
    return ORB_OK;
  }

  base::LockGuard<base::Mutex> lg(lock_);

  device_node = GetDeviceNodeLocked(meta, static_cast<uint8_t>(instance));
  if (device_node) {
    *out = device_node;
    return ORB_OK;
  }

  void *raw = base::Allocator::Alloc(sizeof(DeviceNode));
  if (!raw) {
    return ORB_ERR_NO_MEM;
  }
  device_node = new (raw) DeviceNode(meta, static_cast<uint8_t>(instance));

  node_list_.push_front(*device_node);
  meta.instances[instance].store(device_node, std::memory_order_release);
  *out = device_node;
  return ORB_OK;
}
