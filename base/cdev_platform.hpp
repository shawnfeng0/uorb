
#pragma once

#include <inttypes.h>

#include "visibility.h"

namespace uORB {
namespace cdev {

struct file_t {
  int f_oflags{0};
  void *f_priv{nullptr}; /** Per file driver private data */
  void *vdev{nullptr};

  file_t() = default;
  file_t(int f, void *c) : f_oflags(f), vdev(c) {}
};

}  // namespace cdev
}  // namespace uORB

extern "C" __EXPORT int register_driver(const char *name, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
