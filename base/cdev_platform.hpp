
#pragma once

#include <inttypes.h>

#include "visibility.h"

#define ATOMIC_ENTER lock()
#define ATOMIC_LEAVE unlock()

namespace cdev {

struct file_t {
  int f_oflags{0};
  void *f_priv{nullptr}; /** Per file driver private data */
  void *vdev{nullptr};

  file_t() = default;
  file_t(int f, void *c) : f_oflags(f), vdev(c) {}
};

}  // namespace cdev

extern "C" __EXPORT int register_driver(const char *name, void *data);
extern "C" __EXPORT int unregister_driver(const char *path);
