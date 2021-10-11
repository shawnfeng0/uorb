//
// Created by shawnfeng on 2021/10/4.
//

#pragma once

#include <uorb/uorb.h>

#include <string>

#include "var_attr.h"

namespace uorb {
namespace listener {

class DataPrinter {
 public:
  explicit DataPrinter(const orb_metadata &meta)
      : vars(uorb::listener::ParseMetaFields(meta)) {
    size_t size = 0;
    for (const auto &v : vars) {
      size += v.PrintToStringWithName(nullptr, nullptr);
    }
    size_ = size;
  }

  std::string Convert2String(void *data, size_t size) {
    if (size != size_) return "Error: Data length error!";
    std::string result;
    auto ptr = reinterpret_cast<char *>(data);
    for (const auto &v : vars) {
      std::string value_string;
      ptr += v.PrintToStringWithName(ptr, &value_string);
      result += value_string + "\n";
    }
    return result;
  }

 private:
  const std::vector<uorb::listener::VarAttr> vars;
  size_t size_;
};

}  // namespace listener
}  // namespace uorb
