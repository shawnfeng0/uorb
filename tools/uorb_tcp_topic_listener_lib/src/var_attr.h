//
// Created by shawnfeng on 2021/10/3.
//

#include <cstdint>
#include <regex>
#include <string>

#include "string_helper.h"

namespace uorb {
namespace listener {

enum class ValueType : std::uint8_t {
  unknown = 0x00,
  int8_t_type = 0x10,
  uint8_t_type = 0x11,
  bool_type = 0x12,
  char_type = 0x13,
  int16_t_type = 0x20,
  uint16_t_type = 0x21,
  int32_t_type = 0x40,
  uint32_t_type = 0x41,
  float_type = 0x42,
  int64_t_type = 0x80,
  uint64_t_type = 0x81,
  double_type = 0x82,
};

/**
 * VariableAttribute
 * Example: "uint16_t[32] name"
 */
class VarAttr {
 public:
  explicit VarAttr(const std::string &type_and_name) {
    static const std::regex var_attr_regex(R"((\w+)(\[(\d+)\])? (\w+))");
    std::smatch matches;
    if (regex_search(type_and_name, matches, var_attr_regex)) {
      type_name_ = matches[1];
      if (matches[3].matched) {
        array_size_ = std::stoi(matches[3]);
      }
      var_name_ = matches[4];
    }
  }

  size_t PrintToStringWithName(void *data, std::string *str) const {
    if (!str) {
      return PrintToString(data, nullptr);
    }

    *str += type_name_;
    if (array_size_) *str += "[" + std::to_string(array_size_) + "]";
    *str += " " + var_name_;

    std::string value_str;
    auto ret = PrintToString(data, &value_str);
    *str += " = " + value_str;

    return ret;
  }

 private:
  std::string type_name_{};
  std::string var_name_{};
  size_t array_size_{0};  // 0 means not an array

  /**
   * Print data to a string according to the attributes.
   * @param data
   * @param out_str
   * @return Data length used
   * @return 0 Error
   */
  size_t PrintToString(void *data, std::string *out_str) const {
#define AUX(type)                                                   \
  if (type_name_ == #type) {                                        \
    if (out_str) *out_str = var_to_string<type>(data, array_size_); \
    return sizeof(type) * std::max(array_size_, size_t(1));         \
  }
    AUX(bool)
    AUX(char)
    AUX(int8_t)
    AUX(uint8_t)
    AUX(int16_t)
    AUX(uint16_t)
    AUX(int32_t)
    AUX(uint32_t)
    AUX(int64_t)
    AUX(uint64_t)
    AUX(float)
    AUX(double)
#undef AUX
    return 0;
  }

  template <typename T>
  static std::string var_to_string(void *data, size_t array_size) {
    // Non-array
    if (array_size < 1) {
      T value = *reinterpret_cast<T *>(data);
      return std::to_string(value);
    } else {
      // array
      std::string result;
      result += "[";
      for (size_t i = 0; i < array_size; ++i) {
        T value = *reinterpret_cast<T *>(data);
        result += std::to_string(value) + (i != array_size - 1 ? ", " : "");
        data = reinterpret_cast<char *>(data) + sizeof(T);
      }
      result += "]";
      return result;
    }
  }
};  // namespace listener

static std::vector<uorb::listener::VarAttr> ParseMetaFields(
    const orb_metadata &meta) {
  std::vector<uorb::listener::VarAttr> result;
  auto vars = uorb::listener::split_string(meta.o_fields, ";");
  result.reserve(vars.size());
  for (const auto &var : vars) {
    result.emplace_back(var);
  }
  return result;
}

}  // namespace listener
}  // namespace uorb
