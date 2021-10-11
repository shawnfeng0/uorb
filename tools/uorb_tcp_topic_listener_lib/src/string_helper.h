//
// Created by shawnfeng on 2021/9/27.
//

#pragma once

#include <string>
#include <vector>

namespace uorb {
namespace listener {

static constexpr char kDefaultStripChars[] = " \t\r\n";

/**
 * Strip function, delete the specified characters on both sides of the string.
 */

inline std::string &left_strip(
    std::string &s, const std::string &characters = kDefaultStripChars) {
  return s.erase(0, s.find_first_not_of(characters));
}

inline std::string left_strip(
    const std::string &s, const std::string &characters = kDefaultStripChars) {
  return std::string{s}.erase(0, s.find_first_not_of(characters));
}

inline std::string &right_strip(
    std::string &s, const std::string &characters = kDefaultStripChars) {
  return s.erase(s.find_last_not_of(characters) + 1);
}

inline std::string right_strip(
    const std::string &s, const std::string &characters = kDefaultStripChars) {
  return std::string{s}.erase(s.find_last_not_of(characters) + 1);
}

template <typename StrType>
inline std::string strip(StrType &&s,
                         const std::string &characters = kDefaultStripChars) {
  return right_strip(left_strip(std::forward<StrType>(s), characters),
                     characters);
}

/**
 * Split string to string vector
 *
 * @code Test:
 * std::string str{"program arg1  arg2 \t arg3 \n"};
 * auto str_vec = split_string(str);
 * EXPECT_EQ(str_vec.size(), 5);
 * EXPECT_EQ(str_vec[0], "program");
 * EXPECT_EQ(str_vec[1], "arg1");
 * EXPECT_EQ(str_vec[2], "arg2");
 * EXPECT_EQ(str_vec[3], "arg3");
 * EXPECT_EQ(str_vec[4], "\n");
 *
 * std::string str;
 * auto str_vec = split_string(str);
 * EXPECT_EQ(str_vec.size(), 0);
 *
 * std::string str{"a"};
 * auto str_vec = split_string(str);
 * EXPECT_EQ(str_vec.size(), 1);
 * EXPECT_EQ(str_vec[0], "a");
 *
 * std::string str{"a \t"};
 * auto str_vec = split_string(str);
 * EXPECT_EQ(str_vec.size(), 1);
 * EXPECT_EQ(str_vec[0], "a");
 * @endcode
 *
 * @param str
 * @param delimiter
 * @return
 */
inline std::vector<std::string> split_string(
    const std::string &str, const std::string &delimiter = " \t") {
  std::vector<std::string> result;

  std::string::size_type start;
  std::string::size_type end = -1;
  while (true) {
    start = str.find_first_not_of(delimiter, end + 1);
    if (start == std::string::npos) break;  // over

    end = str.find_first_of(delimiter, start + 1);

    if (end == std::string::npos) {
      result.push_back(str.substr(start));
      break;
    }
    result.push_back(str.substr(start, end - start));
  }
  return result;
}

/**
 * Get string line from string
 *
 * @code
 *  std::string str("123\n456\n2\n3\n");
 *  auto line = get_line(&str);
 *  EXPECT_EQ(line, "123\n");
 *  EXPECT_EQ(str, "456\n2\n3\n");
 *
 *  std::string str("\n2\n3\n");
 *  auto line = get_line(&str);
 *  EXPECT_EQ(line, "\n");
 *  EXPECT_EQ(str, "2\n3\n");
 *
 *  std::string str("\n");
 *  auto line = get_line(&str);
 *  EXPECT_EQ(line, "\n");
 *  EXPECT_EQ(str, "");
 * @endcode
 *
 * @param str
 * @return one line string
 *
 */
std::string get_line(std::string *str) {
  if (!str) return "";
  std::string &str_raw = *str;
  std::string line;
  auto pos = str_raw.find_first_of('\n');
  if (pos != std::string::npos) {
    line = str_raw.substr(0, pos + 1);
    str_raw.erase(0, pos + 1);
  }
  return line;
}

}  // namespace listener
}  // namespace uorb
