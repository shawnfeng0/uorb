//
// Created by shawnfeng on 2021/9/27.
//

#pragma once

#include <map>
#include <string>
#include <vector>

#include "fd_stream.h"

namespace uorb {

class CommandManager {
 public:
  using CommandFunction = void (*)(uorb::Fd& fd,
                                   const std::vector<std::string>& args);

  auto AddCommand(const std::string& command, CommandFunction function)
      -> decltype(*this) {
    command_map_[command] = function;
    return *this;
  }

  bool ExecuteCommand(const std::string& command, uorb::Fd& fd,
                      const std::vector<std::string>& args) {
    if (command_map_.count(command) == 0) {
      return false;
    }
    command_map_[command](fd, args);
    return true;
  }

 private:
  std::map<std::string, CommandFunction> command_map_;
};

}  // namespace uorb
