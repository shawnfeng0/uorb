//
// Created by shawnfeng on 2021/9/27.
//

#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "fd_stream.h"

namespace uorb {

class CommandManager {
 public:
  CommandManager() {
    command_map_["help"].entry = [&](uorb::Fd& fd,
                                     const std::vector<std::string>& args) {
      ExecuteHelp(fd, args);
    };
    command_map_["help"].comment = "Print command list";
  }
  using CommandFunction =
      std::function<void(uorb::Fd& fd, const std::vector<std::string>& args)>;

  auto AddCommand(const std::string& command, CommandFunction function,
                  const std::string& comment = "") -> decltype(*this) {
    command_map_[command].entry = std::move(function);
    command_map_[command].comment = comment;
    return *this;
  }

  void ExecuteHelp(uorb::Fd& fd, const std::vector<std::string>&) const {
    fd.write("Command list: \n");
    for (const auto& i : command_map_) {
      fd.write("\t" + i.first + ": " + i.second.comment + "\n");
    }
  }

  bool ExecuteCommand(const std::string& command, uorb::Fd& fd,
                      const std::vector<std::string>& args) const {
    if (!command_map_.count(command)) {
      return false;
    }
    command_map_.at(command).entry(fd, args);
    return true;
  }

 private:
  struct CommandInfo {
    std::string comment;
    std::function<void(uorb::Fd& fd, const std::vector<std::string>& args)>
        entry;
  };
  std::map<std::string, CommandInfo> command_map_;
};

}  // namespace uorb
