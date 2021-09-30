//
// Created by shawnfeng on 2021/9/30.
//

#include "uorb_tcp_listener.h"

#include <thread>

#include "command_manager.h"
#include "fd_stream.h"
#include "string_helper.h"
#include "tcp_server.h"
#include "uorb/abs_time.h"
#include "uorb/publication.h"
#include "uorb/publication_multi.h"
#include "uorb/subscription.h"
#include "uorb/subscription_interval.h"
#include "uorb/topics/uorb_topics.h"

static void CmdGetVersion(uorb::Fd &fd, const std::vector<std::string> &) {
  fd.write(orb_version());
  fd.write("\n");
}

static void CmdStatus(uorb::Fd &fd, const std::vector<std::string> &) {
  char send_buffer[256];
  snprintf(send_buffer, sizeof(send_buffer),
           "%-20s %-10s %-10s %-10s %-10s %-10s\n", "topic", "instance",
           "queue", "sub", "pub", "index");
  fd.write(send_buffer);
  auto topics = orb_get_topics();
  for (size_t i = 0; i < ORB_TOPICS_COUNT; ++i) {
    for (size_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; ++instance) {
      orb_status status{};
      if (orb_get_topic_status(topics[i], instance, &status)) {
        std::string sub_count_str = std::to_string(status.subscriber_count);
        if (status.has_anonymous_subscriber) sub_count_str += "+";

        std::string pub_count_str = std::to_string(status.publisher_count);
        if (status.has_anonymous_publisher) pub_count_str += "+";

        snprintf(send_buffer, sizeof(send_buffer),
                 "%-20s %-10zu %-10d %-10s %-10s %-10d\n", topics[i]->o_name,
                 instance, status.queue_size, sub_count_str.c_str(),
                 pub_count_str.c_str(), status.latest_data_index);
        fd.write(send_buffer);
      }
    }
  }
}

static void TcpSocketSendThread(int socket_fd,
                                const uorb::CommandManager &command_manager) {
  uorb::Fd socket(socket_fd);

  std::string read_buffer;

  while (true) {
    if (socket.poll_in(-1) <= 0) {
      continue;
    }

    char buffer[1024];
    auto n = socket.read(buffer, sizeof(buffer));
    if (n == 0) break;
    if (n < 0) continue;

    read_buffer.append(buffer, buffer + n);
    auto line = uorb::get_line(&read_buffer);
    if (line.empty()) continue;

    auto args = uorb::split_string(line);
    std::string command;
    if (!args.empty()) {
      command = *args.begin();
      args.erase(args.begin());
    }
    uorb::strip(command);
    if (!command.empty()) {
      if (!command_manager.ExecuteCommand(command, socket, args)) {
        socket.write("Command not found: " + command + "\n");
        command_manager.ExecuteHelp(socket, args);
      }
      break;  // Execute the command only once
    }
  }
}

static void TcpServerThread(uint16_t port) {
  uorb::CommandManager command_manager;
  command_manager.AddCommand("version", CmdGetVersion, "Print uorb version");
  command_manager.AddCommand("status", CmdStatus, "Print uorb status");

  TcpServer tcp_server(port);

  int new_socket;
  while ((new_socket = tcp_server.accept()) >= 0) {
    std::thread{TcpSocketSendThread, new_socket, command_manager}.detach();
  }
  printf("TCP server finished");
}

void orb_tcp_listener_init() { std::thread{TcpServerThread, 10924}.detach(); }
