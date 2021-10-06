//
// Created by shawnfeng on 2021/9/30.
//

#include "uorb_tcp_listener.h"

#include <uorb/topics/example_string.h>
#include <uorb/topics/msg_template.h>

#include <thread>
#include <utility>

#include "command_manager.h"
#include "data_printer.h"
#include "fd_stream.h"
#include "tcp_server.h"
#include "uorb/topics/uorb_topics.h"

class FunctionMarker {
 public:
  explicit FunctionMarker(std::string note) : note_(std::move(note)) {
    printf("%s %s\n", note_.c_str(), "start.");
  }
  ~FunctionMarker() { printf("%s %s\n", note_.c_str(), "over."); }

 private:
  const std::string note_;
};

static void CmdGetVersion(uorb::Fd &fd, const std::vector<std::string> &) {
  fd.write(orb_version());
  fd.write("\n");
}

static void CmdTest(uorb::Fd &fd, const std::vector<std::string> &) {
  msg_template_s msg_template{1,
                              2,
                              3,
                              4,
                              5,
                              6,
                              7,
                              {8, 9, 10, 11},
                              18,
                              19,
                              20,
                              true,
                              22,
                              23,
                              {1, 2, 3, 4, 5},
                              {24, 25, 26, 27, 28, 29, 30}};
  DataPrinter data_printer(uorb::msg::msg_template);
  fd.write(data_printer.Convert2String(&msg_template, sizeof(msg_template)));
}

static const orb_metadata *find_meta(const std::string &topic_name) {
  auto topics = orb_get_topics();
  for (size_t i = 0; i < ORB_TOPICS_COUNT; ++i)
    if (topic_name == topics[i]->o_name) return topics[i];
  return nullptr;
}

static void CmdListener(uorb::Fd &fd, const std::vector<std::string> &argv) {
  //  FunctionMarker marker(__FUNCTION__);

  if (argv.empty()) {
    fd.write("Need topic name.\n");
  }
  auto topic_name = argv[0];

  auto meta = find_meta(topic_name);
  if (!meta) {
    fd.write("Can't find topic: " + topic_name + "\n");
    return;
  }

  auto sub = orb_create_subscription(meta);
  std::vector<uint8_t> data(meta->o_size);

  orb_pollfd fds{.fd = sub, .events = POLLIN, .revents = 0};

  do {
    if (orb_poll(&fds, 1, 100) > 0) {
      if (orb_check_and_copy(sub, data.data())) {
        fd.write(DataPrinter(*meta).Convert2String(data.data(), data.size()));
      }
    }
    char c;
    auto ret = fd.read(&c, 1);
    // == 0 means socket is closed; > 0 means need quit the function.
    if (ret >= 0) {
      break;
    }
    if (ret == -1 && errno == EAGAIN) {
      continue;
    }
  } while (true);

  orb_destroy_subscription(&sub);
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

    auto args = uorb::split_string(line, " \t\n");
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
  command_manager.AddCommand("test", CmdTest, "Test");
  command_manager.AddCommand("listener", CmdListener,
                             "topic listener, example: listener topic_name");

  TcpServer tcp_server(port);

  int new_socket;
  while ((new_socket = tcp_server.accept()) >= 0) {
    std::thread{TcpSocketSendThread, new_socket, command_manager}.detach();
  }
  printf("TCP server finished");
}

void orb_tcp_listener_init() { std::thread{TcpServerThread, 10924}.detach(); }
