//
// Created by shawnfeng on 2021/9/30.
//

#include "uorb_tcp_listener.h"

#include <uorb/abs_time.h>
#include <uorb/uorb.h>

#include <csignal>
#include <thread>
#include <utility>

#include "command_manager.h"
#include "data_printer.h"
#include "fd_stream.h"
#include "tcp_server.h"

static orb_get_topics_callback global_topics_callback_ = nullptr;

/*
 * Returns array of topics metadata
 */
static const struct orb_metadata *const *orb_get_topics(size_t *size) {
  if (global_topics_callback_ != nullptr) {
    return global_topics_callback_(size);
  } else {
    if (size) *size = 0;
    return nullptr;
  }
}

static void CmdGetVersion(uorb::listener::Fd &fd,
                          const std::vector<std::string> &) {
  fd.write(orb_version());
  fd.write("\n");
}

static const orb_metadata *find_meta(const std::string &topic_name) {
  size_t orb_topics_count = 0;
  auto topics = orb_get_topics(&orb_topics_count);
  for (size_t i = 0; i < orb_topics_count; ++i)
    if (topic_name == topics[i]->o_name) return topics[i];
  return nullptr;
}

static void CmdListener(uorb::listener::Fd &fd,
                        const std::vector<std::string> &argv) {
  if (argv.empty()) {
    fd.write("Need topic name, example: listener <topic_name>\n");
    return;
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

  uorb::listener::DataPrinter data_printer(*meta);
  orb_abstime_us last_write_timestamp{};
  const int timeout_ms = 1000;
  do {
    if (orb_poll(&fds, 1, timeout_ms) > 0) {
      if (orb_check_and_copy(sub, data.data())) {
        using namespace uorb::time_literals;
        if (orb_elapsed_time_us(last_write_timestamp) > 100_ms) {
          last_write_timestamp = orb_absolute_time_us();
          fd.write(data_printer.Convert2String(data.data(), data.size()));
        }
      } else {
        fd.write("Error: Polling was successful but no data was read.");
      }
    } else {
      fd.write(std::to_string(timeout_ms) + "ms" +
               std::string{" timeout for polling data"});
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

static void CmdStatus(uorb::listener::Fd &fd,
                      const std::vector<std::string> &) {
  char send_buffer[256];
  snprintf(send_buffer, sizeof(send_buffer),
           "%-20s %-10s %-10s %-10s %-10s %-10s\n", "topic", "instance",
           "queue", "sub", "pub", "index");
  fd.write(send_buffer);

  size_t orb_topics_count = 0;
  auto topics = orb_get_topics(&orb_topics_count);
  for (size_t i = 0; i < orb_topics_count; ++i) {
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

static void TcpSocketSendThread(
    int socket_fd, const uorb::listener::CommandManager &command_manager) {
  uorb::listener::Fd socket(socket_fd);

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
    auto line = uorb::listener::get_line(&read_buffer);
    if (line.empty()) continue;

    auto args = uorb::listener::split_string(line, " \t\n");
    std::string command;
    if (!args.empty()) {
      command = *args.begin();
      args.erase(args.begin());
    }
    uorb::listener::strip(command);
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
  uorb::listener::CommandManager command_manager;
  command_manager.AddCommand("version", CmdGetVersion, "Print uorb version");
  command_manager.AddCommand("status", CmdStatus, "Print uorb status");
  command_manager.AddCommand("listener", CmdListener,
                             "topic listener, example: listener topic_name");

  // Prevent process shutdown due to SIGPIPE signal:
  // https://stackoverflow.com/questions/54871085/is-it-a-good-practice-to-call-pthread-sigmask-in-a-thread-created-by-stdthread
  sigset_t mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGPIPE);
  pthread_sigmask(SIG_BLOCK, &mask, nullptr);

  uorb::listener::TcpServer tcp_server(port);

  int new_socket;
  while ((new_socket = tcp_server.accept()) >= 0) {
    std::thread{TcpSocketSendThread, new_socket, command_manager}.detach();
  }
  printf("TCP server finished");
}

void orb_tcp_listener_init(orb_get_topics_callback callback, uint16_t port) {
  global_topics_callback_ = callback;
  std::thread{TcpServerThread, port}.detach();
}
