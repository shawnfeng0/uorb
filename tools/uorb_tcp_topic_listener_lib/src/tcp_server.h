//
// Created by shawnfeng on 2021/9/30.
//

#pragma once

#include <netinet/in.h>

namespace uorb {
namespace listener {

class TcpServer {
 public:
  explicit TcpServer(uint16_t port) {
    // Creating socket file descriptor
    if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }

    int opt = 1;
    // Forcefully attaching socket to the port
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
      perror("setsockopt");
      exit(EXIT_FAILURE);
    }

    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(port);

    // Forcefully attaching socket to the port
    if (bind(server_fd_, (struct sockaddr *)&address_, sizeof(address_)) < 0) {
      perror("bind failed");
      exit(EXIT_FAILURE);
    }
    if (listen(server_fd_, 30) < 0) {
      perror("listen");
      exit(EXIT_FAILURE);
    }
  }
  int accept() {
    int addrlen = sizeof(address_);
    return ::accept(server_fd_, (struct sockaddr *)&address_,
                    (socklen_t *)&addrlen);
  }

  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

 private:
  struct sockaddr_in address_ {};
  int server_fd_;
};

}  // namespace listener
}  // namespace uorb
