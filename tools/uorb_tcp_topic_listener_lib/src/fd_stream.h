//
// Created by shawnfeng on 2021/9/27.
//

#pragma once

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

namespace uorb {

class Fd {
 public:
  explicit Fd(int fd) : fd_(fd) { mark_non_block(fd_); }
  ~Fd() { ::close(fd_); }

  ssize_t read(void *buf, size_t nbytes) const {
    return ::read(fd_, buf, nbytes);
  }

  ssize_t write(const void *buf, size_t n) const {
    return ::write(fd_, buf, n);
  }

  ssize_t write(const std::string &data) const {
    return ::write(fd_, data.data(), data.size());
  }

  int poll_in(int timeout_ms) const {
    pollfd fd{};
    fd.fd = fd_;
    fd.events = POLLIN;
    return poll(&fd, 1, timeout_ms);
  }

  int get_fd() const { return fd_; }

  Fd(const Fd &) = delete;
  Fd &operator=(const Fd &) = delete;

 private:
  static void mark_non_block(int fd) {
    assert(-1 != fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK));
  }

  int fd_;
};

}  // namespace uorb
