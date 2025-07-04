//
// Created by shawnfeng on 2021/9/9.
//

#pragma once

#define UORB_NONCOPYABLE(Class)                  \
  Class(const Class&) = delete;                  \
  Class(Class&&) = delete;                       \
  const Class& operator=(const Class&) = delete; \
  void operator=(Class&&) = delete;
