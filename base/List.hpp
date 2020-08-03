/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file List.hpp
 *
 * An intrusive linked list.
 */

#pragma once

#include <cstdlib>

namespace uORB {

template <class T>
class ListNode {
 public:
  void set_next(T sibling) { _list_node_sibling = sibling; }
  T get_next() const { return _list_node_sibling; }

 protected:
  T _list_node_sibling{nullptr};
};

template <class T>
class List {
 public:
  bool Add(T newNode) {
    if (newNode) {
      newNode->set_next(head_);
      head_ = newNode;
      return true;
    }
    return false;
  }

  bool Remove(T removeNode) {
    if (removeNode == nullptr || head_ == nullptr) {
      return false;
    }

    // base case
    if (removeNode == head_) {
      head_ = head_->get_next();
      return true;
    }

    for (T node : *this) {
      if (node->get_next() == removeNode) {
        node->set_next(node->get_next()->get_next());
        return true;
      }
    }

    return false;
  }

  struct Iterator {
    T node;
    explicit Iterator(T v) : node(v) {}
    explicit operator T() const { return node; }
    explicit operator T &() const { return node; }
    inline bool operator!=(const Iterator &y) { return !(node == y.node); }
    T operator*() const { return node; }
    Iterator &operator++() {
      if (node) {
        node = node->get_next();
      }

      return *this;
    }
  };
  Iterator begin() { return Iterator(head()); }
  Iterator end() { return Iterator((T) nullptr); }

  T head() const { return head_; }

  bool Empty() const { return head() == nullptr; }

  size_t Size() const {
    size_t sz = 0;

    for (auto node = head(); node != nullptr; node = node->get_next()) {
      sz++;
    }

    return sz;
  }

  void DeleteNode(T node) {
    if (Remove(node)) {
      // only delete if node was successfully removed
      delete node;
    }
  }

  void Clear() {
    auto node = head();

    while (node != nullptr) {
      auto next = node->get_next();
      delete node;
      node = next;
    }

    head_ = nullptr;
  }

 protected:
  T head_{nullptr};
};

}  // namespace uORB
