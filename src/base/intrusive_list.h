/****************************************************************************
 *
 *   Copyright (C) 2012-2020 PX4 Development Team. All rights reserved.
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

namespace uorb {
template <class T>
class List;

template <class T>
class ListNode {
  friend List<T>;

 private:
  void set_next(T sibling) { list_node_sibling_ = sibling; }
  T get_next() const { return list_node_sibling_; }
  T list_node_sibling_{nullptr};
};

template <class T>
class List {
 public:
  bool Add(T new_node) {
    if (!new_node) {
      return false;
    }
    new_node->set_next(head_);
    head_ = new_node;
    return true;
  }

  bool Remove(T remove_node) {
    if (!remove_node || !head_) {
      return false;
    }

    // base case
    if (remove_node == head_) {
      head_ = head_->get_next();
      return true;
    }

    for (T node : *this) {
      if (node->get_next() == remove_node) {
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
    inline bool operator!=(const Iterator &rhs) { return node != rhs.node; }
    T operator*() const { return node; }
    Iterator &operator++() {
      if (node) {
        node = node->get_next();
      }

      return *this;
    }
  };
  Iterator begin() { return Iterator(head_); }
  Iterator begin() const { return Iterator(head_); }
  Iterator end() { return Iterator(nullptr); }
  Iterator end() const { return Iterator(nullptr); }

  bool empty() const { return head_ == nullptr; }

  unsigned size() const {
    unsigned sz = 0;
    for (auto node : *this) sz++;
    return sz;
  }

  void DeleteNode(T node) {
    if (Remove(node)) {
      // only delete if node was successfully removed
      delete node;
    }
  }

  void DeleteAllNode() {
    while (head_) {
      auto next = head_->get_next();
      delete head_;
      head_ = next;
    }
  }

 protected:
  T head_{nullptr};
};

}  // namespace uorb
