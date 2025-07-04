#pragma once

#include "common.h"

namespace intrusive_list {

struct forward_list_node {
  forward_list_node *next;
};

template <typename T, forward_list_node T::*node_field>
class forward_list {
  forward_list_node head_;

 public:
  forward_list() : head_({nullptr}) {}

  /**
   * insert item at the front of list.
   * @param item item to insert in list.
   */
  void push_front(T &item) {
    get_node(&item)->next = head_.next;
    head_.next = get_node(&item);
  }

  bool is_singular() { return (head_.next && head_.next->next == nullptr); }

  /**
   * remove the first item in the list.
   */
  void pop_front() { head_.next = head_.next->next; }

  /**
   * return first item in list.
   * @return first item in list
   *
   * Note list need not empty.
   */
  T &front() { return *get_owner(head_.next); }

  int remove(const T &item) {
    return remove_if([&](const T &i) { return &item == &i; });
  }

  template <typename C>
  int remove_if(const C &condition) {
    int removed = 0;
    auto node = &head_.next;
    while (*node) {
      if (condition(*get_owner(*node))) {
        *node = (*node)->next;
        removed++;
      } else {
        node = &(*node)->next;
      }
    }
    return removed;
  }

  /**
   * check if the list is empty.
   * @return true if list is empty.
   */
  bool empty() const { return head_.next == nullptr; }

  struct Iterator {
    explicit Iterator(forward_list_node *v) : node(v) {}
    explicit operator forward_list_node *() const { return node; }
    inline bool operator!=(const Iterator &rhs) const {
      return node != rhs.node;
    }
    inline bool operator==(const Iterator &rhs) const {
      return node == rhs.node;
    }
    T &operator*() const { return *get_owner(node); }
    T *operator->() const { return get_owner(node); }
    Iterator &operator++() {
      node = node->next;
      return *this;
    }
    forward_list_node *node;
  };

  Iterator begin() { return Iterator{head_.next}; }
  Iterator begin() const { return Iterator{head_.next}; }
  Iterator end() { return Iterator{nullptr}; }
  Iterator end() const { return Iterator{nullptr}; }

 private:
  static inline constexpr forward_list_node *get_node(T *item) {
    return &(item->*node_field);
  }

  static inline constexpr T *get_owner(forward_list_node *member) {
    return internal::owner_of(member, node_field);
  }
};

}  // namespace intrusive_list
