#pragma once

#include <type_traits>

#include "common.h"

namespace intrusive_list {

struct list_node {
  list_node* next;
  list_node* prev;
};

namespace internal {

/*
 * Insert a new entry between two known consecutive entries.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
template <typename Node>
static inline void _list_add(Node *new_, Node *prev, Node *next) {
  next->prev = new_;
  new_->next = next;
  new_->prev = prev;
  prev->next = new_;
}

/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list front to add it after
 *
 * Insert a new entry after the specified front.
 * This is good for implementing stacks.
 */
template <typename Node>
static inline void list_add(Node *new_, Node *head) {
  _list_add(new_, head, head->next);
}

/**
 * list_add_tail - add a new entry
 * @new: new entry to be added
 * @head: list front to add it before
 *
 * Insert a new entry before the specified front.
 * This is useful for implementing queues.
 */
template <typename Node>
static inline void list_add_tail(Node *new_, Node *head) {
  _list_add(new_, head->prev, head);
}

/**
 * Note that the node must already be in a list
 */
template <typename Node>
static inline void list_remove_self_from_list(Node *node) {
  node->next->prev = node->prev;
  node->prev->next = node->next;
  node->next = nullptr;
  node->prev = nullptr;
}

/**
 * list_move_tail - delete from one list and add as another's back
 * @list: the entry to move
 * @head: the front that will follow our entry
 */
template <typename Node>
static inline void list_move_tail(Node *list, Node *head) {
  list_remove_self_from_list(list);
  list_add_tail(list, head);
}

/**
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
template <typename Node>
static inline int list_empty(const Node *head) {
  return head->next == head;
}

/**
 * list_rotate_left - rotate the list to the left
 * @head: the front of the list
 */
template <typename Node>
static inline void list_rotate_left(Node *head) {
  Node *first;

  if (!list_empty(head)) {
    first = head->next;
    list_move_tail(first, head);
  }
}

/**
 * list_is_singular - tests whether a list has just one entry.
 * @head: the list to test.
 */
template <typename Node>
static inline int list_is_singular(const Node *head) {
  return !list_empty(head) && (head->next == head->prev);
}

}  // namespace internal

/**
 * list double linked list.
 */
template <typename T, decltype(auto) node_field>
class list {
  using Node = std::remove_reference_t<decltype((T *)nullptr->*node_field)>;

  Node head_;

 public:
  list() noexcept : head_({&head_, &head_}) {}

  /**
   * insert item at the front of list.
   * @param item item to insert in list.
   */
  void push_front(T &item) { internal::list_add(get_node(&item), &head_); }

  /**
   * insert item at the back of list.
   * @param item item to insert in list.
   */
  void push_back(T &item) { internal::list_add_tail(get_node(&item), &head_); }

  /**
   * Note that the node must already be in a list
   * @param item item to remove
   * @return true When the deletion is successful
   * @return false When the deletion fails
   */
  bool remove_if_exists(T &item) {
    decltype(auto) node = get_node(&item);
    if (node->next && node->prev) {
      internal::list_remove_self_from_list(node);
      return true;
    }
    return false;
  }

  void rotate_left() { internal::list_rotate_left(&head_); }
  bool is_singular() { return internal::list_is_singular(&head_); }

  /**
   * remove the first item in the list.
   */
  void pop_front() { internal::list_remove_self_from_list(get_node(&front())); }

  /**
   * remove the last item in the list.
   */
  void pop_back() { internal::list_remove_self_from_list(get_node(&back())); }

  /**
   * return first item in list.
   * @return first item in list
   *
   * Note list need not empty.
   */
  T &front() { return *get_owner(head_.next); }

  /**
   * return last item in list.
   * @return last item in list
   *
   * Note list need not empty.
   */
  T &back() { return *get_owner(head_.prev); }

  /**
   * check if the list is empty.
   * @return true if list is empty.
   */
  [[nodiscard]] bool empty() const { return internal::list_empty(&head_); }

  struct Iterator {
    explicit Iterator(Node *v) : node(v) {}
    explicit operator Node *() const { return node; }
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
    Node *node;
  };

  Iterator begin() { return Iterator{head_.next}; }
  Iterator begin() const { return Iterator{head_.next}; }
  Iterator end() { return Iterator{&head_}; }
  Iterator end() const { return Iterator{&head_}; }

  Iterator erase(Iterator position) {
    Iterator ret = Iterator((position.node->next));
    internal::list_remove_self_from_list(position.node);
    return ret;
  }

 private:
  static inline constexpr Node *get_node(T *item) {
    return &(item->*node_field);
  }

  static inline constexpr T *get_owner(Node *member) {
    return internal::owner_of(member, node_field);
  }
};

}  // namespace intrusive_list
