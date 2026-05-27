# uORB 待优化事项 (TODO)

本文档汇总代码评审中发现的问题及修复进展。以下条目已完成并附提交号，便于后续追溯。

---

## 已修复

- `EventPoll::Wait(timeout_ms == 0)` 非阻塞快速路径不检查已就绪订阅 — `src/event_poll.h`
- `EventLoop` 文档 / 示例 / 单元测试补齐 — `docs/getting_started{,_cn}.md`、`examples/cpp_pub_sub/cpp_pub_sub_event_loop.cc`、`tests/event_loop_test.cc`

### P1 — 正确性 / 契约

1. `ConditionVariable::wait_for(lock, ms, pred)` 谓词重载下总超时不收敛
   - 提交：`fd1ea61`
   - 变更：`src/base/condition_variable.h`、`src/base/condition_variable_test.cc`

2. `DeviceNode::publisher_count_` / `subscriber_count_` (`uint8_t`) 下溢
   - 提交：`210a435`
   - 变更：`src/device_node.cc`

3. `ReceiverLocal::notifier_` 单绑定契约不明确
   - 提交：`05cf2db`
   - 变更：`src/receiver_local.h`、`src/event_poll.h`、`src/uorb.cc`、`include/uorb/uorb.h`、`include/uorb/event_loop.h`、`tests/event_loop_test.cc`

4. `EventPoll` / `EventLoop` 线程安全边界未完全文档化
   - 提交：`6b9ec02`
   - 变更：`include/uorb/uorb.h`

5. `orb_poll` / `EventPoll` 超时语义与 POSIX `poll` 的差异
   - 提交：`4730566`
   - 变更：`include/uorb/uorb.h`、`tests/uorb_unit_test.cc`

---

### P2 — 可移植性 / 编译器兼容

6. 大量使用 GCC/Clang 扩展、未考虑 MSVC
   - 提交：`e00d159`
   - 变更：`README.md`、`README_CN.md`

7. Apple 与 Linux 的 `pthread_cond_timedwait` 双分支
   - 提交：`7e4fc72`
   - 变更：`src/base/condition_variable.h`

### P3 — 代码质量 / 可维护性

8. `DeviceMaster` 查找节点时的锁粒度（先加 benchmark）
   - 提交：`de9e112`
   - 变更：`tests/device_master_bench.cc`、`tests/CMakeLists.txt`

9. `EventPoll` 循环遍历 `event_poll_list_` 的 O(N) 扫描
   - 提交：`8b043c8`
   - 变更：`src/receiver_local.h`、`src/event_poll.h`

10. `src/git_version.cc.in` 生成路径写死在 `${CMAKE_CURRENT_BINARY_DIR}/src/`
   - 提交：`d95cc19`
   - 变更：`CMakeLists.txt`

11. 进一步的 `EventLoop` / `EventPoll` 内部测试
   - 提交：`a693423`
   - 变更：`tests/event_poll_internal_test.cc`

12. `uint8_t` 计数的上限
   - 提交：`349e750`
   - 变更：`src/device_node.h`、`src/device_node.cc`、`include/uorb/uorb.h`、`tests/uorb_unit_test.cc`

### P4 — 文档 / 示例

- `docs/comparison_with_px4_uorb.md` 补充 `EventLoop/EventPoll` 扩展 API 说明
  - 提交：`ecaa097`
- `README` / `README_CN` 补充多发布/多订阅错误处理示例
  - 提交：`ecaa097`

---

## 待优化

- 当前清单已完成；新增问题请补充到本节并按优先级维护。

---

## 跟进方式

1. 每个条目建议开独立 issue，便于分 PR 逐项推进。
2. P1 / P2 类问题涉及 ABI / 行为变化，建议在一次小版本中集中发布并在 CHANGELOG 中注明。
3. P3 / P4 可以穿插到日常修复中。
