# uORB 待优化事项 (TODO)

本文档汇总了代码评审中发现、但**尚未**在当前 PR 中修复的问题，按优先级排序，供后续迭代跟进。已在当前 PR 中修复的内容见 CHANGELOG / PR 描述，不再重复。

---

## 已修复（参考，仅列标题）

- `EventPoll::Wait(timeout_ms == 0)` 非阻塞快速路径不检查已就绪订阅 — `src/event_poll.h`
- `ConditionVariable::wait_for(lock, ms, pred)` 谓词重载下总超时不收敛（每次 spurious wake 重启完整 `ms`）— `src/base/condition_variable.h`
- `DeviceNode::publisher_count_` / `subscriber_count_` (`uint8_t`) 在 `remove_*` 不配对时会下溢到 255 — `src/device_node.cc`

---

## 待优化

### P1 — 正确性 / 契约

#### 1. `ReceiverLocal::notifier_` 单绑定契约不明确
- **位置**：`src/receiver_local.h`、`src/event_poll.h`、`src/base/lite_notifier.h`
- **问题**：一个 `ReceiverLocal`（订阅）在任意时刻只能被一个 `notifier_`（通常是 `EventPoll` 或 `LiteNotifier`）"注册"。如果用户把同一个 `Subscription` 同时加入多个 `EventPoll`，或同时配合 `orb_poll` 使用，后注册者会静默覆盖前者，前者再也收不到通知。
- **影响**：行为违反最小意外原则，排查困难。
- **建议**：
  - 文档化该约束（在 `include/uorb/event_loop.h`、`orb_poll` 声明处注明）。
  - 在 `register_notifier` 时若已绑定则：(a) `assert`；或 (b) 改为链式多路通知器 (`std::vector<LiteNotifier*>`)。
  - 调试构建下增加 `ORB_ASSERT` 检查。

#### 2. `EventPoll` / `EventLoop` 线程安全边界未文档化
- **位置**：`include/uorb/event_loop.h`、`src/event_poll.h`
- **问题**：`AddSubscription` / `RemoveSubscription` 与 `Wait`、`Run`、`Stop` 在多线程下的可调用关系没有写明。`event_poll_list_` 的增删与遍历之间的同步依赖 `notifier_` 内部锁，但用户从 API 视角并不清楚。
- **建议**：
  - 在头文件上加线程安全说明（"AddSubscription/RemoveSubscription 可从任意线程调用；Run/Wait 只能从单一事件线程调用"之类）。
  - 给出 `Stop()` 后能否再 `Run()` 的明确回答。
  - 增加 `EventLoop` 的单元测试（目前只有 `condition_variable_test.cc` 和 `tests/` 下的 orb 接口测试）。

#### 3. `orb_poll` / `EventPoll` 超时语义与 POSIX `poll` 的差异
- **位置**：`src/uorb.cc`（`orb_poll`）、`include/uorb/uorb.h`
- **问题**：POSIX `poll` 的 `timeout` 是 `int` 毫秒，`< 0` 永久阻塞、`0` 立即返回。uORB 的对应语义需要在公开头文件中明确（尤其是 `timeout_ms` 的整型类型与边界）。
- **建议**：统一文档说明并补充边界用例测试。

---

### P2 — 可移植性 / 编译器兼容

#### 4. 大量使用 GCC/Clang 扩展、未考虑 MSVC
- **位置**：`src/base/**`、`src/device_node.cc` 等
- **具体点**：
  - `__attribute__((xxx))` / `__builtin_*` / 可变长数组（VLA）等若存在需要替换为标准 C++ 等价物或用宏包装。
  - `struct timespec` / `pthread_*` 的使用让 Windows 原生构建不可行（目前 CMake 也直接链 `pthread`）。
- **建议**：
  - 若项目不打算支持 MSVC，请在 README 显式声明"仅支持 POSIX + GCC/Clang"。
  - 若计划支持 Windows，抽象一个薄的 `base/platform.h` 层，`pthread_cond_t` / `pthread_mutex_t` 用 `std::condition_variable` + `std::mutex` 封装，或者用 `_WIN32` 下的 SRWLock/CondVar。

#### 5. Apple 与 Linux 的 `pthread_cond_timedwait` 双分支
- **位置**：`src/base/condition_variable.h`
- **问题**：`__APPLE__` 用 `pthread_cond_timedwait_relative_np`，Linux 用绝对 `CLOCK_REALTIME`。Linux 分支应当使用 `pthread_condattr_setclock(CLOCK_MONOTONIC)`，否则系统时间被调整（NTP、手动改表）时 `wait_for` 的超时会失真。
- **建议**：
  - 构造 `pthread_cond_t` 时设置 `CLOCK_MONOTONIC`（若可用），`get_now_time` 同步改为 `clock_gettime(CLOCK_MONOTONIC, ...)`。
  - 这个改动要和 P1-已修复的 `std::chrono::steady_clock` deadline 对齐，二者都基于单调时钟。

---

### P3 — 代码质量 / 可维护性

#### 6. `DeviceMaster` 查找节点时的锁粒度
- **位置**：`src/device_master.cc`
- **问题**：`GetDeviceNode` 在大锁内查找；订阅/发布路径在高 QPS 下有锁争用隐患。节点集合通常是"只增不减"的场景，适合 RCU / `shared_mutex`。
- **建议**：剖析后再决定，暂不动。先加 benchmark。

#### 7. `EventPoll` 循环遍历 `event_poll_list_` 的 O(N) 扫描
- **位置**：`src/event_poll.h::Wait`
- **问题**：每次唤醒都遍历整张订阅表判断 `updates_available()`，规模大时成本高。`LiteNotifier::notify` 已经知道是哪个节点更新，可以把"就绪集合"直接传给 poll，避免全表扫描。
- **建议**：引入每次 `notify` 时的 ready queue（线程安全的 `intrusive_list` 或 `mpmc queue`），`Wait` 只出队。

#### 8. `src/git_version.cc.in` 生成路径写死在 `${CMAKE_CURRENT_BINARY_DIR}/src/`
- **位置**：`CMakeLists.txt`
- **问题**：作为子项目被 `add_subdirectory` 引入时，`CMAKE_CURRENT_BINARY_DIR` 可能不是预期位置，导致 IDE 索引重复。
- **建议**：用 `target_sources(... PRIVATE $<TARGET_OBJECTS:...>)` 或把生成物放到目标专属目录。

#### 9. 缺少 `EventLoop` / `EventPoll` 的单元测试
- **位置**：`src/base/condition_variable_test.cc` 是目前唯一的内部单测。
- **建议**：补
  - `EventPoll::Wait(0)` 非阻塞路径在"有/无就绪数据"两种情况下的返回值；
  - `Wait` 在 `stop_` 期间的中断；
  - 多线程并发 `AddSubscription` / `RemoveSubscription` 与 `Run` 的压力测试。

#### 10. `uint8_t` 计数的上限
- **位置**：`src/device_node.h`（`publisher_count_`、`subscriber_count_`）
- **问题**：当前 PR 只修了下溢，但上溢（>255 个订阅者）仍会悄悄回绕。
- **建议**：改为 `uint16_t` / `uint32_t`，或在 `add_*` 时做上限检查并返回错误。

---

### P4 — 文档 / 示例

- `docs/comparison_with_px4_uorb.md` 是否覆盖了 `EventLoop` 这个 PX4-uORB 所没有的扩展 API？
- `examples/` 目前没有使用 `EventLoop` 的示例，建议补一个最小示例，演示跨线程订阅 + 统一事件循环的用法。
- `README` / `README_CN` 的多发布/多订阅示例应加上错误处理路径示例（当前示例大多忽略返回值）。

---

## 跟进方式

1. 每个条目建议开独立 issue，便于分 PR 逐项推进。
2. P1 / P2 类问题涉及 ABI / 行为变化，建议在一次小版本中集中发布并在 CHANGELOG 中注明。
3. P3 / P4 可以穿插到日常修复中。
