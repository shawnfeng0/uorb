# uORB 代码库问题清单

基于整体 review 发现的问题，按优先级排序。

---

## 高优先级

### 1. EventPoll::Wait 中手动 lock/unlock

- **文件**: `components/uevent/src/event_poll.h:77-178`
- **问题**: `Wait()` 方法直接使用 `mu_.lock()` / `mu_.unlock()` 而非 RAII 守卫。有 6+ 个 unlock/return 路径，任何异常或提前返回的 bug 都会导致互斥量仍被锁定。
- **修复方案**: 引入 RAII 守卫模式，或使用 `unique_lock` 风格的锁管理。
- **状态**: 待修复

### 2. AddSubscription 的 use-after-free 风险

- **文件**: `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h:107-161`
- **问题**: `AddSubscription` / `RemoveSubscription` 的 dispatch lambda 捕获外部 `Sub&sub` 的 `handle`（原始 `orb_subscriber_t*`）。若调用者在未调用 `RemoveSubscription` 的情况下销毁订阅，悬垂指针会导致 crash。
- **决策**: 移除 `AddSubscription` / `RemoveSubscription`，只保留 `Subscribe<meta>(cb)`。需要外部管理订阅的用户可用 C API 直接组合。
- **状态**: 待修复

### 3. CreateAdvertiser 中的锁顺序未文档化

- **文件**: `src/device_master.cc:17-51`
- **问题**: 持有 `DeviceMaster::lock_` 的同时调用 `device_node->add_publisher()`，后者获取 `DeviceNode::lock_`。嵌套的 master→node 锁顺序。任何未来代码先获取 node 锁再调用 DeviceMaster 就会死锁。
- **修复方案**: 在 DeviceMaster 和 DeviceNode 的头文件中显式文档化锁顺序不变量。
- **状态**: 待修复

### 4. 无节点清理机制

- **文件**: `src/device_master.cc`, `src/device_node.cc`
- **问题**: `DeviceNode` 永远不销毁。`orb_publisher_destroy` 只递减发布者计数。`OpenDeviceNode` 为订阅者按需创建节点。长时间运行的进程中动态 topic 多时，侵入式链表无限增长。
- **决策**: 设计如此，仅文档化说明 topic 节点持久化。
- **状态**: 待修复（文档）

---

## 中优先级

### 5. 索引计算不一致

- **文件**: `src/device_node.cc:88` vs `:61`
- **问题**: `Publish()` 用 `generation_ % queue_size_`，`Copy()` 用 `sub_generation & (queue_size_ - 1)`。两者都正确（queue_size 始终为 2 的幂），但不一致是可读性陷阱。
- **修复方案**: 统一使用位运算 AND。
- **状态**: 待修复

### 6. Subscription 使用虚函数

- **文件**: `include/uorb/subscription.h:49-56`
- **问题**: `Updated()`、`Update()`、`Copy()` 是虚函数，增加 vtable 开销。本质上只有 `SubscriptionInterval` 需要覆写。
- **决策**: 改用 CRTP 消除虚函数开销。
- **状态**: 待修复

### 7. Publication::Publish 错误不透明

- **文件**: `include/uorb/publication.h:31-36`
- **问题**: 返回 `bool`，无法区分"创建失败"和"发布失败"。
- **决策**: 改为返回 `orb_err`，能区分创建失败和发布失败。
- **状态**: 待修复

### 8. orb_subscriber_set_callback 重入风险

- **文件**: `include/uorb/uorb.h:390-393`
- **问题**: 回调在 `DeviceNode::lock_` 持有时调用。若回调对同一 topic 调用 `orb_subscriber_copy`，会死锁（互斥量不可重入）。
- **决策**: 确认双锁方案。拆分为 `data_lock_`（保护环形缓冲区+generation+计数器）和 `callback_lock_`（保护回调列表）。Publish 先写数据再触发回调，Copy 只需 `data_lock_`。回调中可安全调用同一 topic 的 Copy。
- **状态**: 待修复

---

## 低优先级

### 9. uevent_loop 的 reinterpret_cast

- **文件**: `components/uevent/src/uevent.cc:92`
- **问题**: 将 `uevent_source_t*` 重解释为 `EventSource**`，依赖布局相同但无编译期断言。
- **修复方案**: 添加 `static_assert(sizeof(uevent_source_t) == sizeof(void*))`。
- **状态**: 待修复

### 10. Wait 中超时过期线性扫描

- **文件**: `components/uevent/src/event_poll.h:92-100`
- **问题**: 每次唤醒遍历所有 deadlines，O(n)。
- **修复方案**: 暂不处理（典型场景几十个源，性能足够）。
- **状态**: 暂不处理

### 11. stop_ 非原子

- **文件**: `components/uevent/src/event_poll.h:200`
- **问题**: `stop_` 是普通 `bool`，虽然在锁下访问但不够防御性。
- **修复方案**: 改为 `std::atomic<bool>`。
- **状态**: 待修复

### 12. AddSubscription 线性重复检查

- **文件**: `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h:116-121`
- **问题**: 扫描 `entries_` map 查找匹配 handle，O(n)。
- **修复方案**: 暂不处理（典型负载影响不大）。
- **状态**: 暂不处理

### 13. Run() 在 entries_ 为空时忙等

- **文件**: `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h:182-184`
- **问题**: `Run()` 在无订阅时立即返回 `false`，若调用者 `while(loop.Run())` 会忙等。
- **修复方案**: 文档说明调用者应在 `Run()` 前添加订阅。
- **状态**: 待修复

### 14. Quit() 后无法重启 Run()

- **文件**: `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h:180-191`
- **问题**: `quit_requested_` 是粘性的，无法重置。
- **决策**: `Run()` 被重新调用时自动重置 `quit_requested_`。
- **状态**: 待修复

### 15. 固定大小 ready 数组

- **文件**: `components/uorb_uevent/include/uorb_uevent/uorb_uevent.h:168`
- **问题**: `ready[32]` 固定大小，超过 32 个源同时就绪时延迟到下次迭代。
- **修复方案**: 暂不处理（实际场景 32 足够）。
- **状态**: 暂不处理

### 16. 重复的 base 工具代码

- **文件**: `src/base/` 和 `components/uevent/src/base/`
- **问题**: `mutex.h`、`condition_variable.h`、`intrusive_list/` 在两个命名空间各有一份副本。
- **修复方案**: 待讨论（提取 header-only 库 vs 接受重复）
- **状态**: 待讨论

### 17. 错误码映射脆弱

- **文件**: `src/uorb.cc:62-65` 等多处
- **问题**: 用 ad-hoc if-else 链映射 `errno` → `orb_err`，非预期值回退到 `ORB_ERR_UNKNOWN`。
- **修复方案**: 待讨论（系统化映射函数 vs 保持现状）
- **状态**: 待讨论

---

## 进度追踪

| # | 问题 | 优先级 | 状态 |
|---|------|--------|------|
| 1 | EventPoll::Wait 手动 lock/unlock | 高 | ✅ 已修复 |
| 2 | 移除 AddSubscription/RemoveSubscription | 高 | ✅ 已修复 |
| 3 | 锁顺序未文档化（合并到 #8） | 高 | ✅ 已修复 |
| 4 | 无节点清理机制（仅文档化） | 高 | ✅ 已修复 |
| 5 | 索引计算不一致 | 中 | ✅ 已修复 |
| 6 | Subscription 改用 CRTP | 中 | ✅ 已修复 |
| 7 | Publication::Publish 返回 orb_err | 中 | ✅ 已修复 |
| 8 | DeviceNode 双锁方案 | 中 | ✅ 已修复 |
| 9 | reinterpret_cast 无断言 | 低 | ✅ 已修复 |
| 10 | Wait 超时线性扫描 | 低 | 暂不处理 |
| 11 | stop_ 改为 atomic | 低 | ✅ 已修复 |
| 12 | AddSubscription 线性检查（随 #2 移除） | 低 | ✅ 随 #2 移除 |
| 13 | Run() 空订阅忙等（文档说明） | 低 | ✅ 已修复 |
| 14 | Run() 重置 quit_requested_ | 低 | ✅ 已修复 |
| 15 | 固定大小 ready 数组 | 低 | 暂不处理 |
| 16 | 重复的 base 工具代码 | 低 | 待讨论 |
| 17 | 错误码映射脆弱 | 低 | 待讨论 |
