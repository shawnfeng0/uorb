// Benchmark for DeviceMaster lock pressure.
//
// This file measures the throughput of concurrent GetDeviceNode,
// Publish and Copy operations and prints a summary so we have a
// baseline before considering lock-strategy changes (see TODO P3-8).
//
// Build:  included via tests/CMakeLists.txt (target uorb_bench)
// Run:    cmake --build <bld> --target uorb_bench && <bld>/tests/uorb_bench
//         Optional: BENCH_THREADS=8 <bld>/tests/uorb_bench

#include <uorb/topics/orb_test.h>
#include <uorb/uorb.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

static bool g_csv_output = false;
static bool g_consume_all = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double elapsed_ms(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

static void print_header() {
  if (g_csv_output) return;
  printf("%-40s %10s %10s\n", "Benchmark", "Threads", "Mops/s");
  printf("%s\n", std::string(64, '-').c_str());
}

static void print_result(const char *name, int threads, long long ops,
                         double ms) {
  double mops = (double)ops / ms / 1000.0;
  if (g_csv_output) {
    printf("%s,%d,%.3f,%lld,%.3f\n", name, threads, mops, ops, ms);
    return;
  }
  printf("%-40s %10d %10.3f\n", name, threads, mops);
}

static void print_ready_hint(int width, long long loops, long long ready_total) {
  if (g_csv_output || loops <= 0) return;
  const double avg_ready = static_cast<double>(ready_total) /
                           static_cast<double>(loops);
  printf("  -> width=%d, avg_ready_per_loop=%.2f\n", width, avg_ready);
}

static bool mode_enabled(const char *selected_mode, const char *mode) {
  return std::strcmp(selected_mode, "all") == 0 ||
         std::strcmp(selected_mode, mode) == 0;
}

static bool compare_poll_mode_enabled(const char *selected_mode) {
  return std::strcmp(selected_mode, "compare_poll") == 0;
}

// ---------------------------------------------------------------------------
// Benchmark body
// ---------------------------------------------------------------------------

// Worker that continuously calls orb_create_subscription / check_update /
// orb_destroy_subscription in a tight loop.
//
// The subscription path funnels through DeviceMaster::GetDeviceNode (via
// OpenDeviceNode) on every create call, giving a realistic lock-pressure
// measurement.

static void run_subscribe_lookup(int num_threads, int duration_ms) {
  // Pre-advertise so the node already exists (we're measuring lookup, not
  // creation).
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  std::atomic<bool> running{true};
  std::atomic<long long> total_ops{0};

  auto worker = [&]() {
    long long ops = 0;
    while (running.load(std::memory_order_relaxed)) {
      orb_subscription_t *sub =
          orb_create_subscription_multi(ORB_ID(orb_test), 0);
      orb_check_update(sub);
      orb_destroy_subscription(&sub);
      ++ops;
    }
    total_ops.fetch_add(ops, std::memory_order_relaxed);
  };

  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < num_threads; ++i) threads.emplace_back(worker);

  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  running = false;
  for (auto &t : threads) t.join();

  double ms = elapsed_ms(t0);
  print_result("subscribe lookup (create+check+destroy)", num_threads,
               total_ops.load(), ms);

  orb_destroy_publication(&pub);
}

// Pure publish throughput — all threads publish; no subscribers waiting.
static void run_publish(int num_threads, int duration_ms) {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  std::atomic<bool> running{true};
  std::atomic<long long> total_ops{0};

  auto worker = [&]() {
    orb_test_s msg{};
    long long ops = 0;
    while (running.load(std::memory_order_relaxed)) {
      orb_publish(pub, &msg);
      ++ops;
    }
    total_ops.fetch_add(ops, std::memory_order_relaxed);
  };

  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < num_threads; ++i) threads.emplace_back(worker);

  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  running = false;
  for (auto &t : threads) t.join();

  double ms = elapsed_ms(t0);
  print_result("publish (N writers, 1 shared pub handle)", num_threads,
               total_ops.load(), ms);

  orb_destroy_publication(&pub);
}

// Mixed: half threads publish, half subscribe and copy.
static void run_mixed(int num_threads, int duration_ms) {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  std::atomic<bool> running{true};
  std::atomic<long long> pub_ops{0};
  std::atomic<long long> sub_ops{0};

  int pub_threads = std::max(1, num_threads / 2);
  int sub_threads = num_threads - pub_threads;

  auto pub_worker = [&]() {
    orb_test_s msg{};
    long long ops = 0;
    while (running.load(std::memory_order_relaxed)) {
      orb_publish(pub, &msg);
      ++ops;
    }
    pub_ops.fetch_add(ops, std::memory_order_relaxed);
  };

  auto sub_worker = [&]() {
    orb_subscription_t *sub =
        orb_create_subscription_multi(ORB_ID(orb_test), 0);
    orb_test_s msg{};
    long long ops = 0;
    while (running.load(std::memory_order_relaxed)) {
      orb_copy(sub, &msg);
      ++ops;
    }
    sub_ops.fetch_add(ops, std::memory_order_relaxed);
    orb_destroy_subscription(&sub);
  };

  std::vector<std::thread> threads;
  threads.reserve(num_threads);

  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < pub_threads; ++i) threads.emplace_back(pub_worker);
  for (int i = 0; i < sub_threads; ++i) threads.emplace_back(sub_worker);

  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  running = false;
  for (auto &t : threads) t.join();

  double ms = elapsed_ms(t0);
  print_result("mixed pub+sub copy", num_threads,
               pub_ops.load() + sub_ops.load(), ms);

  orb_destroy_publication(&pub);
}

// orb_poll throughput on many fds with timeout=0.
static void run_orb_poll_many_fds(int num_fds, int duration_ms,
                                  const char *label =
                                      "orb_poll timeout=0 (many fds)") {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  std::vector<orb_subscription_t *> subs;
  subs.reserve(num_fds);
  std::vector<orb_pollfd_t> fds(num_fds);

  for (int i = 0; i < num_fds; ++i) {
    orb_subscription_t *sub =
        orb_create_subscription_multi(ORB_ID(orb_test), 0);
    subs.push_back(sub);
    fds[i].fd = sub;
    fds[i].events = POLLIN;
    fds[i].revents = 0;
  }

  // Ensure at least one fd is ready so we measure scanning/marking cost.
  orb_test_s msg{};
  orb_publish(pub, &msg);

  const auto t0 = std::chrono::steady_clock::now();
  long long loops = 0;
  long long ready_total = 0;
  while (elapsed_ms(t0) < duration_ms) {
    const int n = orb_poll(fds.data(), static_cast<unsigned>(fds.size()), 0);
    if (n > 0) {
      if (g_consume_all) {
        for (int i = 0; i < n; ++i) {
          orb_copy(fds[i].fd, &msg);
        }
      } else {
        orb_copy(subs[0], &msg);
      }
      orb_publish(pub, &msg);
      ++loops;
      ready_total += n;
    }
  }

  print_result(label, num_fds, loops, elapsed_ms(t0));
  print_ready_hint(num_fds, loops, ready_total);

  for (auto *sub : subs) {
    orb_destroy_subscription(&sub);
  }
  orb_destroy_publication(&pub);
}

// orb_event_poll throughput on many subscriptions with timeout=0.
static void run_event_poll_many_subs(
    int num_subs, int duration_ms,
    const char *label = "event_poll timeout=0 (many subs)") {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);
  orb_event_poll_t *poll = orb_event_poll_create();

  std::vector<orb_subscription_t *> subs;
  subs.reserve(num_subs);

  for (int i = 0; i < num_subs; ++i) {
    orb_subscription_t *sub =
        orb_create_subscription_multi(ORB_ID(orb_test), 0);
    subs.push_back(sub);
    orb_event_poll_add(poll, sub);
  }

  orb_test_s msg{};
  orb_publish(pub, &msg);

  std::vector<orb_subscription_t *> ready(static_cast<size_t>(num_subs));
  const auto t0 = std::chrono::steady_clock::now();
  long long loops = 0;
  long long ready_total = 0;
  while (elapsed_ms(t0) < duration_ms) {
    const int n = orb_event_poll_wait(poll, ready.data(), num_subs, 0);
    if (n > 0) {
      if (g_consume_all) {
        for (int i = 0; i < n; ++i) {
          orb_copy(ready[i], &msg);
        }
      } else {
        orb_copy(ready[0], &msg);
      }
      orb_publish(pub, &msg);
      ++loops;
      ready_total += n;
    }
  }

  print_result(label, num_subs, loops, elapsed_ms(t0));
  print_ready_hint(num_subs, loops, ready_total);

  for (auto *sub : subs) {
    orb_event_poll_remove(poll, sub);
    orb_destroy_subscription(&sub);
  }
  orb_event_poll_destroy(&poll);
  orb_destroy_publication(&pub);
}

// Blocking poll: publisher thread publishes, consumer blocks on orb_poll.
static void run_orb_poll_blocking(int num_fds, int duration_ms) {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);

  std::vector<orb_subscription_t *> subs;
  subs.reserve(num_fds);
  std::vector<orb_pollfd_t> fds(num_fds);
  for (int i = 0; i < num_fds; ++i) {
    orb_subscription_t *sub =
        orb_create_subscription_multi(ORB_ID(orb_test), 0);
    subs.push_back(sub);
    fds[i].fd = sub;
    fds[i].events = POLLIN;
    fds[i].revents = 0;
  }

  // Drain initial state.
  orb_test_s msg{};
  orb_publish(pub, &msg);
  orb_poll(fds.data(), static_cast<unsigned>(fds.size()), 0);
  for (auto *sub : subs) orb_copy(sub, &msg);

  std::atomic<bool> running{true};
  std::atomic<long long> pub_count{0};

  // Publisher thread: publish as fast as consumer can keep up.
  std::thread publisher([&]() {
    orb_test_s m{};
    while (running.load(std::memory_order_relaxed)) {
      m.val++;
      orb_publish(pub, &m);
      pub_count.fetch_add(1, std::memory_order_relaxed);
      // Yield to let consumer wake and process.
      std::this_thread::yield();
    }
  });

  const auto t0 = std::chrono::steady_clock::now();
  long long loops = 0;
  while (elapsed_ms(t0) < duration_ms) {
    const int n = orb_poll(fds.data(), static_cast<unsigned>(fds.size()), 100);
    if (n > 0) {
      for (int i = 0; i < num_fds; ++i) {
        if (fds[i].revents & POLLIN) orb_copy(fds[i].fd, &msg);
      }
      ++loops;
    }
  }

  running = false;
  publisher.join();
  print_result("blocking orb_poll (pub+wait+copy)", num_fds, loops,
               elapsed_ms(t0));

  for (auto *sub : subs) orb_destroy_subscription(&sub);
  orb_destroy_publication(&pub);
}

// Blocking event_poll: publisher thread publishes, consumer blocks on wait.
static void run_event_poll_blocking(int num_subs, int duration_ms) {
  orb_publication_t *pub =
      orb_create_publication_multi(ORB_ID(orb_test), nullptr);
  orb_event_poll_t *poll = orb_event_poll_create();

  std::vector<orb_subscription_t *> subs;
  subs.reserve(num_subs);
  for (int i = 0; i < num_subs; ++i) {
    orb_subscription_t *sub =
        orb_create_subscription_multi(ORB_ID(orb_test), 0);
    subs.push_back(sub);
    orb_event_poll_add(poll, sub);
  }

  // Drain initial state.
  orb_test_s msg{};
  orb_publish(pub, &msg);
  std::vector<orb_subscription_t *> ready(static_cast<size_t>(num_subs));
  orb_event_poll_wait(poll, ready.data(), num_subs, 0);
  for (auto *sub : subs) orb_copy(sub, &msg);

  std::atomic<bool> running{true};

  std::thread publisher([&]() {
    orb_test_s m{};
    while (running.load(std::memory_order_relaxed)) {
      m.val++;
      orb_publish(pub, &m);
      std::this_thread::yield();
    }
  });

  const auto t0 = std::chrono::steady_clock::now();
  long long loops = 0;
  while (elapsed_ms(t0) < duration_ms) {
    const int n = orb_event_poll_wait(poll, ready.data(), num_subs, 100);
    if (n > 0) {
      for (int i = 0; i < n; ++i) orb_copy(ready[i], &msg);
      ++loops;
    }
  }

  running = false;
  publisher.join();
  print_result("blocking event_poll (pub+wait+copy)", num_subs, loops,
               elapsed_ms(t0));

  for (auto *sub : subs) {
    orb_event_poll_remove(poll, sub);
    orb_destroy_subscription(&sub);
  }
  orb_event_poll_destroy(&poll);
  orb_destroy_publication(&pub);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
  const char *env_mode = std::getenv("BENCH_MODE");
  const char *mode = env_mode ? env_mode : "all";
  const char *env_threads = std::getenv("BENCH_THREADS");
  const int max_threads = env_threads ? std::atoi(env_threads)
                                      : (int)std::thread::hardware_concurrency();
  const char *env_duration = std::getenv("BENCH_DURATION_MS");
  const int duration_ms = env_duration ? std::atoi(env_duration) : 500;
  const char *env_csv = std::getenv("BENCH_CSV");
  g_csv_output = env_csv && std::strcmp(env_csv, "0") != 0;
  const char *env_consume_all = std::getenv("BENCH_CONSUME_ALL");
  g_consume_all = env_consume_all && std::strcmp(env_consume_all, "0") != 0;

  if (g_csv_output) {
    printf("benchmark,threads,mops,ops,elapsed_ms\n");
  } else {
    printf("uORB DeviceMaster benchmark  (hardware_concurrency=%u)\n",
           std::thread::hardware_concurrency());
    printf("mode=%s, duration=%dms per scenario, consume_all=%d\n\n",
           mode, duration_ms, g_consume_all ? 1 : 0);
  }

  if (mode_enabled(mode, "lookup")) {
    print_header();
    for (int t : {1, 2, 4, max_threads}) {
      if (t < 1 || t > max_threads) continue;
      run_subscribe_lookup(t, duration_ms);
    }
  }

  if (mode_enabled(mode, "publish")) {
    if (!g_csv_output) printf("\n");
    print_header();
    for (int t : {1, 2, 4, max_threads}) {
      if (t < 1 || t > max_threads) continue;
      run_publish(t, duration_ms);
    }
  }

  if (mode_enabled(mode, "mixed")) {
    if (!g_csv_output) printf("\n");
    print_header();
    for (int t : {2, 4, max_threads}) {
      if (t < 2 || t > max_threads) continue;
      run_mixed(t, duration_ms);
    }
  }

  if (mode_enabled(mode, "poll")) {
    if (!g_csv_output) printf("\n");
    print_header();
    for (int fds : {1, 8, 32, 128}) {
      run_orb_poll_many_fds(fds, duration_ms);
    }
  }

  if (mode_enabled(mode, "event_poll")) {
    if (!g_csv_output) printf("\n");
    print_header();
    for (int subs : {1, 8, 32, 128}) {
      run_event_poll_many_subs(subs, duration_ms);
    }
  }

  if (compare_poll_mode_enabled(mode)) {
    if (!g_csv_output) {
      printf("\n");
      printf("Poll path comparison on identical sizes\n");
    }
    print_header();
    for (int n : {1, 8, 32, 128}) {
      run_orb_poll_many_fds(n, duration_ms,
                            "compare_poll/orb_poll timeout=0");
      run_event_poll_many_subs(n, duration_ms,
                               "compare_poll/event_poll timeout=0");
    }
  }

  if (mode_enabled(mode, "blocking")) {
    if (!g_csv_output) {
      printf("\n");
      printf("Blocking poll comparison (publisher thread + blocking wait)\n");
    }
    print_header();
    for (int n : {1, 8, 32, 128}) {
      run_orb_poll_blocking(n, duration_ms);
      run_event_poll_blocking(n, duration_ms);
    }
  }

  return 0;
}

