/**
 * @file px4_compat_test.cc
 *
 * GTest suite for the PX4-compatible uORB API shim.
 */

#include <gtest/gtest.h>
#include <pthread.h>
#include <unistd.h>

#include <uorb/topics/orb_test.h>
#include <uorb_compat/px4_compat.h>

/* -----------------------------------------------------------------------
 * Publication tests
 * ----------------------------------------------------------------------- */

TEST(Px4Compat, AdvertisePublishUnadvertise) {
  struct orb_test_s data{};
  data.val = 42;

  orb_advert_t advert = orb_advertise(ORB_ID(orb_test), &data);
  ASSERT_TRUE(advert != nullptr);

  data.val = 99;
  EXPECT_EQ(orb_publish(ORB_ID(orb_test), advert, &data), 0);

  EXPECT_EQ(orb_unadvertise(&advert), 0);
  EXPECT_TRUE(advert == nullptr);
}

TEST(Px4Compat, AdvertiseMulti) {
  struct orb_test_s data{};
  data.val = 1;
  int instance = -1;
  orb_advert_t advert = orb_advertise_multi(ORB_ID(orb_multitest), &data, &instance);
  ASSERT_TRUE(advert != nullptr);
  EXPECT_GE(instance, 0);
  EXPECT_EQ(orb_unadvertise(&advert), 0);
}

/* -----------------------------------------------------------------------
 * Subscription tests
 * ----------------------------------------------------------------------- */

TEST(Px4Compat, SubscribeUnsubscribe) {
  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);
  EXPECT_EQ(orb_unsubscribe(fd), 0);
}

TEST(Px4Compat, CheckAndCopy) {
  /* Advertise and publish a message. */
  struct orb_test_s pub_data{};
  pub_data.val = 77;
  orb_advert_t advert = orb_advertise(ORB_ID(orb_test), &pub_data);
  ASSERT_TRUE(advert != nullptr);

  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);

  bool updated = false;
  EXPECT_EQ(orb_check(fd, &updated), 0);
  EXPECT_TRUE(updated);

  struct orb_test_s rx{};
  EXPECT_EQ(orb_copy(ORB_ID(orb_test), fd, &rx), 0);
  EXPECT_EQ(rx.val, 77);

  /* After copy the update flag should be clear. */
  EXPECT_EQ(orb_check(fd, &updated), 0);
  EXPECT_FALSE(updated);

  EXPECT_EQ(orb_unsubscribe(fd), 0);
  EXPECT_EQ(orb_unadvertise(&advert), 0);
}

TEST(Px4Compat, SetGetInterval) {
  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);

  EXPECT_EQ(orb_set_interval(fd, 100), 0);
  unsigned interval = 0;
  EXPECT_EQ(orb_get_interval(fd, &interval), 0);
  EXPECT_EQ(interval, 100u);

  EXPECT_EQ(orb_unsubscribe(fd), 0);
}

/* -----------------------------------------------------------------------
 * Poll tests
 * ----------------------------------------------------------------------- */

/* Helper thread: publishes one message after a short delay. */
struct PollThreadArg {
  const struct orb_metadata *meta;
  int delay_us;
  int val;
};

static void *publish_thread(void *arg) {
  auto *a = static_cast<PollThreadArg *>(arg);
  usleep(a->delay_us);
  struct orb_test_s data{};
  data.val = a->val;
  orb_publisher_publish_once(a->meta, &data);
  return nullptr;
}

TEST(Px4Compat, PollTimedOut) {
  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);

  /* Drain any data left by previous tests before checking for timeout. */
  {
    struct orb_test_s tmp{};
    orb_copy(ORB_ID(orb_test), fd, &tmp);
  }

  /* No new publisher — poll should time out. */
  px4_pollfd_struct_t pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;

  int ret = px4_poll(&pfd, 1, 50 /* ms */);
  EXPECT_EQ(ret, 0);
  EXPECT_EQ(pfd.revents, 0);

  EXPECT_EQ(orb_unsubscribe(fd), 0);
}

TEST(Px4Compat, PollReceivesData) {
  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);

  /* Drain any stale data so we block in poll until the thread publishes. */
  {
    struct orb_test_s tmp{};
    orb_copy(ORB_ID(orb_test), fd, &tmp);
  }

  PollThreadArg arg{ORB_ID(orb_test), 20000 /* 20 ms */, 55};
  pthread_t thr;
  pthread_create(&thr, nullptr, publish_thread, &arg);

  px4_pollfd_struct_t pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;

  int ret = px4_poll(&pfd, 1, 1000 /* ms */);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(pfd.revents, POLLIN);

  if (ret > 0) {
    struct orb_test_s rx{};
    EXPECT_EQ(orb_copy(ORB_ID(orb_test), fd, &rx), 0);
    EXPECT_EQ(rx.val, 55);
  }

  pthread_join(thr, nullptr);
  EXPECT_EQ(orb_unsubscribe(fd), 0);
}

TEST(Px4Compat, PollAlreadyAvailable) {
  /* Publish first, then poll — should return immediately with data. */
  struct orb_test_s pub_data{};
  pub_data.val = 88;
  orb_advert_t advert = orb_advertise(ORB_ID(orb_test), &pub_data);
  ASSERT_TRUE(advert != nullptr);

  int fd = orb_subscribe(ORB_ID(orb_test));
  ASSERT_GE(fd, 0);

  px4_pollfd_struct_t pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;

  int ret = px4_poll(&pfd, 1, 0 /* immediate */);
  EXPECT_EQ(ret, 1);
  EXPECT_EQ(pfd.revents, POLLIN);

  EXPECT_EQ(orb_unsubscribe(fd), 0);
  EXPECT_EQ(orb_unadvertise(&advert), 0);
}

/* -----------------------------------------------------------------------
 * Invalid handle guard tests
 * ----------------------------------------------------------------------- */

TEST(Px4Compat, InvalidHandles) {
  EXPECT_EQ(orb_publish(ORB_ID(orb_test), ORB_ADVERT_INVALID, nullptr), -1);
  EXPECT_EQ(orb_unsubscribe(-1), -1);
  EXPECT_EQ(orb_unsubscribe(9999), -1);

  bool updated = false;
  EXPECT_EQ(orb_check(-1, &updated), -1);
  EXPECT_EQ(orb_check(9999, &updated), -1);

  EXPECT_EQ(orb_copy(ORB_ID(orb_test), -1, nullptr), -1);
}
