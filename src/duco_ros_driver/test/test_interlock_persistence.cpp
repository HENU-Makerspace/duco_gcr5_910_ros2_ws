// Unit tests for the durable interlock marker.
//
// These are pure C++ + POSIX: no ROS graph, no rclcpp, so they can be compiled
// and run standalone as well as through colcon.

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

#include "duco_ros_driver/InterlockPersistence.hpp"

namespace {

using duco_reliability::InterlockPersistence;

/// A directory that no user can create files in, on every platform we run on.
const char * kUnusableDirectory = "/proc/self/duco_persistence_test";

std::string testRoot()
{
  return "/tmp/duco_interlock_persistence_test_" + std::to_string(::getpid());
}

std::string testPath(const std::string & name)
{
  return testRoot() + "/nested/" + name;
}

void removeTestRoot()
{
  // Bounded, explicit cleanup: only the paths this test owns.  Restore
  // permissions first in case a test deliberately made a directory read-only.
  ::chmod((testRoot() + "/nested").c_str(), 0700);
  ::chmod(testRoot().c_str(), 0700);
  ::unlink(testPath("interlock.state").c_str());
  ::unlink((testPath("interlock.state") + ".tmp").c_str());
  ::rmdir((testRoot() + "/nested").c_str());
  ::rmdir(testRoot().c_str());
}

bool fileExists(const std::string & path)
{
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0;
}

class InterlockPersistenceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    removeTestRoot();
  }

  void TearDown() override
  {
    removeTestRoot();
  }
};

TEST_F(InterlockPersistenceTest, DefaultPathIsAPerUserStateDirectory)
{
  const std::string path = InterlockPersistence::defaultPath();
  ASSERT_FALSE(path.empty()) << "a home or XDG_STATE_HOME must be available";
  EXPECT_NE(path.rfind("/tmp/", 0), 0u)
    << "the default must not live in a temporary directory: " << path;
  EXPECT_NE(path.find("/duco/"), std::string::npos) << path;
}

TEST_F(InterlockPersistenceTest, DefaultPathFollowsXdgStateHome)
{
  const char * previous = std::getenv("XDG_STATE_HOME");
  const std::string saved = previous == nullptr ? std::string() : std::string(previous);
  const bool had = previous != nullptr;

  setenv("XDG_STATE_HOME", "/var/lib/duco-state", 1);
  EXPECT_EQ(
    InterlockPersistence::defaultPath(),
    "/var/lib/duco-state/duco/dual_trajectory_interlock.state");

  if (had) {
    setenv("XDG_STATE_HOME", saved.c_str(), 1);
  } else {
    unsetenv("XDG_STATE_HOME");
  }
}

TEST_F(InterlockPersistenceTest, PrepareFailsWithoutAConfiguredPath)
{
  InterlockPersistence persistence;
  std::string error;
  EXPECT_FALSE(persistence.configured());
  EXPECT_FALSE(persistence.prepare(&error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(persistence.path().empty());
  EXPECT_FALSE(persistence.exists());
}

TEST_F(InterlockPersistenceTest, PrepareCreatesTheDirectoryChain)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;
  struct stat info {};
  ASSERT_EQ(::stat((testRoot() + "/nested").c_str(), &info), 0);
  EXPECT_TRUE(S_ISDIR(info.st_mode));
}

TEST_F(InterlockPersistenceTest, PrepareFailsOnAnUnusableDirectory)
{
  InterlockPersistence persistence(std::string(kUnusableDirectory) + "/interlock.state");
  std::string error;
  EXPECT_FALSE(persistence.prepare(&error));
  EXPECT_FALSE(error.empty()) << "a failure must explain itself";
}

TEST_F(InterlockPersistenceTest, WriteThenReadRoundTrips)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;

  const std::string payload = "goal-42 uncertain: feedback lost\n";
  ASSERT_TRUE(persistence.write(payload, &error)) << error;
  EXPECT_TRUE(persistence.exists());

  std::string read_back;
  EXPECT_TRUE(persistence.read(read_back, &error)) << error;
  EXPECT_EQ(read_back, payload);
}

TEST_F(InterlockPersistenceTest, WriteAtomicallyReplacesPreviousContent)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;

  std::string long_payload(4000, 'x');
  long_payload += "\n";
  ASSERT_TRUE(persistence.write(long_payload, &error)) << error;
  ASSERT_TRUE(persistence.write("short\n", &error)) << error;

  std::string read_back;
  ASSERT_TRUE(persistence.read(read_back, &error)) << error;
  EXPECT_EQ(read_back, "short\n")
    << "the replacement must not leave bytes from the longer record behind";
}

TEST_F(InterlockPersistenceTest, WriteLeavesNoTemporaryFileBehind)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;
  ASSERT_TRUE(persistence.write("latched\n", &error)) << error;
  EXPECT_FALSE(fileExists(persistence.path() + ".tmp"));
  EXPECT_TRUE(fileExists(persistence.path()));
}

TEST_F(InterlockPersistenceTest, AStaleTemporaryFileIsIgnored)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;

  // Simulate a crash between the temporary write and the rename.
  {
    std::ofstream stale(persistence.path() + ".tmp", std::ios::trunc);
    stale << "half written garbage";
  }
  ASSERT_TRUE(persistence.write("authoritative\n", &error)) << error;
  EXPECT_FALSE(fileExists(persistence.path() + ".tmp"));

  std::string read_back;
  ASSERT_TRUE(persistence.read(read_back, &error)) << error;
  EXPECT_EQ(read_back, "authoritative\n");
}

TEST_F(InterlockPersistenceTest, ReadOfAMissingMarkerIsNotAnError)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;

  std::string payload = "untouched";
  EXPECT_FALSE(persistence.read(payload, &error));
  EXPECT_TRUE(error.empty()) << "absence is a normal state, not a failure: " << error;
  EXPECT_TRUE(payload.empty());
  EXPECT_FALSE(persistence.exists());
}

TEST_F(InterlockPersistenceTest, AnEmptyMarkerStillExistsSoARestartLatches)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;

  // A record that exists at all means "not confirmed"; an empty one must not be
  // read as "clean".
  {
    std::ofstream empty(persistence.path(), std::ios::trunc);
  }
  EXPECT_TRUE(persistence.exists());

  std::string payload;
  EXPECT_TRUE(persistence.read(payload, &error)) << error;
  EXPECT_TRUE(payload.empty());
}

TEST_F(InterlockPersistenceTest, RemoveIsIdempotentAndDropsTemporaryFiles)
{
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;
  ASSERT_TRUE(persistence.write("latched\n", &error)) << error;
  {
    std::ofstream stale(persistence.path() + ".tmp", std::ios::trunc);
    stale << "stale";
  }

  EXPECT_TRUE(persistence.remove(&error)) << error;
  EXPECT_FALSE(persistence.exists());
  EXPECT_FALSE(fileExists(persistence.path() + ".tmp"));
  // Removing again is not a failure: recovery must be idempotent.
  EXPECT_TRUE(persistence.remove(&error)) << error;
}

TEST_F(InterlockPersistenceTest, WriteFailsWhenThePathCannotBeUsed)
{
  InterlockPersistence persistence(std::string(kUnusableDirectory) + "/interlock.state");
  std::string error;
  ASSERT_FALSE(persistence.prepare(&error));
  error.clear();
  EXPECT_FALSE(persistence.write("latched\n", &error));
  EXPECT_FALSE(error.empty()) << "a refused write must explain itself";
}

TEST_F(InterlockPersistenceTest, RemoveFailsWhenTheDirectoryBecomesUnwritable)
{
  if (::geteuid() == 0) {
    GTEST_SKIP() << "root bypasses directory permissions, so this case cannot be exercised";
  }
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;
  ASSERT_TRUE(persistence.write("latched\n", &error)) << error;

  // Removing a file needs write permission on the directory, so a marker inside
  // a directory that became read-only cannot be cleared either.
  const std::string directory = testRoot() + "/nested";
  ASSERT_EQ(::chmod(directory.c_str(), 0500), 0);
  const bool removed = persistence.remove(&error);
  ::chmod(directory.c_str(), 0700);
  EXPECT_FALSE(removed);
  EXPECT_FALSE(error.empty()) << "a refused removal must explain itself";
  EXPECT_TRUE(persistence.exists());
}

TEST_F(InterlockPersistenceTest, AnUnreadableMarkerIsReportedInsteadOfAssumedClean)
{
  if (::geteuid() == 0) {
    GTEST_SKIP() << "root bypasses file permissions, so this case cannot be exercised";
  }
  InterlockPersistence persistence(testPath("interlock.state"));
  std::string error;
  ASSERT_TRUE(persistence.prepare(&error)) << error;
  ASSERT_TRUE(persistence.write("latched\n", &error)) << error;

  const std::string path = persistence.path();
  ASSERT_EQ(::chmod(path.c_str(), 0000), 0);
  std::string payload;
  EXPECT_FALSE(persistence.read(payload, &error));
  EXPECT_FALSE(error.empty()) << "the caller must be able to stay conservative";
  EXPECT_TRUE(persistence.exists()) << "existence alone is enough to latch";
  ::chmod(path.c_str(), 0600);
}

}  // namespace
