// InterlockPersistence.hpp
//
// ROS-independent durable storage for the real motion interlock.
//
// The interlock is only useful if it survives the process that created it.  A
// kill -9, a power cycle or a container restart must not make an unconfirmed
// real motion look like a clean state, so the latch is mirrored into a small
// state file that is read again on startup.
//
// This is *auxiliary* durability, not a safety mechanism: it cannot confirm
// anything about the robot, and a lost file only makes the node more
// conservative (it latches again), never less.
//
// Guarantees implemented here:
//   * a predictable, per-user state directory is used by default instead of a
//     shared temporary directory that a tmp cleaner may empty;
//   * the file is replaced atomically (write a temporary file in the same
//     directory, fsync, then rename) so a crash never leaves a half-written
//     record that could be misread;
//   * every failure is reported to the caller instead of being swallowed, so
//     the node can refuse real motion when durability was promised but cannot
//     be provided;
//   * a leftover marker counts as latched even when it cannot be parsed or is
//     empty, because a record that exists at all means "not confirmed".
//
// The default location honours XDG_STATE_HOME and falls back to
// $HOME/.local/state, so the state survives a reboot and is not shared between
// users.

#ifndef DUCO_ROS_DRIVER_INTERLOCK_PERSISTENCE_HPP
#define DUCO_ROS_DRIVER_INTERLOCK_PERSISTENCE_HPP

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace duco_reliability
{

class InterlockPersistence
{
public:
  InterlockPersistence() = default;
  explicit InterlockPersistence(std::string path)
  : path_(std::move(path)) {}

  /**
   * \brief The location used when the operator does not configure one.
   *
   * Deliberately not a shared temporary directory: /tmp is cleared by tmpwatch
   * style services and by reboots, which would silently erase the memory of an
   * unconfirmed real motion.  Returns an empty string when no per-user state
   * directory can be determined, in which case the caller must not pretend that
   * persistence is available.
   */
  static std::string defaultPath()
  {
    const char * xdg_state = std::getenv("XDG_STATE_HOME");
    if (xdg_state != nullptr && *xdg_state != '\0') {
      return std::string(xdg_state) + "/duco/dual_trajectory_interlock.state";
    }
    const char * home = std::getenv("HOME");
    if (home != nullptr && *home != '\0') {
      return std::string(home) + "/.local/state/duco/dual_trajectory_interlock.state";
    }
    return {};
  }

  const std::string & path() const noexcept {return path_;}
  bool configured() const noexcept {return !path_.empty();}

  void setPath(std::string path) {path_ = std::move(path);}

  /**
   * \brief Create the state directory and verify that the marker can be
   * created, replaced and removed.
   *
   * Returns false with \p error filled in when durability cannot be provided.
   * A caller that promised durability must then refuse real motion.
   */
  bool prepare(std::string * error = nullptr)
  {
    if (path_.empty()) {
      return fail(error, "no interlock state file path is configured");
    }
    const std::string directory = directoryOf(path_);
    if (!directory.empty() && !makeDirectories(directory, error)) {
      return false;
    }
    // Probe the directory itself: the atomic replace needs write permission on
    // the directory, not only on the old file.
    const std::string probe_dir = directory.empty() ? std::string(".") : directory;
    const std::string probe = probe_dir + "/.duco_interlock_probe_" + std::to_string(::getpid());
    const int fd = ::open(probe.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
      return fail(
        error, "cannot create a file in [" + probe_dir + "]: " + errnoText(errno));
    }
    const int close_result = ::close(fd);
    const int unlink_result = ::unlink(probe.c_str());
    if (unlink_result != 0) {
      return fail(
        error, "cannot remove a probe file from [" + probe_dir + "]: " + errnoText(errno));
    }
    if (close_result != 0) {
      return fail(error, "cannot close the probe file in [" + probe_dir + "]: " + errnoText(errno));
    }

    // A pre-existing marker must stay readable for the restart path, and must be
    // replaceable.  Reading it is best effort: an unreadable marker still counts
    // as latched, it only costs diagnostics.
    if (fileExists(path_)) {
      const int read_fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
      if (read_fd < 0) {
        return fail(
          error, "an interlock state file exists at [" + path_ + "] but cannot be read: " +
          errnoText(errno));
      }
      ::close(read_fd);
    }
    return true;
  }

  /**
   * \brief Atomically create or replace the marker.
   *
   * The temporary file lives in the same directory as the target, so the
   * rename(2) that publishes it is atomic and never exposes partial content.
   */
  bool write(const std::string & payload, std::string * error = nullptr)
  {
    if (path_.empty()) {
      return fail(error, "no interlock state file path is configured");
    }
    const std::string temporary = path_ + ".tmp";
    const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0) {
      return fail(
        error, "cannot open [" + temporary + "] for writing: " + errnoText(errno));
    }

    std::string reason;
    bool ok = true;
    std::size_t offset = 0;
    while (offset < payload.size()) {
      const ssize_t written = ::write(fd, payload.data() + offset, payload.size() - offset);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        reason = "cannot write [" + temporary + "]: " + errnoText(errno);
        ok = false;
        break;
      }
      if (written == 0) {
        reason = "short write to [" + temporary + "]";
        ok = false;
        break;
      }
      offset += static_cast<std::size_t>(written);
    }
    if (ok && ::fsync(fd) != 0) {
      reason = "cannot flush [" + temporary + "]: " + errnoText(errno);
      ok = false;
    }
    if (::close(fd) != 0 && ok) {
      reason = "cannot close [" + temporary + "]: " + errnoText(errno);
      ok = false;
    }
    if (!ok) {
      ::unlink(temporary.c_str());
      return fail(error, reason);
    }
    if (::rename(temporary.c_str(), path_.c_str()) != 0) {
      const std::string rename_error = errnoText(errno);
      ::unlink(temporary.c_str());
      return fail(error, "cannot publish [" + path_ + "]: " + rename_error);
    }
    return true;
  }

  /**
   * \brief Remove the marker.
   *
   * Returns true when the marker is gone afterwards, including when it never
   * existed, so callers can treat it as idempotent.
   */
  bool remove(std::string * error = nullptr)
  {
    if (path_.empty()) {
      return fail(error, "no interlock state file path is configured");
    }
    if (::unlink(path_.c_str()) == 0 || errno == ENOENT) {
      // Also drop a leftover temporary file from a crash between write and
      // rename, so it can never be mistaken for state.
      ::unlink((path_ + ".tmp").c_str());
      return true;
    }
    return fail(error, "cannot remove [" + path_ + "]: " + errnoText(errno));
  }

  /**
   * \brief Read the marker.
   *
   * Returns false when there is no marker.  When the file exists but cannot be
   * read, \p error is filled and the caller must stay conservative (treat it as
   * latched) rather than assume a clean state.
   */
  bool read(std::string & payload, std::string * error = nullptr) const
  {
    payload.clear();
    if (path_.empty()) {
      return fail(error, "no interlock state file path is configured");
    }
    const int fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      if (errno == ENOENT) {
        return false;
      }
      return fail(error, "cannot read [" + path_ + "]: " + errnoText(errno));
    }
    char buffer[512];
    bool ok = true;
    while (true) {
      const ssize_t count = ::read(fd, buffer, sizeof(buffer));
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        ok = fail(error, "cannot read [" + path_ + "]: " + errnoText(errno));
        break;
      }
      if (count == 0) {
        break;
      }
      payload.append(buffer, static_cast<std::size_t>(count));
      if (payload.size() > 4096) {
        // A marker is a handful of bytes; anything larger is not ours.
        break;
      }
    }
    ::close(fd);
    if (!ok) {
      payload.clear();
      return false;
    }
    return true;
  }

  /// True when a marker file exists, even if it cannot be read.  Existence
  /// alone is enough to start latched.
  bool exists() const
  {
    if (path_.empty()) {
      return false;
    }
    return fileExists(path_);
  }

private:
  static bool fileExists(const std::string & path)
  {
    struct stat info {};
    return ::stat(path.c_str(), &info) == 0;
  }

  static std::string directoryOf(const std::string & path)
  {
    const std::size_t slash = path.find_last_of('/');
    if (slash == std::string::npos) {
      return {};
    }
    if (slash == 0) {
      return "/";
    }
    return path.substr(0, slash);
  }

  /// mkdir -p with a private mode: the marker describes the cell's state and
  /// has no business being world readable.
  static bool makeDirectories(const std::string & directory, std::string * error)
  {
    if (directory.empty() || directory == "/") {
      return true;
    }
    std::string built;
    std::size_t index = 0;
    if (directory[0] == '/') {
      built = "/";
      index = 1;
    }
    while (index <= directory.size()) {
      const std::size_t slash = directory.find('/', index);
      const bool last = slash == std::string::npos;
      const std::string part = directory.substr(
        index, last ? std::string::npos : slash - index);
      if (!part.empty()) {
        if (built.empty()) {
          built = part;
        } else if (built == "/") {
          built += part;
        } else {
          built += "/" + part;
        }
        if (::mkdir(built.c_str(), 0700) != 0 && errno != EEXIST) {
          return fail(
            error, "cannot create the interlock state directory [" + built + "]: " +
            errnoText(errno));
        }
      }
      if (last) {
        break;
      }
      index = slash + 1;
    }
    struct stat info {};
    if (::stat(directory.c_str(), &info) != 0 || !S_ISDIR(info.st_mode)) {
      return fail(error, "[" + directory + "] is not a directory");
    }
    return true;
  }

  static bool fail(std::string * error, const std::string & message)
  {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }

  static std::string errnoText(int value)
  {
    const char * text = std::strerror(value);
    return text == nullptr ? std::string("errno ") + std::to_string(value) : std::string(text);
  }

  std::string path_;
};

}  // namespace duco_reliability

#endif  // DUCO_ROS_DRIVER_INTERLOCK_PERSISTENCE_HPP
