#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <string>
#include <unistd.h>

namespace lite3_recovery {

// The deployment creates this directory with mode 0700. The latch stays
// present even after success: removing it is an explicit external operation.
inline bool reserve_trial(const std::string& directory, std::uint64_t session) {
    if (directory.empty() || !session) return false;
    int dir = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    if (dir < 0) return false;
    int fd = ::openat(dir, "unfinished-sdk-recovery", O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) {
        ::close(dir);
        return false;
    }
    const auto text=std::to_string(session)+"\n";
    std::size_t n{};
    while (n<text.size()) {
        const auto wrote=::write(fd,text.data()+n,text.size()-n);
        if (wrote<0 && errno==EINTR) continue;
        if (wrote<=0) break;
        n+=static_cast<std::size_t>(wrote);
    }
    const bool written=(n==text.size() && ::fsync(fd)==0);
    const bool closed=::close(fd)==0;
    const bool synced=::fsync(dir)==0;
    ::close(dir);
    // A failed write intentionally leaves the file blocking a new session.
    return written && closed && synced;
}

} // namespace lite3_recovery
