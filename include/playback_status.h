#ifndef PLAYBACK_STATUS_H
#define PLAYBACK_STATUS_H

#include <algorithm>
#include <cstddef>
#include <string>

inline bool isPlaybackTerminal(bool eof, bool failed) noexcept
{
    return eof || failed;
}

// Runtime logs use whitespace-delimited key=value fields. Keep an exception
// message on one bounded token so malformed/user-provided text cannot inject
// extra records or grow the log without limit.
inline std::string sanitizePlaybackErrorForLog(
    const std::string& message, std::size_t max_length = 255)
{
    const std::size_t length = std::min(message.size(), max_length);
    std::string result;
    result.reserve(length == 0 ? 7 : length);
    for (std::size_t i = 0; i < length; ++i) {
        const unsigned char ch = static_cast<unsigned char>(message[i]);
        const bool safe =
            (ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '.' || ch == '_' || ch == '-' || ch == ':' || ch == '/';
        result.push_back(safe ? static_cast<char>(ch) : '_');
    }
    if (result.empty()) result = "unknown";
    return result;
}

#endif // PLAYBACK_STATUS_H
