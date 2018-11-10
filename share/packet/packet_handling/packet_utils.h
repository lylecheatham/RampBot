/*******************************************************************************
 *
 * FILENAME: packet_utils.h
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/3/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace packet_utils {


template <typename T>
inline void pack(std::vector<char> &dst, T &data) {
    auto src = reinterpret_cast<char *>(&data);
    dst.insert(dst.end(), src, src + sizeof(T));
}

template <typename T>
inline void unpack(const std::vector<char> &src, size_t index, T &data) {
    std::copy(&src[index], &src[index + sizeof(T)], &data);
}

template <typename T>
inline void unpack(const char *src, size_t index, T &data) {
    std::copy(&src[index], &src[index + sizeof(T)], reinterpret_cast<char *>(&data));
}

template <typename T>
inline void unpack_increment(const std::vector<char> &src, size_t &index, T &data) {
    std::copy(&src[index], &src[index + sizeof(T)], &data);
    index += sizeof(T);
}

template <typename T>
inline void unpack_increment(const char *src, size_t &index, T &data) {
    std::copy(&src[index], &src[index + sizeof(T)], reinterpret_cast<char *>(&data));
    index += sizeof(T);
}

}  // namespace packet_utils
