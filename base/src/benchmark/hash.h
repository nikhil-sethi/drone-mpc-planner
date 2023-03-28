#pragma once
#include <string>

struct HashableObj {
    std::string string;
    std::size_t previous_hash;
};

struct MyHash
{
    std::size_t operator()(HashableObj const &s) const noexcept
    {
        std::size_t h1 = std::hash<std::string> {}(s.string);
        std::size_t h2 = s.previous_hash;
        return h1 ^ (h2 << 1);
    }
};
