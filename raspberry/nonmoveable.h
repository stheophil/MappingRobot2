#pragma once

namespace rbt {
    struct nonmoveable {
        nonmoveable() = default;
        
        nonmoveable(nonmoveable&&) = delete;
        nonmoveable(nonmoveable const&) = delete;
        
        void operator=(nonmoveable&&) = delete;
        void operator=(nonmoveable const&) = delete;
    };
}
