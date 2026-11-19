#pragma once
#include <cstddef>
#include <new>
#ifdef SIM_V2_ALLOC_TRACE
#include "sim_alloc.h"
#endif

// Host allocator boundary; target pool accounting is handled separately.
class MemoryPool {
public:
    explicit MemoryPool(int pool_id = 0) : pool_id_(pool_id) {}
    void *alloc(std::size_t n) {
        void *p = ::operator new(n);
#ifdef SIM_V2_ALLOC_TRACE
        sim_alloc_note_pool(p, pool_id_);
#endif
        return p;
    }
    void *alloc_array(std::size_t n) {
        void *p = ::operator new[](n);
#ifdef SIM_V2_ALLOC_TRACE
        sim_alloc_note_pool(p, pool_id_);
#endif
        return p;
    }
    void dealloc(void *p) { ::operator delete(p); }
    void dealloc_array(void *p) { ::operator delete[](p); }
private:
    int pool_id_;
};
inline void *operator new(std::size_t n, MemoryPool &pool) { return pool.alloc(n); }
inline void *operator new[](std::size_t n, MemoryPool &pool) { return pool.alloc_array(n); }
inline void operator delete(void *p, MemoryPool &pool) noexcept { pool.dealloc(p); }
inline void operator delete[](void *p, MemoryPool &pool) noexcept { pool.dealloc_array(p); }
extern MemoryPool *_DTCMRAM;
extern MemoryPool *_SRAM_1;
