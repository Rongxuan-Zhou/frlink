// rt/triple_buffer.hpp - lock-free single-producer / single-consumer triple buffer.
//
// Three slots. At every instant the writer owns one slot, the reader owns one slot and
// the third is "ready" (published, owned by nobody). write() fills the writer slot and
// swaps it with "ready"; read() swaps its slot with "ready" if "ready" is fresh. Because
// every step is a swap of an owned index with the shared one, the three indices are
// always a permutation of {0,1,2}: no slot is ever touched by both threads -> no tearing,
// no locks, no allocation. The reader always sees the most recently published value.
#pragma once
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace rt {

template <typename T>
class TripleBuffer {
  static_assert(std::is_trivially_copyable<T>::value,
                "TripleBuffer<T> requires a trivially copyable T");

 public:
  TripleBuffer() : slots_{} {}
  explicit TripleBuffer(const T& init) {
    for (auto& s : slots_) s = init;
  }
  TripleBuffer(const TripleBuffer&) = delete;
  TripleBuffer& operator=(const TripleBuffer&) = delete;

  // Producer only.
  void write(const T& v) {
    slots_[write_idx_] = v;
    unsigned prev = ready_.exchange(write_idx_ | kFresh, std::memory_order_acq_rel);
    write_idx_ = prev & kIdxMask;
  }

  // Consumer only. Returns the latest published value (or the value passed to the
  // constructor / the last returned value if nothing new was published).
  T read() {
    unsigned r = ready_.load(std::memory_order_acquire);
    if (r & kFresh) {
      unsigned prev = ready_.exchange(read_idx_, std::memory_order_acq_rel);
      read_idx_ = prev & kIdxMask;
    }
    return slots_[read_idx_];
  }

  bool has_fresh() const { return (ready_.load(std::memory_order_acquire) & kFresh) != 0; }

 private:
  static constexpr unsigned kIdxMask = 3u;
  static constexpr unsigned kFresh = 4u;
  T slots_[3];
  unsigned write_idx_ = 0;  // writer-private
  unsigned read_idx_ = 1;   // reader-private
  alignas(64) std::atomic<unsigned> ready_{2};
};

}  // namespace rt
