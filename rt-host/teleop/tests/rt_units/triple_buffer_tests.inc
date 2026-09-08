// tests/rt_units/triple_buffer_tests.inc - included by tests/test_rt_units.cpp
#include <array>
#include <cstdint>
#include <thread>

#include "../../rt/triple_buffer.hpp"

namespace {
struct Payload {
  uint64_t seq;
  uint64_t a[7];
  uint64_t checksum;
  static Payload make(uint64_t s) {
    Payload p{};
    p.seq = s;
    uint64_t c = s * 0x9E3779B97F4A7C15ull;
    for (int i = 0; i < 7; ++i) {
      p.a[i] = c ^ (s + static_cast<uint64_t>(i) * 1315423911ull);
      c += p.a[i];
    }
    p.checksum = c;
    return p;
  }
  bool valid() const {
    uint64_t c = seq * 0x9E3779B97F4A7C15ull;
    for (int i = 0; i < 7; ++i) {
      if (a[i] != (c ^ (seq + static_cast<uint64_t>(i) * 1315423911ull))) return false;
      c += a[i];
    }
    return checksum == c;
  }
};
}  // namespace

static void test_triple_buffer() {
  SECTION("triple_buffer");
  // initial value, repeated reads, latest-wins
  std::array<double, 16> init{};
  init[12] = 0.5;
  rt::TripleBuffer<std::array<double, 16>> tb(init);
  CHECK(!tb.has_fresh());
  CHECK(tb.read()[12] == 0.5);
  CHECK(tb.read()[12] == 0.5);
  std::array<double, 16> v = init;
  v[12] = 1.0;
  tb.write(v);
  v[12] = 2.0;
  tb.write(v);
  v[12] = 3.0;
  tb.write(v);
  CHECK(tb.has_fresh());
  CHECK(tb.read()[12] == 3.0);
  CHECK(!tb.has_fresh());
  CHECK(tb.read()[12] == 3.0);
  v[12] = 4.0;
  tb.write(v);
  CHECK(tb.read()[12] == 4.0);

  // concurrent torture: 10^6 writes vs continuous reads; every read must be a consistent
  // snapshot (checksum) and sequence numbers must never go backwards.
  constexpr uint64_t kWrites = 1000000;
  rt::TripleBuffer<Payload> buf(Payload::make(0));
  std::atomic<bool> writer_done{false};
  uint64_t reads = 0, bad = 0, backwards = 0, last = 0;
  std::thread writer([&] {
    for (uint64_t s = 1; s <= kWrites; ++s) buf.write(Payload::make(s));
    writer_done.store(true, std::memory_order_release);
  });
  for (;;) {
    Payload p = buf.read();
    ++reads;
    if (!p.valid()) ++bad;
    if (p.seq < last) ++backwards;
    last = p.seq;
    if (writer_done.load(std::memory_order_acquire) && p.seq == kWrites) break;
  }
  writer.join();
  CHECK(bad == 0);
  CHECK(backwards == 0);
  CHECK(last == kWrites);
  CHECK(reads > 0);
  std::printf("  torture: %llu reads, %llu torn, %llu backwards\n",
              static_cast<unsigned long long>(reads), static_cast<unsigned long long>(bad),
              static_cast<unsigned long long>(backwards));
}
