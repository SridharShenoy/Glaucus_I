// Host-side test for the wire protocol. Build & run:
//   g++ -std=c++17 -O1 -Wall shared/protocol/test_protocol.cpp -o /tmp/tp && /tmp/tp
#include <cstdio>
#include <cstring>
#include <random>

#include "glaucus_protocol.h"

using namespace glaucus_proto;

static int fails = 0;
#define EXPECT(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++fails; } } while (0)

int main() {
  static_assert(sizeof(CommandPacket) == 20, "CommandPacket layout changed - bump PROTOCOL_VERSION");
  static_assert(sizeof(TelemetryPacket) == 54, "TelemetryPacket layout changed - bump PROTOCOL_VERSION");

  CommandPacket c;
  c.seq = 42; c.mode = MODE_DRIVE; c.brush_on = 1; c.v = 0.15f; c.heading_cmd = 3.14159f;
  seal(c);
  EXPECT(verify(c));

  uint8_t enc[64], dec[64];
  size_t ne = cobs_encode(reinterpret_cast<uint8_t*>(&c), sizeof c, enc);
  for (size_t i = 0; i < ne; ++i) EXPECT(enc[i] != 0);   // no delimiters inside a frame
  size_t nd = cobs_decode(enc, ne, dec);
  EXPECT(nd == sizeof c);
  CommandPacket c2;
  std::memcpy(&c2, dec, sizeof c2);
  EXPECT(verify(c2) && c2.seq == 42 && c2.v == 0.15f);

  // Random round trips including lots of zero bytes and long non-zero runs.
  std::mt19937 rng(7);
  for (int trial = 0; trial < 2000; ++trial) {
    uint8_t in[600], e[700], d[700];
    size_t n = 1 + rng() % 600;
    for (size_t i = 0; i < n; ++i) in[i] = (rng() % 4 == 0) ? 0 : uint8_t(rng());
    if (trial % 10 == 0) for (size_t i = 0; i < n; ++i) in[i] = uint8_t(1 + i % 250);
    size_t m = cobs_encode(in, n, e);
    size_t k = cobs_decode(e, m, d);
    EXPECT(k == n && std::memcmp(in, d, n) == 0);
    if (fails) break;
  }

  // Corruption is caught by the CRC.
  c2.v = 0.16f;
  EXPECT(!verify(c2));

  std::printf(fails ? "%d failure(s)\n" : "protocol ok\n", fails);
  return fails ? 1 : 0;
}
