// Host tests for the Modbus RTU slave responder (ac_modbus.h).
// Build+run:  make test
// Plain asserts, no framework. The two request frames below are real, captured off the
// wall bus, so the CRC is pinned against hardware rather than against itself.
#include "ac_modbus.h"
#include <cassert>
#include <cstring>

// Real master requests observed on the wire, CRC bytes included.
static const uint8_t REQ_REG15[MB_REQ_LEN] = {0x07, 0x03, 0x00, 0x0F, 0x00, 0x05, 0xB5, 0xAC};
static const uint8_t REQ_REG31[MB_REQ_LEN] = {0x07, 0x03, 0x00, 0x1F, 0x00, 0x0A, 0xF4, 0x6D};

static std::map<uint16_t, uint16_t> seed() {
  // enough of the real register set for these tests: the identity block + the reg31 handshake gate
  std::map<uint16_t, uint16_t> r;
  r[15] = 111; r[16] = 54; r[17] = 1; r[18] = 6; r[19] = 78;
  for (uint16_t k = 0; k < 10; k++) r[(uint16_t)(31 + k)] = k;
  return r;
}

// A frame the master really sent must validate against our CRC -- catches a byte-order or
// polynomial mistake that a self-consistent round-trip test would happily agree with.
static void crc_matches_real_captured_frames() {
  for (const uint8_t *req : {REQ_REG15, REQ_REG31}) {
    uint16_t crc = modbus_crc(req, MB_REQ_LEN - 2);
    assert((crc & 0xFF) == req[MB_REQ_LEN - 2]);
    assert((crc >> 8) == req[MB_REQ_LEN - 1]);
  }
}

static void reads_holding_registers() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];
  size_t n = modbus_build_reply(REQ_REG15, regs, 0, out);

  assert(n == 3 + 5 * 2 + 2);                  // header + 5 regs + crc
  assert(out[0] == MB_ADDR && out[1] == MB_FC_READ_HOLDING && out[2] == 10);
  assert(((out[3] << 8) | out[4]) == 111);     // r15, big-endian on the wire
  assert(((out[5] << 8) | out[6]) == 54);      // r16
  assert(((out[11] << 8) | out[12]) == 78);    // r19
  uint16_t crc = modbus_crc(out, n - 2);       // our own reply must carry a valid CRC
  assert((crc & 0xFF) == out[n - 2] && (crc >> 8) == out[n - 1]);
}

// Answering a block we do not serve (with zeros) bounces the master back to
// reg 15 forever. An unserved start address must produce silence.
static void unknown_start_address_is_silent() {
  std::map<uint16_t, uint16_t> regs;           // nothing seeded at all
  uint8_t out[MB_REPLY_MAX];
  assert(modbus_build_reply(REQ_REG15, regs, 0, out) == 0);

  // ...but a gap INSIDE a block we do serve reads as 0 rather than killing the whole reply
  regs[15] = 111;                              // 16..19 missing
  size_t n = modbus_build_reply(REQ_REG15, regs, 0, out);
  assert(n == 15);
  assert(((out[3] << 8) | out[4]) == 111);
  assert(((out[5] << 8) | out[6]) == 0);
}

static void reads_coils() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];
  uint8_t req[MB_REQ_LEN] = {MB_ADDR, MB_FC_READ_COILS, 0, 0, 0, 5, 0, 0};
  modbus_append_crc(req, MB_REQ_LEN - 2);

  size_t n = modbus_build_reply(req, regs, 0x1C, out);   // fan HIGH + compressor + valve
  assert(n == 3 + 1 + 2);
  assert(out[0] == MB_ADDR && out[1] == MB_FC_READ_COILS && out[2] == 1);
  assert(out[3] == 0x1C);
}

static void rejects_bad_requests() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];
  uint8_t req[MB_REQ_LEN];

  // unsupported function code (fc06 write single -- there are no writes on this bus)
  memcpy(req, REQ_REG15, MB_REQ_LEN); req[1] = 6; modbus_append_crc(req, 6);
  assert(modbus_build_reply(req, regs, 0, out) == 0);

  // quantity 0, and one past our cap
  memcpy(req, REQ_REG15, MB_REQ_LEN); req[4] = 0; req[5] = 0; modbus_append_crc(req, 6);
  assert(modbus_build_reply(req, regs, 0, out) == 0);
  memcpy(req, REQ_REG15, MB_REQ_LEN);
  req[4] = (uint8_t)((MB_MAX_REGS + 1) >> 8); req[5] = (uint8_t)((MB_MAX_REGS + 1) & 0xFF);
  modbus_append_crc(req, 6);
  assert(modbus_build_reply(req, regs, 0, out) == 0);

  // coils are only ever read from 0
  uint8_t c[MB_REQ_LEN] = {MB_ADDR, MB_FC_READ_COILS, 0, 1, 0, 5, 0, 0};
  modbus_append_crc(c, 6);
  assert(modbus_build_reply(c, regs, 0, out) == 0);
}

// The largest reply we can be asked for must still fit the stack buffer we hand in.
static void max_reply_fits_buffer() {
  std::map<uint16_t, uint16_t> regs;
  for (uint16_t k = 0; k < MB_MAX_REGS; k++) regs[k] = k;
  uint8_t out[MB_REPLY_MAX];
  uint8_t req[MB_REQ_LEN] = {MB_ADDR, MB_FC_READ_HOLDING, 0, 0,
                             (uint8_t)(MB_MAX_REGS >> 8), (uint8_t)(MB_MAX_REGS & 0xFF), 0, 0};
  modbus_append_crc(req, 6);
  size_t n = modbus_build_reply(req, regs, 0, out);
  assert(n == 3 + MB_MAX_REGS * 2 + 2);
  assert(n <= MB_REPLY_MAX);
}

static void poll_finds_frame_after_garbage() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];
  std::vector<uint8_t> buf;

  // traffic for another slave, then our request -> must resync and answer
  for (uint8_t g : {0x2C, 0x03, 0x00, 0x01, 0xFF}) buf.push_back(g);
  buf.insert(buf.end(), REQ_REG31, REQ_REG31 + MB_REQ_LEN);
  size_t n = modbus_poll(buf, regs, 0, out);
  assert(n == 3 + 10 * 2 + 2);
  assert(buf.empty());                          // consumed on reply
}

static void poll_waits_for_a_complete_frame() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];
  std::vector<uint8_t> buf(REQ_REG15, REQ_REG15 + MB_REQ_LEN - 1);   // one byte short

  assert(modbus_poll(buf, regs, 0, out) == 0);
  assert(buf.size() == MB_REQ_LEN - 1);         // held, not dropped
  buf.push_back(REQ_REG15[MB_REQ_LEN - 1]);     // last byte arrives
  assert(modbus_poll(buf, regs, 0, out) == 15);
}

static void poll_ignores_other_addresses_and_bad_crc() {
  auto regs = seed();
  uint8_t out[MB_REPLY_MAX];

  uint8_t other[MB_REQ_LEN];                    // well-formed, but addressed to slave 44
  memcpy(other, REQ_REG15, MB_REQ_LEN); other[0] = 44; modbus_append_crc(other, 6);
  std::vector<uint8_t> buf(other, other + MB_REQ_LEN);
  assert(modbus_poll(buf, regs, 0, out) == 0);

  uint8_t corrupt[MB_REQ_LEN];                  // ours, but a flipped CRC byte
  memcpy(corrupt, REQ_REG15, MB_REQ_LEN); corrupt[MB_REQ_LEN - 1] ^= 0xFF;
  buf.assign(corrupt, corrupt + MB_REQ_LEN);
  assert(modbus_poll(buf, regs, 0, out) == 0);
}

// A bus full of traffic we never answer must not grow the buffer without bound.
static void rx_buffer_is_capped() {
  std::map<uint16_t, uint16_t> regs;            // serve nothing -> never replies, never clears
  uint8_t out[MB_REPLY_MAX];
  std::vector<uint8_t> buf;
  for (size_t k = 0; k < MB_RX_MAX * 3; k++) buf.push_back((uint8_t) (k & 0xFF));
  assert(modbus_poll(buf, regs, 0, out) == 0);
  assert(buf.size() <= MB_RX_MAX);
}

int main() {
  crc_matches_real_captured_frames();
  reads_holding_registers();
  unknown_start_address_is_silent();
  reads_coils();
  rejects_bad_requests();
  max_reply_fits_buffer();
  poll_finds_frame_after_garbage();
  poll_waits_for_a_complete_frame();
  poll_ignores_other_addresses_and_bad_crc();
  rx_buffer_is_capped();
  return 0;
}
