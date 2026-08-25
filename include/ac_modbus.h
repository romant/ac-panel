#pragma once
#include <cstdint>
#include <cstddef>
#include <map>
#include <vector>

// Modbus RTU slave for the TZT-100's address on the wall bus.
// The UC8 is the master and only ever READS, so only fc01 (read coils) and
// fc03 (read holding) exist. Anything else is ignored rather than answered.

constexpr uint8_t  MB_ADDR            = 7;
constexpr uint8_t  MB_FC_READ_COILS   = 1;
constexpr uint8_t  MB_FC_READ_HOLDING = 3;
constexpr uint16_t MB_CRC_POLY        = 0xA001;  // reversed 0x8005
constexpr uint16_t MB_CRC_INIT        = 0xFFFF;
constexpr size_t   MB_REQ_LEN         = 8;       // addr + fc + start(2) + qty(2) + crc(2)

constexpr uint16_t MB_MAX_REGS  = 120;   // 120*2+5 = 245, inside MB_REPLY_MAX
constexpr uint16_t MB_MAX_COILS = 200;
constexpr size_t   MB_REPLY_MAX = 253;   // largest RTU frame we will build
// Cap the RX buffer so traffic for OTHER addresses cannot grow it without bound.
// Keeping the last MB_REQ_LEN bytes preserves a request that is mid-arrival.
constexpr size_t   MB_RX_MAX    = 256;

inline uint16_t modbus_crc(const uint8_t *d, size_t n) {
  uint16_t crc = MB_CRC_INIT;
  for (size_t k = 0; k < n; k++) {
    crc ^= d[k];
    for (int j = 0; j < 8; ++j) crc = (crc & 1) ? (crc >> 1) ^ MB_CRC_POLY : crc >> 1;
  }
  return crc;
}

// Append the CRC (low byte first, per RTU) and return the new length.
inline size_t modbus_append_crc(uint8_t *frame, size_t n) {
  uint16_t c = modbus_crc(frame, n);
  frame[n++] = c & 0xFF;
  frame[n++] = c >> 8;
  return n;
}

// Reply to one already-CRC-checked request. Returns bytes written to out, 0 = stay silent.
//
// An unknown start address MUST produce silence. Answering a block we do not hold
// with zeros makes the master abandon the handshake and restart it indefinitely.
inline size_t modbus_build_reply(const uint8_t *req, const std::map<uint16_t, uint16_t> &regs,
                                 uint8_t coils, uint8_t *out) {
  const uint8_t  fc    = req[1];
  const uint16_t start = (uint16_t) ((req[2] << 8) | req[3]);
  const uint16_t qty   = (uint16_t) ((req[4] << 8) | req[5]);
  size_t n = 0;

  if (fc == MB_FC_READ_HOLDING && qty >= 1 && qty <= MB_MAX_REGS && regs.count(start)) {
    out[0] = MB_ADDR; out[1] = MB_FC_READ_HOLDING; out[2] = (uint8_t) (qty * 2); n = 3;
    for (uint16_t k = 0; k < qty; k++) {
      auto it = regs.find((uint16_t) (start + k));
      uint16_t v = (it == regs.end()) ? 0 : it->second;   // gaps inside a served block read as 0
      out[n++] = v >> 8;
      out[n++] = v & 0xFF;
    }
  } else if (fc == MB_FC_READ_COILS && start == 0 && qty >= 1 && qty <= MB_MAX_COILS) {
    uint8_t nb = (uint8_t) ((qty + 7) / 8);
    out[0] = MB_ADDR; out[1] = MB_FC_READ_COILS; out[2] = nb; n = 3;
    out[n++] = coils;                                     // the 5 relay outputs are byte 0
    for (uint8_t k = 1; k < nb; k++) out[n++] = 0;
  }
  return n ? modbus_append_crc(out, n) : 0;
}

// Scan buf for one complete CRC-valid request addressed to us, build its reply into out,
// and erase what was consumed. Returns the reply length (0 = nothing to send).
//
// Only ONE request is answered per call, and the rest of the buffer is then dropped: those
// bytes are the master's traffic to other slaves, and keeping them risks a false CRC match
// across a frame boundary.
inline size_t modbus_poll(std::vector<uint8_t> &buf, const std::map<uint16_t, uint16_t> &regs,
                          uint8_t coils, uint8_t *out) {
  size_t i = 0;
  while (buf.size() - i >= MB_REQ_LEN) {
    const uint8_t *f = &buf[i];
    if (f[0] == MB_ADDR && (f[1] == MB_FC_READ_HOLDING || f[1] == MB_FC_READ_COILS)) {
      uint16_t crc = modbus_crc(f, MB_REQ_LEN - 2);
      if ((crc & 0xFF) == f[MB_REQ_LEN - 2] && (crc >> 8) == f[MB_REQ_LEN - 1]) {
        size_t n = modbus_build_reply(f, regs, coils, out);
        if (n) { buf.clear(); return n; }
        i += MB_REQ_LEN;                  // valid frame, nothing to say -> skip it whole
        continue;
      }
    }
    i++;                                  // not ours, or bad CRC -> resync one byte
  }
  if (i) buf.erase(buf.begin(), buf.begin() + i);
  if (buf.size() > MB_RX_MAX) buf.erase(buf.begin(), buf.end() - MB_REQ_LEN);
  return 0;
}
