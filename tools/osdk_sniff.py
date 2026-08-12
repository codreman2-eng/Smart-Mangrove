#!/usr/bin/env python3
"""
DJI OSDK API-port sniffer — Smart Launcher

Listen-only analyser for the DJI A3 Pro's Onboard SDK UART, run from a PC
through a USB-TTL adapter (FT232 etc.) set to 3.3V.

WHY A PC AND NOT A MICROCONTROLLER
  * Iteration: edit and rerun in seconds, no flash cycle.
  * Capture: every byte is saved raw, so a single session with the aircraft
    powered can be re-analysed offline as many times as you like.
  * The Mega 2560 physically cannot do 921600. At 16MHz in U2X mode the
    nearest achievable rates are 1,000,000 (+8.5%) and 666,667 (-27%);
    8N1 needs better than ~2%.

DESIGN NOTE — WHY CRC IS NOT THE SYNC MECHANISM
  Frame sync uses the header's own `length` field: if a 0xAA is found at
  offset i, and offset i+length also holds 0xAA, and that chains repeatedly,
  you are almost certainly looking at real framing. That test needs no
  knowledge of DJI's CRC polynomials or seeds.

  CRC checking is a DIAGNOSTIC, never a gate — and that decision paid off
  immediately. The first run against a real A3 found 601/601 frames with
  zero resyncs while scoring 0% on CRC, because the assumed polynomial was
  wrong. Had CRC gated acceptance, a perfectly healthy link would have
  reported as dead. The constants have since been recovered FROM that
  capture and now validate 601/601; --crc16-init / --crc32-init remain for
  any future firmware that changes them.

WIRING (listen-only for this tool)
    A3 API TX  ->  adapter RX
    A3 GND     ->  adapter GND
    adapter VCC:  NOT CONNECTED
  Confirm the API connector pinout from the A3 manual. Verify the adapter is
  on 3.3V with a meter BEFORE it goes near the flight controller.

USAGE
    python osdk_sniff.py --port COM7                    # live sniff @921600
    python osdk_sniff.py --port COM7 --seconds 30       # timed capture
    python osdk_sniff.py --scan --port COM7             # baud sweep
    python osdk_sniff.py --replay capture.bin           # offline re-analysis
    python osdk_sniff.py --port COM7 --hexdump 512      # raw bytes only
"""

import argparse
import struct
import sys
import time
from collections import Counter
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")


# ─── CRC — VERIFIED against a live A3 Pro capture, 2026-08-12 ───────────
# Recovered from 601 real broadcast frames, not from documentation:
#   CRC16 seed brute-forced over the 16-bit space per candidate polynomial.
#   CRC32 seed solved algebraically (CRC is linear over GF(2), so the init
#   is recoverable from a single known frame rather than searched).
# Both validate 601/601 frames. The earlier guess had the right seed for
# CRC16 but the wrong POLYNOMIAL (0x8408 reflected-CCITT rather than 0xA001
# reflected-IBM), and the wrong seed for CRC32 — which is precisely why the
# sniffer was built to sync without needing either.
CRC16_POLY = 0xA001          # reflected CRC-16/IBM (not CCITT)
CRC32_POLY = 0xEDB88320      # reflected CRC-32
CRC16_INIT_DEFAULT = 0x3AA3
CRC32_INIT_DEFAULT = 0x00003AA3   # NOT 0x3AA3AA3A


def crc16(data: bytes, init: int) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ CRC16_POLY if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc32(data: bytes, init: int) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ CRC32_POLY if crc & 1 else crc >> 1
    return crc & 0xFFFFFFFF


# ─── OSDK OpenProtocol header (12 bytes, little-endian bitfields) ───────
#   byte  0      SOF = 0xAA
#   bits  8-17   length   (10) — TOTAL frame length, header + payload + CRC32
#   bits 18-23   version  (6)
#   bits 24-28   sessionID(5)
#   bit  29      isAck    (1)
#   bytes 4-7    padding(5) | enc(3) | reserved(24)
#   bytes 8-9    sequence (16)
#   bytes 10-11  header CRC16
HEADER_LEN = 12
SOF = 0xAA
MIN_FRAME = HEADER_LEN + 4       # header + CRC32, no payload
MAX_FRAME = 1023                 # length field is 10 bits


class Frame:
    __slots__ = ("length", "version", "session", "is_ack", "enc",
                 "seq", "head_crc", "payload", "raw", "hdr_crc_ok", "crc32_ok")

    def __repr__(self):
        cs = f"{self.payload[0]:02X}" if len(self.payload) > 0 else "--"
        ci = f"{self.payload[1]:02X}" if len(self.payload) > 1 else "--"
        return (f"len={self.length:4d} v={self.version} ses={self.session:2d} "
                f"ack={self.is_ack} enc={self.enc} seq={self.seq:5d} "
                f"set=0x{cs} id=0x{ci} "
                f"hcrc={'ok ' if self.hdr_crc_ok else 'BAD'} "
                f"crc32={'ok ' if self.crc32_ok else 'BAD'}")


def parse_header(buf: bytes, i: int):
    """Decode a header at buf[i:]. Returns a partly-filled Frame or None."""
    if i + HEADER_LEN > len(buf):
        return None
    w0 = int.from_bytes(buf[i:i + 4], "little")
    if (w0 & 0xFF) != SOF:
        return None
    f = Frame()
    f.length = (w0 >> 8) & 0x3FF
    f.version = (w0 >> 18) & 0x3F
    f.session = (w0 >> 24) & 0x1F
    f.is_ack = (w0 >> 29) & 0x1
    w1 = int.from_bytes(buf[i + 4:i + 8], "little")
    f.enc = (w1 >> 5) & 0x7
    w2 = int.from_bytes(buf[i + 8:i + 12], "little")
    f.seq = w2 & 0xFFFF
    f.head_crc = (w2 >> 16) & 0xFFFF
    return f


def plausible(f: Frame) -> bool:
    return f is not None and MIN_FRAME <= f.length <= MAX_FRAME


def extract_frames(buf: bytes, crc16_init: int, crc32_init: int):
    """
    Walk the buffer emitting frames. Sync is by SOF + length-chaining; CRC is
    computed for reporting only and never used to accept or reject.
    Returns (frames, consumed_upto, resync_count).
    """
    frames, i, resyncs = [], 0, 0
    n = len(buf)
    while i < n:
        if buf[i] != SOF:
            i += 1
            continue
        f = parse_header(buf, i)
        if not plausible(f):
            i += 1
            resyncs += 1
            continue
        if i + f.length > n:
            break                                  # wait for more bytes

        raw = buf[i:i + f.length]
        hdr_ok = (crc16(raw[:10], crc16_init) == f.head_crc)

        # CORROBORATION — a plausible length is not enough on its own. Any
        # stray 0xAA in the stream yields some in-range length and, if taken
        # at face value, swallows the real frames behind it. Accept only if
        # the header CRC passes OR the next frame begins exactly where this
        # one's length says it should (the chaining test).
        end = i + f.length
        if not hdr_ok:
            if end >= n:
                break                              # can't chain-check yet
            if buf[end] != SOF:
                i += 1
                resyncs += 1
                continue

        f.raw = raw
        f.payload = raw[HEADER_LEN:-4]
        f.hdr_crc_ok = hdr_ok
        trailer = int.from_bytes(raw[-4:], "little")
        f.crc32_ok = (crc32(raw[:-4], crc32_init) == trailer)
        frames.append(f)
        i += f.length
    return frames, i, resyncs


# ─── Broadcast payload ──────────────────────────────────────────────────
#   cmd_set(1) | cmd_id(1) | passFlag(uint16) | enabled fields in BIT ORDER
#
# The passFlag semantics are CONFIRMED: on a live A3 the bits matched the
# DJI Assistant 2 rate configuration exactly (RTK and MAG clear because both
# were "Do Not Send"), and the three observed frame sizes correspond to
# 0x067F/231B, 0x0E7F/235B (+STATUS) and 0x3E7F/250B (+BATTERY,+DEVICE).
#
# The field SIZES below were derived empirically from that capture, not read
# out of the SDK headers, and they are self-consistent in a way that is hard
# to achieve by accident:
#   * POSITION lands on offset 72, where a 16-byte run of zeros sits (lat and
#     lon as float64 0.0 — the aircraft had no GPS fix indoors), immediately
#     followed by a stable 78.282 barometric altitude.
#   * QUATERNION lands on offset 19, and the four float32s there have a norm
#     of 1.0000. A wrong offset cannot produce a unit quaternion.
# decode() re-checks that norm at runtime and warns if it drifts, so a
# firmware change that shifts the layout announces itself instead of quietly
# producing plausible nonsense.
#
# Offsets past POSITION (GPS detail, RC, gimbal) are NOT pinned down. Use
# --diff to locate RC empirically rather than trusting a guess here.
FLAG_NAMES = ["TIME", "QUATERNION", "ACCEL", "VELOCITY", "ANG_RATE", "POSITION",
              "GPS", "RTK", "MAG", "RC", "GIMBAL", "STATUS", "BATTERY", "DEVICE"]

OFF_TIME, OFF_QUAT, OFF_ACCEL, OFF_VEL, OFF_RATE, OFF_POS, OFF_GPS = \
    4, 19, 35, 47, 60, 72, 97
RAD2DEG = 57.29577951308232


def decode_broadcast(payload: bytes):
    """Decode what is pinned down. Returns (dict, list_of_warnings)."""
    d, warn = {}, []
    if len(payload) < 4:
        return d, ["payload too short"]
    d["cmd_set"], d["cmd_id"] = payload[0], payload[1]
    flag = struct.unpack_from("<H", payload, 2)[0]
    d["passFlag"] = flag
    d["fields"] = [n for i, n in enumerate(FLAG_NAMES) if flag >> i & 1]

    def f32(o):
        return struct.unpack_from("<f", payload, o)[0]

    try:
        if flag & (1 << 0):
            d["time_2p5ms"] = struct.unpack_from("<I", payload, OFF_TIME)[0]
            d["time_ns"] = struct.unpack_from("<I", payload, OFF_TIME + 4)[0]
        if flag & (1 << 1):
            q = [f32(OFF_QUAT + 4 * i) for i in range(4)]
            d["quaternion"] = q
            norm = sum(x * x for x in q) ** 0.5
            d["quat_norm"] = norm
            # The layout self-check. Only a correct offset yields |q| = 1.
            if abs(norm - 1.0) > 0.01:
                warn.append(f"quaternion norm {norm:.4f} != 1.0 - "
                            "field layout is probably WRONG for this firmware")
        if flag & (1 << 2):
            d["accel"] = [f32(OFF_ACCEL + 4 * i) for i in range(3)]
        if flag & (1 << 3):
            d["velocity"] = [f32(OFF_VEL + 4 * i) for i in range(3)]
            d["vel_health"] = payload[OFF_VEL + 12]
        if flag & (1 << 4):
            d["ang_rate"] = [f32(OFF_RATE + 4 * i) for i in range(3)]
        if flag & (1 << 5):
            lat, lon = struct.unpack_from("<dd", payload, OFF_POS)
            d["lat_rad"], d["lon_rad"] = lat, lon
            d["lat_deg"], d["lon_deg"] = lat * RAD2DEG, lon * RAD2DEG
            d["altitude_m"] = f32(OFF_POS + 16)
            d["height_m"] = f32(OFF_POS + 20)
            d["pos_health"] = payload[OFF_POS + 24]
            if lat == 0.0 and lon == 0.0:
                warn.append("lat/lon are exactly 0 - no GPS fix "
                            "(expected indoors; recapture outside to confirm)")
        # Everything from OFF_GPS on is unmapped — expose it raw so --diff
        # has something to work against without pretending it is decoded.
        d["tail_hex"] = payload[OFF_GPS:].hex()
    except struct.error as e:
        warn.append(f"truncated payload: {e}")
    return d, warn


def print_decoded(d, warn):
    print(f"  passFlag 0x{d['passFlag']:04X} -> {', '.join(d['fields'])}")
    if "time_2p5ms" in d:
        print(f"  time      : {d['time_2p5ms']} x2.5ms  ({d['time_2p5ms']*2.5/1000:.1f}s)")
    if "quaternion" in d:
        q = d["quaternion"]
        print(f"  quaternion: [{q[0]:+.4f} {q[1]:+.4f} {q[2]:+.4f} {q[3]:+.4f}]"
              f"  |q|={d['quat_norm']:.5f}")
    for k, lbl in (("accel", "accel     "), ("velocity", "velocity  "),
                   ("ang_rate", "ang_rate  ")):
        if k in d:
            v = d[k]
            print(f"  {lbl}: [{v[0]:+9.4f} {v[1]:+9.4f} {v[2]:+9.4f}]")
    if "lat_deg" in d:
        print(f"  position  : {d['lat_deg']:.7f}, {d['lon_deg']:.7f} deg"
              f"   alt={d['altitude_m']:.2f}m height={d['height_m']:.2f}m"
              f" health={d['pos_health']}")
    for w in warn:
        print(f"  !! {w}")


def run_decode(args):
    data = open(args.replay, "rb").read() if args.replay else None
    if data is None:
        sys.exit("--decode needs --replay <capture.bin>")
    frames, _, _ = extract_frames(data, args.crc16_init, args.crc32_init)
    print(f"{len(frames)} frames in {args.replay}\n")
    n = args.decode if args.decode > 0 else 3
    for f in frames[:n]:
        d, w = decode_broadcast(f.payload)
        print(f"--- seq {f.seq} len {f.length} ---")
        print_decoded(d, w)
        print()
    # Layout confidence over the whole capture, not just the samples shown
    norms = [decode_broadcast(f.payload)[0].get("quat_norm")
             for f in frames if f.payload and len(f.payload) > OFF_QUAT + 16]
    norms = [x for x in norms if x is not None]
    if norms:
        bad = sum(1 for x in norms if abs(x - 1.0) > 0.01)
        print(f"LAYOUT CHECK: quaternion norm across {len(norms)} frames — "
              f"mean {sum(norms)/len(norms):.5f}, {bad} outside 1.0+/-0.01")
        print("  -> offsets are correct." if bad == 0 else
              "  -> LAYOUT IS WRONG for this firmware; re-derive the offsets.")


# ─── Diff mode: locate fields by changing them ─────────────────────────
def _payloads(path, crc16_init, crc32_init):
    data = open(path, "rb").read()
    frames, _, _ = extract_frames(data, crc16_init, crc32_init)
    by_flag = {}
    for f in frames:
        if len(f.payload) < 4:
            continue
        flag = struct.unpack_from("<H", f.payload, 2)[0]
        by_flag.setdefault(flag, []).append(f.payload)
    return by_flag


def run_diff(args):
    """
    Find which payload bytes changed between two captures.

    The intended use is locating a field with no documentation at all: record
    one capture with the pilot switch in one position, another with it moved,
    and whatever bytes are STABLE WITHIN each capture but DIFFERENT BETWEEN
    them are that switch. Requires no SDK headers and no guessing.

    Constantly-changing bytes (timers, attitude noise, the CRC) are filtered
    out by the within-capture stability test, which is what makes the result
    readable instead of a wall of diffs.
    """
    a = _payloads(args.diff[0], args.crc16_init, args.crc32_init)
    b = _payloads(args.diff[1], args.crc16_init, args.crc32_init)
    common = sorted(set(a) & set(b))
    if not common:
        sys.exit("No frame type common to both captures.")
    flag = max(common, key=lambda f: min(len(a[f]), len(b[f])))
    pa, pb = a[flag], b[flag]
    print(f"A: {args.diff[0]}  ({len(pa)} frames of passFlag 0x{flag:04X})")
    print(f"B: {args.diff[1]}  ({len(pb)} frames)\n")

    n = min(min(len(p) for p in pa), min(len(p) for p in pb))
    rows = []
    for off in range(n):
        va = {p[off] for p in pa}
        vb = {p[off] for p in pb}
        stable = len(va) == 1 and len(vb) == 1
        if stable and va != vb:
            rows.append((off, va.pop(), vb.pop(), "STABLE-DIFF"))
        elif len(va) <= 3 and len(vb) <= 3 and not (va & vb):
            rows.append((off, sorted(va)[0], sorted(vb)[0], "near-stable"))

    if not rows:
        print("  No stable byte differences. Either nothing was changed between"
              "\n  captures, or the field is one that also moves on its own.")
        return

    print("  off   A     B    delta  as int16(LE) A->B         kind")
    print("  " + "-" * 66)
    for off, x, y, kind in rows:
        i16a = i16b = ""
        if off + 1 < n:
            try:
                i16a = struct.unpack_from("<h", pa[0], off)[0]
                i16b = struct.unpack_from("<h", pb[0], off)[0]
            except struct.error:
                pass
        print(f"  {off:4d}  0x{x:02X}  0x{y:02X}  {y-x:+5d}  "
              f"{str(i16a):>8} -> {str(i16b):<8}   {kind}")

    grouped, run = [], []
    for off, *_ in rows:
        if run and off == run[-1] + 1:
            run.append(off)
        else:
            if run:
                grouped.append(run)
            run = [off]
    if run:
        grouped.append(run)
    print("\n  contiguous regions: " +
          ", ".join(f"{r[0]}..{r[-1]} ({len(r)}B)" for r in grouped))
    print("  A 2-byte region that tracks a stick or switch is an RC channel")
    print("  (DJI reports these as int16, roughly -10000..+10000).")


class Stats:
    def __init__(self):
        self.bytes = 0
        self.sof = 0
        self.frames = 0
        self.hdr_ok = 0
        self.crc32_ok = 0
        self.resyncs = 0
        self.cmds = Counter()
        self.lengths = Counter()
        self.t0 = time.time()

    def add(self, frames):
        for f in frames:
            self.frames += 1
            self.hdr_ok += f.hdr_crc_ok
            self.crc32_ok += f.crc32_ok
            self.lengths[f.length] += 1
            if len(f.payload) >= 2:
                self.cmds[(f.payload[0], f.payload[1])] += 1

    def report(self):
        el = max(time.time() - self.t0, 1e-6)
        print("\n" + "=" * 62)
        print("  OSDK SNIFF SUMMARY")
        print("=" * 62)
        print(f"  elapsed         : {el:.1f} s")
        print(f"  bytes           : {self.bytes}  ({self.bytes/el:.0f} B/s)")
        print(f"  0xAA seen       : {self.sof}")
        print(f"  frames          : {self.frames}  ({self.frames/el:.1f}/s)")
        print(f"  resyncs         : {self.resyncs}")
        if self.frames:
            print(f"  header CRC ok   : {self.hdr_ok}/{self.frames} "
                  f"({100*self.hdr_ok/self.frames:.1f}%)")
            print(f"  frame CRC32 ok  : {self.crc32_ok}/{self.frames} "
                  f"({100*self.crc32_ok/self.frames:.1f}%)")
            print("  frame lengths   : " +
                  ", ".join(f"{L}({c})" for L, c in self.lengths.most_common(8)))
            print("  cmd set/id      : " +
                  ", ".join(f"0x{a:02X}/0x{b:02X}({c})"
                            for (a, b), c in self.cmds.most_common(10)))
        print("=" * 62)
        self.verdict()

    def verdict(self):
        if self.bytes == 0:
            print("  VERDICT: NOTHING RECEIVED.")
            print("    -> Check TX/RX orientation, common ground, and that")
            print("       API Control is enabled in DJI Assistant 2.")
            print("    -> Broadcast should be continuous once ACTIVATED; if")
            print("       the aircraft has never been activated, silence here")
            print("       is the expected result, not a wiring fault.")
        elif self.frames == 0:
            print("  VERDICT: BYTES BUT NO FRAMING — almost certainly wrong baud.")
            print("    -> Try --scan to sweep common rates.")
        elif self.hdr_ok == 0:
            print("  VERDICT: FRAMING GOOD, CRC SEEDS WRONG.")
            print("    -> Length-chaining locked on, so this IS OSDK framing.")
            print("       The CRC constants this script assumed are not the")
            print("       ones in use. Frames above are still valid; retry")
            print("       with --crc16-init / --crc32-init to pin them down.")
        elif self.hdr_ok < self.frames * 0.9:
            print(f"  VERDICT: PARTIAL — {100*self.hdr_ok/self.frames:.0f}% header CRC.")
            print("    -> Real link with bit errors, or a marginal adapter.")
        else:
            print("  VERDICT: HEALTHY OSDK LINK. Framing and CRC both check out.")
            print("    -> Safe to move on to decoding the broadcast payload.")


def run_live(args):
    stats = Stats()
    cap_name = args.capture or f"osdk_{datetime.now():%Y%m%d_%H%M%S}.bin"
    print(f"Listening on {args.port} @ {args.baud} 8N1 — Ctrl-C to stop")
    print(f"Raw capture -> {cap_name}\n")

    buf = bytearray()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser, \
             open(cap_name, "wb") as cap:
            deadline = time.time() + args.seconds if args.seconds else None
            last_tick = time.time()
            while True:
                if deadline and time.time() > deadline:
                    break
                chunk = ser.read(4096)
                if chunk:
                    cap.write(chunk)
                    cap.flush()          # survive a Ctrl-C or a yanked cable
                    stats.bytes += len(chunk)
                    stats.sof += chunk.count(SOF)
                    buf.extend(chunk)
                    frames, used, rs = extract_frames(
                        buf, args.crc16_init, args.crc32_init)
                    stats.add(frames)
                    stats.resyncs += rs
                    if args.verbose:
                        for f in frames:
                            print("  " + repr(f))
                    del buf[:used]
                if time.time() - last_tick >= 1.0:
                    last_tick = time.time()
                    print(f"  {stats.bytes:>9} B | {stats.frames:>6} frames | "
                          f"hdrCRC {stats.hdr_ok:>6} | resync {stats.resyncs}")
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped.")
    stats.report()
    print(f"\n  Capture saved: {cap_name}")
    print(f"  Re-analyse without the aircraft:  python {sys.argv[0]} --replay {cap_name}")


def run_replay(args):
    with open(args.replay, "rb") as f:
        data = f.read()
    print(f"Replaying {args.replay} ({len(data)} bytes)\n")
    stats = Stats()
    stats.bytes = len(data)
    stats.sof = data.count(SOF)
    frames, _, rs = extract_frames(data, args.crc16_init, args.crc32_init)
    stats.add(frames)
    stats.resyncs = rs
    if args.verbose:
        for f in frames[:args.verbose]:
            print("  " + repr(f))
    stats.report()


def run_scan(args):
    """Sweep bauds. The winner is whichever yields the most chained frames."""
    rates = [921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600]
    print("Baud sweep — 3s each. Best framing wins.\n")
    results = []
    for baud in rates:
        try:
            with serial.Serial(args.port, baud, timeout=0.2) as ser:
                buf = bytearray()
                end = time.time() + 3
                while time.time() < end:
                    buf.extend(ser.read(4096))
            frames, _, _ = extract_frames(bytes(buf), args.crc16_init, args.crc32_init)
            ok = sum(f.hdr_crc_ok for f in frames)
            results.append((baud, len(buf), len(frames), ok))
            print(f"  {baud:>7} : {len(buf):>7} B  {len(frames):>5} frames  {ok:>5} hdrCRC ok")
        except serial.SerialException as e:
            print(f"  {baud:>7} : error — {e}")
    good = [r for r in results if r[2] > 0]
    print()
    if not good:
        print("  No framing at any rate. Check wiring/activation before baud.")
    else:
        best = max(good, key=lambda r: (r[3], r[2]))
        print(f"  BEST: {best[0]} baud ({best[2]} frames, {best[3]} header CRC ok)")


def run_hexdump(args):
    print(f"Raw hexdump — first {args.hexdump} bytes @ {args.baud}\n")
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        data = bytearray()
        end = time.time() + 10
        while len(data) < args.hexdump and time.time() < end:
            data.extend(ser.read(args.hexdump - len(data)))
    if not data:
        print("  (nothing received in 10s)")
        return
    for off in range(0, len(data), 16):
        row = data[off:off + 16]
        hexs = " ".join(f"{b:02X}" for b in row)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print(f"  {off:04X}  {hexs:<47}  {text}")
    print(f"\n  {len(data)} bytes, {data.count(SOF)} x 0xAA")


def main():
    p = argparse.ArgumentParser(description="DJI OSDK API-port sniffer (listen-only)")
    p.add_argument("--port", help="serial port, e.g. COM7")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--seconds", type=int, default=0, help="stop after N seconds")
    p.add_argument("--capture", help="raw capture filename")
    p.add_argument("--replay", help="analyse a saved capture instead of a port")
    p.add_argument("--scan", action="store_true", help="sweep common baud rates")
    p.add_argument("--decode", type=int, nargs="?", const=3, default=0,
                   metavar="N", help="decode N frames from --replay (default 3)")
    p.add_argument("--diff", nargs=2, metavar=("A.bin", "B.bin"),
                   help="find bytes that differ between two captures "
                        "(move a switch between them to locate its field)")
    p.add_argument("--hexdump", type=int, metavar="N", help="dump N raw bytes and exit")
    p.add_argument("--verbose", type=int, nargs="?", const=10**9, default=0,
                   help="print each frame (optionally limit count)")
    p.add_argument("--crc16-init", type=lambda x: int(x, 0), default=CRC16_INIT_DEFAULT)
    p.add_argument("--crc32-init", type=lambda x: int(x, 0), default=CRC32_INIT_DEFAULT)
    args = p.parse_args()

    if args.diff:
        run_diff(args)
    elif args.decode and args.replay:
        run_decode(args)
    elif args.replay:
        run_replay(args)
    elif not args.port:
        p.error("--port is required unless --replay is used")
    elif args.scan:
        run_scan(args)
    elif args.hexdump:
        run_hexdump(args)
    else:
        run_live(args)


if __name__ == "__main__":
    main()
