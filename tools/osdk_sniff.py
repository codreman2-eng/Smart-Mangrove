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

  CRC checking is included but is a DIAGNOSTIC, never a gate. The seeds
  below are the commonly cited OSDK values but are NOT verified against this
  aircraft — so the tool reports a pass RATE and lets you confirm them
  empirically, rather than silently discarding frames because a constant
  this script assumed turned out to be wrong. Override with --crc16-init /
  --crc32-init if the rate comes back at zero.

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
import sys
import time
from collections import Counter
from datetime import datetime

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")


# ─── CRC — UNVERIFIED SEEDS, see the design note above ──────────────────
CRC16_INIT_DEFAULT = 0x3AA3
CRC32_INIT_DEFAULT = 0x3AA3AA3A


def crc16(data: bytes, init: int) -> int:
    """Reflected CCITT (poly 0x8408), DJI seed."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc32(data: bytes, init: int) -> int:
    """Reflected CRC-32 (poly 0xEDB88320), DJI seed."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
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
    p.add_argument("--hexdump", type=int, metavar="N", help="dump N raw bytes and exit")
    p.add_argument("--verbose", type=int, nargs="?", const=10**9, default=0,
                   help="print each frame (optionally limit count)")
    p.add_argument("--crc16-init", type=lambda x: int(x, 0), default=CRC16_INIT_DEFAULT)
    p.add_argument("--crc32-init", type=lambda x: int(x, 0), default=CRC32_INIT_DEFAULT)
    args = p.parse_args()

    if args.replay:
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
