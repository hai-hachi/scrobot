#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import time

import serial

SOF = b"\xAA\x55"
VER = 2

TYPE_SETPOINT = 0x10
TYPE_ARM = 0x11
TYPE_DISARM = 0x12
TYPE_FEEDBACK = 0x20
TYPE_DIAGNOSTICS = 0x21
TYPE_INFO_REQUEST = 0x50
TYPE_INFO_RESPONSE = 0x51
TYPE_ERROR = 0x7F

RADIUS_M = 0.050
TRACK_M = 0.420
CPR_WR = 3264.0
CPR_WL = 3264.0
ENC_SIGN_WR = -1.0
ENC_SIGN_WL = 1.0

STATUS = {
    0: "ARMED",
    1: "ESTOP",
    2: "COMM_TIMEOUT",
    3: "SYSID_MODE",
    4: "UART_ERROR",
    5: "INVALID_OUTPUT",
    6: "TX_QUEUE_DROP",
    7: "INVALID_COMMAND",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BBHB", VER, msg_type, seq & 0xFFFF, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


def status_text(flags: int) -> str:
    names = [name for bit, name in STATUS.items() if flags & (1 << bit)]
    return "|".join(names) if names else "OK"


class Parser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                self.buf = self.buf[-1:]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < 7:
                break
            payload_len = self.buf[6]
            total = 9 + payload_len
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            body = raw[2:-2]
            rx_crc = struct.unpack_from("<H", raw, total - 2)[0]
            if crc16_ccitt_false(body) != rx_crc:
                continue
            ver, typ, seq, plen = struct.unpack_from("<BBHB", raw, 2)
            if ver != VER or plen != payload_len:
                continue
            out.append((typ, seq, raw[7:7 + plen]))
        return out


def parse_feedback(payload: bytes):
    if len(payload) != 50:
        return None
    tick, flags, last_seq = struct.unpack_from("<IIH", payload, 0)
    counts = struct.unpack_from("<iiiii", payload, 10)
    rpms = struct.unpack_from("<fffff", payload, 30)
    return tick, flags, last_seq, counts, rpms


def delta_i32(now: int, prev: int) -> int:
    d = (now - prev) & 0xFFFFFFFF
    if d & 0x80000000:
        d -= 0x100000000
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--baud", type=int, default=1_000_000)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("monitor")
    p = sub.add_parser("wheels")
    p.add_argument("--rpm", type=float, default=30.0)
    p.add_argument("--seconds", type=float, default=3.0)

    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    ser.reset_input_buffer()
    parser = Parser()
    seq = 1

    def send(t, payload=b""):
        nonlocal seq
        ser.write(frame(t, seq, payload))
        seq = (seq + 1) & 0xFFFF

    send(TYPE_INFO_REQUEST)

    if args.cmd == "monitor":
        print("Monitoring feedback. Ctrl-C to stop.")
        try:
            while True:
                for typ, rxseq, payload in parser.feed(ser.read(4096)):
                    if typ == TYPE_FEEDBACK:
                        fb = parse_feedback(payload)
                        if fb:
                            tick, flags, last_seq, counts, rpms = fb
                            print(
                                f"tick={tick:8d} status={status_text(flags):20s} "
                                f"WR={rpms[0]:7.2f} rpm WL={rpms[1]:7.2f} rpm "
                                f"countWR={counts[0]:11d} countWL={counts[1]:11d}"
                            )
                    elif typ == TYPE_INFO_RESPONSE and len(payload) == 4:
                        print(f"FW={payload[0]}.{payload[1]}.{payload[2]} protocol={payload[3]}")
                    elif typ == TYPE_ERROR:
                        print(f"ERROR seq={rxseq} payload={payload.hex()}")
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
        return

    rpm = float(args.rpm)
    if abs(rpm) > 100.0:
        raise SystemExit("|rpm| must be <= 100 for WR/WL firmware limits")

    print("WARNING: jack the drive wheels off the ground and keep E-stop reachable.")
    print(f"Commanding WR=WL={rpm:.1f} rpm for {args.seconds:.1f} s")

    send(TYPE_ARM)
    time.sleep(0.05)

    start = time.monotonic()
    next_tx = start
    prev_counts = None
    x = y = yaw = 0.0
    last_print = 0.0

    try:
        while time.monotonic() - start < args.seconds:
            now = time.monotonic()
            if now >= next_tx:
                payload = struct.pack("<fffff", rpm, rpm, 0.0, 0.0, 0.0)
                send(TYPE_SETPOINT, payload)
                next_tx += 0.02

            for typ, rxseq, payload in parser.feed(ser.read(4096)):
                if typ == TYPE_ERROR:
                    print(f"ERROR seq={rxseq} payload={payload.hex()}")
                    continue
                if typ != TYPE_FEEDBACK:
                    continue
                fb = parse_feedback(payload)
                if not fb:
                    continue
                tick, flags, last_seq, counts, rpms = fb

                if prev_counts is not None:
                    dcr = delta_i32(counts[0], prev_counts[0]) * ENC_SIGN_WR
                    dcl = delta_i32(counts[1], prev_counts[1]) * ENC_SIGN_WL
                    dr = dcr * (2.0 * math.pi * RADIUS_M / CPR_WR)
                    dl = dcl * (2.0 * math.pi * RADIUS_M / CPR_WL)
                    ds = 0.5 * (dr + dl)
                    dyaw = (dr - dl) / TRACK_M
                    yaw_mid = yaw + 0.5 * dyaw
                    x += ds * math.cos(yaw_mid)
                    y += ds * math.sin(yaw_mid)
                    yaw += dyaw

                prev_counts = counts

                if now - last_print >= 0.20:
                    print(
                        f"status={status_text(flags):16s} "
                        f"WR={rpms[0]:7.2f} WL={rpms[1]:7.2f} rpm  "
                        f"odom x={x:+.4f} m y={y:+.4f} m yaw={math.degrees(yaw):+.2f} deg"
                    )
                    last_print = now
    finally:
        for _ in range(10):
            send(TYPE_SETPOINT, struct.pack("<fffff", 0.0, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.02)
        send(TYPE_DISARM)
        ser.close()

    expected_v = rpm * 2.0 * math.pi * RADIUS_M / 60.0
    print(f"Done. Expected straight speed at {rpm:.1f} rpm: {expected_v:.4f} m/s")
    print(f"Integrated odom: x={x:+.4f} m y={y:+.4f} m yaw={math.degrees(yaw):+.2f} deg")


if __name__ == "__main__":
    main()
