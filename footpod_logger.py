#!/usr/bin/env python3
"""Record raw IMU data from a FootPod over BLE to CSV.

    pip install bleak
    python footpod_logger.py --name FootPod-L --out run1.csv
    (Ctrl-C to stop, or pass --duration seconds)
"""
import argparse
import asyncio
import csv
import struct
import time

from bleak import BleakClient, BleakScanner

IMU_CHAR = "a3c40001-5f1d-4b8e-9c3a-7d1e2f3b4c50"
SAMPLES_PER_PKT = 19
HDR = struct.Struct("<IHBB")             # t0_us, seq, n, odr_div8
PKT_LEN = HDR.size + SAMPLES_PER_PKT * 12
ACC_G_PER_LSB = 0.000488                 # ±16 g
GYR_DPS_PER_LSB = 0.070                  # ±2000 dps


class Recorder:
    def __init__(self, writer):
        self.w = writer
        self.last_seq = None
        self.last_t0 = None
        self.wrap = 0
        self.samples = 0
        self.lost_pkts = 0
        self.bad_len = 0

    def handle(self, _sender, data: bytearray):
        if len(data) != PKT_LEN:
            self.bad_len += 1
            return
        t0, seq, n, odr_div8 = HDR.unpack_from(data)
        if self.last_t0 is not None and t0 < self.last_t0 - 2**31:
            self.wrap += 2**32                      # micros() rollover (~71 min)
        self.last_t0 = t0
        if self.last_seq is not None:
            self.lost_pkts += (seq - self.last_seq - 1) & 0xFFFF
        self.last_seq = seq

        dt = 1e6 / (odr_div8 * 8)
        vals = struct.unpack_from(f"<{n * 6}h", data, HDR.size)
        base = t0 + self.wrap
        for i in range(n):
            gx, gy, gz, ax, ay, az = vals[i * 6:(i + 1) * 6]
            self.w.writerow([
                round(base + i * dt), seq,
                round(ax * ACC_G_PER_LSB, 4), round(ay * ACC_G_PER_LSB, 4), round(az * ACC_G_PER_LSB, 4),
                round(gx * GYR_DPS_PER_LSB, 2), round(gy * GYR_DPS_PER_LSB, 2), round(gz * GYR_DPS_PER_LSB, 2),
            ])
        self.samples += n


async def run(args):
    print(f"Scanning for {args.name} ...")
    dev = await BleakScanner.find_device_by_name(args.name, timeout=20)
    if dev is None:
        raise SystemExit(f"{args.name} not found — is it powered and advertising?")

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_us", "seq", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"])
        rec = Recorder(w)

        async with BleakClient(dev) as client:
            print(f"Connected (MTU {client.mtu_size}). Recording to {args.out} — Ctrl-C to stop.")
            if client.mtu_size and client.mtu_size < PKT_LEN + 3:
                print("WARNING: MTU too small for full packets; data will be rejected.")
            await client.start_notify(IMU_CHAR, rec.handle)
            start = time.time()
            last = 0
            try:
                while args.duration is None or time.time() - start < args.duration:
                    await asyncio.sleep(1)
                    rate = rec.samples - last
                    last = rec.samples
                    print(f"{time.time() - start:6.0f}s  {rate:4d} samples/s  "
                          f"lost pkts {rec.lost_pkts}  bad pkts {rec.bad_len}", end="\r")
            finally:
                try:
                    await client.stop_notify(IMU_CHAR)
                except Exception:
                    pass
                f.flush()
                print(f"\nSaved {rec.samples} samples, lost {rec.lost_pkts} packets.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="FootPod-L")
    ap.add_argument("--out", default="footpod_run.csv")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default: until Ctrl-C)")
    args = ap.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
