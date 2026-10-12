#!/usr/bin/env python3
"""Sanity-check a FootPod recording and estimate cadence.

    pip install pandas numpy scipy matplotlib
    python footpod_plot.py run1.csv
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import find_peaks

ACC_CLIP_G = 15.9
GYR_CLIP_DPS = 1990


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--save", help="save the figure to this PNG instead of showing it")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    t = (df.t_us - df.t_us.iloc[0]).to_numpy() / 1e6
    fs = 1.0 / np.median(np.diff(t))
    amag = np.sqrt(df.ax_g**2 + df.ay_g**2 + df.az_g**2).to_numpy()

    # The gyro axis with the most variance is the foot's pitch (sagittal) axis.
    gcols = ["gx_dps", "gy_dps", "gz_dps"]
    pitch_col = df[gcols].var().idxmax()
    w = df[pitch_col].to_numpy()
    sign = 1 if w.max() >= -w.min() else -1
    ws = sign * w

    # One big pitch-rate peak per stride (mid-swing). Min spacing 0.4 s.
    height = 0.5 * np.percentile(ws, 99)
    peaks, _ = find_peaks(ws, height=height, distance=max(1, int(0.4 * fs)))
    stride_t = t[peaks]
    cad = 2 * 60 / np.diff(stride_t) if len(peaks) > 1 else np.array([])
    cad = cad[(cad > 60) & (cad < 260)]

    acc_clip = int((df[["ax_g", "ay_g", "az_g"]].abs() >= ACC_CLIP_G).any(axis=1).sum())
    gyr_clip = int((df[gcols].abs() >= GYR_CLIP_DPS).any(axis=1).sum())

    print(f"Duration        {t[-1]:.1f} s, {len(df)} samples, {fs:.0f} Hz effective")
    print(f"Accel peak      {amag.max():.1f} g   (samples at clip: {acc_clip})")
    print(f"Gyro peak       {np.abs(df[gcols].to_numpy()).max():.0f} dps   (samples at clip: {gyr_clip})")
    print(f"Pitch axis      {pitch_col} (sign {sign:+d})")
    print(f"Strides found   {len(peaks)}")
    if len(cad):
        print(f"Cadence         median {np.median(cad):.0f} spm  "
              f"(p10 {np.percentile(cad, 10):.0f}, p90 {np.percentile(cad, 90):.0f})")

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    axs[0].plot(t, amag, lw=0.6)
    axs[0].set_ylabel("|accel| (g)")
    for c in gcols:
        axs[1].plot(t, df[c], lw=0.6, label=c)
    axs[1].plot(t[peaks], w[peaks], "kx", label="stride")
    axs[1].set_ylabel("gyro (dps)")
    axs[1].legend(loc="upper right", fontsize=8)
    if len(peaks) > 1:
        mid = (stride_t[1:] + stride_t[:-1]) / 2
        c_all = 2 * 60 / np.diff(stride_t)
        axs[2].plot(mid, c_all, ".-", lw=0.6)
    axs[2].set_ylabel("cadence (spm)")
    axs[2].set_xlabel("time (s)")
    fig.suptitle(args.csv)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120)
    else:
        plt.show()


if __name__ == "__main__":
    main()
