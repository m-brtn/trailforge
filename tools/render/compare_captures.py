#!/usr/bin/env python3
"""Pixel-diff statistics between two directories of captures with the same file names.

Usage: compare_captures.py BEFORE_DIR AFTER_DIR [--diff-dir DIR]

Per image: mean absolute difference (0..255, over RGB), max difference, share of pixels whose largest channel
difference exceeds 2 and 8 levels, PSNR. With --diff-dir an amplified (x8) difference image is written per pair.
Needs numpy and Pillow.
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image


def load(path):
    return np.asarray(Image.open(path).convert('RGB'), dtype=np.int16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('before')
    ap.add_argument('after')
    ap.add_argument('--diff-dir')
    args = ap.parse_args()
    names = sorted(n for n in os.listdir(args.before) if n.lower().endswith('.png'))
    if not names:
        print('no PNG files in', args.before)
        return 1
    if args.diff_dir:
        os.makedirs(args.diff_dir, exist_ok=True)
    print('%-52s %7s %5s %8s %8s %7s' % ('image', 'mean', 'max', '>2 (%)', '>8 (%)', 'PSNR'))
    totals = []
    missing = 0
    for n in names:
        b_path, a_path = os.path.join(args.before, n), os.path.join(args.after, n)
        if not os.path.exists(a_path):
            print('%-52s missing in %s' % (n, args.after))
            missing += 1
            continue
        b, a = load(b_path), load(a_path)
        if b.shape != a.shape:
            print('%-52s size differs %s vs %s' % (n, b.shape, a.shape))
            missing += 1
            continue
        d = np.abs(a - b)
        dmax = d.max(axis=2)
        mse = float(np.mean((a - b).astype(np.float64) ** 2))
        psnr = float('inf') if mse == 0 else 10 * math.log10(255.0 ** 2 / mse)
        row = (float(d.mean()), int(d.max()), 100.0 * float(np.mean(dmax > 2)), 100.0 * float(np.mean(dmax > 8)), psnr)
        totals.append(row)
        print('%-52s %7.3f %5d %8.3f %8.3f %7.1f' % ((n,) + row))
        if args.diff_dir:
            Image.fromarray(np.clip(d * 8, 0, 255).astype(np.uint8)).save(os.path.join(args.diff_dir, n))
    if totals:
        t = np.array(totals)
        finite = t[np.isfinite(t[:, 4]), 4]
        print('%-52s %7.3f %5d %8.3f %8.3f %7.1f' % ('ALL (%d images, mean; max of max)' % len(totals), t[:, 0].mean(),
                                                     int(t[:, 1].max()), t[:, 2].mean(), t[:, 3].mean(),
                                                     finite.mean() if len(finite) else float('inf')))
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
