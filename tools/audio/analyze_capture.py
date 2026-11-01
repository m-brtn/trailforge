#!/usr/bin/env python3
"""Analyze an offline audio capture made with `trailforge --audio-capture out.wav`.

Reads out.wav (float32 stereo) and the per-tick mixer parameters in out.wav.csv, prints:
  - sample peak, clipped samples, integrated loudness and true peak (ffmpeg ebur128, if ffmpeg is installed);
  - engine pitch tracking during the first full-throttle run: the fundamental found in the audio vs. the firing
    frequency of the inline-4 at the played engine speed (rpm_src / 30, two firings per revolution);
  - the 1-6 kHz band level while the tires skid compared with the run before braking;
  - every tire squeal segment (mixer skid > 0.05 for >= 0.2 s): rise time, largest 5 ms level step, held roughness,
    release, spectral centroid and share above 3 kHz;
  - spectrum balance of the day and the night ambience segments;
  - body / suspension events (bump-stop knocks, body rattle, body impacts): count, rate, level against the sound
    before each event, the spectral centroid of the event's own sound, its decay to -20 dB, the 1-6 kHz transients
    after its onset and its crest factor.
Writes spectrograms with sox (if installed) to <prefix>_*.png.

Usage: analyze_capture.py out.wav [--png-prefix /tmp/trailforge_audio]
Requires numpy.
"""

import argparse
import csv
import shutil
import struct
import subprocess
import sys

import numpy as np


def read_wav(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise ValueError("not a RIFF/WAVE file")
    pos, fmt, pcm = 12, None, None
    while pos + 8 <= len(data):
        cid, size = data[pos:pos + 4], struct.unpack("<I", data[pos + 4:pos + 8])[0]
        body = data[pos + 8:pos + 8 + size]
        if cid == b"fmt ":
            tag, ch, rate, _, _, bits = struct.unpack("<HHIIHH", body[:16])
            if tag == 0xFFFE:
                tag = struct.unpack("<H", body[24:26])[0]
            fmt = (tag, ch, rate, bits)
        elif cid == b"data":
            pcm = body
        pos += 8 + size + (size & 1)
    tag, ch, rate, bits = fmt
    if tag == 3 and bits == 32:
        x = np.frombuffer(pcm, dtype="<f4")
    elif tag == 1 and bits == 16:
        x = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
    else:
        raise ValueError(f"unsupported WAV format tag={tag} bits={bits}")
    return x.reshape(-1, ch), rate


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def db(x):
    return 20.0 * np.log10(max(float(x), 1e-12))


def band_db(mono, rate, t0, t1, lo, hi):
    seg = mono[int(t0 * rate):int(t1 * rate)]
    spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg)))) ** 2
    freqs = np.fft.rfftfreq(len(seg), 1.0 / rate)
    band = spec[(freqs >= lo) & (freqs < hi)].sum()
    return 10.0 * np.log10(max(band / len(seg), 1e-20))


def harmonic_f0(seg, rate, lo, hi, harmonics=4):
    n = 1 << 15
    spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg)), n))
    df = rate / n
    cands = np.arange(lo, hi, 0.25)
    score = np.zeros_like(cands)
    for k in range(1, harmonics + 1):
        idx = np.clip(np.round(cands * k / df).astype(int), 0, len(spec) - 1)
        score += np.log(spec[idx] + 1e-9)
    return cands[int(np.argmax(score))]


def first(mask, start=0):
    idx = np.nonzero(mask[start:])[0]
    return start + int(idx[0]) if len(idx) else None


def ebur128(path):
    if not shutil.which("ffmpeg"):
        return None
    out = subprocess.run(["ffmpeg", "-hide_banner", "-nostats", "-i", path, "-af", "ebur128=peak=true",
                          "-f", "null", "-"], capture_output=True, text=True).stderr
    summary = out[out.rfind("Summary:"):]
    vals = {}
    for line in summary.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] in ("I:", "LRA:", "Peak:"):
            vals[parts[0][:-1]] = float(parts[1])
    return vals


def spectrogram(path, png, title, t0=None, dur=None, max_hz=None):
    if not shutil.which("sox"):
        return False
    cmd = ["sox", path, "-n", "remix", "1,2"]
    if t0 is not None:
        cmd += ["trim", f"{t0:.3f}", f"{dur:.3f}"]
    if max_hz:
        cmd += ["rate", str(int(max_hz * 2))]
    cmd += ["spectrogram", "-o", png, "-t", title, "-x", "1400", "-y", "513", "-z", "90"]
    return subprocess.run(cmd, capture_output=True).returncode == 0


def bandpass(x, rate, lo, hi):
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1.0 / rate)
    spec[(f < lo) | (f >= hi)] = 0.0
    return np.fft.irfft(spec, len(x))


def envelope_db(x, rate, win):
    n = max(1, int(win * rate))
    k = len(x) // n
    e = np.sqrt(np.mean(x[:k * n].reshape(k, n) ** 2, axis=1))
    return 20.0 * np.log10(np.maximum(e, 1e-9))


def event_shape(mono, rate, o):
    """Decay and clatter of one body event at onset o (s): time until the event's own 5 ms envelope (power above the
    bed of the 300 ms before) stays 20 dB below its peak; the number of distinct 1-6 kHz transients in the 400 ms
    after the onset (2 ms envelope peaks 6 dB above the band's bed, >= 8 ms apart); crest factor of the 250 ms."""
    pre, post = 0.3, 0.6
    if o < pre or int((o + post) * rate) > len(mono):
        return (float("nan"), 0.0, float("nan"))
    seg = mono[int((o - pre) * rate):int((o + post) * rate)]
    e = 10 ** (envelope_db(seg, rate, 0.005) / 10.0)
    k0 = int(pre / 0.005)
    bed = np.median(e[:k0 - 4])
    own = np.clip(e[k0:] - bed, 0.0, None)
    decay = float("nan")
    if own.max() > 0:
        ipk = int(np.argmax(own))
        above = np.nonzero(own > own[ipk] * 0.01)[0]
        decay = (above[-1] + 1) * 0.005
    hb = bandpass(seg, rate, 1000, 6000)
    eh = envelope_db(hb, rate, 0.002)
    kb = int(pre / 0.002)
    bedh = np.median(eh[:kb - 10])
    win = eh[kb:kb + int(0.4 / 0.002)]
    count, last = 0, -100
    for i in range(1, len(win) - 1):
        if win[i] > bedh + 6.0 and win[i] >= win[i - 1] and win[i] > win[i + 1] and i - last >= 4:
            count += 1
            last = i
    ev = mono[int(o * rate):int((o + 0.25) * rate)]
    crest = db(np.abs(ev).max()) - db(np.sqrt(np.mean(ev ** 2)))
    return (decay, float(count), crest)


def runs(mask, t, min_len):
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_len:
                out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def analyze_squeal(mono, rate, p):
    """Tire squeal segments (mixer `skid` > 0.05 for >= 0.2 s). For each: the 0.7-6 kHz band of the capture, its own
    power above the bed of the 0.4 s before the segment; onset rise time 10 -> 90 % of the own amplitude peak, the
    largest 5 ms step of the band envelope in the first 0.4 s (abruptness), the roughness of the held part (std of
    10 ms envelope differences, dB), the release time (end of the segment until the own power is 20 dB below the
    held level) and the centroid / share above 3 kHz of the squeal's own spectrum."""
    t, skid = p["t"], p["skid"]
    segs = runs(skid > 0.05, t, 0.2)
    if not segs:
        print("tire squeal: no squeal segments")
        return
    rows = []
    for i, j in segs:
        a, b = t[i], t[j]
        if a < 0.6 or int((b + 0.8) * rate) > len(mono):
            continue
        t0 = a - 0.5
        seg = mono[int(t0 * rate):int((b + 0.8) * rate)]
        band = bandpass(seg, rate, 700, 6000)
        e5 = 10 ** (envelope_db(band, rate, 0.005) / 10.0)
        bed = np.median(e5[int(0.05 / 0.005):int(0.45 / 0.005)])
        own = np.sqrt(np.clip(e5 - bed, 0.0, None))
        k0 = int(0.5 / 0.005)
        kb = int((b - t0) / 0.005)
        held = own[k0:kb + 1]
        if held.max() <= 0:
            continue
        pk = held.max()
        i10 = np.nonzero(own[k0 - 10:] >= 0.1 * pk)[0]
        i90 = np.nonzero(own[k0 - 10:] >= 0.9 * pk)[0]
        rise = (i90[0] - i10[0]) * 0.005 if len(i10) and len(i90) else float("nan")
        edb = 20 * np.log10(np.maximum(np.sqrt(e5), 1e-9))
        step = float(np.max(np.diff(edb[k0 - 4:k0 + 80]))) if kb > k0 + 2 else float("nan")
        e10 = envelope_db(band, rate, 0.010)
        h0, h1 = int((a - t0 + 0.3) / 0.01), int((b - t0 - 0.05) / 0.01)
        rough = float(np.std(np.diff(e10[h0:h1]))) if h1 - h0 > 5 else float("nan")
        level = np.median(held[len(held) // 3:]) if len(held) > 6 else pk
        quiet = own[kb:] < 0.1 * level
        release = float("nan")
        for k in range(len(quiet) - 10):
            if quiet[k:k + 10].all():
                release = k * 0.005
                break
        s = mono[int(a * rate):int(b * rate)]
        bseg = mono[int((a - 0.45) * rate):int((a - 0.05) * rate)]
        n = len(s)
        ss = np.abs(np.fft.rfft(s * np.hanning(n))) ** 2 / n
        sb = np.abs(np.fft.rfft(np.resize(bseg, n) * np.hanning(n))) ** 2 / n
        f = np.fft.rfftfreq(n, 1.0 / rate)
        d = np.clip(ss - sb, 0.0, None)
        m = (f >= 200) & (f <= 16000)
        tot = d[m].sum()
        centroid = float((d[m] * f[m]).sum() / tot) if tot > 0 else float("nan")
        hi = float(d[(f >= 3000) & (f <= 16000)].sum() / tot) if tot > 0 else float("nan")
        rows.append((a, b - a, float(skid[i:j + 1].max()), rise, step, rough, release, centroid, hi,
                     db(np.sqrt(np.mean(band[k0 * 240:kb * 240] ** 2)))))
    if not rows:
        print("tire squeal: no analysable segments")
        return
    r = np.array(rows)
    print(f"tire squeal: {len(r)} segments (mixer skid > 0.05 for >= 0.2 s)")
    for row in r:
        print(f"    t={row[0]:6.2f} s {row[1]:4.2f} s skid max {row[2]:.2f}: rise {1000 * row[3]:4.0f} ms, max 5 ms step "
              f"{row[4]:+5.1f} dB, held roughness {row[5]:4.2f} dB, release {1000 * row[6]:4.0f} ms, centroid "
              f"{row[7]:5.0f} Hz, >3 kHz {100 * row[8]:4.1f} %, 0.7-6 kHz {row[9]:6.1f} dBFS")
    print(f"  median: rise {1000 * np.nanmedian(r[:, 3]):.0f} ms, max 5 ms step {np.nanmedian(r[:, 4]):+.1f} dB, held "
          f"roughness {np.nanmedian(r[:, 5]):.2f} dB, release {1000 * np.nanmedian(r[:, 6]):.0f} ms, centroid "
          f"{np.nanmedian(r[:, 7]):.0f} Hz, >3 kHz {100 * np.nanmedian(r[:, 8]):.1f} %")


def analyze_impacts(mono, rate, p, prefix, wav):
    """Body / suspension events (CSV columns ev_bump, ev_rattle, ev_impact: cumulative counts per tick). For every
    onset: peak and RMS of the 250 ms after it against the 500 ms before it (engine, tires and wind), the event's own
    power (event window minus the window before, in power), and the spectral centroid and the share below 200 Hz of
    that difference spectrum (the event's own sound with the steady bed removed)."""
    if "ev_rattle" not in p:
        return
    t = p["t"]
    dur = len(mono) / rate
    win, pre = 0.25, 0.5
    names = (("ev_bump", "bump-stop / knock"), ("ev_rattle", "rattle (before: suspension thump)"),
             ("ev_impact", "body impact"))
    onsets_all = []
    print(f"body / suspension events over {dur:.1f} s:")
    for col, label in names:
        c = p[col]
        rows = np.nonzero(np.diff(c) > 0)[0] + 1
        onsets = [max(0.0, t[i] - 1.0 / 120.0) for i in rows for _ in range(int(c[i] - c[i - 1]))]
        onsets_all += onsets
        if not onsets:
            print(f"  {label:34s}: 0 events")
            continue
        ons = np.array(onsets)
        per_s = max(int(((ons >= o) & (ons < o + 1.0)).sum()) for o in ons)
        stats = []
        for o in sorted(set(onsets)):
            if o < pre or o + win > dur:
                continue
            ev = mono[int(o * rate):int((o + win) * rate)]
            base = mono[int((o - pre) * rate):int((o - 0.02) * rate)]
            pe, pb = float(np.mean(ev ** 2)), float(np.mean(base ** 2))
            n = len(ev)
            se = np.abs(np.fft.rfft(ev * np.hanning(n))) ** 2
            sb = np.abs(np.fft.rfft(base[-n:] * np.hanning(n), n)) ** 2 if len(base) >= n else \
                np.abs(np.fft.rfft(np.resize(base, n) * np.hanning(n))) ** 2
            f = np.fft.rfftfreq(n, 1.0 / rate)
            d = np.clip(se - sb, 0.0, None)
            band = (f >= 20) & (f <= 16000)
            tot = d[band].sum()
            centroid = float((d[band] * f[band]).sum() / tot) if tot > 0 else float("nan")
            low = float(d[(f >= 20) & (f < 200)].sum() / tot) if tot > 0 else float("nan")
            stats.append((o, db(np.abs(ev).max()), 10 * np.log10(max(pe, 1e-20)), 10 * np.log10(max(pb, 1e-20)),
                          10 * np.log10(max(pe - pb, 1e-20)), centroid, low))
            stats[-1] += event_shape(mono, rate, o)
        s = np.array(stats) if stats else np.zeros((0, 10))
        print(f"  {label:34s}: {len(onsets)} events, {len(onsets) / dur:.2f}/s over the capture, max {per_s} in 1 s")
        if len(s):
            own_rel = s[:, 4] - s[:, 3]
            print(f"    peak {np.median(s[:, 1]):6.1f} dBFS median / {s[:, 1].max():6.1f} max; window RMS "
                  f"{np.median(s[:, 2]):6.1f} dB vs {np.median(s[:, 3]):6.1f} dB before (median "
                  f"{np.median(s[:, 2] - s[:, 3]):+.1f} dB, max {(s[:, 2] - s[:, 3]).max():+.1f} dB)")
            print(f"    event's own power {np.median(own_rel):+.1f} dB re the bed before (median), max "
                  f"{own_rel.max():+.1f} dB; centroid of the event's own spectrum {np.nanmedian(s[:, 5]):.0f} Hz "
                  f"median ({np.nanmin(s[:, 5]):.0f}-{np.nanmax(s[:, 5]):.0f}), share below 200 Hz "
                  f"{100 * np.nanmedian(s[:, 6]):.0f} % median")
            print(f"    decay to -20 dB of the event's own envelope {1000 * np.nanmedian(s[:, 7]):.0f} ms median "
                  f"(max {1000 * np.nanmax(s[:, 7]):.0f}); 1-6 kHz transients in the 400 ms after the onset "
                  f"{np.median(s[:, 8]):.0f} median (max {s[:, 8].max():.0f}); crest factor {np.median(s[:, 9]):.1f} dB")
    if onsets_all:
        o = min(onsets_all)
        spectrogram(wav, f"{prefix}_impacts.png", "body / suspension events (first 12 s from the first event)",
                    max(0.0, o - 1.0), min(12.0, dur - max(0.0, o - 1.0)))


def analyze_electric(mono, rate, p, prefix, wav):
    """Electric drive: the motor order line found in the audio vs. the mixer's motor frequency (motor_hz, the
    electrical order of the faster motor) while it sounds, the inverter carrier level at low and high motor
    frequency, and the levels of the regen phase."""
    t, hz, lvl = p["t"], p["motor_hz"], p["motor_level"]
    sounding = (hz > 60.0) & (lvl > 0.004) & (p["gameplay"] > 0.99)
    idx = np.nonzero(sounding)[0]
    if not len(idx):
        print("electric: the motor never sounds")
        return
    ta, tb = t[idx[0]], t[idx[-1]]
    win, hop = 0.10, 0.05
    rows = []
    for c in np.arange(ta + win / 2, tb - win / 2, hop):
        k = int(np.searchsorted(t, c))
        if k >= len(t) or not sounding[k]:
            continue
        seg = mono[int((c - win / 2) * rate):int((c + win / 2) * rate)]
        expected = float(np.interp(c, t, hz))
        near = harmonic_f0(seg, rate, expected * 0.85, expected * 1.15, harmonics=2)
        rows.append((c, expected, near, float(np.interp(c, t, lvl))))
    if not rows:
        print("electric: no analysis windows")
        return
    r = np.array(rows)
    err = r[:, 2] / r[:, 1] - 1.0
    print(f"electric motor {ta:.2f}-{tb:.2f} s: motor order {r[:, 1].min():.0f}-{r[:, 1].max():.0f} Hz, "
          f"{len(r)} windows of {win * 1000:.0f} ms")
    print(f"  order line searched within +-15 %: median error {np.median(err) * 100:+.2f} %, |error| < 2 % in "
          f"{np.mean(np.abs(err) < 0.02) * 100:.0f} % of windows, correlation {np.corrcoef(r[:, 1], r[:, 2])[0, 1]:.4f}")
    for c, e, n_, l_ in r[::max(1, len(r) // 8)]:
        print(f"    t={c:6.2f} s  expected {e:7.1f} Hz  found {n_:7.1f} Hz  mixer level {l_:.4f}")
    inv = p["inverter_level"]
    low = sounding & (hz < 150.0)
    high = sounding & (hz > 500.0)
    if low.any() and high.any():
        print(f"  inverter carrier level: {inv[low].mean():.4f} at motor order < 150 Hz, {inv[high].mean():.4f} above "
              f"500 Hz (fades out with speed)")
        tl = t[low]
        th = t[high]
        print(f"  3.0-3.5 kHz band at low speed {band_db(mono, rate, tl[0], tl[-1] + 0.05, 3000, 3500):.1f} dB, at "
              f"high speed {band_db(mono, rate, th[0], th[-1], 3000, 3500):.1f} dB")
    # Regen: the motor sounds while the throttle is released (load ~0).
    regen = sounding & (p["load"] < 0.02)
    drive = sounding & (p["load"] > 0.5)
    if regen.any() and drive.any():
        print(f"  mixer motor level: drive {lvl[drive].mean():.4f}, regen / coast {lvl[regen].mean():.4f} "
              f"({regen.sum()} rows)")
    if "road" in p:
        moving = p["road"] > 0.0
        print(f"  road hum level: max {p['road'].max():.3f}, mean while rolling {p['road'][moving].mean() if moving.any() else 0:.3f}")
    spectrogram(wav, f"{prefix}_ev_motor.png", "electric drive (0-5 kHz)", max(0.0, ta - 0.5), tb - ta + 1.0, 5000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav")
    ap.add_argument("--png-prefix", default="/tmp/trailforge_audio")
    args = ap.parse_args()

    x, rate = read_wav(args.wav)
    p = read_csv(args.wav + ".csv")
    t = p["t"]
    mono = x.mean(axis=1)
    print(f"capture: {len(x) / rate:.2f} s, {rate} Hz, {x.shape[1]} ch, {len(t)} parameter rows")
    peak = float(np.abs(x).max())
    print(f"sample peak {db(peak):.2f} dBFS, clipped samples (|x| >= 1) {int((np.abs(x) >= 1.0).sum())}, "
          f"limiter max reduction {p['limiter_db'].min():.2f} dB")
    loud = ebur128(args.wav)
    if loud:
        print(f"integrated loudness {loud.get('I', float('nan')):.1f} LUFS, loudness range "
              f"{loud.get('LRA', float('nan')):.1f} LU, true peak {loud.get('Peak', float('nan')):.1f} dBTP")

    electric = "motor_hz" in p and p["motor_hz"].max() > 0.0
    if electric:
        analyze_electric(mono, rate, p, args.png_prefix, args.wav)

    # ---- engine pitch tracking: first full-throttle run until the throttle is released for braking
    i0 = first(p["load"] > 0.5) if not electric else None
    i1 = first(p["load"] < 0.3, i0) if i0 is not None else None
    if i0 is not None and i1 is not None:
        ta, tb = t[i0], t[i1] - 0.2
        win, hop = 0.12, 0.05
        rows = []
        for c in np.arange(ta + win / 2, tb - win / 2, hop):
            seg = mono[int((c - win / 2) * rate):int((c + win / 2) * rate)]
            expected = float(np.interp(c, t, p["rpm_src"])) / 30.0
            near = harmonic_f0(seg, rate, expected * 0.7, expected * 1.4)
            free = harmonic_f0(seg, rate, 15.0, 250.0)
            rows.append((c, float(np.interp(c, t, p["rpm"])), expected, near, free))
        r = np.array(rows)
        err_near = r[:, 3] / r[:, 2] - 1.0
        err_free = r[:, 4] / r[:, 2] - 1.0
        corr = np.corrcoef(r[:, 2], r[:, 3])[0, 1]
        print(f"engine run {ta:.2f}-{tb:.2f} s: sim rpm {r[:, 1].min():.0f}-{r[:, 1].max():.0f}, expected firing "
              f"{r[:, 2].min():.1f}-{r[:, 2].max():.1f} Hz, {len(r)} windows of {win * 1000:.0f} ms")
        print(f"  fundamental searched near the firing frequency: median error {np.median(err_near) * 100:+.1f} %, "
              f"|error| < 5 % in {np.mean(np.abs(err_near) < 0.05) * 100:.0f} % of windows, "
              f"correlation {corr:.3f}")
        ratio = r[:, 4] / r[:, 2]
        crank = np.abs(ratio / 0.5 - 1.0) < 0.05
        print(f"  unconstrained search 15-250 Hz: within 5 % of the firing frequency in "
              f"{np.mean(np.abs(err_free) < 0.05) * 100:.0f} % of windows, of the crank frequency (rpm_src / 60) in "
              f"{np.mean(crank) * 100:.0f} %, correlation with the expected firing frequency "
              f"{np.corrcoef(r[:, 2], r[:, 4])[0, 1]:.3f}")
        for c, rpm, e, n_, f_ in r[::max(1, len(r) // 8)]:
            print(f"    t={c:6.2f} s  rpm {rpm:5.0f}  expected {e:6.1f} Hz  found {n_:6.1f} Hz  free {f_:6.1f} Hz")
        spectrogram(args.wav, f"{args.png_prefix}_engine_run.png", "engine run (0-1 kHz)", ta - 0.5,
                    tb - ta + 1.0, 1000)

        # ---- skid while braking on concrete vs. the run before
        i2 = first((p["skid"] < 0.05) & (t > t[i1] + 0.3), i1 + 1)
        tc = t[i2] if i2 is not None else t[i1] + 1.5
        sk = band_db(mono, rate, t[i1], tc, 1000, 6000)
        before = band_db(mono, rate, max(ta, t[i1] - 1.5), t[i1] - 0.1, 1000, 6000)
        print(f"braking {t[i1]:.2f}-{tc:.2f} s: mixer skid max {p['skid'][i1:i2].max():.2f}, 1-6 kHz band "
              f"{sk - before:+.1f} dB vs. the {min(1.4, t[i1] - 0.1 - ta):.1f} s before braking")
        spectrogram(args.wav, f"{args.png_prefix}_brake.png", "acceleration and skid braking on concrete",
                    ta - 0.5, tc - ta + 1.5)

    # ---- day / night ambience: last stretch of each
    def last_run(mask):
        idx = np.nonzero(mask)[0]
        if not len(idx):
            return None
        end = idx[-1]
        start = end
        while start > 0 and mask[start - 1]:
            start -= 1
        return t[start], t[end]

    ok = p["gameplay"] > 0.99
    day = last_run((p["day"] > 0.99) & ok & (p["load"] < 0.05))
    night = last_run((p["night"] > 0.99) & ok & (p["load"] < 0.05))
    if day and night:
        d0, d1 = max(day[0], day[1] - 3.0), day[1]
        n0, n1 = max(night[0], night[1] - 3.0), night[1]
        print(f"ambience day {d0:.1f}-{d1:.1f} s vs night {n0:.1f}-{n1:.1f} s (engine idling in both):")
        for lo, hi in ((100, 1000), (1000, 3000), (3000, 6000), (6000, 12000)):
            print(f"  {lo:5d}-{hi:5d} Hz: day {band_db(mono, rate, d0, d1, lo, hi):6.1f} dB, "
                  f"night {band_db(mono, rate, n0, n1, lo, hi):6.1f} dB")
        spectrogram(args.wav, f"{args.png_prefix}_day_night.png", "day ambience -> night ambience", d0,
                    n1 - d0)

    analyze_squeal(mono, rate, p)
    analyze_impacts(mono, rate, p, args.png_prefix, args.wav)
    spectrogram(args.wav, f"{args.png_prefix}_full.png", "full capture")
    return 0


if __name__ == "__main__":
    sys.exit(main())
