#!/usr/bin/env python3
"""Rebuild assets/audio/ from a pinned list of Freesound sounds.

Downloads the HQ OGG previews of the pinned sounds (token auth, no OAuth), cuts the
selected windows, converts to 48 kHz 16-bit PCM WAV, normalizes loudness, builds
seamless loops and writes assets/audio/manifest.json and assets/audio/CREDITS.md.

Requirements: Python 3.9+, `sox` and `ffmpeg` on PATH (brew install sox ffmpeg).
The API key is read from the FREESOUND_API_KEY environment variable, or from the
env file given with --env-file (default ~/.config/trailforge/freesound.env).
The key is sent in the Authorization header and never printed or written anywhere.

Usage:
    python3 tools/audio/fetch_freesound.py                # fetch (cached) + process all
    python3 tools/audio/fetch_freesound.py --only engine  # one category
    python3 tools/audio/fetch_freesound.py --verify       # only re-check existing files
"""

import argparse
import array
import json
import math
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
import wave
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "assets" / "audio"
CACHE_DIR = REPO / "build" / "audio_cache"
API = "https://freesound.org/apiv2"
SR = 48000

LOOP_LUFS = -18.0       # integrated loudness target for loops
LOOP_MAX_TP = -1.0      # loops never exceed this true peak (gain is reduced instead)
ONESHOT_TP = -1.0       # one-shots are true-peak normalized to this
LOOP_MAX_LIMIT_DB = 6.0  # most gain reduction a limiter may apply to reach the loop target

LICENSES = {
    "http://creativecommons.org/publicdomain/zero/1.0/": ("CC0 1.0", "https://creativecommons.org/publicdomain/zero/1.0/"),
    "https://creativecommons.org/publicdomain/zero/1.0/": ("CC0 1.0", "https://creativecommons.org/publicdomain/zero/1.0/"),
    "https://creativecommons.org/licenses/by/4.0/": ("CC BY 4.0", "https://creativecommons.org/licenses/by/4.0/"),
    "http://creativecommons.org/licenses/by/4.0/": ("CC BY 4.0", "https://creativecommons.org/licenses/by/4.0/"),
}

# Chippy569 "Kickstarter Dyno" pack: Mercedes-Benz 190E 2.3-16V (inline-4) on a chassis dyno.
# rpm_label comes from the source file name/description; rpm_measured from the firing frequency
# (dominant spectral peak / harmonic product spectrum, inline-4 = 2 firings per revolution).
ENGINE_NOTE = "Mercedes-Benz 190E 2.3-16V (inline-4, petrol) on a chassis dyno"

# Asset table. Times are in seconds of the source preview.
#   kind "loop":        window [start, start + length + xfade] -> loop of `length` with crossfaded seam
#   kind "concat_loop": several windows overlap-added, then looped like "loop"
#   kind "oneshot":     window [start, start + length], silence-trimmed, faded, true-peak normalized
# Options: mono (downmix), highpass (Hz), align (search loop length for best seam correlation, s),
#          mix (extra simultaneous source ids mixed in at equal gain).
ASSETS = [
    # ---------------- engine: exhaust mic (Audix D6, 1 ft behind the tailpipe) ----------------
    dict(id="engine_exhaust_idle_loop", cat="engine", src=162515, start=0.05, length=8.0, xfade=0.15, align=0.05,
         mono=True, rpm_label=500, rpm_measured=1010, load="off"),
    dict(id="engine_exhaust_offload_mid_loop", cat="engine", src=162518, start=0.05, length=3.7, xfade=0.15, align=0.05,
         mono=True, rpm_label=2500, rpm_measured=2430, load="off"),
    dict(id="engine_exhaust_offload_high_loop", cat="engine", src=162521, start=0.05, length=1.6, xfade=0.12, align=0.03,
         mono=True, rpm_label=6500, rpm_measured=6290, load="off"),
    dict(id="engine_exhaust_onload_low_loop", cat="engine", src=162506, start=0.05, length=1.55, xfade=0.12, align=0.05,
         mono=True, rpm_label=500, rpm_measured=1290, load="full"),
    dict(id="engine_exhaust_onload_mid_loop", cat="engine", src=162509, start=0.05, length=4.4, xfade=0.15, align=0.05,
         mono=True, rpm_label=2500, rpm_measured=2560, load="full"),
    dict(id="engine_exhaust_onload_high_loop", cat="engine", src=162512, start=0.05, length=1.35, xfade=0.12, align=0.03,
         mono=True, rpm_label=6500, rpm_measured=6210, load="full"),
    # ---------------- engine: engine-bay mic (DPA 4062 on the radiator support) ----------------
    dict(id="engine_bay_idle_loop", cat="engine", src=162516, start=0.05, length=8.0, xfade=0.15, align=0.05,
         mono=True, rpm_label=500, rpm_measured=1020, load="off"),
    dict(id="engine_bay_offload_mid_loop", cat="engine", src=162519, start=0.05, length=3.7, xfade=0.15, align=0.05,
         mono=True, rpm_label=2500, rpm_measured=2430, load="off"),
    dict(id="engine_bay_offload_high_loop", cat="engine", src=162522, start=0.05, length=1.6, xfade=0.12, align=0.03,
         mono=True, rpm_label=6500, rpm_measured=6290, load="off"),
    dict(id="engine_bay_onload_low_loop", cat="engine", src=162507, start=0.05, length=1.55, xfade=0.12, align=0.05,
         mono=True, rpm_label=500, rpm_measured=1290, load="full"),
    dict(id="engine_bay_onload_mid_loop", cat="engine", src=162510, start=0.05, length=4.4, xfade=0.15, align=0.05,
         mono=True, rpm_label=2500, rpm_measured=2560, load="full"),
    dict(id="engine_bay_onload_high_loop", cat="engine", src=162513, start=0.05, length=1.35, xfade=0.12, align=0.03,
         mono=True, rpm_label=6500, rpm_measured=6210, load="full"),
    # ---------------- engine: start / stop ----------------
    dict(id="engine_start", cat="engine", src=162528, mix=[162523], start=1.0, length=6.6, kind="oneshot", mono=True,
         note="starter crank, catch and settle to idle; exhaust (D6) and engine-bay (4062) mics of the same take mixed"),
    dict(id="engine_stop", cat="engine", src=405322, start=9.0, length=3.5, kind="oneshot", mono=True,
         note="2004 Saturn Vue (petrol, manual): last second of idle and shut-off; different vehicle than the loops"),
    # ---------------- transmission ----------------
    dict(id="gear_shift_01", cat="transmission", src=697836, start=2.6, length=0.8, kind="oneshot", mono=True),
    dict(id="gear_shift_02", cat="transmission", src=697836, start=4.4, length=0.9, kind="oneshot", mono=True),
    dict(id="gear_shift_03", cat="transmission", src=697836, start=8.1, length=0.8, kind="oneshot", mono=True),
    # ---------------- tires ----------------
    dict(id="tire_skid_asphalt_loop", cat="tires", src=71739, start=9.8, length=3.6, xfade=0.4, mono=True, highpass=40),
    dict(id="tire_skid_short", cat="tires", src=104026, start=0.0, length=1.7, kind="oneshot", mono=True),
    dict(id="tire_gravel_roll_loop", cat="tires", src=251661, start=0.5, length=11.0, xfade=1.0, mono=True, highpass=30),
    dict(id="tire_dirt_roll_loop", cat="tires", src=685231, start=3.0, length=9.0, xfade=1.0, mono=True, highpass=30),
    dict(id="tire_mud_slosh_loop", cat="tires", src=140477, start=12.0, length=9.0, xfade=1.0, mono=True, highpass=80),
    dict(id="mud_squelch_01", cat="tires", src=207142, start=2.65, length=0.65, kind="oneshot", mono=True, highpass=40),
    dict(id="mud_squelch_02", cat="tires", src=207142, start=3.38, length=0.65, kind="oneshot", mono=True, highpass=40),
    dict(id="mud_squelch_03", cat="tires", src=207142, start=5.45, length=0.65, kind="oneshot", mono=True, highpass=40),
    dict(id="mud_squelch_04", cat="tires", src=207142, start=7.68, length=0.65, kind="oneshot", mono=True, highpass=40),
    dict(id="water_splash_01", cat="tires", src=563021, start=0.3, length=4.5, kind="oneshot", mono=True),
    dict(id="water_splash_02", cat="tires", src=207139, start=0.8, length=1.1, kind="oneshot", mono=True, highpass=40),
    dict(id="water_splash_03", cat="tires", src=207139, start=3.1, length=1.3, kind="oneshot", mono=True, highpass=40),
    dict(id="grass_rustle_loop", cat="tires", src=559082, kind="concat_loop", mono=True, xfade=0.45, highpass=60,
         segments=[[0.0, 1.1], [1.8, 1.1], [3.75, 1.1], [5.35, 1.1], [6.85, 1.1], [8.45, 1.1], [9.95, 1.1], [11.35, 1.1], [12.85, 1.0]]),
    dict(id="grass_swish_01", cat="tires", src=559082, start=5.35, length=1.1, kind="oneshot", mono=True, highpass=60),
    dict(id="grass_swish_02", cat="tires", src=559082, start=8.45, length=1.1, kind="oneshot", mono=True, highpass=60),
    dict(id="grass_swish_03", cat="tires", src=559082, start=9.95, length=1.1, kind="oneshot", mono=True, highpass=60),
    # ---------------- suspension / body ----------------
    dict(id="suspension_thump_01", cat="suspension", src=640204, start=0.0, length=0.65, kind="oneshot", mono=True),
    dict(id="suspension_bottom_out_01", cat="suspension", src=452550, start=0.25, length=0.9, kind="oneshot", mono=True,
         note="car trunk closed hard (proxy for a bump-stop / bottom-out clunk)"),
    dict(id="suspension_bottom_out_02", cat="suspension", src=450766, start=36.1, length=1.7, kind="oneshot", mono=True,
         note="heavy station-wagon door slam with rattle (proxy for a bottom-out clunk)"),
    dict(id="suspension_bottom_out_03", cat="suspension", src=450766, start=8.1, length=1.8, kind="oneshot", mono=True,
         note="heavy station-wagon door slam with rattle (proxy for a bottom-out clunk)"),
    dict(id="body_impact_metal_01", cat="suspension", src=798843, start=6.55, length=1.25, kind="oneshot", mono=True),
    dict(id="body_impact_metal_02", cat="suspension", src=798843, start=7.85, length=1.45, kind="oneshot", mono=True),
    dict(id="body_impact_metal_03", cat="suspension", src=798843, start=9.35, length=1.7, kind="oneshot", mono=True),
    dict(id="body_impact_clank", cat="suspension", src=151624, start=0.0, length=0.5, kind="oneshot", mono=True),
    dict(id="chassis_creak_01", cat="suspension", src=86234, start=0.6, length=1.3, kind="oneshot", mono=True, highpass=40),
    dict(id="chassis_creak_02", cat="suspension", src=86234, start=11.8, length=1.0, kind="oneshot", mono=True, highpass=40),
    dict(id="chassis_creak_03", cat="suspension", src=86234, start=17.0, length=1.6, kind="oneshot", mono=True, highpass=40),
    # ---------------- ambience ----------------
    dict(id="wind_speed_loop", cat="ambience", src=438885, start=8.0, length=20.0, xfade=2.0, mono=True, highpass=40),
    dict(id="ambience_day_meadow_loop", cat="ambience", src=409143, start=10.0, length=45.0, xfade=3.0, highpass=50),
    dict(id="ambience_night_crickets_loop", cat="ambience", src=333221, start=100.0, length=45.0, xfade=3.0, highpass=50),
    # ---------------- ui ----------------
    dict(id="ui_click", cat="ui", src=528561, start=0.0, length=0.23, kind="oneshot", mono=True),
]

META_FIELDS = "id,name,username,license,url,previews,duration,samplerate,channels,description"


# ------------------------------------------------------------------------------------------------
# Freesound access
# ------------------------------------------------------------------------------------------------

def load_key(env_file):
    key = os.environ.get("FREESOUND_API_KEY")
    if not key and env_file and Path(env_file).expanduser().exists():
        for line in Path(env_file).expanduser().read_text().splitlines():
            m = re.match(r"\s*(?:export\s+)?FREESOUND_API_KEY\s*=\s*['\"]?([^'\"\s]+)", line)
            if m:
                key = m.group(1)
    if not key:
        sys.exit("FREESOUND_API_KEY is not set (export it or pass --env-file)")
    return key


_last_api_call = [0.0]


def api_get(path, key):
    wait = 1.1 - (time.time() - _last_api_call[0])  # stay well under 60 requests/minute
    if wait > 0:
        time.sleep(wait)
    req = urllib.request.Request(API + path, headers={"Authorization": "Token " + key})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        sys.exit(f"Freesound API error {e.code} for {path.split('?')[0]}")
    finally:
        _last_api_call[0] = time.time()


def fetch_source(sid, key, refresh):
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    meta_path = CACHE_DIR / f"{sid}.json"
    ogg_path = CACHE_DIR / f"{sid}.ogg"
    if refresh or not meta_path.exists():
        meta = api_get(f"/sounds/{sid}/?fields={META_FIELDS}", key)
        meta_path.write_text(json.dumps(meta, indent=1))
    meta = json.loads(meta_path.read_text())
    if meta["license"] not in LICENSES:
        sys.exit(f"sound {sid} has license {meta['license']}, which is not allowed (CC0 / CC BY 4.0 only)")
    if refresh or not ogg_path.exists():
        print(f"  downloading preview {sid} ({meta['name']})", flush=True)
        tmp = ogg_path.with_suffix(".part")
        urllib.request.urlretrieve(meta["previews"]["preview-hq-ogg"], tmp)
        tmp.rename(ogg_path)
    return meta, ogg_path


# ------------------------------------------------------------------------------------------------
# Audio helpers (float32 interleaved arrays)
# ------------------------------------------------------------------------------------------------

def decode(ogg, channels, highpass):
    """Decode the whole preview to float32 at 48 kHz (resampled before cutting to avoid edge effects)."""
    effects = []
    if channels == 1:
        effects += ["remix", "-"]
    if highpass:
        effects += ["highpass", "-2", str(highpass)]
    effects += ["rate", "-v", str(SR)]
    cmd = ["sox", "-V1", str(ogg), "-t", "raw", "-e", "floating-point", "-b", "32", "-c", str(channels), "-"] + effects
    raw = subprocess.run(cmd, check=True, capture_output=True).stdout
    a = array.array("f")
    a.frombytes(raw)
    return a


def source_channels(ogg):
    out = subprocess.run(["soxi", "-c", str(ogg)], check=True, capture_output=True, text=True).stdout
    return int(out.strip())


def cut(a, ch, start, length):
    s = int(round(start * SR)) * ch
    e = s + int(round(length * SR)) * ch
    if e > len(a):
        raise ValueError(f"window {start}+{length}s exceeds source length {len(a) / ch / SR:.2f}s")
    return a[s:e]


def correlation(a, b):
    num = sum(x * y for x, y in zip(a, b))
    den = math.sqrt(sum(x * x for x in a) * sum(y * y for y in b)) + 1e-12
    return num / den


def best_loop_length(y, ch, nominal, xfade, search):
    """Pick a loop length in [nominal - search, nominal] whose seam region best matches the start."""
    if search <= 0:
        return nominal
    head = y[0:xfade * ch:ch]
    best, best_c = nominal, -2.0
    for L in range(nominal - search, nominal + 1, 2):
        c = correlation(head, y[L * ch:(L + xfade) * ch:ch])
        if c > best_c:
            best, best_c = L, c
    return best


def make_loop(y, ch, length_s, xfade_s, align_s):
    """y holds at least length + xfade seconds. Returns a seamless loop, its sample length and the
    correlation of the crossfaded segments. The crossfade keeps the expected power constant for that
    correlation (linear for identical material, equal-power for uncorrelated material)."""
    X = int(round(xfade_s * SR))
    L = int(round(length_s * SR))
    L = best_loop_length(y, ch, L, X, int(round(align_s * SR)))
    rho = min(1.0, max(0.0, correlation(y[:X * ch], y[L * ch:(L + X) * ch])))
    out = array.array("f", y[:L * ch])
    for i in range(X):
        t = (i + 0.5) / X
        norm = math.sqrt(t * t + (1 - t) * (1 - t) + 2 * rho * t * (1 - t))
        win, wout = t / norm, (1 - t) / norm
        for c in range(ch):
            out[i * ch + c] = y[i * ch + c] * win + y[(L + i) * ch + c] * wout
    return out, L, rho


def overlap_concat(parts, ch, xfade_s):
    """Join takes with equal-power crossfades (the takes are unrelated, so uncorrelated)."""
    X = int(round(xfade_s * SR))
    win = [math.sin((i + 0.5) / X * math.pi / 2) for i in range(X)]
    wout = [math.cos((i + 0.5) / X * math.pi / 2) for i in range(X)]
    out = array.array("f", parts[0])
    for p in parts[1:]:
        base = len(out) - X * ch
        for i in range(X):
            for c in range(ch):
                k = base + i * ch + c
                out[k] = out[k] * wout[i] + p[i * ch + c] * win[i]
        out.extend(p[X * ch:])
    return out


def trim_oneshot(y, ch, threshold_db=-50.0):
    """Trim leading/trailing material quieter than threshold_db below the peak, then fade the edges."""
    n = len(y) // ch
    peak = max(abs(v) for v in y) or 1.0
    thr = peak * 10 ** (threshold_db / 20)
    frames_amp = [max(abs(y[i * ch + c]) for c in range(ch)) for i in range(n)]
    first = next((i for i in range(n) if frames_amp[i] > thr), 0)
    last = next((i for i in range(n - 1, -1, -1) if frames_amp[i] > thr), n - 1)
    pre = int(0.005 * SR)
    first = max(0, first - pre)
    out = array.array("f", y[first * ch:(last + 1) * ch])
    m = len(out) // ch
    fin = min(int(0.002 * SR), m // 4) if first > 0 else 0
    fout = min(int(0.03 * SR), m // 4)
    for i in range(fin):
        g = i / fin
        for c in range(ch):
            out[i * ch + c] *= g
    for i in range(fout):
        g = (i + 1) / fout
        for c in range(ch):
            out[(m - 1 - i) * ch + c] *= g
    return out, first / SR, (last + 1) / SR


def write_float_wav(a, ch, path):
    cmd = ["sox", "-V1", "-t", "raw", "-r", str(SR), "-e", "floating-point", "-b", "32", "-c", str(ch), "-",
           "-e", "floating-point", "-b", "32", str(path)]
    subprocess.run(cmd, input=a.tobytes(), check=True)


def write_pcm16_wav(a, ch, path):
    cmd = ["sox", "-V1", "-R", "-t", "raw", "-r", str(SR), "-e", "floating-point", "-b", "32", "-c", str(ch), "-",
           "-e", "signed-integer", "-b", "16", str(path), "dither"]
    subprocess.run(cmd, input=a.tobytes(), check=True)


def ebur128(path):
    """Integrated loudness (LUFS, None if too short to gate) and true peak (dBTP)."""
    out = subprocess.run(["ffmpeg", "-hide_banner", "-nostats", "-i", str(path), "-af", "ebur128=peak=true",
                          "-f", "null", "-"], capture_output=True, text=True).stderr
    summary = out[out.rfind("Summary:"):]
    i = re.search(r"I:\s+(-?[\d.]+|-inf) LUFS", summary)
    p = re.search(r"Peak:\s+(-?[\d.]+|-inf) dBFS", summary)
    lufs = float(i.group(1)) if i and i.group(1) != "-inf" else None
    if lufs is not None and lufs <= -69.9:
        lufs = None
    tp = float(p.group(1)) if p and p.group(1) != "-inf" else -200.0
    return lufs, tp


def scale(a, gain_db):
    g = 10 ** (gain_db / 20)
    return array.array("f", (v * g for v in a))


def limit_loop(a, ch, pre_gain_db, ceiling_db, tmp_dir):
    """Look-ahead peak limiter applied to three back-to-back copies; the middle copy is returned so the
    limiter state at the seam is the same as anywhere else in the loop."""
    tiled = array.array("f", a)
    tiled.extend(a)
    tiled.extend(a)
    src = tmp_dir / "limit_in.wav"
    dst = tmp_dir / "limit_out.wav"
    write_float_wav(tiled, ch, src)
    af = (f"volume={pre_gain_db:.3f}dB,"
          f"alimiter=limit={10 ** (ceiling_db / 20):.5f}:attack=2:release=60:level=false:latency=true")
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(src), "-af", af, "-c:a", "pcm_f32le", str(dst)], check=True)
    raw = subprocess.run(["sox", "-V1", str(dst), "-t", "raw", "-e", "floating-point", "-b", "32", "-"],
                         check=True, capture_output=True).stdout
    out = array.array("f")
    out.frombytes(raw)
    n = len(a)
    src.unlink()
    dst.unlink()
    return out[n:2 * n]


# ------------------------------------------------------------------------------------------------
# Verification
# ------------------------------------------------------------------------------------------------

def verify_wav(path, loop):
    with wave.open(str(path), "rb") as w:
        ch, sw, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        data = array.array("h", w.readframes(n))
    if sys.byteorder == "big":
        data.byteswap()
    peak = max(abs(v) for v in data) if data else 0
    clipped = sum(1 for v in data if v >= 32767 or v <= -32768)
    rms = math.sqrt(sum(v * v for v in data) / max(1, len(data))) if data else 0.0
    res = dict(channels=ch, sample_rate=sr, bits=sw * 8, frames=n, duration_s=round(n / sr, 4),
               peak_dbfs=round(20 * math.log10(max(peak, 1) / 32768), 2),
               rms_dbfs=round(20 * math.log10(max(rms, 1e-9) / 32768), 2), clipped_samples=clipped)
    if loop:
        # Seam jump (last frame -> first frame) relative to the 99th percentile of in-file steps, and
        # RMS of the last vs the first 50 ms. A clean loop has a ratio around or below 1 and ~0 dB.
        worst = 0.0
        for c in range(ch):
            x = data[c::ch]
            steps = sorted(abs(x[i + 1] - x[i]) for i in range(0, len(x) - 1, 3))
            p99 = steps[int(len(steps) * 0.99)] or 1
            worst = max(worst, abs(x[0] - x[-1]) / p99)
        k = int(0.05 * sr)
        head = data[:k * ch]
        tail = data[-k * ch:]
        rh = math.sqrt(sum(v * v for v in head) / len(head)) or 1e-9
        rt = math.sqrt(sum(v * v for v in tail) / len(tail)) or 1e-9
        res["seam_jump_vs_p99"] = round(worst, 2)
        res["seam_rms_delta_db"] = round(20 * math.log10(rh / rt), 2)
    return res


# ------------------------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------------------------

def process(asset, metas, oggs, tmp_dir):
    kind = asset.get("kind", "loop")
    src = asset["src"]
    ogg = oggs[src]
    ch = 1 if asset.get("mono") else source_channels(ogg)
    hp = asset.get("highpass")
    full = decode(ogg, ch, hp)
    steps = [f"decoded Freesound HQ preview (OGG), resampled to {SR} Hz (sox rate -v)"]
    if asset.get("mono"):
        steps.append("downmixed to mono" if source_channels(ogg) > 1 else "mono source")
    if hp:
        steps.append(f"high-pass {hp} Hz (2-pole)")

    if kind == "concat_loop":
        parts = [cut(full, ch, s, d) for s, d in asset["segments"]]
        y = overlap_concat(parts, ch, asset["xfade"])
        total = len(y) / ch / SR
        length = total - asset["xfade"]
        steps.append(f"{len(parts)} takes overlap-added with {asset['xfade']} s equal-power crossfades")
        out, L, rho = make_loop(y, ch, length, asset["xfade"], 0.0)
        steps.append(f"loop seam crossfaded ({asset['xfade']} s, constant-power for segment correlation {rho:.2f})")
        window = asset["segments"]
    elif kind == "loop":
        y = cut(full, ch, asset["start"], asset["length"] + asset["xfade"])
        out, L, rho = make_loop(y, ch, asset["length"], asset["xfade"], asset.get("align", 0.0))
        if asset.get("align"):
            steps.append(f"loop length chosen within {asset['align']} s for best seam correlation")
        steps.append(f"loop seam crossfaded ({asset['xfade']} s, constant-power for segment correlation {rho:.2f})")
        window = [asset["start"], round(L / SR + asset["xfade"], 4)]
    else:
        y = cut(full, ch, asset["start"], asset["length"])
        for extra in asset.get("mix", []):
            other = cut(decode(oggs[extra], ch, hp), ch, asset["start"], asset["length"])
            y = array.array("f", (a + b for a, b in zip(y, other)))
            steps.append(f"mixed with simultaneous take {extra}")
        out, t0, t1 = trim_oneshot(y, ch)
        steps.append("silence trimmed (-50 dB below peak), 2 ms fade-in when trimmed, 30 ms fade-out")
        window = [round(asset["start"] + t0, 4), round(t1 - t0, 4)]

    tmp = tmp_dir / f"{asset['id']}_float.wav"
    write_float_wav(out, ch, tmp)
    lufs, tp = ebur128(tmp)
    if kind == "oneshot":
        gain = ONESHOT_TP - tp
        steps.append(f"true-peak normalized to {ONESHOT_TP} dBTP")
        out = scale(out, gain)
    else:
        gain = LOOP_LUFS - lufs
        shortfall = tp + gain - LOOP_MAX_TP
        if shortfall > 1.0:
            # Transient-heavy textures: a look-ahead limiter takes up to LOOP_MAX_LIMIT_DB of the excess,
            # then a static gain keeps the true peak at the ceiling.
            reduction = min(shortfall, LOOP_MAX_LIMIT_DB)
            pre_gain = LOOP_MAX_TP - tp + reduction
            out = limit_loop(out, ch, pre_gain, LOOP_MAX_TP - 0.5, tmp_dir)
            write_float_wav(out, ch, tmp)
            lufs2, tp2 = ebur128(tmp)
            post_gain = min(LOOP_LUFS - lufs2, LOOP_MAX_TP - tp2)
            out = scale(out, post_gain)
            gain = pre_gain + post_gain
            steps.append(f"peak-limited (look-ahead, up to {reduction:.1f} dB reduction, processed circularly) "
                         f"to approach {LOOP_LUFS} LUFS within {LOOP_MAX_TP} dBTP")
        else:
            if shortfall > 0:
                gain = LOOP_MAX_TP - tp
                steps.append(f"loudness normalized toward {LOOP_LUFS} LUFS, capped by {LOOP_MAX_TP} dBTP true peak (static gain)")
            else:
                steps.append(f"loudness normalized to {LOOP_LUFS} LUFS (static gain)")
            out = scale(out, gain)
    path = OUT_DIR / asset["cat"] / f"{asset['id']}.wav"
    path.parent.mkdir(parents=True, exist_ok=True)
    write_pcm16_wav(out, ch, path)
    steps.append("written as 48 kHz 16-bit PCM WAV with TPDF dither")
    tmp.unlink()
    return path, steps, window, lufs, gain


def manifest_entry(asset, meta, path, steps, window, src_lufs, gain):
    lic_name, lic_url = LICENSES[meta["license"]]
    kind = asset.get("kind", "loop")
    final_lufs, final_tp = ebur128(path)
    check = verify_wav(path, kind != "oneshot")
    e = dict(
        id=asset["id"],
        category=asset["cat"],
        file=str(path.relative_to(OUT_DIR)),
        loop=kind != "oneshot",
        duration_s=check["duration_s"],
        channels=check["channels"],
        sample_rate=check["sample_rate"],
        bits=check["bits"],
        freesound_id=meta["id"],
        original_name=meta["name"],
        author=meta["username"],
        license=lic_name,
        license_url=lic_url,
        source_url=meta["url"],
        source_window_s=window,
        processing=steps,
        loudness_lufs=final_lufs,
        true_peak_dbtp=final_tp,
        source_loudness_lufs=None if src_lufs is None else round(src_lufs, 2),
        applied_gain_db=round(gain, 2),
        verification=check,
    )
    if asset.get("mix"):
        e["mixed_freesound_ids"] = asset["mix"]
    if "rpm_measured" in asset:
        e["engine"] = dict(vehicle=ENGINE_NOTE, rpm_label=asset["rpm_label"], rpm_measured=asset["rpm_measured"],
                           load=asset["load"], mic="exhaust (Audix D6)" if "exhaust" in asset["id"] else "engine bay (DPA 4062)")
    if asset.get("note"):
        e["note"] = asset["note"]
    return e


def add_engine_relative_levels(entries):
    """Level of each engine loop relative to the full-load high-rpm loop of the same mic, before normalization."""
    for mic in ("exhaust", "bay"):
        ref = next(e for e in entries if e["id"] == f"engine_{mic}_onload_high_loop")
        for e in entries:
            if e["id"].startswith(f"engine_{mic}_") and e["loop"]:
                e["engine"]["source_level_vs_onload_high_db"] = round(e["source_loudness_lufs"] - ref["source_loudness_lufs"], 2)


def write_credits(entries, metas):
    by_src = {}
    for e in entries:
        ids = [e["freesound_id"]] + e.get("mixed_freesound_ids", [])
        for sid in ids:
            by_src.setdefault(sid, []).append(e["id"])
    lines = [
        "# Audio credits",
        "",
        "All sounds in `assets/audio/` are derived from recordings on [Freesound](https://freesound.org).",
        "They were built from the Freesound HQ previews (OGG) by `tools/audio/fetch_freesound.py`:",
        "cut, resampled to 48 kHz, high-pass filtered where noted, loudness-normalized, and looped",
        "with crossfaded seams. Per-file details are in `manifest.json`.",
        "",
        "Only CC0 and CC BY 4.0 sources are used. CC BY 4.0 sounds require attribution (below) when",
        "the game or its assets are distributed.",
        "",
        "## Attribution required (CC BY 4.0)",
        "",
    ]
    by = [sid for sid in by_src if LICENSES[metas[sid]["license"]][0] == "CC BY 4.0"]
    cc0 = [sid for sid in by_src if LICENSES[metas[sid]["license"]][0] == "CC0 1.0"]
    for sid in sorted(by):
        m = metas[sid]
        lines.append(f"- \"{m['name']}\" by {m['username']} ({m['url']}), licensed under "
                     f"[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Modified: cut, filtered, "
                     f"normalized, looped. Used in: {', '.join(f'`{x}`' for x in by_src[sid])}.")
    if not by:
        lines.append("- None.")
    lines += ["", "## Public domain (CC0 1.0)", "",
              "No attribution is required; the authors are credited here with thanks.", ""]
    for sid in sorted(cc0):
        m = metas[sid]
        lines.append(f"- \"{m['name']}\" by {m['username']} ({m['url']}). Used in: "
                     f"{', '.join(f'`{x}`' for x in by_src[sid])}.")
    lines.append("")
    (OUT_DIR / "CREDITS.md").write_text("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--env-file", default="~/.config/trailforge/freesound.env")
    ap.add_argument("--only", help="process only this category or asset id (manifest is merged)")
    ap.add_argument("--refresh", action="store_true", help="re-download metadata and previews")
    ap.add_argument("--verify", action="store_true", help="only verify existing files listed in manifest.json")
    args = ap.parse_args()

    manifest_path = OUT_DIR / "manifest.json"
    if args.verify:
        manifest = json.loads(manifest_path.read_text())
        bad = 0
        for e in manifest["sounds"]:
            p = OUT_DIR / e["file"]
            c = verify_wav(p, e["loop"])
            lufs, tp = ebur128(p)
            flag = ""
            if c["clipped_samples"] or tp > -0.9 or (e["loop"] and (c["seam_jump_vs_p99"] > 1.5 or abs(c["seam_rms_delta_db"]) > 4)):
                flag = "  <-- CHECK"
                bad += 1
            seam = f" seam={c['seam_jump_vs_p99']} dRMS={c['seam_rms_delta_db']}dB" if e["loop"] else ""
            print(f"{e['id']:34s} {c['duration_s']:7.3f}s ch={c['channels']} peak={c['peak_dbfs']:6.2f} "
                  f"rms={c['rms_dbfs']:6.2f} I={lufs} TP={tp}{seam}{flag}")
        print(f"{len(manifest['sounds'])} files, {bad} flagged")
        sys.exit(1 if bad else 0)

    for tool in ("sox", "soxi", "ffmpeg"):
        if subprocess.run(["which", tool], capture_output=True).returncode != 0:
            sys.exit(f"{tool} not found (brew install sox ffmpeg)")
    key = load_key(args.env_file)

    assets = [a for a in ASSETS if not args.only or a["cat"] == args.only or a["id"] == args.only]
    if not assets:
        sys.exit(f"nothing matches --only {args.only}")
    src_ids = sorted({a["src"] for a in assets} | {m for a in assets for m in a.get("mix", [])})
    metas, oggs = {}, {}
    print(f"fetching {len(src_ids)} Freesound sources (cache: {CACHE_DIR.relative_to(REPO)})")
    for sid in src_ids:
        metas[sid], oggs[sid] = fetch_source(sid, key, args.refresh)

    tmp_dir = CACHE_DIR / "tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    old = {e["id"]: e for e in json.loads(manifest_path.read_text())["sounds"]} if manifest_path.exists() and args.only else {}
    entries = []
    for a in assets:
        print(f"  {a['id']}", flush=True)
        path, steps, window, src_lufs, gain = process(a, metas, oggs, tmp_dir)
        entries.append(manifest_entry(a, metas[a["src"]], path, steps, window, src_lufs, gain))
    if old:
        new_ids = {e["id"] for e in entries}
        entries = [old[a["id"]] if a["id"] not in new_ids else next(e for e in entries if e["id"] == a["id"])
                   for a in ASSETS if a["id"] in old or a["id"] in new_ids]
        for sid in {e["freesound_id"] for e in entries} | {m for e in entries for m in e.get("mixed_freesound_ids", [])}:
            if sid not in metas:
                metas[sid], oggs[sid] = fetch_source(sid, key, False)
    if all(any(e["id"] == f"engine_{m}_onload_high_loop" for e in entries) for m in ("exhaust", "bay")):
        add_engine_relative_levels(entries)

    manifest = dict(
        description="TrailForge audio assets built from Freesound HQ previews by tools/audio/fetch_freesound.py",
        format=dict(container="WAV", encoding="PCM signed 16-bit", sample_rate=SR),
        source_window_note="source_window_s is [start, duration] in seconds of the Freesound preview "
                           "(a list of such pairs for concatenated loops; loops include the crossfade tail)",
        loudness=dict(loops_lufs=LOOP_LUFS, loops_max_true_peak_dbtp=LOOP_MAX_TP, oneshots_true_peak_dbtp=ONESHOT_TP),
        sounds=entries,
    )
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    write_credits(entries, metas)
    total = sum((OUT_DIR / e["file"]).stat().st_size for e in entries)
    print(f"wrote {len(entries)} files, {total / 1e6:.1f} MB, manifest.json and CREDITS.md")


if __name__ == "__main__":
    main()
