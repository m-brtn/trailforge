#!/usr/bin/env python3
"""Rebuilds the terrain material textures in assets/textures/terrain/ from Poly Haven (CC0).

For every layer of assets/textures/terrain/terrain.json it downloads the diffuse and displacement maps at the
albedo size and the DirectX normal and ARM (AO / roughness / metal) maps at the normal size, checks their md5
against the Poly Haven API and against the md5 recorded in assets/textures/terrain/sources.lock.json, and encodes
them with tools/texc (built here into build/texc) to <id>_albedo.ktx2 (BC7 sRGB, height in alpha) and
<id>_nra.ktx2 (BC7: normal x / y, roughness, AO). Writes assets/textures/CREDITS.md from the asset info.

  python3 tools/textures/fetch_textures.py            # download (cached in build/texture_cache), encode
  python3 tools/textures/fetch_textures.py --lock     # record the md5 of the current Poly Haven files
  python3 tools/textures/fetch_textures.py --only rock

Downloads are cached; the encoded output is deterministic for the same inputs. No API key is needed.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TERRAIN = os.path.join(ROOT, "assets", "textures", "terrain")
MANIFEST = os.path.join(TERRAIN, "terrain.json")
LOCK = os.path.join(TERRAIN, "sources.lock.json")
CREDITS = os.path.join(ROOT, "assets", "textures", "CREDITS.md")
CACHE = os.path.join(ROOT, "build", "texture_cache")
TEXC_BUILD = os.path.join(ROOT, "build", "texc")
API = "https://api.polyhaven.com"
UA = {"User-Agent": "TrailForge-texture-fetch/1.0"}


def get_json(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=60) as r:
        return json.load(r)


def md5_of(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def resolution_key(size):
    return {1024: "1k", 2048: "2k", 4096: "4k", 8192: "8k"}[size]


def fetch(asset, files, kind, size, lock, record):
    """Downloads one map (jpg) into the cache and returns its path."""
    res = resolution_key(size)
    entry = files[kind][res]["jpg"]
    url, md5 = entry["url"], entry["md5"]
    key = f"{asset}/{kind}_{res}"
    if record:
        lock[key] = md5
    elif lock.get(key) != md5:
        sys.exit(f"{key}: Poly Haven md5 {md5} differs from the lock file ({lock.get(key)}); "
                 "check the change and rerun with --lock")
    path = os.path.join(CACHE, asset, os.path.basename(url))
    if not (os.path.exists(path) and md5_of(path) == md5):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        print(f"  download {url} ({entry['size'] / 1e6:.1f} MB)")
        with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=300) as r, \
                open(path + ".part", "wb") as f:
            while True:
                chunk = r.read(1 << 20)
                if not chunk:
                    break
                f.write(chunk)
        os.replace(path + ".part", path)
        if md5_of(path) != md5:
            sys.exit(f"{path}: md5 mismatch after download")
    return path


def build_texc():
    exe = os.path.join(TEXC_BUILD, "trailforge_texc" + (".exe" if os.name == "nt" else ""))
    subprocess.run(["cmake", "-S", os.path.join(ROOT, "tools", "texc"), "-B", TEXC_BUILD,
                    "-DCMAKE_BUILD_TYPE=Release"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", TEXC_BUILD, "--config", "Release"], check=True)
    return exe


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lock", action="store_true", help="record the md5 of the current Poly Haven files")
    ap.add_argument("--only", help="rebuild only this layer id")
    args = ap.parse_args()

    with open(MANIFEST) as f:
        manifest = json.load(f)
    lock = {}
    if os.path.exists(LOCK):
        with open(LOCK) as f:
            lock = json.load(f)
    texc = build_texc()

    credits = []
    for layer in manifest["layers"]:
        asset = layer["asset"]
        info = get_json(f"{API}/info/{asset}")
        credits.append((layer["id"], asset, info.get("name", asset), info.get("authors", {}), info.get("dimensions")))
        if args.only and layer["id"] != args.only:
            continue
        print(f"{layer['id']}: {asset}")
        files = get_json(f"{API}/files/{asset}")
        a, n = manifest["albedoSize"], manifest["nraSize"]
        diff = fetch(asset, files, "Diffuse", a, lock, args.lock)
        disp = fetch(asset, files, "Displacement", a, lock, args.lock)
        nor = fetch(asset, files, "nor_dx", n, lock, args.lock)
        arm = fetch(asset, files, "arm", n, lock, args.lock)
        out = os.path.join(TERRAIN, f"{layer['id']}_albedo.ktx2")
        subprocess.run([texc, "albedo", "--color", diff, "--height", disp, "--size", str(a), "--out", out], check=True)
        out = os.path.join(TERRAIN, f"{layer['id']}_nra.ktx2")
        subprocess.run([texc, "nra", "--normal", nor, "--arm", arm, "--size", str(n), "--out", out], check=True)

    if args.lock:
        with open(LOCK, "w") as f:
            json.dump(dict(sorted(lock.items())), f, indent=2)
            f.write("\n")

    lines = [
        "# Texture credits",
        "",
        "All terrain material textures in `terrain/` are from [Poly Haven](https://polyhaven.com) and are released",
        "under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/) (public domain; attribution is not",
        "required). They were downscaled and re-encoded to BC7 KTX2 by `tools/textures/fetch_textures.py`",
        "(`tools/texc`); the source md5 values are in `terrain/sources.lock.json`.",
        "",
        "| Layer | Poly Haven asset | Authors | Real-world size |",
        "|---|---|---|---|",
    ]
    for lid, asset, name, authors, dims in credits:
        who = ", ".join(f"{k} ({v})" if v != "All" else k for k, v in authors.items())
        size = f"{dims[0] / 1000:.1f} m x {dims[1] / 1000:.1f} m" if dims else "-"
        lines.append(f"| {lid} | [{name}](https://polyhaven.com/a/{asset}) | {who} | {size} |")
    with open(CREDITS, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {CREDITS}")


if __name__ == "__main__":
    main()
