#!/usr/bin/env python3
"""Fetches the source data of the Achna Speedway level (assets/levels/achna_speedway/source/).

  osm.osm     OpenStreetMap API 0.6 `map` response for the venue bounding box (raw XML; ODbL,
              (c) OpenStreetMap contributors). The circuit is the raceway way 734690065 and its neighbours.
  dem.json    Copernicus GLO-30 DEM (DSM) samples around the venue, cropped from the 1 degree COG tile, heights in m
              above the EGM2008 geoid (Copernicus programme licence, attribution in the level CREDITS.md).

sources.lock.json records the sha256 of both files and of the DEM tile. A fetch whose result differs from the lock
fails (OpenStreetMap is edited all the time) unless --update-lock is given; --check only verifies the files on disk.

  python3 -m venv build/levels_venv && build/levels_venv/bin/pip install -r tools/levels/requirements.txt
  build/levels_venv/bin/python tools/levels/fetch_osm_track.py              # fetch, compare with the lock
  build/levels_venv/bin/python tools/levels/fetch_osm_track.py --check      # verify the committed files
  build/levels_venv/bin/python tools/levels/fetch_osm_track.py --update-lock

The DEM tile (16 MB) is cached in build/level_cache. tifffile + imagecodecs decode it (deflate, float predictor).
"""
import argparse
import hashlib
import json
import os
import sys
import time
import urllib.parse
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LEVEL = os.path.join(ROOT, "assets", "levels", "achna_speedway")
SOURCE = os.path.join(LEVEL, "source")
LOCK = os.path.join(SOURCE, "sources.lock.json")
CACHE = os.path.join(ROOT, "build", "level_cache")
UA = {"User-Agent": "TrailForge-level-fetch/1.0 (https://github.com/; offline game level build)"}

# Venue bounding box (lon / lat) and the DEM crop (a bit larger than the 1 km level square).
BBOX = (33.7925, 35.0205, 33.8100, 35.0315)  # min lon, min lat, max lon, max lat
DEM_CENTER = (35.02546, 33.80017)              # lat, lon (the level origin, make_achna_speedway.py)
DEM_HALF = 0.0075                              # degrees of latitude / longitude around the center
OSM_API = "https://api.openstreetmap.org/api/0.6/map?bbox={:.4f},{:.4f},{:.4f},{:.4f}"
OVERPASS = "https://overpass-api.de/api/interpreter"
DEM_TILE = "Copernicus_DSM_COG_10_N35_00_E033_00_DEM"
DEM_URL = "https://copernicus-dem-30m.s3.amazonaws.com/{0}/{0}.tif".format(DEM_TILE)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url, path, data=None):
    for attempt in range(5):
        try:
            req = urllib.request.Request(url, data=data, headers=UA)
            with urllib.request.urlopen(req, timeout=180) as r, open(path + ".part", "wb") as f:
                while True:
                    chunk = r.read(1 << 20)
                    if not chunk:
                        break
                    f.write(chunk)
            os.replace(path + ".part", path)
            return
        except Exception as e:  # noqa: BLE001 - network errors of any kind are retried
            wait = 5 * 2 ** attempt
            print(f"  {url[:80]}: {e}; retry in {wait} s")
            time.sleep(wait)
    raise SystemExit(f"download failed: {url}")


def fetch_osm(path):
    try:
        download(OSM_API.format(*BBOX), path)
    except SystemExit:
        # Overpass fallback: the same elements (out meta, XML) for the bounding box.
        q = "[timeout:120];(node({1},{0},{3},{2});way({1},{0},{3},{2});relation({1},{0},{3},{2}););(._;>;);out meta;"
        download(OVERPASS, path, urllib.parse.urlencode({"data": q.format(*BBOX)}).encode())


def crop_dem(tile_path, path):
    import numpy as np
    import tifffile

    with tifffile.TiffFile(tile_path) as tif:
        page = tif.pages[0]
        scale = page.tags["ModelPixelScaleTag"].value
        tie = page.tags["ModelTiepointTag"].value
        a = page.asarray()
    # Upper-left corner of pixel (0, 0) (GeoTIFF RasterPixelIsArea); pixel (r, c) center:
    # lon = tie[3] + (c + 0.5) * scale[0], lat = tie[4] - (r + 0.5) * scale[1].
    lat0, lon0 = DEM_CENTER
    r0 = int(np.floor((tie[4] - (lat0 + DEM_HALF)) / scale[1]))
    r1 = int(np.ceil((tie[4] - (lat0 - DEM_HALF)) / scale[1]))
    c0 = int(np.floor((lon0 - DEM_HALF - tie[3]) / scale[0]))
    c1 = int(np.ceil((lon0 + DEM_HALF - tie[3]) / scale[0]))
    win = a[r0:r1 + 1, c0:c1 + 1].astype(float)
    out = {
        "source": DEM_URL,
        "note": "Copernicus GLO-30 DSM, heights in m (EGM2008); rows north to south, columns west to east",
        "lat0": round(tie[4] - (r0 + 0.5) * scale[1], 9), "lon0": round(tie[3] + (c0 + 0.5) * scale[0], 9),
        "dlat": scale[1], "dlon": scale[0], "rows": int(win.shape[0]), "cols": int(win.shape[1]),
        "heights": [[round(float(v), 2) for v in row] for row in win],
    }
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("{\n")
        keys = [k for k in out if k != "heights"]
        for k in keys:
            f.write(f'  "{k}": {json.dumps(out[k])},\n')
        f.write('  "heights": [\n')
        for i, row in enumerate(out["heights"]):
            f.write("    " + json.dumps(row, separators=(",", ":")) + ("," if i + 1 < len(out["heights"]) else "") + "\n")
        f.write("  ]\n}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only verify the files on disk against the lock")
    ap.add_argument("--update-lock", action="store_true", help="accept the fetched files and rewrite the lock")
    args = ap.parse_args()
    lock = json.load(open(LOCK)) if os.path.exists(LOCK) else {}
    osm, dem = os.path.join(SOURCE, "osm.osm"), os.path.join(SOURCE, "dem.json")
    if args.check:
        bad = [n for n, p in (("osm.osm", osm), ("dem.json", dem)) if lock.get(n) != (sha256(p) if os.path.exists(p) else None)]
        print("source files match the lock" if not bad else f"MISMATCH: {', '.join(bad)}")
        return 1 if bad else 0

    os.makedirs(SOURCE, exist_ok=True)
    os.makedirs(CACHE, exist_ok=True)
    print("OpenStreetMap", OSM_API.format(*BBOX))
    fetch_osm(osm + ".new")
    tile = os.path.join(CACHE, DEM_TILE + ".tif")
    if not os.path.exists(tile):
        print("Copernicus DEM", DEM_URL)
        download(DEM_URL, tile)
    crop_dem(tile, dem + ".new")
    got = {"osm.osm": sha256(osm + ".new"), "dem.json": sha256(dem + ".new"), "dem_tile": sha256(tile)}
    diff = [k for k in got if lock.get(k) not in (None, got[k])]
    if diff and not args.update_lock:
        print(f"fetched data differs from sources.lock.json: {', '.join(diff)} (OSM edits?); kept the committed files,"
              f" new ones in *.new; rerun with --update-lock to accept")
        return 1
    os.replace(osm + ".new", osm)
    os.replace(dem + ".new", dem)
    lock.update(got)
    lock["osm_url"] = OSM_API.format(*BBOX)
    lock["dem_url"] = DEM_URL
    with open(LOCK, "w", encoding="utf-8", newline="\n") as f:
        json.dump(lock, f, indent=2, sort_keys=True)
        f.write("\n")
    print("wrote", osm, dem, LOCK)
    return 0


if __name__ == "__main__":
    sys.exit(main())
