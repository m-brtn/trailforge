#!/usr/bin/env python3
"""Rebuilds the 3D models in assets/models/ from Poly Haven (CC0).

For every entry of assets/models/models.json it downloads the 1k (or "res") glTF of the asset with its binary
buffer and its diffuse, OpenGL normal and ARM maps, plus the alpha map of alpha-tested materials, checks every
file's md5 against the Poly Haven API and against assets/models/sources.lock.json, and converts each variant with
tools/texc's trailforge_modelc (built here into build/texc) to <out>/<id>.glb (mesh LODs, optional collision mesh,
optional impostor atlas) and the material maps <out>/<stem>_albedo.ktx2 / <stem>_nra.ktx2 (BC7). Writes
assets/models/CREDITS.md from the asset info and prints the size of every output file.

  python3 tools/models/fetch_models.py               # download (cached in build/model_cache), convert
  python3 tools/models/fetch_models.py --lock        # record the md5 of the current Poly Haven files
  python3 tools/models/fetch_models.py --only rock_face_01

Downloads are cached; the converted output is deterministic for the same inputs. No API key is needed.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MODELS = os.path.join(ROOT, "assets", "models")
MANIFEST = os.path.join(MODELS, "models.json")
LOCK = os.path.join(MODELS, "sources.lock.json")
CREDITS = os.path.join(MODELS, "CREDITS.md")
CACHE = os.path.join(ROOT, "build", "model_cache")
TEXC_BUILD = os.path.join(ROOT, "build", "texc")
API = "https://api.polyhaven.com"
UA = {"User-Agent": "TrailForge-model-fetch/1.0"}


def get_json(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=60) as r:
        return json.load(r)


def md5_of(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(asset, key, entry, rel, lock, record):
    """Downloads one file into build/model_cache/<asset>/<rel> after the md5 checks and returns its path."""
    url, md5 = entry["url"], entry["md5"]
    lock_key = f"{asset}/{key}"
    if record:
        lock[lock_key] = md5
    elif lock.get(lock_key) != md5:
        sys.exit(f"{lock_key}: Poly Haven md5 {md5} differs from the lock file ({lock.get(lock_key)}); "
                 "check the change and rerun with --lock")
    path = os.path.join(CACHE, asset, rel)
    if not (os.path.exists(path) and md5_of(path) == md5):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        print(f"  download {url} ({entry['size'] / 1e6:.1f} MB)")
        with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=600) as r, \
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


def build_modelc():
    exe = os.path.join(TEXC_BUILD, "trailforge_modelc" + (".exe" if os.name == "nt" else ""))
    subprocess.run(["cmake", "-S", os.path.join(ROOT, "tools", "texc"), "-B", TEXC_BUILD,
                    "-DCMAKE_BUILD_TYPE=Release"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["cmake", "--build", TEXC_BUILD, "--config", "Release", "--target", "trailforge_modelc"],
                   check=True)
    return exe


def image_uri(gltf, texture_ref):
    if not texture_ref:
        return None
    return gltf["images"][gltf["textures"][texture_ref["index"]]["source"]]["uri"]


def convert(model, modelc, lock, record):
    asset, res = model["asset"], model.get("res", "1k")
    files = get_json(f"{API}/files/{asset}")
    entry = files["gltf"][res]["gltf"]
    gltf_path = fetch(asset, f"gltf_{res}", entry, os.path.basename(entry["url"]), lock, record)
    includes = entry["include"]
    with open(gltf_path) as f:
        gltf = json.load(f)
    # Binary buffers.
    for buf in gltf.get("buffers", []):
        uri = buf["uri"]
        fetch(asset, uri, includes[uri], uri, lock, record)

    out_dir = os.path.join(MODELS, model["out"])
    os.makedirs(out_dir, exist_ok=True)
    materials, written = {}, set()
    for mat in gltf["materials"]:
        pbr = mat.get("pbrMetallicRoughness", {})
        diff = image_uri(gltf, pbr.get("baseColorTexture"))
        nor = image_uri(gltf, mat.get("normalTexture"))
        arm = image_uri(gltf, pbr.get("metallicRoughnessTexture"))
        if not diff:
            sys.exit(f"{asset}: material {mat['name']} has no base colour texture")
        stem = os.path.basename(diff).rsplit("_diff_", 1)[0]
        spec = {"name": stem, "normalGl": True}
        spec["albedo"] = fetch(asset, diff, includes[diff], diff, lock, record)
        spec["normal"] = fetch(asset, nor, includes[nor], nor, lock, record)
        rough_key = "rough" if "_rough_" in os.path.basename(arm) else "arm"
        spec[rough_key] = fetch(asset, arm, includes[arm], arm, lock, record)
        if mat.get("alphaMode", "OPAQUE") != "OPAQUE" or "twig" in mat["name"]:
            prefix = stem[len(asset) + 1:] if stem.startswith(asset + "_") else ""
            key = f"{prefix}_alpha" if prefix else "Alpha"
            if key not in files:
                sys.exit(f"{asset}: no {key} map for the alpha-tested material {mat['name']}")
            alpha_res = files[key][res]
            fmt = "png" if "png" in alpha_res else "jpg"
            e = alpha_res[fmt]
            spec["alpha"] = fetch(asset, f"{key}_{res}_{fmt}", e, "alpha/" + os.path.basename(e["url"]), lock, record)
        materials[mat["name"]] = spec

    for variant in model["variants"]:
        mats = {}
        for name, spec in materials.items():
            s = dict(spec)
            s["write"] = spec["name"] not in written
            mats[name] = s
        spec = {
            "id": variant["id"], "gltf": gltf_path, "out": out_dir, "kind": model["kind"],
            "nodes": variant.get("nodes", []), "lods": model["lods"],
            "collisionLod": model.get("collisionLod", -1), "textureSize": model.get("textureSize", 1024),
            "normalSize": model.get("normalSize", 512), "materials": mats,
        }
        if "impostor" in model:
            spec["impostor"] = model["impostor"]
        spec_path = os.path.join(CACHE, asset, f"{variant['id']}.spec.json")
        with open(spec_path, "w") as f:
            json.dump(spec, f, indent=2)
        print(f"{variant['id']}: {asset}")
        r = subprocess.run([modelc, "--spec", spec_path], check=True, capture_output=True, text=True)
        sys.stdout.write(r.stdout)
        # Textures written by this variant (only materials used by its nodes).
        for line in r.stdout.splitlines():
            if line.strip().endswith("dB") and "_albedo.ktx2" in line:
                written.add(os.path.basename(line.split(":")[0].strip()).rsplit("_albedo", 1)[0])
    return files


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lock", action="store_true", help="record the md5 of the current Poly Haven files")
    ap.add_argument("--only", help="rebuild only this Poly Haven asset")
    args = ap.parse_args()

    with open(MANIFEST) as f:
        manifest = json.load(f)
    lock = {}
    if os.path.exists(LOCK):
        with open(LOCK) as f:
            lock = json.load(f)
    modelc = build_modelc()

    credits = []
    for model in manifest["models"]:
        asset = model["asset"]
        info = get_json(f"{API}/info/{asset}")
        credits.append((model["out"], asset, info.get("name", asset), info.get("authors", {}),
                        [v["id"] for v in model["variants"]]))
        if args.only and asset != args.only:
            continue
        convert(model, modelc, lock, args.lock)

    if args.lock:
        with open(LOCK, "w") as f:
            json.dump(dict(sorted(lock.items())), f, indent=2)
            f.write("\n")

    lines = [
        "# Model credits",
        "",
        "All 3D models in this folder are from [Poly Haven](https://polyhaven.com) and are released under",
        "[CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/) (public domain; attribution is not required).",
        "They were converted by `tools/models/fetch_models.py` (`tools/texc`, `trailforge_modelc`): mesh LODs and",
        "collision meshes simplified with meshoptimizer, foliage cards thinned (trees: clump cards baked from clusters of",
        "the source foliage), impostor atlases rendered from the full models, textures downscaled and re-encoded to",
        "BC7 KTX2. The source md5 values are in `sources.lock.json`.",
        "",
        "| Folder | Poly Haven asset | Authors | Variants |",
        "|---|---|---|---|",
    ]
    for out, asset, name, authors, variants in credits:
        who = ", ".join(f"{k} ({v})" if v != "All" else k for k, v in authors.items())
        lines.append(f"| {out} | [{name}](https://polyhaven.com/a/{asset}) | {who} | {', '.join(variants)} |")
    with open(CREDITS, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {CREDITS}")

    total = 0
    for dirpath, _, names in os.walk(MODELS):
        for n in sorted(names):
            if n.endswith((".glb", ".ktx2")):
                total += os.path.getsize(os.path.join(dirpath, n))
    print(f"assets/models: {total / 1e6:.1f} MB in .glb + .ktx2")


if __name__ == "__main__":
    main()
