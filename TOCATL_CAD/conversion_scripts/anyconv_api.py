"""Convert CAD files via anyconv.com's JSON API (reverse-engineered from their app JS).

Usage: anyconv_api.py <input file> <out dir> [target format, default stl]
"""
import random
import sys
import time
import urllib.parse
from pathlib import Path

import requests

UA = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36"


def make_fid() -> str:
    n = int(time.time() * 1000)
    out = []
    for ch in "xxxxx3xxxxxxxxxxxxxxxxxxx5xxxxxx":
        t = int((n + 16 * random.random()) % 16)
        n //= 16
        out.append(format(t if ch == "x" else (t & 7 | 8), "x") if ch == "x" else
                   (ch if ch in "35" else format(t, "x")))
    # pattern chars '3' and '5' are literals in the template
    return "".join(c if c in "35" else c for c in out)


def convert(src: Path, out_dir: Path, to: str = "stl", quiet=False) -> Path:
    s = requests.Session()
    s.headers["User-Agent"] = UA
    page = f"https://anyconv.com/{src.suffix[1:].lower()}-to-{to}-converter/"
    r = s.get(page, timeout=30)
    if r.status_code != 200:
        s.get("https://anyconv.com/cad-converter/", timeout=30)
    xsrf = s.cookies.get("XSRF-TOKEN", "")
    headers = {
        "Origin": "https://anyconv.com",
        "Referer": page,
        "X-Requested-With": "XMLHttpRequest",
    }
    if xsrf:
        headers["X-XSRF-TOKEN"] = urllib.parse.unquote(xsrf)

    fid = make_fid()
    with open(src, "rb") as f:
        r = s.post(
            f"https://anyconv.com/api/action/add/{fid}/",
            files={"file": (src.name, f, "application/octet-stream")},
            data={"to": to},
            headers=headers,
            timeout=180,
        )
    if not quiet:
        print("add:", r.status_code, r.text[:200])
    r.raise_for_status()

    deadline = time.time() + 300
    while time.time() < deadline:
        time.sleep(2.5)
        r = s.get(f"https://anyconv.com/api/action/status/{fid}/",
                  headers=headers, timeout=30)
        data = r.json()
        if data.get("errors"):
            raise RuntimeError(f"conversion failed: {data['errors']}")
        if data.get("status"):
            url = data["url"]
            name = data.get("result_name") or (src.stem + "." + to)
            out_dir.mkdir(parents=True, exist_ok=True)
            dest = out_dir / name
            dl = s.get(url if url.startswith("http") else "https://anyconv.com" + url,
                       headers=headers, timeout=300)
            dl.raise_for_status()
            dest.write_bytes(dl.content)
            if not quiet:
                print(f"saved: {dest} ({dest.stat().st_size} bytes)")
            return dest
    raise TimeoutError("conversion did not finish in 300 s")


if __name__ == "__main__":
    src = Path(sys.argv[1])
    out = Path(sys.argv[2])
    fmt = sys.argv[3] if len(sys.argv) > 3 else "stl"
    convert(src, out, fmt)
