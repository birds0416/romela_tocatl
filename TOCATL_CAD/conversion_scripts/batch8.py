"""Convert via convert3d.org, extracting the result from the in-page blob URL."""
import asyncio
import base64
import shutil
import sys
from pathlib import Path

from playwright.async_api import async_playwright

CAD = Path("/home/hojung/Desktop/romela_tocatl/TOCATL_CAD")
OUT = CAD / "converted_stl"
PARTS = [
    "Hip Turret v3.SLDPRT", "Hip Turret Shaft.SLDPRT",
    "Femur Beam.SLDPRT", "Femur Knee.SLDPRT", "Femur Mounting Point.SLDPRT",
    "Femur Bearing Attachment v3.SLDPRT", "Femur Bearing Attachment v3- MIrrored.SLDPRT",
    "Knee Motor Adapter.SLDPRT", "Knee Bar 2.0.SLDPRT", "Knee Hinge.SLDPRT",
    "Tibia 3.5.SLDPRT", "Tarsus.SLDPRT",
    "Body Shell - Left.SLDPRT", "Body Shell - Right.SLDPRT",
]

GRAB_BLOB = """async (url) => {
  const r = await fetch(url);
  const buf = await r.arrayBuffer();
  const b = new Uint8Array(buf);
  let s = '';
  const chunk = 32768;
  for (let i = 0; i < b.length; i += chunk)
    s += String.fromCharCode.apply(null, b.subarray(i, i + chunk));
  return btoa(s);
}"""


async def convert_one(page, src: Path, dest: Path) -> bool:
    await page.goto("https://convert3d.org/convert", wait_until="domcontentloaded")
    await page.wait_for_timeout(4000)
    await page.locator("input[type=file]").first.set_input_files(str(src))
    stl = page.locator("text=STL (Stereolithography)").first
    await stl.wait_for(state="visible", timeout=240000)
    await page.wait_for_timeout(1500)
    await stl.click()
    done = page.locator("text=Done!").first
    await done.wait_for(state="visible", timeout=300000)
    await page.wait_for_timeout(1500)
    hrefs = await page.evaluate(
        "Array.from(document.querySelectorAll('a')).map(a=>a.href)"
        ".filter(h=>h.startsWith('blob:'))")
    if not hrefs:
        raise RuntimeError("no blob url after Done")
    b64 = await page.evaluate(GRAB_BLOB, hrefs[0])
    data = base64.b64decode(b64)
    if len(data) < 200:
        print(f"  warning: tiny result ({len(data)} B)", flush=True)
    dest.write_bytes(data)
    return True


async def main():
    todo = [n for n in (sys.argv[1:] or PARTS) if not (OUT / (Path(n).stem + ".stl")).exists()]
    print(f"converting {len(todo)} parts", flush=True)
    ok = 0
    async with async_playwright() as p:
        shutil.rmtree("/tmp/cad_convert/prof_b8", ignore_errors=True)
        ctx = await p.chromium.launch_persistent_context(
            "/tmp/cad_convert/prof_b8", headless=False,
            viewport={"width": 1500, "height": 950})
        page = ctx.pages[0] if ctx.pages else await ctx.new_page()
        for name in todo:
            dest = OUT / (Path(name).stem + ".stl")
            done = False
            for attempt in (1, 2):
                try:
                    done = await convert_one(page, CAD / name, dest)
                    break
                except Exception as e:
                    print(f"  attempt {attempt}: {type(e).__name__} {str(e)[:90]}", flush=True)
            ok += bool(done)
            size = dest.stat().st_size if dest.exists() else 0
            print(("OK  " if done else "GAVE_UP") + f": {name} ({size//1024} KiB)", flush=True)
        await ctx.close()
    print(f"finished {ok}/{len(todo)}", flush=True)


asyncio.run(main())
