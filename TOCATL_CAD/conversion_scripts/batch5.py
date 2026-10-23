"""Convert via convert3d.org app. Correct flow: upload -> click STL -> wait -> Download."""
import asyncio, shutil, sys
from pathlib import Path
from playwright.async_api import async_playwright

CAD = Path("/home/hojung/Desktop/romela_tocatl/TOCATL_CAD")
OUT = CAD / "converted_stl"
PARTS = [
    "Body Shell - Left.SLDPRT", "Body Shell - Right.SLDPRT",
    "Hip Mount v4.SLDPRT", "Hip Turret v3.SLDPRT", "Hip Turret Shaft.SLDPRT",
    "Femur Beam.SLDPRT", "Femur Knee.SLDPRT", "Femur Mounting Point.SLDPRT",
    "Femur Bearing Attachment v3.SLDPRT", "Femur Bearing Attachment v3- MIrrored.SLDPRT",
    "Knee Motor Adapter.SLDPRT", "Knee Bar 2.0.SLDPRT", "Knee Hinge.SLDPRT",
    "Tibia 3.5.SLDPRT", "Tarsus.SLDPRT",
]

async def convert_one(page, src: Path, dest: Path) -> bool:
    await page.goto("https://convert3d.org/convert", wait_until="domcontentloaded")
    await page.wait_for_timeout(4000)
    await page.locator("input[type=file]").first.set_input_files(str(src))
    stl = page.locator("text=STL (Stereolithography)").first
    await stl.wait_for(state="visible", timeout=180000)
    await page.wait_for_timeout(1500)
    await stl.click()
    dl = page.locator("a:has-text('Download'):visible, button:has-text('Download'):visible").first
    await dl.wait_for(state="visible", timeout=300000)
    async with page.expect_download(timeout=120000) as dli:
        await dl.click()
    d = await dli.value
    await d.save_as(str(dest))
    return True

async def main():
    todo = [n for n in (sys.argv[1:] or PARTS) if not (OUT / (Path(n).stem + ".stl")).exists()]
    print(f"converting {len(todo)} parts", flush=True)
    ok = 0
    async with async_playwright() as p:
        ctx = await p.chromium.launch_persistent_context("/tmp/cad_convert/prof_b5",
            headless=False, viewport={"width":1500,"height":950}, accept_downloads=True)
        page = ctx.pages[0] if ctx.pages else await ctx.new_page()
        for name in todo:
            dest = OUT / (Path(name).stem + ".stl")
            try:
                ok += bool(await convert_one(page, CAD / name, dest))
                print(f"OK  : {name}", flush=True)
            except Exception as e:
                print(f"ERR : {name}: {type(e).__name__} {str(e)[:100]}", flush=True)
        await ctx.close()
    print(f"finished {ok}/{len(todo)}", flush=True)

asyncio.run(main())
