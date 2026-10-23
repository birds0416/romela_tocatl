"""Convert a SolidWorks subassembly (with its referenced files) via convert3d.org."""
import asyncio
import base64
import shutil
import sys
from pathlib import Path

from playwright.async_api import async_playwright

sys.path.insert(0, "/tmp/cad_convert")
from batch8 import GRAB_BLOB

CAD = Path("/home/hojung/Desktop/romela_tocatl/TOCATL_CAD")


async def convert_asm(files: list[Path], out: Path, target_label: str) -> None:
    async with async_playwright() as p:
        shutil.rmtree("/tmp/cad_convert/prof_asm", ignore_errors=True)
        ctx = await p.chromium.launch_persistent_context(
            "/tmp/cad_convert/prof_asm", headless=False,
            viewport={"width": 1500, "height": 950})
        page = ctx.pages[0] if ctx.pages else await ctx.new_page()
        await page.goto("https://convert3d.org/convert", wait_until="domcontentloaded")
        await page.wait_for_timeout(4000)
        await page.locator("input[type=file]").first.set_input_files([str(f) for f in files])
        target = page.locator(f"text={target_label}").first
        await target.wait_for(state="visible", timeout=300000)
        await page.wait_for_timeout(2000)
        await page.screenshot(path="/tmp/cad_convert/asm_loaded.png")
        body = await page.inner_text("body")
        print("pre-convert state:", " | ".join(l for l in body.splitlines() if l.strip())[:300])
        await target.click()
        done = page.locator("text=Done!").first
        await done.wait_for(state="visible", timeout=600000)
        await page.wait_for_timeout(2000)
        hrefs = await page.evaluate(
        "Array.from(document.querySelectorAll('a')).map(a=>a.href).filter(h=>h.startsWith('blob:'))")
        print("blobs:", len(hrefs))
        for i, h in enumerate(hrefs):
            data = base64.b64decode(await page.evaluate(GRAB_BLOB, h))
            dest = out if len(hrefs) == 1 else out.with_stem(out.stem + f"_{i}")
            dest.write_bytes(data)
            print("saved:", dest, len(data), "bytes")
        await page.screenshot(path="/tmp/cad_convert/asm_done.png", full_page=True)
        await ctx.close()


if __name__ == "__main__":
    files = [
        CAD / "Subassemblies/Subassembly - Lower Leg.SLDASM",
        CAD / "Subassemblies/Tibia with Inserts.SLDASM",
        CAD / "Tibia 3.5.SLDPRT",
        CAD / "Tarsus.SLDPRT",
        CAD / "Toe.SLDPRT",
        CAD / "Shell Inserts.SLDPRT",
    ]
    asyncio.run(convert_asm(files, Path("/tmp/cad_convert/asm/lower_leg.stl"),
                            "STL (Stereolithography)"))
