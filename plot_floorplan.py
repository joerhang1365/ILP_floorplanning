import argparse, re
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

def parse_sizes(testcase_path: Path):
    sizes = {}
    lines = testcase_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    module_size = None; start_idx = None
    for i, line in enumerate(lines):
        m = re.match(r"\s*MODULE_SIZE\s+(\d+)", line)
        if m:
            module_size = int(m.group(1)); start_idx = i + 2; break
    if module_size is None:
        raise RuntimeError("Cannot find MODULE_SIZE in testcase file.")
    read_count = 0; i = start_idx
    while read_count < module_size and i < len(lines):
        line = lines[i].strip(); i += 1
        if not line: continue
        parts = re.split(r"[ \t]+", line)
        if len(parts) < 3: continue
        try:
            mid = int(parts[0]); w = int(parts[1]); h = int(parts[2])
        except:
            continue
        sizes[mid] = (w, h); read_count += 1
    if read_count != module_size:
        raise RuntimeError(f"Parsed {read_count} modules, expected {module_size}.")
    return sizes

def parse_placement(placement_path: Path):
    placements = []
    for raw in placement_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line: continue
        m = re.match(r"^\s*(\d+)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+(\d+)\s*$", line)
        if m:
            mid = int(m.group(1)); x = int(m.group(2)); y = int(m.group(3)); rot = int(m.group(4))
        else:
            m2 = re.match(r"\s*(\d+)\s+([+-]?\d+(?:\.\d+)?)[ ,]\s*([+-]?\d+(?:\.\d+)?)\s+(\d+)", line)
            if not m2: continue
            mid = int(m2.group(1)); x = int(m2.group(2)); y = int(m2.group(3)); rot = int(m2.group(4))
        placements.append({"id": mid, "x": x, "y": y, "rot": rot})
    if not placements:
        raise RuntimeError("No placements parsed from placement file.")
    return placements

def compute_bbox(sizes, placements):
    max_x = 0.0; max_y = 0.0
    for pl in placements:
        if pl["id"] not in sizes: continue
        w, h = sizes[pl["id"]]
        if pl["rot"] != 0: w, h = h, w
        x2 = pl["x"] + w
        y2 = pl["y"] + h
        if x2 > max_x: max_x = x2
        if y2 > max_y: max_y = y2
    return max_x, max_y

def _text_wh(draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont):
    try:
        left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
        return right - left, bottom - top
    except Exception:
        try:
            return font.getsize(text)
        except Exception:
            return (len(text) * 6, 10)

def plot_floorplan_pillow(testcase, placement, output="floorplan.png",
                          canvas_w=1600, canvas_h=1200, margin_px=40):
    sizes = parse_sizes(Path(testcase))
    placements = parse_placement(Path(placement))
    max_x, max_y = compute_bbox(sizes, placements)
    if max_x <= 0 or max_y <= 0:
        raise RuntimeError("Invalid bounding box from placements.")


    sx = (canvas_w - 2*margin_px) / max_x
    sy = (canvas_h - 2*margin_px) / max_y
    s = min(sx, sy)


    img = Image.new("RGB", (canvas_w, canvas_h), "white")
    draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()

    
    for pl in placements:
        mid = pl["id"]
        if mid not in sizes: continue
        w, h = sizes[mid]
        if pl["rot"] != 0: w, h = h, w
        ww = int(round(w * s))
        hh = int(round(h * s))
        x = margin_px + int(round(pl["x"] * s))
        y = canvas_h - margin_px - int(round(pl["y"] * s)) - hh
        draw.rectangle([x, y, x+ww, y+hh], outline="black", width=1)
        text = str(mid)
        tw, th = _text_wh(draw, text, font)
        tx = x + (ww - tw)//2
        ty = y + (hh - th)//2
        draw.text((tx, ty), text, fill="black", font=font)

    
    draw.line([margin_px-10, canvas_h-margin_px, margin_px+10, canvas_h-margin_px], fill="gray")
    draw.line([margin_px, canvas_h-margin_px-10, margin_px, canvas_h-margin_px+10], fill="gray")

    
    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"Saved to {out}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Plot floorplan without matplotlib (uses Pillow).")
    ap.add_argument("testcase", help="Path to testcase file (contains module sizes).")
    ap.add_argument("placement", help="Path to placement file (format: ID X Y ROT).")
    ap.add_argument("-o", "--output", default="floorplan.png", help="Output image path (PNG).")
    args = ap.parse_args()
    plot_floorplan_pillow(args.testcase, args.placement, args.output)
