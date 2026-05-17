"""
colour_calibration.py 

Calibrates the HSV values based on the images inputted. 
This is done 

The images were taken as screenshots of each colour in the reaction on a white background. 

The script outputs HSV range constants ready to paste directly into colour_monitor.py.
 
Usage
-----
    # Basic — point at your three images:
    python calibrate_hsv.py --green green.png --red red.png --yellow yellow.png
 
    # Adjust ROI and padding interactively:
    python calibrate_hsv.py --green green.png --red red.png --yellow yellow.png \
                            --padding 20 --show-masks
 
    # Save output to a file as well as printing to terminal:
    python calibrate_hsv.py --green green.png --red red.png --yellow yellow.png \
                            --output hsv_ranges.py
 
How it works
------------
1.  Loads each image and crops to the same ROI used by ColourMonitor.
2.  Converts to HSV and computes per-channel statistics (mean, std, percentiles).
3.  Sets lower/upper bounds as  mean ± (N * std),  clamped to valid HSV ranges.
    The padding multiplier N is tunable via --padding (default 2.5σ).
4.  Adds a fixed floor on Saturation and Value so pale backgrounds are excluded.
5.  Prints a verification table showing what fraction of each image's ROI
    the derived range captures — you want >70% for the matching colour and
    <5% cross-contamination with the other colours.
6.  Prints the final HSV_RANGES and COLOUR_DOMINANCE_THRESHOLD blocks,
    formatted to paste straight into colour_monitor.py.
"""
 
import cv2
import numpy as np
import argparse
import sys
from pathlib import Path
 
 
# ──────────────────────────────────────────────────────────────────────────────
# DEFAULTS  (mirror colour_monitor.py so the output is a drop-in replacement)
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_ROI         = (0.40, 0.25, 0.60, 0.75)   # (x0, y0, x1, y1) as fractions
DEFAULT_PADDING     = 2.5    # standard-deviation multiplier for hue band width
DEFAULT_S_FLOOR     = 40     # minimum saturation — excludes near-white/grey
DEFAULT_V_FLOOR     = 40     # minimum value     — excludes near-black
DEFAULT_THRESHOLD   = 0.15   # dominance threshold written into output
 
 
# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────
 
def load_roi_hsv(path: str, roi: tuple) -> np.ndarray:
    # load image, crop to ROI, return HSV array.
    img = cv2.imread(path)
    if img is None:
        sys.exit(f"[Error] Cannot read image: {path}")
    h, w = img.shape[:2]
    x1 = int(roi[0] * w);  y1 = int(roi[1] * h)
    x2 = int(roi[2] * w);  y2 = int(roi[3] * h)
    crop = img[y1:y2, x1:x2]
    if crop.size == 0:
        sys.exit(f"[Error] ROI produces an empty crop for {path}. "
                 "Check --roi values are between 0 and 1.")
    return cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
 
 
def compute_range(hsv: np.ndarray, padding: float,
                  s_floor: int, v_floor: int,
                  is_red: bool = False) -> tuple:
    """
    Derive (lower, upper) HSV bounds from an HSV ROI array.
 
    For red, hue wraps around 0/179.  We detect this by checking whether
    the circular mean of the hue channel is near 0 or 179, and if so we
    split into two ranges (lo hue and hi hue) as colour_monitor.py expects.
 
    Returns
    -------
    ranges : list of (lower, upper) tuples — length 1 normally, 2 for red wrap
    wrap   : bool — True when two ranges were emitted
    """
    h_ch = hsv[:, :, 0].flatten().astype(float)
    s_ch = hsv[:, :, 1].flatten().astype(float)
    v_ch = hsv[:, :, 2].flatten().astype(float)
 
    # Filter out low-saturation / low-value pixels (background bleed)
    mask = (s_ch >= s_floor) & (v_ch >= v_floor)
    if mask.sum() < 50:
        print(f"  [Warning] Very few well-saturated pixels ({mask.sum()}) after "
              f"S>={s_floor} V>={v_floor} filter. "
              "The sample region may be washed out — try lowering --s-floor / --v-floor.")
        mask = np.ones_like(mask, dtype=bool)   # fall back to all pixels
 
    h_f = h_ch[mask]
    s_f = s_ch[mask]
    v_f = v_ch[mask]
 
    # ── Hue: handle circular wrap for red ──────────────────────────────────
    # Convert hue (0-179) to radians and compute circular mean
    angles = h_f * (2 * np.pi / 180)
    circ_mean_rad = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
    circ_mean_h   = float(circ_mean_rad * 180 / (2 * np.pi)) % 180
 
    # Circular std
    R = np.sqrt(np.sin(angles).mean()**2 + np.cos(angles).mean()**2)
    R = min(R, 0.9999)
    circ_std_h = float(np.sqrt(-2 * np.log(R)) * 180 / (2 * np.pi))
 
    half_band = padding * circ_std_h
 
    s_mean = float(s_f.mean());  s_std = float(s_f.std())
    v_mean = float(v_f.mean());  v_std = float(v_f.std())
 
    s_lo = max(s_floor,  int(s_mean - padding * s_std))
    v_lo = max(v_floor,  int(v_mean - padding * v_std))
 
    # Decide whether hue wraps (only relevant for red, near 0 or 179)
    wraps = is_red and (circ_mean_h < half_band or circ_mean_h > (180 - half_band))
 
    if wraps:
        # Split into two bands: [0 .. mean+band] and [179-band .. 179]
        h_hi_lo = int(min(179, circ_mean_h + half_band)) if circ_mean_h < 90 \
                  else int(min(179, circ_mean_h + half_band) % 180)
        h_lo_hi = int(max(0,   180 - half_band))
 
        # Primary range (low end, e.g. 0-10)
        r1_lo = (0,          s_lo, v_lo)
        r1_hi = (min(179, int(half_band)), 255, 255)
 
        # Secondary range (high end, e.g. 168-179)
        r2_lo = (max(0, int(180 - half_band)), s_lo, v_lo)
        r2_hi = (179, 255, 255)
 
        return [r1_lo, r1_hi, r2_lo, r2_hi], True
    else:
        h_lo = int(max(0,   circ_mean_h - half_band))
        h_hi = int(min(179, circ_mean_h + half_band))
        return [(h_lo, s_lo, v_lo), (h_hi, 255, 255)], False
 
 
def coverage(hsv_roi: np.ndarray, ranges: list, wrap: bool) -> float:
    # return fraction of ROI pixels captured by the given HSV range(s).
    total = hsv_roi.shape[0] * hsv_roi.shape[1]
    if total == 0:
        return 0.0
    if wrap:
        lo1 = np.array(ranges[0], dtype=np.uint8)
        hi1 = np.array(ranges[1], dtype=np.uint8)
        lo2 = np.array(ranges[2], dtype=np.uint8)
        hi2 = np.array(ranges[3], dtype=np.uint8)
        mask = cv2.inRange(hsv_roi, lo1, hi1) | cv2.inRange(hsv_roi, lo2, hi2)
    else:
        lo = np.array(ranges[0], dtype=np.uint8)
        hi = np.array(ranges[1], dtype=np.uint8)
        mask = cv2.inRange(hsv_roi, lo, hi)
    return float(np.count_nonzero(mask)) / total
 
 
def show_masks(images: dict, ranges_map: dict):
    # display colour-isolation masks for visual verification (press any key).
    for img_name, hsv in images.items():
        vis_rows = []
        for colour_name, (rng, wrap) in ranges_map.items():
            if wrap:
                lo1 = np.array(rng[0], dtype=np.uint8)
                hi1 = np.array(rng[1], dtype=np.uint8)
                lo2 = np.array(rng[2], dtype=np.uint8)
                hi2 = np.array(rng[3], dtype=np.uint8)
                mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
            else:
                lo = np.array(rng[0], dtype=np.uint8)
                hi = np.array(rng[1], dtype=np.uint8)
                mask = cv2.inRange(hsv, lo, hi)
            # Upscale small ROI for visibility
            mask_big = cv2.resize(mask, (200, 300), interpolation=cv2.INTER_NEAREST)
            bgr_orig = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            orig_big = cv2.resize(bgr_orig, (200, 300), interpolation=cv2.INTER_NEAREST)
            combined = np.hstack([orig_big, cv2.cvtColor(mask_big, cv2.COLOR_GRAY2BGR)])
            cv2.putText(combined, f"{colour_name} range on {img_name}",
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            vis_rows.append(combined)
        panel = np.vstack(vis_rows)
        cv2.imshow(f"Masks — {img_name} image", panel)
    print("\n[Calibration] Showing mask windows — press any key to close and continue.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
 
 
# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
 
def main():
    parser = argparse.ArgumentParser(
        description="Derive HSV colour ranges from labelled vial images.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--green",   required=True, help="Path to GREEN vial image")
    parser.add_argument("--red",     required=True, help="Path to RED vial image")
    parser.add_argument("--yellow",  required=True, help="Path to YELLOW vial image")
    parser.add_argument("--roi",     nargs=4, type=float,
                        default=list(DEFAULT_ROI),
                        metavar=("X0","Y0","X1","Y1"),
                        help="ROI as fractions of frame width/height "
                             f"(default: {DEFAULT_ROI})")
    parser.add_argument("--padding", type=float, default=DEFAULT_PADDING,
                        help=f"Std-dev multiplier for band width (default: {DEFAULT_PADDING})")
    parser.add_argument("--s-floor", type=int, default=DEFAULT_S_FLOOR,
                        help=f"Minimum saturation (default: {DEFAULT_S_FLOOR})")
    parser.add_argument("--v-floor", type=int, default=DEFAULT_V_FLOOR,
                        help=f"Minimum value/brightness (default: {DEFAULT_V_FLOOR})")
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD,
                        help=f"COLOUR_DOMINANCE_THRESHOLD to write out (default: {DEFAULT_THRESHOLD})")
    parser.add_argument("--show-masks", action="store_true",
                        help="Open OpenCV windows showing colour masks for each image")
    parser.add_argument("--output", type=str, default=None,
                        help="Optional path to save the generated code block")
    args = parser.parse_args()
 
    roi = tuple(args.roi)
 
    # ── Load images ────────────────────────────────────────────────────────
    print("\n[Calibration] Loading images ...")
    hsv_images = {
        "GREEN":  load_roi_hsv(args.green,  roi),
        "RED":    load_roi_hsv(args.red,    roi),
        "YELLOW": load_roi_hsv(args.yellow, roi),
    }
 
    # ── Compute ranges ─────────────────────────────────────────────────────
    print("[Calibration] Computing HSV ranges ...")
    results = {}
    for colour, hsv in hsv_images.items():
        rng, wrap = compute_range(
            hsv,
            padding  = args.padding,
            s_floor  = args.s_floor,
            v_floor  = args.v_floor,
            is_red   = (colour == "RED"),
        )
        results[colour] = (rng, wrap)
 
        h_ch = hsv[:,:,0].flatten().astype(float)
        s_ch = hsv[:,:,1].flatten().astype(float)
        v_ch = hsv[:,:,2].flatten().astype(float)
        print(f"  {colour:6s}  H_mean={h_ch.mean():.1f}  S_mean={s_ch.mean():.1f}"
              f"  V_mean={v_ch.mean():.1f}  wrap={wrap}")
 
    # ── Verification table ─────────────────────────────────────────────────
    print("\n── Coverage verification ──────────────────────────────────────────")
    print(f"  {'':8s}  {'GREEN range':>14}  {'RED range':>14}  {'YELLOW range':>14}")
    print(f"  {'':8s}  {'(want >70%)':>14}  {'(want >70%)':>14}  {'(want >70%)':>14}")
    print(f"  {'Image':8s}  {'GREEN%':>14}  {'RED%':>14}  {'YELLOW%':>14}")
    print("  " + "-"*56)
 
    all_ok = True
    for img_name, hsv in hsv_images.items():
        row = f"  {img_name:8s}"
        for colour in ["GREEN", "RED", "YELLOW"]:
            rng, wrap = results[colour]
            cov = coverage(hsv, rng, wrap) * 100
            flag = ""
            if img_name == colour and cov < 50:
                flag = " ✗"
                all_ok = False
            elif img_name != colour and cov > 20:
                flag = " !"   # cross-contamination warning
            row += f"  {cov:12.1f}%{flag}"
        print(row)
 
    if not all_ok:
        print("\n  [Warning] Some self-coverage values are low (<50%).")
        print("  Try increasing --padding, or run --calibrate in colour_monitor.py")
        print("  to inspect your actual camera feed.\n")
    else:
        print("\n  ✓ All self-coverage values look good.\n")
 
    # ── Build output code block ────────────────────────────────────────────
    green_rng, _       = results["GREEN"]
    red_rng,   red_w   = results["RED"]
    yellow_rng, _      = results["YELLOW"]
 
    # Format single-range entries
    def fmt_range(lo, hi):
        return f"[({lo[0]}, {lo[1]:3d}, {lo[2]:3d}), ({hi[0]}, {hi[1]:3d}, {hi[2]:3d})]"
 
    green_lo,  green_hi  = green_rng[0],  green_rng[1]
    yellow_lo, yellow_hi = yellow_rng[0], yellow_rng[1]
 
    if red_w:
        red_lo1, red_hi1, red_lo2, red_hi2 = red_rng
    else:
        red_lo1, red_hi1 = red_rng[0], red_rng[1]
        red_lo2 = (168, red_lo1[1], red_lo1[2])
        red_hi2 = (179, 255, 255)
 
    output_lines = [
        "# ── HSV ranges generated by NEW_colour_calibration.py ───────────────────────────────",
        "# Re-run NEW_colour_calibration any time lighting or camera settings change.",
        "#",
        f"# Source images:  GREEN={args.green}  RED={args.red}  YELLOW={args.yellow}",
        f"# ROI used:       {roi}",
        f"# Padding (σ):    {args.padding}",
        f"# S floor:        {args.s_floor}",
        f"# V floor:        {args.v_floor}",
        "#",
        "HSV_RANGES = {",
        f'    "GREEN":  [({green_lo[0]}, {green_lo[1]:3d}, {green_lo[2]:3d}), ({green_hi[0]}, {green_hi[1]:3d}, {green_hi[2]:3d})],',
        f'    "RED":    [({red_lo1[0]}, {red_lo1[1]:3d}, {red_lo1[2]:3d}), ({red_hi1[0]}, {red_hi1[1]:3d}, {red_hi1[2]:3d})],   # red wraps; second range below',
        f'    "RED2":   [({red_lo2[0]}, {red_lo2[1]:3d}, {red_lo2[2]:3d}), ({red_hi2[0]}, {red_hi2[1]:3d}, {red_hi2[2]:3d})],   # upper red hue wrap-around',
        f'    "YELLOW": [({yellow_lo[0]}, {yellow_lo[1]:3d}, {yellow_lo[2]:3d}), ({yellow_hi[0]}, {yellow_hi[1]:3d}, {yellow_hi[2]:3d})],',
        "}",
        "",
        f"COLOUR_DOMINANCE_THRESHOLD = {args.threshold}   # {int(args.threshold*100)}% of ROI pixels",
    ]
 
    code_block = "\n".join(output_lines)
 
    print("── Generated constants (paste into NEW_colour_monitor.py) ─────────────────────")
    print()
    print(code_block)
    print()
 
    if args.output:
        Path(args.output).write_text(code_block)
        print(f"[Calibration] Constants also saved to: {args.output}")
 
    if args.show_masks:
        show_masks(hsv_images, results)
 
    print("[Calibration] Done.")
 
 
if __name__ == "__main__":
    main()
