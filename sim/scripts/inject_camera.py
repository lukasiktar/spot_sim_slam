#!/usr/bin/env python3
"""
Injects the simulated D435i xacro into the champ Spot robot description.

Run at Docker build time. Finds the main Spot .urdf.xacro in the cloned
simulation repo, detects the base link name (champ Spot descriptions use
'base_link' or 'body' depending on lineage), copies the overlay next to it,
and appends the include + macro instantiation before </robot>.

Idempotent: skips files already containing 'd435i_sim'.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

CANDIDATE_BASE_LINKS = ["base_link", "body", "base"]


def find_robot_xacro(root: Path):
    """Pick the most plausible top-level Spot description file."""
    candidates = []
    for pattern in ("*.urdf.xacro", "*.xacro", "*.urdf"):
        candidates += list(root.rglob(pattern))
    # Prefer files in a *description* package mentioning spot, largest first
    def score(p: Path):
        s = 0
        text = p.read_text(errors="ignore")
        low = str(p).lower()
        if "spot" in low: s += 4
        if "description" in low: s += 3
        if "<robot" in text: s += 2
        if "gazebo" in low: s -= 1  # prefer the main description over gazebo-only files
        s += min(len(text) // 20000, 3)
        return s
    candidates = [c for c in candidates if "<robot" in c.read_text(errors="ignore")]
    if not candidates:
        return None
    return sorted(candidates, key=score, reverse=True)[0]


def detect_base_link(text: str) -> str:
    links = re.findall(r'<link\s+name="([^"]+)"', text)
    for cand in CANDIDATE_BASE_LINKS:
        if cand in links:
            return cand
    return links[0] if links else "base_link"


def find_model_sdf(root: Path):
    """Find the SDF model file Gazebo actually spawns (not the xacro/URDF,
    which only feeds robot_state_publisher's TF view)."""
    candidates = [p for p in root.rglob("model.sdf") if "<model" in p.read_text(errors="ignore")]
    if not candidates:
        return None

    def score(p: Path):
        low = str(p).lower()
        s = 0
        if "spot" in low:
            s += 4
        if "description" in low:
            s += 3
        return s

    return sorted(candidates, key=score, reverse=True)[0]


def inject_sdf_sensors(search_root: Path, sdf_overlay: Path):
    """Splice native <sensor> elements for the D435i straight into
    model.sdf, since that's the file Gazebo actually spawns from."""
    target = find_model_sdf(search_root)
    if target is None:
        sys.exit(f"ERROR: no model.sdf found under {search_root}")

    text = target.read_text()
    if "camera_infra1_frame" in text:
        print(f"[inject_camera] {target} already has the D435i SDF sensors — skipping.")
        return

    snippet = sdf_overlay.read_text()
    if "</model>" not in text:
        sys.exit(f"ERROR: {target} has no closing </model> tag.")
    text = text.replace("</model>", snippet + "\n  </model>")
    target.write_text(text)
    print(f"[inject_camera] Injected D435i SDF sensors into {target}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--search-root", required=True, type=Path)
    ap.add_argument("--overlay", required=True, type=Path)
    ap.add_argument("--sdf-overlay", required=True, type=Path)
    ap.add_argument("--xyz", default="0.30 0 0.12")
    ap.add_argument("--rpy", default="0 0 0")
    args = ap.parse_args()

    target = find_robot_xacro(args.search_root)
    if target is None:
        sys.exit(f"ERROR: no robot description found under {args.search_root}")

    text = target.read_text()
    if "d435i_sim" in text:
        print(f"[inject_camera] {target} already has the D435i overlay — skipping.")
    else:
        base_link = detect_base_link(text)
        overlay_copy = target.parent / args.overlay.name
        shutil.copy(args.overlay, overlay_copy)

        snippet = (
            f'\n  <!-- injected by spot_sim_slam/scripts/inject_camera.py -->\n'
            f'  <xacro:include filename="{args.overlay.name}"/>\n'
            f'  <xacro:d435i_sim parent="{base_link}" xyz="{args.xyz}" rpy="{args.rpy}"/>\n'
        )

        if "</robot>" not in text:
            sys.exit(f"ERROR: {target} has no closing </robot> tag.")
        # Ensure the xacro namespace exists on the <robot> tag
        if "xmlns:xacro" not in text:
            text = re.sub(
                r"<robot\b",
                '<robot xmlns:xacro="http://ros.org/wiki/xacro"',
                text,
                count=1,
            )
        text = text.replace("</robot>", snippet + "</robot>")
        target.write_text(text)
        print(
            f"[inject_camera] Injected D435i into {target}\n"
            f"[inject_camera]   parent link : {base_link}\n"
            f"[inject_camera]   mount xyz   : {args.xyz}   rpy: {args.rpy}\n"
            f"[inject_camera] If the parent link is wrong, re-run with a manual edit."
        )

    # The xacro/URDF injection above only affects robot_state_publisher's TF
    # view — it does nothing in Gazebo, which spawns the robot from model.sdf
    # directly. The actual camera+IMU <sensor> elements have to go there too.
    inject_sdf_sensors(args.search_root, args.sdf_overlay)


if __name__ == "__main__":
    main()
