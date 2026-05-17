"""
Step 4 — Plot the reachable workspace and save to outputs/workspace.png.

Usage:
    python tools/plot_workspace.py
    python tools/plot_workspace.py --steps 15   # faster, less detail
    python tools/plot_workspace.py --steps 30   # slower, more detail

No robot required.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinematics.workspace import sample_workspace


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot SO-101 reachable workspace")
    parser.add_argument("--steps", type=int, default=20,
                        help="Samples per joint axis (default 20, ~160k points)")
    args = parser.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")   # no display needed
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("ERROR: matplotlib not installed. Run:  pip install matplotlib")
        sys.exit(1)

    print(f"Sampling workspace ({args.steps} steps/joint) …")
    pts = sample_workspace(steps_per_joint=args.steps)
    total = len(pts)
    print(f"  {total:,} points generated.")

    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    zs = [p.z for p in pts]
    rs = [p.r for p in pts]

    out_dir = Path(__file__).resolve().parents[1] / "outputs"
    out_dir.mkdir(exist_ok=True)

    fig = plt.figure(figsize=(16, 5))
    fig.suptitle("SO-101 Reachable Workspace (placeholder geometry)", fontsize=13)

    # ── 3-D scatter ───────────────────────────────────────────────────────────
    ax3 = fig.add_subplot(131, projection="3d")
    ax3.scatter(xs, ys, zs, c=zs, cmap="viridis", s=0.3, alpha=0.3)
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_zlabel("Z (m)")
    ax3.set_title("3-D view")

    # ── Side view: radial reach vs height ────────────────────────────────────
    ax2 = fig.add_subplot(132)
    ax2.scatter(rs, zs, c=zs, cmap="viridis", s=0.3, alpha=0.2)
    ax2.set_xlabel("Radial reach r (m)")
    ax2.set_ylabel("Height z (m)")
    ax2.set_title("Side view (r–z plane)")
    ax2.grid(True, linewidth=0.4)

    # ── Top view: X–Y footprint ───────────────────────────────────────────────
    ax1 = fig.add_subplot(133)
    ax1.scatter(xs, ys, c=rs, cmap="plasma", s=0.3, alpha=0.2)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Top view (X–Y plane)")
    ax1.set_aspect("equal")
    ax1.grid(True, linewidth=0.4)

    plt.tight_layout()
    out_path = out_dir / "workspace.png"
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"  Saved -> {out_path}")

    # Stats
    print(f"\nWorkspace bounds:")
    print(f"  X : [{min(xs):.3f}, {max(xs):.3f}] m")
    print(f"  Y : [{min(ys):.3f}, {max(ys):.3f}] m")
    print(f"  Z : [{min(zs):.3f}, {max(zs):.3f}] m")
    print(f"  r : [{min(rs):.3f}, {max(rs):.3f}] m")


if __name__ == "__main__":
    main()
