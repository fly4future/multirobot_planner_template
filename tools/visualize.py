#!/usr/bin/env python3
"""
Visualize planner output: search area polygon, robot start positions, and planned paths.

Usage:
    # Run planner with JSON output, then visualize:
    ./build/planner_demo test_data/scenario_simple.json --output /tmp/result.json
    python3 tools/visualize.py /tmp/result.json

    # Save to file instead of showing interactively:
    python3 tools/visualize.py /tmp/result.json --save plan.png

Requirements:
    pip install matplotlib
"""

import argparse
import json
import sys

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# Distinct colors for up to 10 robots
COLORS = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4",
    "#42d4f4", "#f032e6", "#bfef45", "#fabed4", "#469990",
]


def load_result(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def plot_result(data: dict, save_path: str | None = None):
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # --- Search area polygon ---
    area = data["search_area"]
    poly_lon = [p["longitude"] for p in area] + [area[0]["longitude"]]
    poly_lat = [p["latitude"] for p in area] + [area[0]["latitude"]]
    ax.fill(poly_lon, poly_lat, alpha=0.12, color="gray")
    ax.plot(poly_lon, poly_lat, "k-", linewidth=1.5, label="Search area")

    # --- No-fly zones ---
    for nfz in data.get("no_fly_zones", []):
        verts = nfz.get("vertices", [])
        if len(verts) < 3:
            continue
        nfz_lon = [v["longitude"] for v in verts] + [verts[0]["longitude"]]
        nfz_lat = [v["latitude"] for v in verts] + [verts[0]["latitude"]]
        ax.fill(nfz_lon, nfz_lat, alpha=0.35, color="red", hatch="//")
        ax.plot(nfz_lon, nfz_lat, "r-", linewidth=1.5)
        name = nfz.get("name", "")
        if name:
            cx = sum(v["longitude"] for v in verts) / len(verts)
            cy = sum(v["latitude"] for v in verts) / len(verts)
            ax.text(cx, cy, name, ha="center", va="center", fontsize=7,
                    fontweight="bold", color="darkred")

    # --- Robot paths ---
    legends = []
    for i, path_data in enumerate(data.get("paths", [])):
        color = COLORS[i % len(COLORS)]
        name = path_data["robot_name"]
        wps = path_data["waypoints"]

        if not wps:
            continue

        lons = [w["longitude"] for w in wps]
        lats = [w["latitude"] for w in wps]

        # Path line
        ax.plot(lons, lats, "-", color=color, linewidth=1.2, alpha=0.8)
        # Waypoint markers
        ax.plot(lons, lats, "o", color=color, markersize=3)
        # Start marker
        ax.plot(lons[0], lats[0], "s", color=color, markersize=8)
        # End marker
        ax.plot(lons[-1], lats[-1], "^", color=color, markersize=8)

        legends.append(mpatches.Patch(color=color, label=f"{name} ({len(wps)} wps)"))

    # --- Robot start positions (from input) ---
    for i, robot in enumerate(data.get("robots", [])):
        color = COLORS[i % len(COLORS)]
        ax.plot(
            robot["longitude"], robot["latitude"],
            "*", color=color, markersize=14, markeredgecolor="black",
            markeredgewidth=0.5, zorder=10,
        )

    # --- Labels ---
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    success = data.get("success", False)
    msg = data.get("message", "")
    height = data.get("flight_height", "?")
    n_robots = len(data.get("paths", []))
    ax.set_title(
        f"Planner Output — {n_robots} robots, {height}m AGL\n"
        f"{'OK' if success else 'FAILED'}: {msg}"
    )

    # Equal aspect so the map isn't distorted
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Legend
    area_patch = mpatches.Patch(facecolor="gray", alpha=0.2, edgecolor="black", label="Search area")
    nfz_patch = mpatches.Patch(facecolor="red", alpha=0.35, edgecolor="red",
                               hatch="//", label="No-fly zone")
    star_legend = plt.Line2D(
        [], [], marker="*", color="gray", markersize=10,
        markeredgecolor="black", linestyle="None", label="Robot start pos",
    )
    legend_handles = [area_patch]
    if data.get("no_fly_zones"):
        legend_handles.append(nfz_patch)
    legend_handles.extend([star_legend] + legends)
    ax.legend(handles=legend_handles, loc="upper right", fontsize=9)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved to {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize planner output")
    parser.add_argument("result_json", help="Path to JSON result file (from planner_demo --output)")
    parser.add_argument("--save", metavar="FILE", help="Save plot to file instead of showing")
    args = parser.parse_args()

    data = load_result(args.result_json)

    if not data.get("paths"):
        print(f"No paths in result (success={data.get('success')}, msg={data.get('message')})")
        sys.exit(1)

    plot_result(data, save_path=args.save)


if __name__ == "__main__":
    main()
