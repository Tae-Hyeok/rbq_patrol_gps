#!/usr/bin/env python3
# latlon_to_route_raw.py
# No filtering, no dedup. Use coordinates exactly as-is.

from __future__ import annotations

import argparse
import csv
import os
import re
from dataclasses import dataclass
from typing import List, Optional

SEP_RE = re.compile(r"[,\s]+")


@dataclass
class Point:
    lat: float
    lon: float


def parse_line(line: str) -> Optional[Point]:
    s = line.strip()
    if not s or s.startswith("#") or s.startswith("//"):
        return None

    parts = [p for p in SEP_RE.split(s) if p]
    if len(parts) < 2:
        return None

    # INPUT FORMAT: lat lon
    lat = float(parts[0])
    lon = float(parts[1])
    return Point(lat=lat, lon=lon)


def escape_xml(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("'", "&apos;")
    )


def write_mymaps_wkt_csv(points: List[Point], out_csv: str, name: str) -> None:
    # WKT order is "lon lat"
    wkt = "LINESTRING(" + ", ".join(f"{p.lon} {p.lat}" for p in points) + ")"
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["name", "geometry"])
        w.writerow([name, wkt])


def write_kml(points: List[Point], out_kml: str, name: str) -> None:
    # KML coordinates order: lon,lat,alt
    coords = "\n".join(f"{p.lon},{p.lat},0" for p in points)

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{escape_xml(name)}</name>
    <Placemark>
      <name>{escape_xml(name)}</name>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>
{coords}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
"""
    os.makedirs(os.path.dirname(out_kml) or ".", exist_ok=True)
    with open(out_kml, "w", encoding="utf-8") as f:
        f.write(kml)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--input", required=True, help="Input txt file: 'lat lon' per line")
    ap.add_argument("--name", default="route1", help="Route name")
    ap.add_argument("--out-csv", default="route_mymaps.csv", help="Output CSV (WKT) for My Maps")
    ap.add_argument("--out-kml", default="route.kml", help="Output KML for Google Earth / My Maps")
    args = ap.parse_args()

    points: List[Point] = []
    with open(args.input, "r", encoding="utf-8") as f:
        for line in f:
            p = parse_line(line)
            if p is not None:
                points.append(p)

    if len(points) < 2:
        raise SystemExit("Need at least 2 valid coordinate lines.")

    write_mymaps_wkt_csv(points, args.out_csv, args.name)
    write_kml(points, args.out_kml, args.name)

    print(f"[OK] points={len(points)}")
    print(f"[OK] My Maps CSV: {args.out_csv}")
    print(f"[OK] KML: {args.out_kml}")


if __name__ == "__main__":
    main()

