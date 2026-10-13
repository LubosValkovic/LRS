# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Local WGS84 geodetic and ENU conversions for the outdoor mission."""

import math


def geodetic_to_enu(lat, lon, origin_lat, origin_lon):
    """WGS84 local tangent approximation; coordinates in degrees and metres."""
    phi = math.radians(origin_lat)
    sin_phi = math.sin(phi)
    a = 6378137.0
    e2 = 0.00669437999014
    prime = a / math.sqrt(1 - e2 * sin_phi * sin_phi)
    meridian = a * (1 - e2) / (1 - e2 * sin_phi * sin_phi) ** 1.5
    return (math.radians(lon - origin_lon) * prime * math.cos(phi),
            math.radians(lat - origin_lat) * meridian)


def enu_to_geodetic(east, north, origin_lat, origin_lon):
    """Convert a local ENU offset back to WGS84 latitude and longitude."""
    phi = math.radians(origin_lat)
    sin_phi = math.sin(phi)
    a = 6378137.0
    e2 = 0.00669437999014
    prime = a / math.sqrt(1 - e2 * sin_phi * sin_phi)
    meridian = a * (1 - e2) / (1 - e2 * sin_phi * sin_phi) ** 1.5
    return (origin_lat + math.degrees(north / meridian),
            origin_lon + math.degrees(east / (prime * math.cos(phi))))

