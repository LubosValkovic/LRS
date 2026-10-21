# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Local WGS84 geodetic and ENU conversions for the outdoor mission."""

import math
from functools import lru_cache


@lru_cache(maxsize=32)
def _metres_per_degree(latitude):
    """Reuse the Earth radii while the mission's home latitude stays fixed."""
    if not math.isfinite(latitude) or not -90 < latitude < 90:
        raise ValueError('local ENU origin must have a finite latitude between the poles')
    phi = math.radians(latitude)
    sin_phi = math.sin(phi)
    a = 6378137.0
    e2 = 0.00669437999014
    prime = a / math.sqrt(1 - e2 * sin_phi * sin_phi)
    meridian = a * (1 - e2) / (1 - e2 * sin_phi * sin_phi) ** 1.5
    return math.radians(1) * prime * math.cos(phi), math.radians(1) * meridian


def geodetic_to_enu(lat, lon, origin_lat, origin_lon):
    """WGS84 local tangent approximation; coordinates in degrees and metres."""
    if (not all(math.isfinite(v) for v in (lat, lon, origin_lon))
            or not -90 <= lat <= 90 or not -180 <= lon <= 180 or not -180 <= origin_lon <= 180):
        raise ValueError('GPS coordinates must be finite and within latitude/longitude bounds')
    east_scale, north_scale = _metres_per_degree(origin_lat)
    longitude_delta = (lon - origin_lon + 180) % 360 - 180
    return longitude_delta * east_scale, (lat - origin_lat) * north_scale


def enu_to_geodetic(east, north, origin_lat, origin_lon):
    """Convert a local ENU offset back to WGS84 latitude and longitude."""
    if not all(math.isfinite(v) for v in (east, north, origin_lon)) or not -180 <= origin_lon <= 180:
        raise ValueError('ENU offsets and origin longitude must be finite and valid')
    east_scale, north_scale = _metres_per_degree(origin_lat)
    return origin_lat + north / north_scale, (origin_lon + east / east_scale + 180) % 360 - 180
