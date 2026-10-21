# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Local GPS conversions, including date-line crossing and invalid fixes."""

import pytest

from assignment1_mission.part03.geo import enu_to_geodetic, geodetic_to_enu


@pytest.mark.parametrize('origin', [(48.15144444, 17.07375), (0., 0.), (10., 179.9999)])
def test_local_offsets_round_trip(origin):
    position = enu_to_geodetic(30., -20., *origin)
    assert geodetic_to_enu(*position, *origin) == pytest.approx((30., -20.), abs=1e-6)


def test_date_line_uses_short_longitude_difference():
    east, north = geodetic_to_enu(10., -179.9999, 10., 179.9999)
    assert 0 < east < 25
    assert north == 0


@pytest.mark.parametrize('position', [(float('nan'), 17.), (91., 17.), (48., 181.)])
def test_invalid_gps_fix_is_rejected(position):
    with pytest.raises(ValueError):
        geodetic_to_enu(*position, 48., 17.)
