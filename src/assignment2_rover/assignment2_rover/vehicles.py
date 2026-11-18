# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Vehicle identities and homes from the supplied rover world/specification."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Vehicle:
    sysid: int
    home: str

    @property
    def namespace(self):
        return f'/drone_{self.sysid}'

    @property
    def instance(self):
        return self.sysid - 1

    @property
    def mavlink_port(self):
        return 14550 + 10 * self.instance

    @property
    def fdm_port(self):
        return 9002 + 10 * self.instance


VEHICLES = (
    Vehicle(1, '48.15135451,17.07361560,150,0'),
    Vehicle(2, '48.15135451,17.07388440,150,0'),
)
