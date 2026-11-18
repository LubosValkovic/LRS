# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Start one already-built SITL directly, with isolated files and UDP output."""

import argparse
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .vehicles import VEHICLES


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle', type=int, choices=[v.sysid for v in VEHICLES], required=True)
    parser.add_argument('--ardupilot-dir', type=Path,
                        default=Path(os.environ.get('ARDUPILOT_DIR', Path.home() / 'ardupilot')))
    parser.add_argument('--run-dir', type=Path, default=Path.cwd() / 'log' / 'sitl',
                        help='parent directory for the separate drone_1 and drone_2 state directories')
    args = parser.parse_args(argv)
    vehicle = next(v for v in VEHICLES if v.sysid == args.vehicle)
    ardupilot = args.ardupilot_dir.expanduser().resolve()
    binary = ardupilot / 'build/sitl/bin/arducopter'
    defaults = [ardupilot / 'Tools/autotest/default_params' / name
                for name in ['copter.parm', 'gazebo-iris.parm']]
    defaults.append(Path(get_package_share_directory('assignment2_rover')) / 'config/rover_sitl.params')
    if not binary.is_file() or not os.access(binary, os.X_OK):
        parser.error(f'build SITL once first: cd {ardupilot} && ./waf configure --board sitl && ./waf copter')
    for path in defaults:
        if not path.is_file():
            parser.error(f'missing SITL defaults: {path}')
    directory = args.run_dir.expanduser().resolve() / f'drone_{vehicle.sysid}'
    directory.mkdir(parents=True, exist_ok=True)
    command = [str(binary), '--model', 'JSON', '--speedup', '1',
               '--defaults', ','.join(map(str, defaults)), '--home', vehicle.home,
               '--sysid', str(vehicle.sysid), '--instance', str(vehicle.instance),
               '--serial0', f'udpclient:127.0.0.1:{vehicle.mavlink_port}']
    print(f'{vehicle.namespace}: SYSID {vehicle.sysid}, FDM {vehicle.fdm_port}, '
          f'MAVLink UDP {vehicle.mavlink_port}; state: {directory}', flush=True)
    os.chdir(directory)
    os.execv(str(binary), command)


if __name__ == '__main__':
    main()
