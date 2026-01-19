# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    pkg_root = Path(__file__).resolve().parents[1]

    # Prefer repo/package-local config
    config_candidates = [
        pkg_root / ".flake8",
        pkg_root / "setup.cfg",
    ]
    config_path = next((p for p in config_candidates if p.exists()), None)

    argv = [str(pkg_root)]
    if config_path is not None:
        argv = ["--config", str(config_path), str(pkg_root)]
    rc, errors = main_with_errors(argv=argv)
    assert rc == 0, "Found %d code style errors / warnings:\n" % len(
        errors
    ) + "\n".join(errors)
