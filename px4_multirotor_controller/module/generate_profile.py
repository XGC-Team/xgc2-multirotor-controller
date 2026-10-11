#!/usr/bin/env python3
"""Compile the actual owning YAML into the core; no second default table."""
import hashlib
import json
from pathlib import Path
import sys
import yaml
source = Path(sys.argv[1])
profile = json.dumps(yaml.safe_load(source.read_text()), separators=(",", ":"), allow_nan=False)
Path(sys.argv[2]).write_text("#pragma once\nnamespace px4_multirotor_controller {\n"
    + 'constexpr const char* kControllerProfile = R"XGC_PROFILE(' + profile + ')XGC_PROFILE";\n'
    + 'constexpr const char* kControllerProfileSha256 = "' + hashlib.sha256(source.read_bytes()).hexdigest() + '";\n}\n')
