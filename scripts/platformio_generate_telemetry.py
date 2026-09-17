Import("env")

import sys
from pathlib import Path


scripts_directory = Path(env.subst("$PROJECT_DIR")) / "scripts"
sys.path.insert(0, str(scripts_directory))

from generate_telemetry_lua import generate


generate()
