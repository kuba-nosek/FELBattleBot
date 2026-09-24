Import("env")

import shutil
import subprocess
from pathlib import Path


project_directory = Path(env.subst("$PROJECT_DIR"))
generator = project_directory / "scripts" / "generate_led_strip_images.py"
system_python = Path("/usr/bin/python3")

if not system_python.exists():
    discovered_python = shutil.which("python3")
    if not discovered_python:
        raise RuntimeError("python3 is required to generate POV LED-strip images")
    system_python = Path(discovered_python)

result = subprocess.run([str(system_python), str(generator)], cwd=project_directory, text=True)
if result.returncode != 0:
    raise RuntimeError(
        "LED-strip image generation failed. If Pillow is missing, install it with: "
        "python3 -m pip install Pillow"
    )
