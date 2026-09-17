#!/usr/bin/env python3

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "include" / "config.h"
TEMPLATE_PATH = ROOT / "edgetx" / "BBOT.template.lua"
SCREEN_OUTPUTS = {
    ROOT / "edgetx" / "SCRIPTS" / "TELEMETRY" / "BBGRPH.lua": "graph",
    ROOT / "edgetx" / "SCRIPTS" / "TELEMETRY" / "BBVALS.lua": "values",
    ROOT / "edgetx" / "SCRIPTS" / "TELEMETRY" / "BBRAW.lua": "raw",
}
SCHEMA_MARKER = "-- {{TELEMETRY_SCHEMA}}"

TYPE_SIZES = {
    "int8_t": 1,
    "uint8_t": 1,
    "int16_t": 2,
    "uint16_t": 2,
    "int32_t": 4,
    "uint32_t": 4,
    "float": 4,
}


def read_fields():
    lines = CONFIG_PATH.read_text(encoding="utf-8").splitlines()
    fields = []
    inside_map = False

    for line in lines:
        if not inside_map:
            if re.match(r"^\s*#define\s+TELEMETRY_FIELD_MAP\(X\)", line):
                inside_map = True
            continue

        match = re.search(r"\bX\(\s*([A-Za-z_]\w*)\s*,\s*([A-Za-z_]\w*)\s*\)", line)
        if match:
            fields.append((match.group(1), match.group(2)))

        if not line.rstrip().endswith("\\"):
            break

    if not fields:
        raise RuntimeError(f"No fields found in TELEMETRY_FIELD_MAP in {CONFIG_PATH}")

    names = [name for name, _ in fields]
    duplicate_names = sorted({name for name in names if names.count(name) > 1})
    if duplicate_names:
        raise RuntimeError(f"Duplicate telemetry field names: {', '.join(duplicate_names)}")

    unsupported_types = sorted({field_type for _, field_type in fields if field_type not in TYPE_SIZES})
    if unsupported_types:
        raise RuntimeError(f"Unsupported telemetry field types: {', '.join(unsupported_types)}")

    if len(fields) > 32:
        raise RuntimeError("Telemetry supports at most 32 configured fields")

    serialized_size = sum(TYPE_SIZES[field_type] for _, field_type in fields)
    frame_size = 11 + serialized_size
    if frame_size > 64:
        raise RuntimeError(f"Configured telemetry requires a {frame_size}-byte CRSF frame; maximum is 64")

    return fields


def render_schema(fields):
    payload_size = 7 + sum(TYPE_SIZES[field_type] for _, field_type in fields)
    offset = 8
    schema_key = ",".join(f"{name}:{field_type}" for name, field_type in fields)
    lines = [
        f'local SCHEMA_KEY = "{schema_key}"',
        f"local PAYLOAD_SIZE = {payload_size}",
        "local fields = {",
    ]

    for name, field_type in fields:
        size = TYPE_SIZES[field_type]
        lines.append(f'  {{ name="{name}", type="{field_type}", size={size}, offset={offset} }},')
        offset += size

    lines.append("}")
    return "\n".join(lines)


def render_screen(screen):
    fields = read_fields()
    template = TEMPLATE_PATH.read_text(encoding="utf-8")
    if template.count(SCHEMA_MARKER) != 1:
        raise RuntimeError(f"{TEMPLATE_PATH} must contain exactly one {SCHEMA_MARKER!r} marker")

    generated_header = "-- Generated from include/config.h by scripts/generate_telemetry_lua.py.\n"
    rendered = template.replace(SCHEMA_MARKER, render_schema(fields))
    return generated_header + rendered.replace("{{TELEMETRY_SCREEN}}", screen)


def generate(check=False):
    outputs = {path: render_screen(screen) for path, screen in SCREEN_OUTPUTS.items()}

    if check:
        stale_outputs = [
            str(path)
            for path, rendered in outputs.items()
            if not path.exists() or path.read_text(encoding="utf-8") != rendered
        ]
        if stale_outputs:
            raise RuntimeError(
                "Generated telemetry files are stale; run scripts/generate_telemetry_lua.py: "
                + ", ".join(stale_outputs)
            )
        return

    for path, rendered in outputs.items():
        current = path.read_text(encoding="utf-8") if path.exists() else None
        if current != rendered:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(rendered, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Generate the EdgeTX telemetry files from TELEMETRY_FIELD_MAP")
    parser.add_argument(
        "--check", action="store_true", help="fail instead of writing when generated Lua files are stale"
    )
    arguments = parser.parse_args()
    generate(check=arguments.check)


if __name__ == "__main__":
    main()
