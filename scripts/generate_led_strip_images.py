#!/usr/bin/env python3

import argparse
import colorsys
import math
import re
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print(
        "Pillow is required to generate POV LED-strip images. "
        "Install it with: python3 -m pip install Pillow",
        file=sys.stderr,
    )
    raise SystemExit(1)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "include" / "config.h"
ASSET_DIRECTORY = ROOT / "assets" / "led-strip"
HEADER_PATH = ROOT / "lib" / "LEDHandler" / "GeneratedLEDStripImages.h"
SOURCE_PATH = ROOT / "lib" / "LEDHandler" / "GeneratedLEDStripImages.cpp"
SUPPORTED_EXTENSIONS = {".jpeg", ".jpg", ".png"}
FIXED_ANIMATIONS = [
    "Off",
    "Bootup",
    "Idle",
    "Forward",
    "Failsafe",
    "LowBattery",
    "HardwareError",
    "TestPattern",
]


def read_integer_constant(config_path, name):
    text = config_path.read_text(encoding="utf-8")
    match = re.search(rf"^\s*constexpr\s+\w+\s+{re.escape(name)}\s*=\s*(\d+)\s*;", text, re.MULTILINE)
    if not match:
        raise RuntimeError(f"Could not read numeric {name} from {config_path}")
    return int(match.group(1))


def read_dimensions(config_path=CONFIG_PATH):
    led_count = read_integer_constant(config_path, "LED_STRIP_LED_COUNT")
    sectors_per_led = read_integer_constant(config_path, "LED_STRIP_SECTORS_PER_LED")
    if led_count < 2:
        raise RuntimeError("LED_STRIP_LED_COUNT must be at least 2")
    if sectors_per_led < 1:
        raise RuntimeError("LED_STRIP_SECTORS_PER_LED must be at least 1")
    return led_count, led_count * sectors_per_led


def filename_to_enum(path):
    words = re.findall(r"[A-Za-z0-9]+", path.stem)
    if not words:
        raise RuntimeError(f"Image filename does not contain an enum name: {path.name}")
    name = "".join(word[0].upper() + word[1:] for word in words)
    if name[0].isdigit():
        name = "Image" + name
    return name


def discover_images(asset_directory=ASSET_DIRECTORY):
    if not asset_directory.exists():
        return []

    images = [path for path in asset_directory.iterdir() if path.is_file() and path.suffix.lower() in SUPPORTED_EXTENSIONS]
    images.sort(key=lambda path: (path.name.casefold(), path.name))

    names = FIXED_ANIMATIONS.copy()
    named_images = []
    for path in images:
        enum_name = filename_to_enum(path)
        if enum_name.lower() in {name.lower() for name in names}:
            raise RuntimeError(f"Duplicate LED-strip animation name {enum_name!r} from {path.name}")
        names.append(enum_name)
        named_images.append((enum_name, path))
    return named_images


def sample_image(path, led_count, sector_count):
    image = Image.open(path).convert("RGBA")
    width, height = image.size
    center_x = width / 2.0
    center_y = height / 2.0
    maximum_radius_leds = (led_count - 1) / 2.0
    scale = min(center_x, center_y) / maximum_radius_leds
    sectors = []

    for sector in range(sector_count):
        angle = 2.0 * math.pi * sector / sector_count - math.pi / 2.0
        pixels = []
        for led in range(led_count):
            radius_leds = led - maximum_radius_leds
            x = int(center_x + radius_leds * scale * math.cos(angle))
            y = int(center_y + radius_leds * scale * math.sin(angle))
            x = max(0, min(width - 1, x))
            y = max(0, min(height - 1, y))
            red, green, blue, alpha = image.getpixel((x, y))
            red = red * alpha // 255
            green = green * alpha // 255
            blue = blue * alpha // 255
            pixels.append((red << 16) | (green << 8) | blue)
        sectors.append(pixels)
    return sectors


def make_test_pattern(led_count, sector_count):
    maximum_radius = (led_count - 1) / 2.0
    sectors = []

    for sector in range(sector_count):
        angle = 2.0 * math.pi * sector / sector_count - math.pi / 2.0
        pixels = []
        for led in range(led_count):
            signed_radius = (led - maximum_radius) / maximum_radius
            x = signed_radius * math.cos(angle)
            y = signed_radius * math.sin(angle)
            radius = abs(signed_radius)

            if abs(x) < 0.06 or abs(y) < 0.06:
                color = 0xFFFFFF
            elif 0.43 < radius < 0.58:
                color = 0x00FFFF
            else:
                hue = (math.atan2(y, x) / (2.0 * math.pi)) % 1.0
                red, green, blue = colorsys.hsv_to_rgb(hue, 1.0, 0.55)
                color = (round(red * 255) << 16) | (round(green * 255) << 8) | round(blue * 255)
            pixels.append(color)
        sectors.append(pixels)
    return sectors


def render_array(name, sectors):
    lines = [f"constexpr uint32_t {name}Pixels[GENERATED_SECTOR_COUNT][GENERATED_LED_COUNT] = {{"]
    for pixels in sectors:
        values = ", ".join(f"0x{pixel:06X}" for pixel in pixels)
        lines.append(f"    {{{values}}},")
    lines.append("};")
    return "\n".join(lines)


def render_files(config_path=CONFIG_PATH, asset_directory=ASSET_DIRECTORY):
    led_count, sector_count = read_dimensions(config_path)
    images = discover_images(asset_directory)
    image_data = [("TestPattern", make_test_pattern(led_count, sector_count))]
    image_data.extend((name, sample_image(path, led_count, sector_count)) for name, path in images)
    enum_names = FIXED_ANIMATIONS + [name for name, _ in images]

    enum_lines = ",\n    ".join(f"{name} = {index}" for index, name in enumerate(enum_names))
    header = f"""#pragma once

// Generated by scripts/generate_led_strip_images.py. Do not edit manually.

#include \"config.h\"

#include <stdint.h>

enum class LEDStripAnimation : uint8_t {{
    {enum_lines}
}};

namespace LEDStripImages {{
constexpr uint16_t GENERATED_LED_COUNT = {led_count};
constexpr uint16_t GENERATED_SECTOR_COUNT = {sector_count};

static_assert(RobotConfig::LED_STRIP_LED_COUNT == GENERATED_LED_COUNT,
              \"Regenerate the LED-strip images after changing LED_STRIP_LED_COUNT\");
static_assert(RobotConfig::LED_STRIP_SECTOR_COUNT == GENERATED_SECTOR_COUNT,
              \"Regenerate the LED-strip images after changing the strip sector count\");

bool hasImage(LEDStripAnimation animation);
const uint32_t* getSectorPixels(LEDStripAnimation animation, uint16_t sector);
}} // namespace LEDStripImages
"""

    arrays = "\n\n".join(render_array(name, sectors) for name, sectors in image_data)
    image_cases = "\n".join(
        f"        case LEDStripAnimation::{name}: return {name}Pixels[sector];" for name, _ in image_data
    )
    has_image_cases = "\n".join(f"        case LEDStripAnimation::{name}: return true;" for name, _ in image_data)
    source = f"""// Generated by scripts/generate_led_strip_images.py. Do not edit manually.

#include \"GeneratedLEDStripImages.h\"

namespace LEDStripImages {{
namespace {{
{arrays}
}} // namespace

bool hasImage(LEDStripAnimation animation) {{
    switch(animation) {{
{has_image_cases}
        default: return false;
    }}
}}

const uint32_t* getSectorPixels(LEDStripAnimation animation, uint16_t sector) {{
    if(sector >= GENERATED_SECTOR_COUNT) return nullptr;

    switch(animation) {{
{image_cases}
        default: return nullptr;
    }}
}}
}} // namespace LEDStripImages
"""
    return header, source


def write_if_changed(path, content):
    existing = path.read_text(encoding="utf-8") if path.exists() else None
    if existing == content:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def generate(check=False, config_path=CONFIG_PATH, asset_directory=ASSET_DIRECTORY, header_path=HEADER_PATH,
             source_path=SOURCE_PATH):
    header, source = render_files(config_path, asset_directory)
    outputs = ((header_path, header), (source_path, source))

    if check:
        stale = [str(path) for path, content in outputs if not path.exists() or path.read_text(encoding="utf-8") != content]
        if stale:
            raise RuntimeError("Generated LED-strip files are stale; run scripts/generate_led_strip_images.py: " + ", ".join(stale))
        return

    for path, content in outputs:
        write_if_changed(path, content)


def main():
    parser = argparse.ArgumentParser(description="Generate compile-time POV LED-strip images")
    parser.add_argument("--check", action="store_true", help="fail instead of writing when generated files are stale")
    arguments = parser.parse_args()
    generate(check=arguments.check)


if __name__ == "__main__":
    main()
