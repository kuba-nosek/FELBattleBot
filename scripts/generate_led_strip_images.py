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
HEADER_PATH = ROOT / "lib" / "POVDisplay" / "GeneratedLEDStripImages.h"
SOURCE_PATH = ROOT / "lib" / "POVDisplay" / "GeneratedLEDStripImages.cpp"
SUPPORTED_EXTENSIONS = {".jpeg", ".jpg", ".png"}
MAX_USER_IMAGES = 4
ANGULAR_SAMPLES_PER_SECTOR = 4
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


def read_integer_array_constant(config_path, name):
    text = config_path.read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*constexpr\s+\w+\s+{re.escape(name)}\s*\[[^]]+\]\s*=\s*\{{([^}}]+)\}}\s*;",
        text,
        re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"Could not read numeric array {name} from {config_path}")
    return [int(value) for value in re.findall(r"\d+", match.group(1))]


def read_dimensions(config_path=CONFIG_PATH):
    led_count = read_integer_constant(config_path, "LED_STRIP_LED_COUNT")
    sector_count = read_integer_constant(config_path, "LED_STRIP_SECTOR_COUNT")
    sector_divisors = read_integer_array_constant(config_path, "LED_STRIP_SECTOR_DIVISORS")
    if led_count < 2:
        raise RuntimeError("LED_STRIP_LED_COUNT must be at least 2")
    if sector_count < 1:
        raise RuntimeError("LED_STRIP_SECTOR_COUNT must be at least 1")
    if len(sector_divisors) != led_count:
        raise RuntimeError("LED_STRIP_SECTOR_DIVISORS must contain one value per LED")
    if any(divisor < 1 or sector_count % divisor != 0 for divisor in sector_divisors):
        raise RuntimeError("Every LED strip sector divisor must be positive and divide LED_STRIP_SECTOR_COUNT")
    return led_count, sector_count, sector_divisors


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

    if len(images) > MAX_USER_IMAGES:
        raise RuntimeError(
            f"At most {MAX_USER_IMAGES} user LED-strip images are supported; found {len(images)}"
        )

    names = FIXED_ANIMATIONS.copy()
    named_images = []
    for path in images:
        enum_name = filename_to_enum(path)
        if enum_name.lower() in {name.lower() for name in names}:
            raise RuntimeError(f"Duplicate LED-strip animation name {enum_name!r} from {path.name}")
        names.append(enum_name)
        named_images.append((enum_name, path))
    return named_images


def sample_image_color(image, x, y):
    width, height = image.size
    x = max(0.0, min(width - 1.0, x))
    y = max(0.0, min(height - 1.0, y))
    x0 = math.floor(x)
    y0 = math.floor(y)
    x1 = min(x0 + 1, width - 1)
    y1 = min(y0 + 1, height - 1)
    x_fraction = x - x0
    y_fraction = y - y0
    samples = (
        (image.getpixel((x0, y0)), (1.0 - x_fraction) * (1.0 - y_fraction)),
        (image.getpixel((x1, y0)), x_fraction * (1.0 - y_fraction)),
        (image.getpixel((x0, y1)), (1.0 - x_fraction) * y_fraction),
        (image.getpixel((x1, y1)), x_fraction * y_fraction),
    )

    channels = []
    for channel in range(3):
        channels.append(sum(pixel[channel] * pixel[3] / 255.0 * weight for pixel, weight in samples))
    return channels


def average_angular_color(group_start, group_width, sector_count, color_at_angle):
    sample_count = ANGULAR_SAMPLES_PER_SECTOR * group_width
    totals = [0.0, 0.0, 0.0]

    for sample in range(sample_count):
        sector_position = group_start + (sample + 0.5) * group_width / sample_count
        angle = 2.0 * math.pi * sector_position / sector_count - math.pi / 2.0
        color = color_at_angle(angle)
        for channel in range(3):
            totals[channel] += color[channel]

    channels = [round(total / sample_count) for total in totals]
    return (channels[0] << 16) | (channels[1] << 8) | channels[2]


def sample_image(path, led_count, sector_count, sector_divisors=None):
    if sector_divisors is None:
        sector_divisors = [1] * led_count

    image = Image.open(path).convert("RGBA")
    width, height = image.size
    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0
    maximum_radius_leds = led_count - 0.5
    scale = min(center_x, center_y) / maximum_radius_leds
    sectors = [[0] * led_count for _ in range(sector_count)]

    for led in range(led_count):
        radius_leds = maximum_radius_leds - led
        divisor = sector_divisors[led]

        def color_at_angle(angle):
            x = center_x + radius_leds * scale * math.cos(angle)
            y = center_y + radius_leds * scale * math.sin(angle)
            return sample_image_color(image, x, y)

        for group_start in range(0, sector_count, divisor):
            color = average_angular_color(group_start, divisor, sector_count, color_at_angle)
            for sector in range(group_start, group_start + divisor):
                sectors[sector][led] = color
    return sectors


def make_test_pattern(led_count, sector_count, sector_divisors=None):
    if sector_divisors is None:
        sector_divisors = [1] * led_count

    maximum_radius = led_count - 0.5
    sectors = [[0] * led_count for _ in range(sector_count)]

    for led in range(led_count):
        radius = (maximum_radius - led) / maximum_radius
        divisor = sector_divisors[led]

        def color_at_angle(angle):
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)

            if abs(x) < 0.06 or abs(y) < 0.06:
                return (255, 255, 255)
            if 0.43 < radius < 0.58:
                return (0, 255, 255)

            hue = (math.atan2(y, x) / (2.0 * math.pi)) % 1.0
            red, green, blue = colorsys.hsv_to_rgb(hue, 1.0, 0.55)
            return (round(red * 255), round(green * 255), round(blue * 255))

        for group_start in range(0, sector_count, divisor):
            color = average_angular_color(group_start, divisor, sector_count, color_at_angle)
            for sector in range(group_start, group_start + divisor):
                sectors[sector][led] = color
    return sectors


def render_array(name, sectors):
    lines = [f"constexpr uint32_t {name}Pixels[GENERATED_SECTOR_COUNT][GENERATED_LED_COUNT] = {{"]
    for pixels in sectors:
        values = ", ".join(f"0x{pixel:06X}" for pixel in pixels)
        lines.append(f"    {{{values}}},")
    lines.append("};")
    return "\n".join(lines)


def render_switch_cases(images):
    cases = [
        "        case 0: return LEDStripAnimation::Off;",
        "        case 1: return LEDStripAnimation::TestPattern;",
    ]

    for image_index in range(MAX_USER_IMAGES):
        animation = images[image_index][0] if image_index < len(images) else "Off"
        cases.append(f"        case {image_index + 2}: return LEDStripAnimation::{animation};")

    return "\n".join(cases)


def render_files(config_path=CONFIG_PATH, asset_directory=ASSET_DIRECTORY):
    led_count, sector_count, sector_divisors = read_dimensions(config_path)
    images = discover_images(asset_directory)
    image_data = [("TestPattern", make_test_pattern(led_count, sector_count, sector_divisors))]
    image_data.extend(
        (name, sample_image(path, led_count, sector_count, sector_divisors)) for name, path in images
    )
    enum_names = FIXED_ANIMATIONS + [name for name, _ in images]

    enum_lines = ",\n    ".join(f"{name} = {index}" for index, name in enumerate(enum_names))
    divisor_assertions = "\n".join(
        f"static_assert(RobotConfig::LED_STRIP_SECTOR_DIVISORS[{index}] == {divisor},\n"
        f"              \"Regenerate the LED-strip images after changing sector divisor {index}\");"
        for index, divisor in enumerate(sector_divisors)
    )
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
constexpr uint8_t GENERATED_USER_IMAGE_COUNT = {len(images)};

static_assert(RobotConfig::LED_STRIP_LED_COUNT == GENERATED_LED_COUNT,
              \"Regenerate the LED-strip images after changing LED_STRIP_LED_COUNT\");
static_assert(RobotConfig::LED_STRIP_SECTOR_COUNT == GENERATED_SECTOR_COUNT,
              \"Regenerate the LED-strip images after changing the strip sector count\");
{divisor_assertions}

inline LEDStripAnimation animationForSpinSwitchPosition(uint8_t position) {{
    switch(position) {{
{render_switch_cases(images)}
        default: return LEDStripAnimation::Off;
    }}
}}

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
