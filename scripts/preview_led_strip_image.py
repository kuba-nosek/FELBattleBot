#!/usr/bin/env python3

import argparse
import math
import re
import sys
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print(
        "Pillow is required to preview POV LED-strip images. "
        "Install it with: python3 -m pip install Pillow",
        file=sys.stderr,
    )
    raise SystemExit(1)


ROOT = Path(__file__).resolve().parents[1]
GENERATED_SOURCE_PATH = ROOT / "lib" / "LEDHandler" / "GeneratedLEDStripImages.cpp"
GENERATED_HEADER_PATH = ROOT / "lib" / "LEDHandler" / "GeneratedLEDStripImages.h"
CONFIG_PATH = ROOT / "include" / "config.h"
DEFAULT_OUTPUT_DIRECTORY = ROOT / ".pio" / "led-strip-previews"


def read_integer_constant(path, name):
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"^\s*constexpr\s+\w+\s+{re.escape(name)}\s*=\s*(\d+)\s*;", text, re.MULTILINE)
    if not match:
        raise RuntimeError(f"Could not read numeric {name} from {path}")
    return int(match.group(1))


def read_generated_arrays(source_path=GENERATED_SOURCE_PATH):
    source = source_path.read_text(encoding="utf-8")
    array_pattern = re.compile(
        r"constexpr\s+uint32_t\s+(\w+)Pixels\s*"
        r"\[GENERATED_SECTOR_COUNT\]\[GENERATED_SPIN_LED_COUNT\]\s*=\s*\{(.*?)\n\};",
        re.DOTALL,
    )
    row_pattern = re.compile(r"\{([^{}]+)\}")
    arrays = {}

    for match in array_pattern.finditer(source):
        name, body = match.groups()
        sectors = []
        for row in row_pattern.findall(body):
            sectors.append([int(value, 16) for value in re.findall(r"0x([0-9A-Fa-f]{6,8})", row)])
        arrays[name] = sectors

    if not arrays:
        raise RuntimeError(f"No generated LED-strip image arrays found in {source_path}")

    return arrays


def validate_dimensions(arrays, header_path=GENERATED_HEADER_PATH):
    expected_sector_count = read_integer_constant(header_path, "GENERATED_SECTOR_COUNT")
    expected_led_count = read_integer_constant(header_path, "GENERATED_SPIN_LED_COUNT")

    for name, sectors in arrays.items():
        if len(sectors) != expected_sector_count:
            raise RuntimeError(
                f"{name} contains {len(sectors)} sectors, but the generated header expects {expected_sector_count}"
            )
        if any(len(sector) != expected_led_count for sector in sectors):
            raise RuntimeError(f"{name} does not contain exactly {expected_led_count} LEDs in every sector")


def scale_color(color, brightness):
    red = (((color >> 16) & 0xFF) * brightness + 127) // 255
    green = (((color >> 8) & 0xFF) * brightness + 127) // 255
    blue = ((color & 0xFF) * brightness + 127) // 255
    return (red << 16) | (green << 8) | blue


def color_tuple(color):
    return (color >> 16 & 0xFF, color >> 8 & 0xFF, color & 0xFF)


def render_pov_image(sectors, size, brightness=None, supersampling=3):
    if size < 32:
        raise ValueError("Preview size must be at least 32 pixels")
    if not sectors or not sectors[0]:
        raise ValueError("The generated image array is empty")

    render_size = size * supersampling
    image = Image.new("RGB", (render_size, render_size), (0, 0, 0))
    draw = ImageDraw.Draw(image)
    center = render_size / 2.0
    outer_radius = render_size * 0.48
    led_count = len(sectors[0])
    maximum_radius_leds = led_count - 0.5
    degrees_per_sector = 360.0 / len(sectors)

    for sector_index, pixels in enumerate(sectors):
        start_angle = -90.0 + sector_index * degrees_per_sector
        end_angle = start_angle + degrees_per_sector

        # Draw outer-to-inner so each smaller disc leaves one radial LED band
        # visible. Pixel zero is the physical outer LED.
        for led_index, color in enumerate(pixels):
            radius_leds = maximum_radius_leds if led_index == 0 else led_count - led_index
            radius = outer_radius * radius_leds / maximum_radius_leds
            box = (center - radius, center - radius, center + radius, center + radius)
            displayed_color = scale_color(color, brightness) if brightness is not None else color
            draw.pieslice(box, start_angle, end_angle, fill=color_tuple(displayed_color))

    if supersampling == 1:
        return image
    return image.resize((size, size), Image.Resampling.LANCZOS)


def create_contact_sheet(rendered_images):
    image_count = len(rendered_images)
    columns = min(3, image_count)
    rows = math.ceil(image_count / columns)
    image_size = next(iter(rendered_images.values())).width
    label_height = max(32, image_size // 14)
    sheet = Image.new("RGB", (columns * image_size, rows * (image_size + label_height)), (0, 0, 0))
    draw = ImageDraw.Draw(sheet)

    try:
        font = ImageFont.truetype("DejaVuSans.ttf", max(18, image_size // 28))
    except OSError:
        font = ImageFont.load_default()

    for index, (name, image) in enumerate(rendered_images.items()):
        column = index % columns
        row = index // columns
        x = column * image_size
        y = row * (image_size + label_height)
        sheet.paste(image, (x, y + label_height))
        draw.text((x + image_size / 2, y + label_height / 2), name, fill=(255, 255, 255), font=font, anchor="mm")

    return sheet


def main():
    parser = argparse.ArgumentParser(
        description="Preview all ideal POV images reconstructed from the generated firmware arrays"
    )
    parser.add_argument("--list", action="store_true", help="list generated image names and exit")
    parser.add_argument("--size", type=int, default=700, help="square preview size in pixels (default: 700)")
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=DEFAULT_OUTPUT_DIRECTORY,
        help="directory for individual previews and AllImages.png",
    )
    parser.add_argument(
        "--apply-runtime-brightness",
        action="store_true",
        help="apply LED_STRIP_BRIGHTNESS exactly as the firmware does",
    )
    parser.add_argument("--no-show", action="store_true", help="save the PNG without opening an image viewer")
    arguments = parser.parse_args()

    arrays = read_generated_arrays()
    validate_dimensions(arrays)

    if arguments.list:
        print("\n".join(arrays))
        return

    brightness = None
    if arguments.apply_runtime_brightness:
        brightness = read_integer_constant(CONFIG_PATH, "LED_STRIP_BRIGHTNESS")

    output_directory = arguments.output_directory.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    rendered_images = {}
    for animation_name, sectors in arrays.items():
        image = render_pov_image(sectors, arguments.size, brightness)
        image.save(output_directory / f"{animation_name}.png")
        rendered_images[animation_name] = image

    contact_sheet = create_contact_sheet(rendered_images)
    contact_sheet_path = output_directory / "AllImages.png"
    contact_sheet.save(contact_sheet_path)
    print(f"Saved {len(rendered_images)} previews and contact sheet to {output_directory}")

    if not arguments.no_show:
        contact_sheet.show(title="POV LED-strip previews")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
