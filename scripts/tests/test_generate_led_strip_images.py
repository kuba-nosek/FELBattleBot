import tempfile
import unittest
from pathlib import Path

from PIL import Image

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import generate_led_strip_images as generator


class GenerateLEDStripImagesTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.config = self.root / "config.h"
        self.assets = self.root / "assets"
        self.header = self.root / "GeneratedLEDStripImages.h"
        self.source = self.root / "GeneratedLEDStripImages.cpp"
        self.assets.mkdir()
        self.config.write_text(
            "constexpr uint16_t LED_STRIP_LED_COUNT = 4;\n"
            "constexpr uint8_t LED_STRIP_SECTORS_PER_LED = 3;\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_parses_dimensions_and_generates_deterministically(self):
        self.assertEqual((4, 2, 12), generator.read_dimensions(self.config))
        first = generator.render_files(self.config, self.assets)
        second = generator.render_files(self.config, self.assets)
        self.assertEqual(first, second)
        self.assertIn("GENERATED_FULL_LED_COUNT = 4", first[0])
        self.assertIn("GENERATED_SPIN_LED_COUNT = 2", first[0])
        self.assertIn("GENERATED_SECTOR_COUNT = 12", first[0])
        self.assertIn("static_assert", first[0])

    def test_rejects_odd_full_strip_led_count(self):
        self.config.write_text(
            "constexpr uint16_t LED_STRIP_LED_COUNT = 5;\n"
            "constexpr uint8_t LED_STRIP_SECTORS_PER_LED = 3;\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(RuntimeError, "must be even"):
            generator.read_dimensions(self.config)

    def test_converts_filename_to_enum_name(self):
        self.assertEqual("TeamLogoV2", generator.filename_to_enum(Path("team-logo-v2.png")))
        self.assertEqual("Image123", generator.filename_to_enum(Path("123.jpg")))

    def test_discovers_images_alphabetically(self):
        Image.new("RGB", (2, 2)).save(self.assets / "zebra.png")
        Image.new("RGB", (2, 2)).save(self.assets / "alpha.png")
        names = [name for name, _ in generator.discover_images(self.assets)]
        self.assertEqual(["Alpha", "Zebra"], names)

    def test_samples_half_strip_outer_to_center_and_applies_transparency(self):
        image_path = self.assets / "radial.png"
        image = Image.new("RGBA", (9, 9), (0, 0, 255, 0))
        image.putpixel((4, 0), (255, 0, 0, 128))
        image.putpixel((4, 3), (0, 255, 0, 255))
        image.save(image_path)

        sampled = generator.sample_image(image_path, 2, 12)
        self.assertEqual(12, len(sampled))
        self.assertTrue(all(len(sector) == 2 for sector in sampled))
        self.assertEqual([0x800000, 0x00FF00], sampled[0])

    def test_rejects_duplicate_generated_names(self):
        Image.new("RGB", (2, 2)).save(self.assets / "my-logo.png")
        Image.new("RGB", (2, 2)).save(self.assets / "my logo.jpg")
        with self.assertRaisesRegex(RuntimeError, "Duplicate LED-strip animation name"):
            generator.discover_images(self.assets)

    def test_accepts_four_images_and_rejects_a_fifth(self):
        for index in range(4):
            Image.new("RGB", (2, 2)).save(self.assets / f"user-{index}.png")
        self.assertEqual(4, len(generator.discover_images(self.assets)))

        Image.new("RGB", (2, 2)).save(self.assets / "user-4.png")
        with self.assertRaisesRegex(RuntimeError, "At most 4 user LED-strip images"):
            generator.discover_images(self.assets)

    def test_generates_six_position_animation_mapping(self):
        Image.new("RGB", (2, 2)).save(self.assets / "zebra.png")
        Image.new("RGB", (2, 2)).save(self.assets / "alpha.png")
        header, _ = generator.render_files(self.config, self.assets)

        self.assertIn("case 0: return LEDStripAnimation::Off;", header)
        self.assertIn("case 1: return LEDStripAnimation::TestPattern;", header)
        self.assertIn("case 2: return LEDStripAnimation::Alpha;", header)
        self.assertIn("case 3: return LEDStripAnimation::Zebra;", header)
        self.assertIn("case 4: return LEDStripAnimation::Off;", header)
        self.assertIn("case 5: return LEDStripAnimation::Off;", header)

    def test_check_mode_detects_stale_output(self):
        generator.generate(False, self.config, self.assets, self.header, self.source)
        generator.generate(True, self.config, self.assets, self.header, self.source)
        self.source.write_text("stale", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            generator.generate(True, self.config, self.assets, self.header, self.source)


if __name__ == "__main__":
    unittest.main()
