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
        self.assertEqual((4, 12), generator.read_dimensions(self.config))
        first = generator.render_files(self.config, self.assets)
        second = generator.render_files(self.config, self.assets)
        self.assertEqual(first, second)
        self.assertIn("GENERATED_LED_COUNT = 4", first[0])
        self.assertIn("GENERATED_SECTOR_COUNT = 12", first[0])
        self.assertIn("static_assert", first[0])

    def test_converts_filename_to_enum_name(self):
        self.assertEqual("TeamLogoV2", generator.filename_to_enum(Path("team-logo-v2.png")))
        self.assertEqual("Image123", generator.filename_to_enum(Path("123.jpg")))

    def test_discovers_images_alphabetically(self):
        Image.new("RGB", (2, 2)).save(self.assets / "zebra.png")
        Image.new("RGB", (2, 2)).save(self.assets / "alpha.png")
        names = [name for name, _ in generator.discover_images(self.assets)]
        self.assertEqual(["Alpha", "Zebra"], names)

    def test_samples_expected_dimensions_and_transparency_as_black(self):
        image_path = self.assets / "transparent.png"
        image = Image.new("RGBA", (9, 9), (255, 0, 0, 0))
        image.putpixel((4, 4), (0, 255, 0, 255))
        image.save(image_path)

        sampled = generator.sample_image(image_path, 4, 12)
        self.assertEqual(12, len(sampled))
        self.assertTrue(all(len(sector) == 4 for sector in sampled))
        self.assertTrue(all(pixel == 0 for sector in sampled for pixel in sector))

    def test_rejects_duplicate_generated_names(self):
        Image.new("RGB", (2, 2)).save(self.assets / "my-logo.png")
        Image.new("RGB", (2, 2)).save(self.assets / "my logo.jpg")
        with self.assertRaisesRegex(RuntimeError, "Duplicate LED-strip animation name"):
            generator.discover_images(self.assets)

    def test_check_mode_detects_stale_output(self):
        generator.generate(False, self.config, self.assets, self.header, self.source)
        generator.generate(True, self.config, self.assets, self.header, self.source)
        self.source.write_text("stale", encoding="utf-8")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            generator.generate(True, self.config, self.assets, self.header, self.source)


if __name__ == "__main__":
    unittest.main()
