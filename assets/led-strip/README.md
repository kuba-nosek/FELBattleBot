# POV LED-strip images

Put `.png`, `.jpg`, or `.jpeg` source images in this directory. The generator
discovers them alphabetically, samples each image across the full strip diameter,
and compiles the result into the firmware. Transparent pixels are rendered as
black.

The filename becomes the `LEDStripAnimation` name. For example,
`team-logo.png` becomes `LEDStripAnimation::TeamLogo`. Names must remain unique
after punctuation and spaces are removed.

Regenerate the C++ files manually with:

```sh
python3 scripts/generate_led_strip_images.py
```

Verify that committed generated files are current without changing them:

```sh
python3 scripts/generate_led_strip_images.py --check
```

Every PlatformIO build runs the generator automatically. If Pillow is missing,
install it with `python3 -m pip install Pillow`.
