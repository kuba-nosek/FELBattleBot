# POV LED-strip images

Put `.png`, `.jpg`, or `.jpeg` source images in this directory. The generator
discovers up to four images alphabetically, samples each image from the outer
edge toward the rotation axis, and compiles the result into the firmware.
Transparent pixels are rendered as black.

The filename becomes the `LEDStripAnimation` name. For example,
`team-logo.png` becomes `LEDStripAnimation::TeamLogo`. Names must remain unique
after punctuation and spaces are removed.

In Spin mode, the six-position switch selects:

| Position | Display |
| ---: | --- |
| 1 | Off |
| 2 | Built-in test pattern |
| 3–6 | User images in alphabetical filename order |

If a selected user-image position is empty, the strip remains off. Adding more
than four user images is a generation error.

Regenerate the C++ files manually with:

```sh
python3 scripts/generate_led_strip_images.py
```

Verify that committed generated files are current without changing them:

```sh
python3 scripts/generate_led_strip_images.py --check
```

Preview the ideal rotary image reconstructed from the actual generated C++
arrays:

```sh
python3 scripts/preview_led_strip_image.py
```

The script saves one PNG per generated image and a combined `AllImages.png`
under `.pio/led-strip-previews/`, then opens the combined preview. Add
`--apply-runtime-brightness` to apply the configured strip brightness or
`--no-show` to only write the PNG files.

Every PlatformIO build runs the generator automatically. If Pillow is missing,
install it with `python3 -m pip install Pillow`.
