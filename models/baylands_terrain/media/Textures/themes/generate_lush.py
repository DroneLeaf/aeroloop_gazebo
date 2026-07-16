#!/usr/bin/env python3
"""Generate the `lush` terrain-theme textures from the stock baylands assets.

The baylands DAE hardcodes three ground texture filenames (Grass.png, Sand.png,
DirtPath.png). Terrain themes work by copying a theme's variants of those three
files over the live ones (see betaloop/common.py apply_terrain_theme). This
script (re)builds themes/lush/ from the sources kept in this media dir:

- Grass.png    <- Grass_original.png (the green pre-desert grass), darkened and
                  upscaled to 2048 for a dark lush lawn.
- Sand.png     <- the desert sand detail, recolored to dark mossy green so open
                  sandy patches read as vegetation.
- DirtPath.png <- the desert dirt detail, recolored to dark wet earth.

Run from this directory: python3 generate_lush.py
"""
from pathlib import Path

from PIL import Image, ImageEnhance

HERE = Path(__file__).resolve().parent
TEXTURES = HERE.parent
LUSH = HERE / "lush"


def colorize(src: Image.Image, rgb: tuple[float, float, float],
             gain: float = 1.9) -> Image.Image:
    """Recolor a texture: keep its luminance detail, remap to a base color."""
    lum = src.convert("L")
    out = Image.merge("RGB", [
        lum.point(lambda v, c=c: max(0, min(255, int(v * c * gain))))
        for c in rgb
    ])
    return out


def main() -> None:
    LUSH.mkdir(parents=True, exist_ok=True)

    # Grass: the original green grass, darkened for a "dark green" look.
    grass = Image.open(TEXTURES / "Grass_original.png").convert("RGB")
    grass = ImageEnhance.Brightness(grass).enhance(0.72)
    grass = ImageEnhance.Color(grass).enhance(1.25)
    grass = grass.resize((2048, 2048), Image.LANCZOS)
    grass.save(LUSH / "Grass.png")

    # Sand patches -> dark mossy green (luminance-preserving recolor).
    sand = Image.open(TEXTURES / "themes" / "desert" / "Sand.png").convert("RGB")
    colorize(sand, (0.16, 0.26, 0.11)).save(LUSH / "Sand.png")

    # Dirt paths -> dark wet earth.
    dirt = Image.open(TEXTURES / "themes" / "desert" / "DirtPath.png").convert("RGB")
    colorize(dirt, (0.30, 0.24, 0.16)).save(LUSH / "DirtPath.png")

    for f in sorted(LUSH.iterdir()):
        print(f"wrote {f} ({f.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
