"""Regenerates heightmap.png (optional - the PNG is already in the project).

Usage:  python3 make_heightmap.py [seed]
Needs:  numpy, pillow

Any greyscale image works as a heightmap; this script is just one way to
make one. Black = low ground, white = high ground. 129x129 pixels gives a
128 m x 128 m terrain (one pixel per metre of grid).
"""
import sys

import numpy as np
from PIL import Image

SIZE = 129


def smooth_noise(rng: np.random.Generator, cells: int) -> np.ndarray:
    """Random values on a coarse grid, smoothly interpolated up to SIZE."""
    grid = rng.random((cells + 1, cells + 1))
    t = np.linspace(0, cells, SIZE)
    i = np.minimum(t.astype(int), cells - 1)
    f = t - i
    f = f * f * (3 - 2 * f)  # smoothstep, removes creases at cell borders
    rows = grid[i] * (1 - f)[:, None] + grid[i + 1] * f[:, None]
    return rows[:, i] * (1 - f)[None, :] + rows[:, i + 1] * f[None, :]


def main() -> None:
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 361
    rng = np.random.default_rng(seed)

    # Broad hills plus two finer layers of detail.
    height = (1.00 * smooth_noise(rng, 3)
              + 0.25 * smooth_noise(rng, 8)
              + 0.06 * smooth_noise(rng, 20))
    height = (height - height.min()) / (height.max() - height.min())

    Image.fromarray(np.round(height * 255).astype(np.uint8), mode="L").save("heightmap.png")
    print(f"wrote heightmap.png ({SIZE}x{SIZE}, seed {seed})")


if __name__ == "__main__":
    main()
