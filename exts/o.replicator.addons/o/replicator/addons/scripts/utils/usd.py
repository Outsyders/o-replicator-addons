import typing
from pathlib import Path


def find_usd_assets(root: typing.Union[str, Path], extensions=["usd", "usdz"]):
    """Find USD and USDZ assets in the root directory and its subdirectories."""

    references = {}

    root = Path(root)
    assets = [p for ext in extensions for p in root.rglob(f"*.{ext}")]

    for asset in assets:
        print(f"Found asset: {asset.stem}")
        references[asset.stem] = asset.as_posix()

    return references
