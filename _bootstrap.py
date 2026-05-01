from pathlib import Path
import sys


def ensure_src_path() -> None:
    src_path = Path(__file__).resolve().parent / "src"
    src_str = str(src_path)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)
