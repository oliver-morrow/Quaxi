from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from quaxi.navigation.traversal import NodeTraversal
from quaxi.runtime import short_term


if __name__ == "__main__":
    traversal = NodeTraversal()
    short_term.init()
    traversal.execute_path("PondsideAve.:QuackSt", 585, 135)
