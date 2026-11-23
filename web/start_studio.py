"""Launch the extracted Vin Trajectory Planner offline workbench."""
from pathlib import Path
import sys

if __name__ == "__main__":
    directory = Path(__file__).resolve().parent
    sys.path.insert(0, str(directory / "python"))
    try:
        from mission_planner.research_studio import main
    except ModuleNotFoundError as exc:
        if exc.name != "numpy":
            raise
        raise SystemExit("Install NumPy once with: python -m pip install numpy") from exc
    main(default_assets=directory)
