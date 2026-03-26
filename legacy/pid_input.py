import runpy
from pathlib import Path

runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts" / "manual_target_input.py"), run_name="__main__")
