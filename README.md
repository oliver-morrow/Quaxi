# Quaxi

Refactored into a conventional Python project layout for robotics and ML-adjacent work.

## Layout

- `src/quaxi/`: installable application package
- `src/quaxi/sensors/`: AI and hardware sensor adapters
- `src/quaxi/perception/`: classical CV code
- `src/quaxi/control/`: low-level control and behavior logic
- `src/quaxi/navigation/`: path planning and traversal
- `src/quaxi/runtime/`: runtime loops and executable flows
- `src/quaxi/hardware/`: direct hardware integrations
- `src/quaxi/config/`: static map and routing data
- `scripts/`: runnable entry-point scripts
- `models/`: deployed model artifacts

## Run

For packaged execution:

```bash
pip install -e .
python scripts/run_short_term.py
```

Legacy top-level module names are still present as compatibility shims.
