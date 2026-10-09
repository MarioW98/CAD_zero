# ADR-0006: PySide6 as primary UI

## Status
Accepted (2025-10-07)

## Context
A CAD desktop app needs a robust GUI toolkit with:
- Mature widget library (panels, trees, property browsers)
- OpenGL/Vulkan viewport integration
- Cross-platform binary distribution
- Python scripting hooks

## Decision
**PySide6 (Python) is the primary UI**. The Qt6 C++ path is removed
from the repository (no `app/src/` directory); the historical
"optional Qt6 C++ not in CI" route is dropped because unmaintained
code rots.

### Rationale
- PySide6 ships official Qt bindings for Python, with commercial
  licensing compatible with most users.
- Python is already the scripting language (Phase A bindings).
  Using the same language for UI and scripting eliminates the
  impedance mismatch.
- The viewport will use Qt's `QWindow`/`QWidget` OpenGL surface;
  Vulkan integration is a Phase G+ goal.

## Consequences
- The Python package gains an optional dependency: `PySide6>=6.6`.
- The desktop app is packaged via Briefcase or PyInstaller (Phase C).
- Headless mode (CLI, batch) does not require PySide6.
- The `app/` directory contains only `resources/` (icons, themes,
  fonts); no C++ code.
