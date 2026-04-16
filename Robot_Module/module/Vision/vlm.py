"""Compatibility wrapper for Bishe vision skills."""

from __future__ import annotations

if __package__:
    from .Task.Bishe.registry import register_tools
    from .Task.Bishe.vlm_observe import execute_vlm_observe
else:  # pragma: no cover
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from module.Vision.Task.Bishe.registry import register_tools
    from module.Vision.Task.Bishe.vlm_observe import execute_vlm_observe


__all__ = ["execute_vlm_observe", "register_tools"]
