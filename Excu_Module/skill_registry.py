"""Global skill registry for SkillBase instances.

Provides a central registry so that Excu_Module (execution, validation,
completion checks) and LLM_Module (parameter calculation) can look up
skill implementations without importing concrete skill classes.

Also provides pluggable hook registration for:
  - context hooks:  build shared planning context for parameter calculation
  - rule planners:  generate rule-based task plans (replacing hardcoded planners)
  - lowlevel prompt: project-specific lowlevel_prompt.yaml path
  - highlevel prompt: project-specific highlevel_prompt.yaml path
  - navigation model-use codes: which model_use values indicate navigation
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from .skill_base import SkillBase

# ── Skill registry ────────────────────────────────────────────────────

_REGISTRY: dict[str, SkillBase] = {}


def register_skill(skill: SkillBase) -> None:
    """Register a SkillBase instance under its ``name``."""
    if not isinstance(skill, SkillBase):
        raise TypeError(f"Expected SkillBase instance, got {type(skill).__name__}")
    _REGISTRY[skill.name] = skill


def get_skill(name: str) -> SkillBase | None:
    """Look up a registered skill by name. Returns None if not found."""
    return _REGISTRY.get(name)


def all_skills() -> dict[str, SkillBase]:
    """Return a shallow copy of the full registry."""
    return dict(_REGISTRY)


# ── Context hooks ─────────────────────────────────────────────────────

_CONTEXT_HOOKS: list[Callable] = []


def register_context_hook(hook: Callable) -> None:
    """Register a context-building hook for parameter calculation.

    ``hook`` signature: (object_facts, tasks) -> dict
    """
    _CONTEXT_HOOKS.append(hook)


def build_planning_context(object_facts: dict[str, Any], tasks: list[dict[str, Any]]) -> dict[str, Any]:
    """Run all registered context hooks and merge results into a single dict.

    If no hooks are registered, returns an empty dict.
    """
    context: dict[str, Any] = {}
    for hook in _CONTEXT_HOOKS:
        result = hook(object_facts, tasks)
        if isinstance(result, dict):
            context.update(result)
    return context


# ── Rule planners ─────────────────────────────────────────────────────

_RULE_PLANNERS: list[Callable] = []


def register_rule_planner(planner_fn: Callable) -> None:
    """Register a rule-based planner.

    ``planner_fn`` signature:
        (user_input, scene_facts, object_facts) -> tuple[tasks, meta] | None
    """
    _RULE_PLANNERS.append(planner_fn)


def try_rule_planners(
    user_input: str,
    scene_facts: dict[str, Any] | None,
    object_facts: dict[str, Any] | None,
) -> tuple[list[dict], dict[str, Any]] | None:
    """Try each registered rule planner in order; return the first non-None result."""
    for planner_fn in _RULE_PLANNERS:
        result = planner_fn(user_input, scene_facts, object_facts)
        if result is not None:
            return result
    return None


# ── Lowlevel prompt path ──────────────────────────────────────────────

_LOWLEVEL_PROMPT_PATH: Path | None = None


def register_lowlevel_prompt(prompt_path: Path) -> None:
    """Register a project-specific lowlevel_prompt.yaml path."""
    global _LOWLEVEL_PROMPT_PATH
    _LOWLEVEL_PROMPT_PATH = Path(prompt_path)


def get_lowlevel_prompt_path() -> Path | None:
    """Return the registered lowlevel prompt path, or None."""
    return _LOWLEVEL_PROMPT_PATH


# ── Navigation model-use codes ────────────────────────────────────────

_NAVIGATION_MODEL_USES: set[int] = set()


def register_navigation_model_uses(codes: set[int]) -> None:
    """Register which model_use integer codes indicate navigation skills."""
    _NAVIGATION_MODEL_USES.update(codes)


def get_navigation_model_uses() -> set[int]:
    """Return the set of navigation model_use codes."""
    return set(_NAVIGATION_MODEL_USES)


# ── Highlevel prompt path ─────────────────────────────────────────────

_HIGHLEVEL_PROMPT_PATH: Path | None = None


def register_highlevel_prompt(prompt_path: Path) -> None:
    """Register a project-specific highlevel_prompt.yaml path."""
    global _HIGHLEVEL_PROMPT_PATH
    _HIGHLEVEL_PROMPT_PATH = Path(prompt_path)


def get_highlevel_prompt_path() -> Path | None:
    """Return the registered highlevel prompt path, or None."""
    return _HIGHLEVEL_PROMPT_PATH
