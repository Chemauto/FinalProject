"""技能注册表。

管理 SkillBase 实例和导航 model_use 码。
"""

from __future__ import annotations

from typing import Any

from .skill_base import SkillBase

# ── 技能实例注册 ──────────────────────────────────────────────────────

_REGISTRY: dict[str, SkillBase] = {}


def register_skill(skill: SkillBase) -> None:
    """注册技能实例。"""
    if not isinstance(skill, SkillBase):
        raise TypeError(f"Expected SkillBase instance, got {type(skill).__name__}")
    _REGISTRY[skill.name] = skill


def get_skill(name: str) -> SkillBase | None:
    """按名称查找已注册技能。"""
    return _REGISTRY.get(name)


def all_skills() -> dict[str, SkillBase]:
    """返回全部已注册技能。"""
    return dict(_REGISTRY)


# ── 导航 model_use 码 ────────────────────────────────────────────────

_NAVIGATION_MODEL_USES: set[int] = set()


def register_navigation_model_uses(codes: set[int]) -> None:
    """注册导航技能的 model_use 码。"""
    _NAVIGATION_MODEL_USES.update(codes)


def get_navigation_model_uses() -> set[int]:
    """返回导航 model_use 码集合。"""
    return set(_NAVIGATION_MODEL_USES)
