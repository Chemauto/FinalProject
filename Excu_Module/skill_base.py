"""SkillBase abstract base class for action skills.

Every concrete skill inherits from SkillBase and provides:
  - name              : unique skill identifier
  - execute()         : async execution entry point
  - check_completion(): polling completion check (returns {"done": False} by default)
  - validate()        : post-execution state validation (returns None by default)
  - calculate_parameters(): parameter calculation from planning context (returns None by default)
  - register_tool()   : MCP tool registration
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class SkillBase(ABC):
    """Abstract base class for all action skills."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique skill identifier, e.g. 'walk', 'climb', 'push_box'."""
        ...

    @abstractmethod
    async def execute(self, **kwargs) -> dict[str, Any]:
        """Async execution entry point. Called by the MCP tool wrapper."""
        ...

    @abstractmethod
    def register_tool(self, mcp) -> dict[str, Any]:
        """Register this skill as an MCP tool. Returns {tool_name: tool_function}."""
        ...

    def check_completion(
        self,
        state: dict[str, Any],
        before_state: dict[str, Any],
        parameters: dict[str, Any],
    ) -> dict[str, Any]:
        """Polling completion check during skill execution.

        Override in subclasses to provide skill-specific completion detection.
        Default always returns {"done": False}, which means timeout-only behavior.
        """
        return {"done": False}

    def validate(
        self,
        parameters: dict[str, Any],
        before_state: dict[str, Any] | None,
        after_state: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        """Post-execution state validation.

        Override in subclasses to provide skill-specific validation.
        Default returns None, which triggers the generic missing-validation fallback.
        """
        return None

    def calculate_parameters(
        self,
        task: dict[str, Any],
        object_facts: dict[str, Any] | None,
        context: dict[str, Any],
    ) -> dict[str, Any] | None:
        """Parameter calculation from planning context.

        Override in subclasses to fill in concrete parameters (coordinates,
        distances, directions, etc.) for a planned task.

        ``context`` is a mutable dict shared across all tasks in the same plan.
        Skills may read/update it to track accumulated state (e.g. current pose).

        Returns None if the skill has no parameter calculation logic.
        """
        return None

    def normalize_tool_arguments(
        self,
        function_args: dict[str, Any],
        task_description: str,
        previous_result: Any,
    ) -> tuple[dict[str, Any], str | None]:
        """Skill-level argument correction before tool execution.

        Override in subclasses to implement skill-specific parameter repair
        (e.g. splitting a tall climb into relative steps).

        Returns (corrected_args, correction_message_or_None).
        Default passes arguments through unchanged.
        """
        return function_args, None
