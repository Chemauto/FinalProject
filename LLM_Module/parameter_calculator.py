from __future__ import annotations

from typing import Any


class ParameterCalculator:

    def annotate_tasks(
        self,
        tasks: list[dict[str, Any]],
        object_facts: dict[str, Any] | None,
    ) -> list[dict[str, Any]]:
        if not tasks or not object_facts:
            return tasks

        from Excu_Module.skill_registry import all_skills, build_planning_context

        context = build_planning_context(object_facts, tasks)
        skills = all_skills()
        annotated_tasks = []

        for task in tasks:
            annotated = dict(task)
            function_name = task.get("function")
            parameter_context = dict(task.get("parameter_context") or {})

            skill = skills.get(function_name)
            if skill is not None:
                calculated = skill.calculate_parameters(task, object_facts, context)
                if calculated is not None:
                    navigation_goal = context.get("navigation_goal")
                    current_pose = context.get("current_pose", [0.0, 0.0, 0.0])
                    if navigation_goal:
                        parameter_context.update(
                            {
                                "start_pose_xyz": [round(v, 3) for v in current_pose],
                                "goal_pose_xyz": navigation_goal,
                            }
                        )
                    annotated["parameter_context"] = parameter_context
                    annotated["calculated_parameters"] = calculated

            annotated_tasks.append(annotated)

        return annotated_tasks
