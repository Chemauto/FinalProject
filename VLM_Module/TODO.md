# VLM TODO

## Current Direction

The agreed architecture is:

- `VLM` only reports what is visible in the image.
- The rule layer or planner decides whether the robot can walk directly, needs to avoid, or needs elevation handling.

This means `VLM` should not directly output conclusions such as:

- "can pass safely"
- "can walk directly"
- "needs climb"

Those are downstream decisions, not visual facts.

## Responsibility Split

### VLM responsibility

`VLM_Module` should only produce structured scene facts, for example:

- ground or base plane description
- left-side structures
- right-side structures
- front-area structures
- visible obstacles
- suspected height-difference cues
- explicit uncertainties

### Rule-layer responsibility

A future rule module, tentatively `LLM_Module/visual_rules.py`, should consume the structured VLM output and produce motion-oriented judgments.

Examples of rule-layer outputs:

- `direct_walkable`
- `left_walk_blocked`
- `right_walk_blocked`
- `requires_elevation_action`

Important:

- `blocked` here means "not usable by pure walking", not necessarily "physically impossible forever".
- `requires_elevation_action` means the scene likely needs height handling, not that `climb` is already proven executable.

## Candidate Integration Path

Recommended future integration:

1. `VLM_Module.vlm_core.VLMCore.describe_structured()` returns structured visual facts.
2. Add `LLM_Module/visual_rules.py`:
   - input: VLM structured dict
   - output A: structured rule result
   - output B: stable summary text for current prompt compatibility
3. Update `Interactive_Module/interactive.py` to:
   - call `describe_structured()`
   - pass the result through the rule layer
   - log both raw VLM facts and rule-layer summary
4. Update `LLM_Module/llm_highlevel.py` fallback logic to rely on rule-layer results instead of keyword matching over free-form prose.

## Why This Matters

This separation should improve:

- stability of planning inputs
- interpretability of failures
- consistency across repeated tests
- reduction of VLM over-claiming about passability

## Non-Goals

This TODO does not imply that RGB-only VLM becomes geometrically precise.

Even with structured output:

- exact height is still uncertain from a single RGB frame
- exact traversability still needs rules, geometry, depth, or simulator state
- rule outputs should remain conservative when evidence is weak

## Status

- Structured VLM JSON output: started
- Rule-layer module: not implemented yet
- Planner integration with rule-layer output: not implemented yet
