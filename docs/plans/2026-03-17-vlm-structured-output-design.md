# VLM Structured Output Design

**Goal:** Make `VLM_Module` output structured scene facts instead of free-form prose so downstream rule/planning code can decide passability.

**Scope:** Only change `VLM_Module` and its tests/docs in this step. Do not change the planner or execution logic yet.

## Decision

`VLM` will report only visible facts and uncertainty. It will not decide whether the robot can walk, climb, or pass through an area.

## Output Shape

`VLMCore.describe()` will return a JSON string with a fixed schema:

```json
{
  "ground": "string",
  "left_side": "string",
  "right_side": "string",
  "front_area": "string",
  "obstacles": ["string"],
  "suspected_height_diff": true,
  "uncertainties": ["string"]
}
```

The values are intentionally coarse but machine-readable:
- `ground`: visible ground or base plane description
- `left_side`: major structures on the left
- `right_side`: major structures on the right
- `front_area`: what is directly ahead
- `obstacles`: visible blocking or raised structures
- `suspected_height_diff`: conservative boolean flag for any visible platform/step/raised surface cues
- `uncertainties`: explicit list of what cannot be confirmed from the image alone

## Prompting Strategy

The prompt should force JSON-only output and forbid passability judgments. It should require the model to say `unknown` or add an uncertainty item instead of guessing.

## Parsing Strategy

`VLMCore` should parse the model output, strip fenced code blocks if needed, normalize missing keys, and return a complete schema every time.

If parsing fails, the module should raise a clear error rather than silently returning prose, because downstream code now depends on structure.

## Verification

Tests should verify:
- prompt wording requires JSON-only factual output
- `describe_structured()` normalizes wrapped JSON responses
- `describe()` returns a JSON string and still uses low sampling temperature
