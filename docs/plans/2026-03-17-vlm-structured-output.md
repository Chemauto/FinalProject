# VLM Structured Output Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Change `VLM_Module` to return structured JSON scene facts instead of free-form prose.

**Architecture:** Keep the remote VLM call, but tighten the prompt and add a normalization layer in `VLMCore`. Expose `describe_structured()` for Python callers and keep `describe()` as a JSON-string wrapper for compatibility with current string-based call sites.

**Tech Stack:** Python, `unittest`, `openai`, `pyyaml`, `json`

---

### Task 1: Add failing tests for structured VLM output

**Files:**
- Modify: `tests/test_vlm_core.py`
- Test: `tests/test_vlm_core.py`

**Step 1: Write the failing test**

Add tests that verify:
- prompt text requests JSON-only structured factual output
- `describe_structured()` parses fenced JSON and returns normalized keys
- `describe()` returns a JSON string, not prose

**Step 2: Run test to verify it fails**

Run: `python3 -m unittest discover -s tests -p 'test_vlm_core.py'`

Expected: FAIL because the current prompt is still prose-focused and `VLMCore` does not expose structured parsing yet.

**Step 3: Write minimal implementation**

Update the prompt and `VLMCore` parsing/serialization logic so the tests pass.

**Step 4: Run test to verify it passes**

Run: `python3 -m unittest discover -s tests -p 'test_vlm_core.py'`

Expected: PASS

### Task 2: Implement structured output in VLM

**Files:**
- Modify: `VLM_Module/prompts/VlmPrompt.yaml`
- Modify: `VLM_Module/vlm_core.py`

**Step 1: Tighten the prompt**

Require strict JSON output with the fixed schema and no passability judgments.

**Step 2: Add normalization logic**

Implement helpers that:
- strip markdown fences
- parse JSON safely
- fill missing keys with defaults
- coerce `obstacles` and `uncertainties` into lists
- coerce `suspected_height_diff` into a boolean

**Step 3: Preserve compatibility**

Keep `describe()` returning a string by serializing the normalized dict with `json.dumps(..., ensure_ascii=False)`.

**Step 4: Run focused tests**

Run: `python3 -m unittest discover -s tests -p 'test_vlm_core.py'`

Expected: PASS

### Task 3: Update docs and run broader verification

**Files:**
- Modify: `VLM_Module/README.md`
- Test: `tests/test_vlm_core.py`

**Step 1: Update README**

Document the new structured JSON output and provide a direct test command.

**Step 2: Run all tests**

Run: `python3 -m unittest discover -s tests`

Expected: PASS
