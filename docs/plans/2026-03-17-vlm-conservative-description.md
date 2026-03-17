# VLM Conservative Description Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make `VLM_Module` describe scene geometry conservatively without asserting passability, and make outputs more stable across repeated runs.

**Architecture:** Tighten the prompt so the model reports visible facts and uncertainty instead of navigation conclusions. Lower the default sampling temperature in `VLM_Module/vlm_core.py`, and add focused unit tests that verify prompt loading and request construction without calling the remote API.

**Tech Stack:** Python, `unittest`, `openai`, `pyyaml`

---

### Task 1: Add regression tests for conservative prompt behavior

**Files:**
- Create: `tests/test_vlm_core.py`
- Test: `tests/test_vlm_core.py`

**Step 1: Write the failing test**

Write tests that verify:
- the loaded system prompt contains conservative instructions such as not directly judging passability
- `VLMCore.describe()` sends a low default temperature to the API client

**Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests/test_vlm_core.py`

Expected: FAIL because the current prompt still asks for walkable areas and the current default temperature is `0.3`.

**Step 3: Write minimal implementation**

Update the prompt file and `VLMCore` constructor/request code so the tests can pass.

**Step 4: Run test to verify it passes**

Run: `python3 -m unittest tests/test_vlm_core.py`

Expected: PASS

### Task 2: Tighten the VLM prompt and stabilize output

**Files:**
- Modify: `VLM_Module/prompts/VlmPrompt.yaml`
- Modify: `VLM_Module/vlm_core.py`

**Step 1: Tighten prompt wording**

Change the prompt to:
- describe only visible geometry, obstacles, relative positions, and uncertainty
- avoid phrases like “可安全通行”“可直接通过” unless visually certain
- explicitly treat platforms, steps, or raised surfaces as possible height differences, not flat ground

**Step 2: Reduce default sampling temperature**

Lower the default temperature used in `describe()` from `0.3` to a more stable value.

**Step 3: Run focused tests**

Run: `python3 -m unittest tests/test_vlm_core.py`

Expected: PASS

### Task 3: Run a broader verification pass

**Files:**
- Test: `tests/test_vlm_core.py`

**Step 1: Run all local tests**

Run: `python3 -m unittest discover -s tests`

Expected: PASS for all discovered tests.
