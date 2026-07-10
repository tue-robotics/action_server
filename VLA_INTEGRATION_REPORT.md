# VLA Integration Report

## Scope

This report lists all files changed during the VLA integration work, including the interface layer, action delegation, and layout configurability updates.

It also explains why each file was changed.

## Repository Summary

- action_server: VLA interface layer, provider selection, task-level delegation, manipulation hooks.
- per-group-mse-vla: action layout and action dimension configurability for inference, evaluation, and training launchers.

## Files Changed in action_server

### Added

1. action_server/src/action_server/vla/__init__.py
   - Why:
   - Exposes public VLA integration helpers as a stable module boundary.
   - Defines explicit exports for VLA entrypoints used by actions and task manager.
   - Interfaces exposed:
   - maybe_run_vla_action
   - maybe_run_vla_manipulation
   - should_delegate_action_to_vla

2. action_server/src/action_server/vla/executor.py
   - Why:
   - Introduces a replaceable VLA backend interface and runtime configuration loading from ROS params.
   - Implements offline/local backend provider path for on-robot GPU inference.
   - Keeps no-op provider for safe fallback behavior.
   - Provides per-action VLA decision and strict vs fallback handling.
   - Key interfaces added:
   - VLAConfig
   - ManipulationRequest
   - ManipulationResponse
   - VLAOutcome
   - BaseVLAProvider.execute_manipulation
   - LocalBackendVLAProvider.execute_manipulation
   - load_vla_config
   - maybe_run_vla_action
   - maybe_run_vla_manipulation
   - should_delegate_action_to_vla

### Modified

3. action_server/setup.py
   - Why:
   - Registers new Python subpackages so catkin/distutils installs and resolves VLA modules correctly.
   - Added packages:
   - action_server.actions
   - action_server.actions.util
   - action_server.vla

4. action_server/src/action_server/task_manager.py
   - Why:
   - Adds generic action delegation to VLA in full vla mode.
   - Allows actions beyond predefined classic classes, including new custom action names.
   - Preserves classic path and fallback behavior.
   - Interfaces added internally:
   - TaskManager._VLADelegatedAction
   - TaskManager._instantiate_action

5. action_server/src/action_server/actions/pick_up.py
   - Why:
   - Adds optional VLA execution path for manipulation in hybrid mode.
   - Preserves classic FSM behavior if VLA is disabled or fallback occurs.
   - Captures raw semantics/context payloads for provider-agnostic VLA requests.

6. action_server/src/action_server/actions/place.py
   - Why:
   - Same integration pattern as pick_up.
   - Adds optional VLA-first path with classic fallback.

7. action_server/src/action_server/actions/hand_over.py
   - Why:
   - Same integration pattern as pick_up/place.
   - Allows VLA execution while keeping classic handover as fallback.

## Files Changed in per-group-mse-vla

### Modified

8. inference/policy_server.py
   - Why:
   - Adds configurable action layout presets and optional action dimension override.
   - Keeps default behavior unchanged for hsr11.
   - Enables arm-only output slicing for compatible checkpoints.
   - Interfaces added:
   - DEFAULT_ACTION_LAYOUT
   - ACTION_LAYOUT_TO_DIM
   - resolve_action_dim
   - CLI flags: --action-layout, --action-dim

9. eval/per_group_mse.py
   - Why:
   - Makes evaluation layout-aware for both hsr11 and arm5-style outputs.
   - Keeps current default metric behavior unchanged.
   - Interfaces added:
   - ACTION_LAYOUT_TO_GROUPS
   - resolve_action_groups
   - CLI flags: --action-layout, --action-dim

10. training/train_smolvla_generalist.py
    - Why:
    - Adds non-breaking hooks to configure future arm-only training runs.
    - Keeps default command path unchanged for hsr11.
    - Interfaces added:
    - ACTION_LAYOUT_TO_DIM
    - build_layout_overrides
    - CLI flags: --action-layout, --action-dim, --policy-override

11. training/train_smolvla_task.py
    - Why:
    - Same as generalist launcher, for task-specific top-up runs.
    - Keeps default hsr11 behavior unchanged.
    - Interfaces added:
    - ACTION_LAYOUT_TO_DIM
    - build_layout_overrides
    - CLI flags: --action-layout, --action-dim, --policy-override

## High-Level Rationale

The design intentionally separates concerns:

1. Provider interface layer:
   - Makes VLA backend replaceable without touching challenge/action logic.

2. Task-level delegation:
   - Supports both predefined and new custom actions in full vla mode.

3. Action-level hooks:
   - Enables safe hybrid rollout on high-impact manipulation actions.

4. Configuration-first behavior:
   - Classic remains default unless params enable VLA.
   - Strict mode can enforce hard-fail behavior when desired.

## Current Offline-Only Status

The action_server VLA path is currently offline-first:

- Supported providers:
  - local
  - none
- Removed provider path:
  - websocket

This matches on-robot GPU execution with a local backend class.
