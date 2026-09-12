from __future__ import annotations

import importlib
import traceback
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

import rospy


@dataclass
class VLAConfig:
    execution_mode: str
    provider: str
    enabled_actions: Set[str]
    output_mode: str
    action_dim: int
    strict_mode: bool
    local_backend_class: str


@dataclass
class ManipulationRequest:
    robot_name: str
    action_name: str
    semantics: Dict
    context: Dict
    output_mode: str
    action_dim: int
    raw_sentence: str = ""


@dataclass
class ManipulationResponse:
    succeeded: bool
    message: str
    joint_trajectory: Optional[List[List[float]]] = None


@dataclass
class VLAOutcome:
    used: bool
    succeeded: bool
    message: str


class BaseVLAProvider(object):
    """Interface for replaceable VLA backends.

    Concrete providers can wrap a local policy, a ROS action bridge, WebSocket
    transport, or any other runtime.
    """

    def execute_manipulation(self, request: ManipulationRequest) -> ManipulationResponse:
        raise NotImplementedError("Implement execute_manipulation in provider")


class NullVLAProvider(BaseVLAProvider):
    """Default provider that intentionally does nothing.

    This keeps the classic pipeline active unless a real provider is configured.
    """

    def execute_manipulation(self, request: ManipulationRequest) -> ManipulationResponse:
        return ManipulationResponse(
            succeeded=False,
            message="VLA provider not configured",
            joint_trajectory=None,
        )


class LocalBackendVLAProvider(BaseVLAProvider):
    """In-process provider for on-robot policy execution without network transport."""

    def __init__(self, robot, config: VLAConfig):
        self._robot = robot
        self.robot_name = robot.robot_name
        self._config = config
        self._backend = None

    @staticmethod
    def _load_class(class_path: str):
        if ":" in class_path:
            module_name, class_name = class_path.split(":", 1)
        else:
            module_name, class_name = class_path.rsplit(".", 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)

    def _backend_instance(self):
        if self._backend is not None:
            return self._backend
        if not self._config.local_backend_class:
            return None

        backend_cls = self._load_class(self._config.local_backend_class)
        # Backends run in-process and need the live robot object for I/O (cameras,
        # joint states, controllers) through the robot_skills interfaces.
        self._backend = backend_cls(robot=self._robot)
        return self._backend

    @staticmethod
    def _normalize_response(result, action_dim):
        if not isinstance(result, dict):
            return ManipulationResponse(False, "Local backend must return dict", None)

        actions = result.get("actions") or result.get("joint_trajectory") or []
        trajectory = []
        for step in actions:
            if isinstance(step, (list, tuple)):
                trajectory.append([float(v) for v in step[:action_dim]])

        succeeded = bool(result.get("succeeded", bool(trajectory)))
        message = str(result.get("message", ""))
        return ManipulationResponse(succeeded=succeeded, message=message, joint_trajectory=trajectory or None)

    def execute_manipulation(self, request: ManipulationRequest) -> ManipulationResponse:
        try:
            backend = self._backend_instance()
            if backend is None:
                return ManipulationResponse(False, "No local backend class configured", None)

            if hasattr(backend, "execute"):
                result = backend.execute(request)
            elif hasattr(backend, "predict"):
                result = backend.predict(request)
            else:
                return ManipulationResponse(False, "Local backend missing execute/predict method", None)

            return self._normalize_response(result, request.action_dim)
        except Exception as e:
            rospy.logerr("[VLA] Backend traceback:\n%s", traceback.format_exc())
            return ManipulationResponse(False, "Local backend call failed: {}".format(e), None)


_PROVIDER_REGISTRY = {
    "none": lambda robot, config: NullVLAProvider(),
    "local": lambda robot, config: LocalBackendVLAProvider(robot, config),
}

# Providers (and the VLA model they load) are expensive to construct, so cache
# one live instance per robot to avoid reloading the policy on every action.
_PROVIDER_CACHE = {}  # type: Dict[tuple, BaseVLAProvider]


def _get_param_with_fallback(names: List[str], default):
    for name in names:
        if rospy.has_param(name):
            return rospy.get_param(name)
    return default


def load_vla_config(robot_name: str) -> VLAConfig:
    base = "/{}/action_server/vla".format(robot_name)

    execution_mode = _get_param_with_fallback(
        ["~execution_mode", base + "/execution_mode", "/action_server/execution_mode"],
        "classic",
    )
    provider = _get_param_with_fallback(
        [base + "/provider", "/action_server/vla/provider"],
        "none",
    )
    enabled_actions = _get_param_with_fallback(
        [base + "/enabled_actions", "/action_server/vla/enabled_actions"],
        ["pick-up", "place", "hand-over"],
    )
    output_mode = _get_param_with_fallback(
        [base + "/output_mode", "/action_server/vla/output_mode"],
        "joints",
    )
    action_dim = _get_param_with_fallback(
        [base + "/action_dim", "/action_server/vla/action_dim"],
        11,
    )
    strict_mode = bool(_get_param_with_fallback(
        [base + "/strict_mode", "/action_server/vla/strict_mode"],
        False,
    ))
    local_backend_class = _get_param_with_fallback(
        [base + "/local_backend_class", "/action_server/vla/local_backend_class"],
        "",
    )

    return VLAConfig(
        execution_mode=str(execution_mode),
        provider=str(provider),
        enabled_actions=set(enabled_actions),
        output_mode=str(output_mode),
        action_dim=int(action_dim),
        strict_mode=strict_mode,
        local_backend_class=str(local_backend_class),
    )


def _provider_for(robot, provider_name: str, config: VLAConfig) -> BaseVLAProvider:
    """A single live provider instance per robot is cached to avoid reloading the model on every action."""
    cache_key = (robot.robot_name, provider_name)
    cached = _PROVIDER_CACHE.get(cache_key)
    if cached is not None:
        return cached

    factory = _PROVIDER_REGISTRY.get(provider_name)
    if not factory:
        rospy.logwarn("[VLA] Unknown provider '%s', falling back to 'none'", provider_name)
        factory = _PROVIDER_REGISTRY["none"]
    provider = factory(robot, config)
    _PROVIDER_CACHE[cache_key] = provider
    return provider


def _is_vla_enabled_for_action(config: VLAConfig, action_name: str) -> bool:
    if config.execution_mode not in ("vla", "hybrid"):
        return False
    if "*" in config.enabled_actions:
        return True
    return action_name in config.enabled_actions


def maybe_run_vla_action(
    robot, action_name: str, semantics: Dict, context: Dict, raw_sentence: str = ""
) -> VLAOutcome:
    """Try VLA execution for one action.

    Returns VLAOutcome(used=False, ...) when classic code should continue.
    """
    config = load_vla_config(robot.robot_name)
    if not _is_vla_enabled_for_action(config, action_name):
        return VLAOutcome(used=False, succeeded=False, message="")

    if config.output_mode != "joints":
        msg = "[VLA] Unsupported output_mode '{}', expected 'joints'".format(config.output_mode)
        rospy.logwarn(msg)
        if config.strict_mode:
            return VLAOutcome(used=True, succeeded=False, message=msg)
        return VLAOutcome(used=False, succeeded=False, message="")

    request = ManipulationRequest(
        robot_name=robot.robot_name,
        action_name=action_name,
        semantics=semantics,
        context=context,
        output_mode=config.output_mode,
        action_dim=config.action_dim,
        raw_sentence=raw_sentence,
    )
    provider = _provider_for(robot, config.provider, config)
    response = provider.execute_manipulation(request)

    if response.succeeded:
        msg = response.message or "VLA manipulation executed successfully."
        return VLAOutcome(used=True, succeeded=True, message=msg)

    if config.strict_mode:
        msg = response.message or "VLA manipulation failed."
        return VLAOutcome(used=True, succeeded=False, message=msg)

    rospy.logwarn("[VLA] Provider returned failure, using classic fallback: %s", response.message)
    return VLAOutcome(used=False, succeeded=False, message=response.message)


def maybe_run_vla_manipulation(
    robot, action_name: str, semantics: Dict, context: Dict, raw_sentence: str = ""
) -> VLAOutcome:
    """Backward-compatible wrapper for manipulation actions."""
    return maybe_run_vla_action(
        robot, action_name, semantics, context, raw_sentence=raw_sentence
    )


def should_delegate_action_to_vla(robot_name: str, action_name: str) -> bool:
    config = load_vla_config(robot_name)
    if config.execution_mode != "vla":
        return False
    return _is_vla_enabled_for_action(config, action_name)
