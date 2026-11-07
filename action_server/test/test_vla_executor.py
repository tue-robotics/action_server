import unittest
from unittest import mock

from action_server.vla import executor


def make_config(**overrides):
    values = dict(execution_mode="hybrid", provider="local", enabled_actions={"pick-up"}, output_mode="joints",
                  action_dim=11, strict_mode=False, local_backend_class="module:Backend")
    values.update(overrides)
    return executor.VLAConfig(**values)


def failing_provider():
    provider = mock.MagicMock()
    provider.execute_manipulation.return_value = executor.ManipulationResponse(False, "max_chunks", None)
    return provider


class TestMaybeRunVlaManipulation(unittest.TestCase):
    def run_vla(self, provider, **config):
        robot = mock.MagicMock(robot_name="hero")
        with mock.patch.object(executor, "load_vla_config", return_value=make_config(**config)), \
                mock.patch.object(executor, "_provider_for", return_value=provider):
            return executor.maybe_run_vla_manipulation(robot, "pick-up", {}, {})

    def test_hybrid_falls_back_to_classic_after_a_failure(self):
        self.assertFalse(self.run_vla(failing_provider()).used)

    def test_strict_mode_reports_the_failure(self):
        outcome = self.run_vla(failing_provider(), strict_mode=True)
        self.assertTrue(outcome.used)
        self.assertFalse(outcome.succeeded)

    def test_vla_mode_never_falls_back_to_classic(self):
        outcome = self.run_vla(failing_provider(), execution_mode="vla")
        self.assertTrue(outcome.used)
        self.assertFalse(outcome.succeeded)
        self.assertEqual(outcome.message, "max_chunks")

    def test_classic_mode_does_not_run_the_vla(self):
        provider = failing_provider()
        self.assertFalse(self.run_vla(provider, execution_mode="classic").used)
        provider.execute_manipulation.assert_not_called()


class TestCancelVla(unittest.TestCase):
    def provider(self, robot_name):
        provider = executor.LocalBackendVLAProvider(mock.MagicMock(robot_name=robot_name), make_config())
        provider._backend = mock.MagicMock()
        return provider

    def test_cancel_reaches_the_backend_of_that_robot_only(self):
        hero, amigo = self.provider("hero"), self.provider("amigo")
        with mock.patch.dict(executor._PROVIDER_CACHE, {("hero", "local"): hero, ("amigo", "local"): amigo},
                             clear=True):
            executor.cancel_vla("hero")
        hero._backend.cancel.assert_called_once_with()
        amigo._backend.cancel.assert_not_called()

    def test_cancel_without_a_backend_is_harmless(self):
        idle = executor.LocalBackendVLAProvider(mock.MagicMock(robot_name="hero"), make_config())
        with mock.patch.dict(executor._PROVIDER_CACHE, {("hero", "local"): idle, ("hero", "none"):
                             executor.NullVLAProvider()}, clear=True):
            executor.cancel_vla("hero")


if __name__ == '__main__':
    unittest.main()
