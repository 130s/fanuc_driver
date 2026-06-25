import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("stmo_recovery.py")
spec = importlib.util.spec_from_file_location("stmo_recovery", MODULE_PATH)
stmo_recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stmo_recovery)


class StmoRecoveryTest(unittest.TestCase):
    def test_detects_stmo_inactive_warning(self) -> None:
        self.assertTrue(
            stmo_recovery.should_trigger_recovery(
                "[WARN] Aborted: STMO is inactive (!motion_possible)"
            )
        )
        self.assertFalse(stmo_recovery.should_trigger_recovery("normal log line"))

    def test_formats_log_entry(self) -> None:
        line = stmo_recovery.format_log_entry("2026-06-24T12:34:56", 3)
        self.assertIn("2026-06-24T12:34:56", line)
        self.assertIn("count=3", line)

    def test_formats_final_report(self) -> None:
        report = stmo_recovery.format_final_report(5)
        self.assertIn("5", report)
        self.assertIn("recoveries", report.lower())

    def test_service_cmd_issues_status_one(self) -> None:
        # The known-good workaround is `status: 1` on the switch_control_state service.
        self.assertIn("/fanuc_gpio_controller/switch_control_state", stmo_recovery.SERVICE_CMD)
        self.assertIn("status: 1", stmo_recovery.SERVICE_CMD)


if __name__ == "__main__":
    unittest.main()
