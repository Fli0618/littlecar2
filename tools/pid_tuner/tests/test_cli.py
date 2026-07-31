import unittest

from pid_tuner.cli import build_parser, main


class CliTests(unittest.TestCase):
    def test_set_pid_requires_apply_for_dry_run(self) -> None:
        result = main(["set-pid", "--port", "COM4", "--kp-pos", "1", "--ki-pos", "0",
                       "--kd-pos", "0", "--kp-yaw", "1", "--ki-yaw", "0", "--kd-yaw", "0"])
        self.assertEqual(result, 0)

    def test_goto_requires_all_motion_arguments(self) -> None:
        with self.assertRaises(SystemExit):
            build_parser().parse_args(["goto", "--port", "COM4", "--x", "1"])

    def test_goto_accepts_yaw_isolation_switch(self) -> None:
        args = build_parser().parse_args(["goto", "--port", "COM4", "--x", "100", "--y", "0",
                                          "--yaw", "0", "--vmax", "300", "--wmax", "60",
                                          "--timeout", "3000", "--no-yaw"])
        self.assertTrue(args.no_yaw)

    def test_profile_save_from_device_requires_port(self) -> None:
        result = main(["profile", "save", "baseline", "--from-device"])
        self.assertEqual(result, 2)
