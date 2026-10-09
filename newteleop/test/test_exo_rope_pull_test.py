"""无需舵机的故障路径测试：python -m unittest discover -s ... -p test_exo_rope_pull_test.py"""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import exo_rope_pull_test as pull


class FakeServo:
    def __init__(self):
        self.packet = self
        self.port = None
        self.sid = 7
        self.reg = {11: 5, 64: 0, 38: 300, 98: 0, 102: 0}
        self.writes = []
        self.fail_enable = False
        self.fail_zero = False

    def ping(self, *args):
        return 1200, 0, 0

    def check(self, result, error):
        assert result == error == 0

    def read(self, address, size=1):
        return self.reg[address]

    def write(self, address, value, size=1):
        self.writes.append((address, value))
        self.reg[address] = value
        if address == 64 and value == 1 and self.fail_enable:
            raise RuntimeError('enable应答丢失但已上电')
        if address == 102 and value == 0 and self.fail_zero:
            self.fail_zero = False
            raise RuntimeError('清零应答丢失')

    def sample(self):
        return 100, 0, 25, 5.0, 0


class PullTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.a = pull.parse_args(['--id', '7', '--current', '30', '--enable-write',
                                  '--csv', str(Path(self.tmp.name) / 'log.csv')])
        self.servo = FakeServo()
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def test_readonly_and_cancel_never_write(self):
        self.a.enable_write = False
        pull.run(self.a, self.servo)
        self.a.enable_write = True
        pull.run(self.a, self.servo, lambda _: 'cancel')
        self.assertEqual(self.servo.writes, [])

    def test_refuse_active_servo_without_cleanup_writes(self):
        self.servo.reg[64] = 1
        with self.assertRaises(RuntimeError):
            pull.run(self.a, self.servo)
        self.assertEqual(self.servo.writes, [])

    def test_refuse_hardware_limit(self):
        self.a.current = 301
        with self.assertRaises(RuntimeError):
            pull.run(self.a, self.servo)
        self.assertEqual(self.servo.writes, [])

    def test_enable_ack_failure_still_disables_torque(self):
        self.servo.fail_enable = True
        with self.assertRaises(RuntimeError):
            pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[64], 0)
        self.assertEqual(self.servo.reg[102], 0)

    def test_zero_failure_does_not_skip_torque_off(self):
        self.servo.fail_zero = True
        pull.release(self.servo)
        self.assertIn((64, 0), self.servo.writes)

    def test_success_ramps_holds_and_releases(self):
        with patch.object(pull.time, 'monotonic', side_effect=[0, 0, 0, 2, 2, 7]), \
                patch.object(pull.time, 'sleep'):
            result = pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertTrue(result.exists())
        self.assertIn((102, 30), self.servo.writes)
        self.assertEqual(self.servo.reg[64], 0)
        self.assertEqual(self.servo.reg[102], 0)

    def test_interrupt_releases(self):
        with patch.object(pull.time, 'monotonic', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[64], 0)

    def test_each_protection(self):
        for sample in [(300, 0, 25, 5, 0), (100, 60, 25, 5, 0),
                       (100, 0, 55, 5, 0), (100, 0, 25, 5, 4)]:
            with self.subTest(sample=sample), self.assertRaises(RuntimeError):
                pull.check_sample(self.a, sample, 100)

    def test_max_resolves_hardware_limit_in_both_directions(self):
        for value, expected in [('max', 300), ('-max', -300)]:
            with self.subTest(value=value):
                a = pull.parse_args(['--id', '7', f'--current={value}'])
                pull.run(a, self.servo)
                self.assertEqual(a.current, expected)
                self.assertEqual(a.actual_current_limit, 330)
        self.assertEqual(self.servo.writes, [])

    def test_max_rejects_insufficient_actual_limit_without_writes(self):
        a = pull.parse_args(['--id', '7', '--current', 'max',
                            '--actual-current-limit', '200', '--enable-write'])
        with self.assertRaises(RuntimeError):
            pull.run(a, self.servo)
        self.assertEqual(self.servo.writes, [])

    def test_long_hold_runs_past_old_limit_and_releases(self):
        self.a.duration = 30
        with patch.object(pull.time, 'monotonic', side_effect=[0, 2, 2, 20, 20, 32]), \
                patch.object(pull.time, 'sleep'):
            path = pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertIn('20,hold', path.read_text())
        self.assertEqual(self.servo.reg[64], 0)
        self.assertEqual(self.servo.reg[102], 0)

    def test_duration_range(self):
        a = pull.parse_args(['--id', '7', '--current', 'max', '--duration', '120'])
        self.assertEqual(a.duration, 120)
        self.assertEqual(pull.parse_args(['--id', '7', '--current', '30', '--duration', '0']).duration, 0)
        for value in ['121', 'nan', 'inf', '-1', '0.01']:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                pull.parse_args(['--id', '7', '--current', 'max', '--duration', value])

    def test_unlimited_holds_past_120_and_manual_stop_releases(self):
        self.a.duration = 0
        with patch.object(pull.time, 'monotonic', side_effect=[0, 2, 2, 500, 500, KeyboardInterrupt]), \
                patch.object(pull.time, 'sleep'):
            path = pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertIn('500,hold', path.read_text())
        self.assertEqual(self.servo.reg[64], 0)
        self.assertEqual(self.servo.reg[102], 0)

    def test_unlimited_still_checks_travel(self):
        self.a.duration = 0
        with patch.object(self.servo, 'sample', side_effect=[(100, 0, 25, 5, 0),
                          (100, 0, 25, 5, 0), (301, 0, 25, 5, 0)]):
            with self.assertRaisesRegex(RuntimeError, '位移'):
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[64], 0)

    def test_sigterm_releases_without_force_prompt(self):
        self.a.duration = 0
        with patch.object(pull.time, 'monotonic', side_effect=lambda: pull.stop_signal(15, None)):
            with self.assertRaises(SystemExit) as ctx:
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(ctx.exception.code, 143)
        self.assertEqual(self.servo.reg[64], 0)

    def configure_600(self):
        self.a.current = 600
        self.a.current_limit = 600
        self.a.actual_current_limit = 630

    def test_600_output_and_restore_after_torque_off(self):
        self.configure_600()
        with patch.object(pull.time, 'monotonic', side_effect=[0, 2, 2, 7]), \
                patch.object(pull.time, 'sleep'):
            pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertIn((102, 600), self.servo.writes)
        self.assertLess(self.servo.writes.index((38, 600)), self.servo.writes.index((64, 1)))
        self.assertLess(self.servo.writes.index((64, 0)), self.servo.writes.index((38, 300)))
        self.assertEqual(self.servo.reg[38], 300)

    def test_limit_ack_failure_still_restores(self):
        self.configure_600()
        original_write = self.servo.write
        def fail_ack(address, value, size=1):
            original_write(address, value, size)
            if address == 38 and value == 600:
                raise RuntimeError('EEPROM应答丢失')
        with patch.object(self.servo, 'write', side_effect=fail_ack):
            with self.assertRaisesRegex(RuntimeError, 'EEPROM'):
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[38], 300)
        self.assertNotIn((64, 1), self.servo.writes)

    def test_undervoltage_stops_and_restores(self):
        self.configure_600()
        with patch.object(self.servo, 'sample', side_effect=[(100, 0, 25, 5, 0),
                          (100, 0, 25, 5, 0), (100, 600, 25, 3.6, 0)]):
            with self.assertRaisesRegex(RuntimeError, '电压过低'):
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[38], 300)
        self.assertEqual(self.servo.reg[64], 0)

    def test_limit_change_readonly_or_cancel_no_writes(self):
        self.configure_600()
        self.a.enable_write = False
        pull.run(self.a, self.servo)
        self.a.enable_write = True
        pull.run(self.a, self.servo, lambda _: 'cancel')
        self.assertEqual(self.servo.writes, [])

    def test_restore_failure_is_reported(self):
        self.configure_600()
        original_write = self.servo.write
        def fail_restore(address, value, size=1):
            if address == 38 and value == 300:
                raise RuntimeError('恢复通信失败')
            original_write(address, value, size)
        with patch.object(self.servo, 'write', side_effect=fail_restore), \
                patch.object(pull.time, 'monotonic', side_effect=[0, 7]), \
                patch.object(pull.time, 'sleep'):
            with self.assertRaisesRegex(RuntimeError, 'Current Limit恢复失败'):
                pull.run(self.a, self.servo, lambda _: 'PULL')
        self.assertEqual(self.servo.reg[64], 0)
        self.assertEqual(self.servo.reg[38], 600)


if __name__ == '__main__':
    unittest.main()
