"""Partial V2 handler evidence must be visible without implying full coverage."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'checkers'))
import report
import modules_check


class V2Report(unittest.TestCase):
    def test_player_task_delay_observations_merge_and_render_separately(self):
        main = {'v2_task_delays': {'PlayThread': {'calls': 2, 'total_us': 300000, 'max_us': 200000}}}
        extra = {'v2_task_delays': {'PlayThread': {'calls': 1, 'total_us': 250000, 'max_us': 250000}}}
        modules_check.merge_run(main, extra)
        self.assertEqual(main['v2_task_delays']['PlayThread'], {'calls': 3, 'total_us': 550000, 'max_us': 250000})
        html = report.build_modules({'data': main}, '')
        self.assertIn('Player task delays', html)
        self.assertIn('550.0 ms', html)
        self.assertIn('250.0 ms', html)
        self.assertIn('not command-handler blocking', html)

    def test_periodic_callback_elapsed_observations_merge_and_render(self):
        main = {'v2_timer_callback_elapsed_us': {'ELS::update_rpm': {'total_us': 30000, 'max_us': 20000}}}
        extra = {'v2_timer_callback_elapsed_us': {'ELS::update_rpm': {'total_us': 25000, 'max_us': 25000}}}
        modules_check.merge_run(main, extra)
        self.assertEqual(main['v2_timer_callback_elapsed_us']['ELS::update_rpm'], {'total_us': 55000, 'max_us': 25000})
        html = report.build_modules({'data': main}, '')
        self.assertIn('Partial periodic callback elapsed time', html)
        self.assertIn('55.0 ms', html)
        self.assertIn('25.0 ms', html)
        self.assertIn('not processor time', html)

    def test_module_callback_elapsed_observations_merge_and_render(self):
        main = {'v2_module_callback_elapsed_us': {'switch::in_command_ctx': {'total_us': 30000, 'max_us': 20000}}}
        extra = {'v2_module_callback_elapsed_us': {'switch::in_command_ctx': {'total_us': 25000, 'max_us': 25000}}}
        modules_check.merge_run(main, extra)
        self.assertEqual(main['v2_module_callback_elapsed_us']['switch::in_command_ctx'], {'total_us': 55000, 'max_us': 25000})
        html = report.build_modules({'data': main}, '')
        self.assertIn('Partial module callback elapsed time', html)
        self.assertIn('55.0 ms', html)
        self.assertIn('25.0 ms', html)
        self.assertIn('not processor time', html)

    def test_partial_instrumentation_preserves_observed_failure(self):
        checks = [
            {'check': 'handlers did not block while running the workload', 'status': 'fail', 'detail': 'forbidden wait observed'},
            {'check': 'no handler ran longer than threshold', 'status': 'pass', 'detail': 'ok'},
            {'check': 'module coverage', 'status': 'warn', 'detail': 'partial'},
            {'check': 'unrelated', 'status': 'pass', 'detail': 'ok'},
        ]
        modules_check.mark_v2_partial_instrumentation(checks)
        self.assertEqual(checks[0]['status'], 'fail')
        self.assertIn('forbidden wait observed', checks[0]['detail'])
        self.assertIn('not measured', checks[0]['detail'])
        self.assertEqual([c['status'] for c in checks[1:]], ['warn', 'warn', 'pass'])

    def test_partial_handler_coverage_is_labeled_and_escaped(self):
        data = {'v2_handler_method_coverage': {'Robot::<motion>': {'calls': 2, 'lines_hit': 3, 'lines_total': 5}}}
        html = report.build_modules({'data': data}, '')
        self.assertIn('Partial command-handler coverage', html)
        self.assertIn('Robot::&lt;motion&gt;', html)
        self.assertIn('3 / 5', html)
        self.assertIn('does not cover all handlers', html)

    def test_partial_handler_elapsed_time_is_visible_and_labeled(self):
        data = {'v2_handler_method_elapsed_us': {'Robot::<motion>': {'total_us': 30000, 'max_us': 20000}}}
        html = report.build_modules({'data': data}, '')
        self.assertIn('Partial handler elapsed time', html)
        self.assertIn('Robot::&lt;motion&gt;', html)
        self.assertIn('30.0 ms', html)
        self.assertIn('20.0 ms', html)
        self.assertIn('includes waits and timer callbacks', html)
        self.assertIn('not processor time', html)

    def test_no_partial_section_without_coverage(self):
        html = report.build_modules({'data': {}}, '')
        self.assertNotIn('Partial command-handler coverage', html)
        self.assertNotIn('Partial periodic callback coverage', html)

    def test_partial_timer_coverage_is_labeled_and_escaped(self):
        data = {'v2_timer_callback_coverage': {'Robot::<periodic>': {'calls': 2, 'lines_hit': 3, 'lines_total': 5}}}
        html = report.build_modules({'data': data}, '')
        self.assertIn('Partial periodic callback coverage', html)
        self.assertIn('Robot::&lt;periodic&gt;', html)
        self.assertIn('3 / 5', html)
        self.assertIn('does not cover all callbacks', html)
