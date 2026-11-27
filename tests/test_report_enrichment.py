import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report
import report_enrichment
from gate_result import make_result


class EnrichmentTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name) / 'validated'
        (self.root / 'review').mkdir(parents=True)
        self.record = self.root / 'review/ai-review.result.json'
        self.record.write_text(json.dumps(make_result('ai-review', 'fail', 'AI review', 'pattern scan')))
        self.binding = {'workflow': {'pr': 3, 'run_id': 9}}
        self.uploads = []

    def github(self, arguments, payload=None):
        if payload:
            self.uploads.append(arguments)
        return b'diff --git a/x b/x\n'

    def enrich(self, script):
        enricher = report_enrichment.Enricher('o/r', '.', run_script=script, github=self.github)
        return enricher(self.root, self.binding, {'title': 't'}, {'profile': 'v2-prime'})

    def test_model_failure_keeps_pattern_scan_record(self):
        def broken(name, arguments, timeout):
            raise ValueError('down')
        before = self.record.read_text()
        self.assertEqual(self.enrich(broken), (None, {}, ''))
        self.assertEqual(self.record.read_text(), before)

    def test_adds_review_message_and_pictures(self):
        def script(name, arguments, timeout):
            out = Path(arguments[arguments.index('--out') + 1])
            out.mkdir(exist_ok=True)
            if name == 'charts.py':
                (out / 'size.svg').write_text('<svg/>')
                (out / 'charts.json').write_text(json.dumps({'size-release': {'file': 'size.svg', 'alt': 'a'}}))
                return
            stage = 'ai-review' if name == 'ai_review.py' else 'guidance'
            (out / (stage + '.result.json')).write_text(json.dumps(make_result(stage, 'warn', stage, 'found')))
        guidance, charts, base = self.enrich(script)
        self.assertEqual(guidance['stage'], 'guidance')
        self.assertIn('size-release', charts)
        self.assertEqual(base, 'https://github.com/o/r/raw/gate-images/pr-3/run-9')
        self.assertEqual(json.loads(self.record.read_text())['summary'], 'found')
        self.assertIn('repos/o/r/contents/pr-3/run-9/size.svg', self.uploads[0])


class LayoutTests(unittest.TestCase):
    def test_message_survives_huge_details_and_passing_build_log_is_left_out(self):
        results = [make_result('build-head-release', 'pass', 'Build', 'ok', details='COMPILER-LOG'),
                   make_result('sim', 'fail', 'Sim', 'bad', details='<details>\n' + 'é' * 200000),
                   make_result('guidance', 'pass', 'Message', 'written', details='CONTRIBUTOR-MESSAGE')]
        text = report.render_comment(results, [], 'https://example.invalid')
        self.assertLessEqual(len(text.encode()), report.MAX_COMMENT)
        self.assertIn('CONTRIBUTOR-MESSAGE', text)
        self.assertNotIn('COMPILER-LOG', text)
        self.assertEqual(text.count('<details'), text.count('</details>'))


if __name__ == '__main__':
    unittest.main()
