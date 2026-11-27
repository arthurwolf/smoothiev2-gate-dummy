"""Comment size limit and distinct build titles."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts'))
import ci_builds
import report


class ReportLimits(unittest.TestCase):
    def test_oversized_details_fit_one_github_comment(self):
        results = [{'stage': 'sim', 'status': 'fail', 'title': f'Sim {index}', 'summary': 's',
                    'details': 'line\n' * 20000} for index in range(30)]
        comment = report.render_comment(results, [], 'https://example.invalid/run')
        self.assertLessEqual(len(comment), report.MAX_COMMENT)
        self.assertEqual(comment.count('<details'), comment.count('</details>'))

    def test_build_titles_name_revision_and_configuration(self):
        titles = {ci_builds.build_title(f'build-{label}-{kind}')
                  for label in ('base', 'pr') for kind in ('release', 'debug')}
        self.assertEqual(len(titles), 4)


if __name__ == '__main__':
    unittest.main()
