import json
import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import ai_review
import copilot_client


def events(*items):
    return '\n'.join(json.dumps(item) for item in items)


ANSWER = events({'type': 'assistant.message', 'data': {'content': '{}', 'model': 'm'}}, {'type': 'result', 'exitCode': 0})


class CopilotTests(unittest.TestCase):
    def test_runs_without_tools_or_unrelated_credentials(self):
        seen = {}

        def run(command, **options):
            seen.update(command=command, **options)
            return subprocess.CompletedProcess(command, 0, ANSWER, '')
        with mock.patch.dict(os.environ, {'COPILOT_GITHUB_TOKEN': 't', 'GH_TOKEN': 'other', 'AI_API_KEY': 'other'}):
            self.assertEqual(copilot_client.ask('@/etc/passwd', 5, run=run), ('{}', 'm'))
        self.assertIn('--available-tools=none-available', seen['command'])
        self.assertEqual(seen['input'], '@/etc/passwd')
        self.assertNotIn('other', seen['env'].values())

    def test_refuses_tool_use_and_incomplete_answers(self):
        for output in (events({'type': 'tool.execution_start'}) + '\n' + ANSWER, events({'type': 'result', 'exitCode': 0})):
            with self.assertRaises(ValueError):
                copilot_client.parse(output)

    def test_copilot_is_first_and_gateway_is_the_fallback(self):
        with mock.patch.object(copilot_client, 'enabled', return_value=True):
            client = ai_review.ModelClient('http://x', ['router'], None, 1000, 5, 1, sleep=lambda s: None)
        messages = [{'role': 'system', 'content': 's'}, {'role': 'user', 'content': 'u'}]
        gateway = mock.Mock(return_value={'choices': [{'message': {'content': 'from gateway'}}]})
        with mock.patch.object(client, '_post', gateway):
            with mock.patch.object(copilot_client, 'ask', return_value=('from copilot', 'm')):
                self.assertEqual(client.complete(messages), ('from copilot', 'github-copilot/m'))
            gateway.assert_not_called()
            with mock.patch.object(copilot_client, 'ask', side_effect=ValueError('quota')) as ask:
                self.assertEqual(client.complete(messages), ('from gateway', 'router'))
                client.complete(messages)
            self.assertEqual(ask.call_count, 1)


if __name__ == '__main__':
    unittest.main()
