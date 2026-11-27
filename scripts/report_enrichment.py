"""Trusted additions to an authenticated report: model review, contributor message and pictures.

Runs only in the report job. Pull request text, diff and artifact data are passed as data to the
existing bounded scripts. Every addition is optional: a failure is logged and the report publishes without it.
"""
import base64
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from gate_result import validate_result

HERE = Path(__file__).resolve().parent
MAX_DIFF = 4 * 1024**2


def warn(operation, error):
    print(json.dumps({'type': 'warning', 'operation': operation, 'message': str(error)[-600:]}), flush=True)


def _gh(arguments, timeout=60, payload=None):
    done = subprocess.run(['gh', 'api', '--hostname', 'github.com', *arguments], input=payload,
                          capture_output=True, timeout=timeout)
    if done.returncode:
        raise ValueError(done.stderr.decode('utf-8', 'replace')[:300] or 'GitHub request failed')
    return done.stdout


def _script(name, arguments, timeout):
    """Run one trusted stage without the GitHub token; its log goes to the job log."""
    environment = {key: value for key, value in os.environ.items() if key not in ('GH_TOKEN', 'GITHUB_TOKEN')}
    done = subprocess.run([sys.executable, '-B', '-I', str(HERE / name), *arguments], env=environment, timeout=timeout)
    if done.returncode not in (0, 1):
        raise ValueError(name + ' exited with ' + str(done.returncode))


def _record(path, stage):
    record = json.loads(Path(path).read_text(encoding='utf-8'))
    if validate_result(record) or record.get('stage') != stage:
        raise ValueError('invalid ' + stage + ' result')
    return record


class Enricher:
    def __init__(self, repository, sim_dir, run_script=_script, github=_gh):
        self.repository, self.sim_dir = repository, Path(sim_dir).resolve()
        self.run_script, self.github = run_script, github

    def __call__(self, root, binding, pr, manifest):
        """Update ROOT's review record in place; return (guidance record or None, charts, image base)."""
        root = Path(root)
        number, run_id = binding['workflow']['pr'], binding['workflow']['run_id']
        work = root.parent / 'enrichment'
        work.mkdir()
        generation = '2' if str(manifest.get('profile', '')).startswith('v2') else '1'
        diff, text = work / 'pr.diff', work / 'pr-text.txt'
        guidance = None
        try:
            payload = self.github(['-H', 'Accept: application/vnd.github.diff',
                                   f'repos/{self.repository}/pulls/{number}'])
            if len(payload) > MAX_DIFF:
                raise ValueError('pull request diff exceeds the review limit')
            diff.write_bytes(payload)
            text.write_text('Title: ' + str(pr.get('title') or '')[:1000] + '\n\n' + str(pr.get('body') or '')[:20000],
                            encoding='utf-8')
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            warn('read pull request diff', error)
            diff = None
        if diff:
            try:
                (work / 'review').mkdir()
                self.run_script('ai_review.py', ['--diff', str(diff), '--pr-text', str(text), '--out', str(work / 'review'),
                                '--sim-dir', str(self.sim_dir), '--firmware-version', generation,
                                '--timeout', '100', '--attempts', '1'], 420)
                record = _record(work / 'review/ai-review.result.json', 'ai-review')
                (root / 'review/ai-review.result.json').write_text(json.dumps(record), encoding='utf-8')
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                warn('model review', error)  # the pattern scan record from the checks stays
            try:
                self.run_script('guidance.py', ['--results', str(root), '--diff', str(diff), '--pr-text', str(text),
                                '--out', str(work / 'guidance'), '--firmware-version', generation,
                                '--timeout', '100', '--attempts', '1'], 240)
                guidance = _record(work / 'guidance/guidance.result.json', 'guidance')
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                warn('contributor message', error)
        charts, base = {}, ''
        try:
            charts, base = self._charts(root, work / 'charts', f'pr-{number}/run-{run_id}')
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
            warn('publish pictures', error)
        return guidance, charts, base

    def _charts(self, root, out, folder):
        self.run_script('charts.py', ['--results', str(root), '--out', str(out)], 120)
        if not (out / 'charts.json').is_file():
            return {}, ''
        published = {}
        for key, chart in json.loads((out / 'charts.json').read_text(encoding='utf-8')).items():
            name = chart['file']
            if name != os.path.basename(name) or not name.endswith('.svg'):
                raise ValueError('unexpected chart file name')
            route = f'repos/{self.repository}/contents/{folder}/{name}'
            body = json.dumps({'message': 'Report picture ' + folder, 'branch': 'gate-images',
                               'content': base64.b64encode((out / name).read_bytes()).decode('ascii')}).encode()
            # Reports of several pull requests commit to this branch at once; a conflict is retried.
            for attempt in range(6):
                try:
                    self.github(['--method', 'PUT', route, '--input', '-'], payload=body)
                    published[key] = chart
                    break
                except (ValueError, subprocess.SubprocessError) as error:
                    try:
                        self.github([route + '?ref=gate-images'])
                        published[key] = chart
                        break
                    except (ValueError, subprocess.SubprocessError):
                        if attempt == 5:
                            warn('publish picture ' + name, error)
                        else:
                            time.sleep(2 * (attempt + 1))
        return published, f'https://github.com/{self.repository}/raw/gate-images/{folder}'
