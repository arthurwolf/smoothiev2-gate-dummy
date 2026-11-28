"""One text answer from GitHub Copilot CLI, with every tool disabled.

The prompt holds untrusted pull request text, so the CLI gets no tools, no repository, no custom
instructions, an empty home folder and only the environment it needs to authenticate.
"""
import json
import os
import subprocess
import tempfile

# A name that matches no tool leaves the model with none.
FLAGS = ('--available-tools=none-available', '--disable-builtin-mcps', '--no-custom-instructions', '--no-ask-user',
         '--no-auto-update', '--output-format', 'json', '--model')


def enabled():
    return os.environ.get('AI_COPILOT') == '1' and bool(os.environ.get('COPILOT_GITHUB_TOKEN'))


def parse(output):
    """Return (text, model) from the CLI's JSON lines; refuse an answer that tried to use a tool."""
    text, model, finished = [], 'unknown', False
    for line in output.splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if not isinstance(event, dict):
            continue
        kind = str(event.get('type', ''))
        data = event.get('data') if isinstance(event.get('data'), dict) else {}
        if kind.startswith('tool.') or data.get('toolRequests'):
            raise ValueError('the model tried to use a tool')
        if kind == 'assistant.message' and isinstance(data.get('content'), str):
            text.append(data['content'])
            model = str(data.get('model') or model)
        elif kind == 'result':
            finished = event.get('exitCode') == 0
    if not finished or not ''.join(text).strip():
        raise ValueError('no complete answer')
    return ''.join(text), model


def ask(prompt, timeout, run=subprocess.run):
    """Return (text, model name); raise ValueError with a short reason."""
    with tempfile.TemporaryDirectory() as home:
        work = os.path.join(home, 'work')
        os.mkdir(work)
        environment = {'PATH': os.environ.get('PATH', os.defpath), 'HOME': home,
                       'COPILOT_HOME': os.path.join(home, '.copilot'),
                       'COPILOT_GITHUB_TOKEN': os.environ['COPILOT_GITHUB_TOKEN']}
        try:
            # The prompt goes through standard input: it can exceed the size limit of one command argument.
            done = run(['copilot', *FLAGS, os.environ.get('AI_COPILOT_MODEL') or 'auto'], input=prompt, cwd=work,
                       env=environment, capture_output=True, text=True, errors='replace', timeout=timeout)
        except subprocess.TimeoutExpired:
            raise ValueError('timed out') from None
        except OSError as error:
            raise ValueError('could not start (' + type(error).__name__ + ')') from None
    if done.returncode:
        reason = done.stderr.strip().splitlines()[-1:] or ['no message']
        raise ValueError(f'exit {done.returncode}: {reason[0][:200]}')
    return parse(done.stdout)
