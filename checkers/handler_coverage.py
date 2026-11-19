"""Which event-handler functions of each module did the workload really run? Uses gcov counters from a simulator built with
COVERAGE=1 (the Makefile default). A handler that was merely registered, or that returned on its first line, does not count."""
import glob, json, os, re, shutil, subprocess, tempfile


def instrumented_v2(binary):
    symbols = subprocess.run(['nm', '--defined-only', binary], capture_output=True, text=True, check=True).stdout
    return any(line.split()[-1:] == ['__gcov_init'] for line in symbols.splitlines())


def prepare_v2(binary, workdir):
    """Make isolated compiler counters for a V2 workload, if this build has them."""
    notes = glob.glob(binary + '-*.gcno')
    if not instrumented_v2(binary):
        return None
    if not notes:
        raise RuntimeError('coverage simulator has no compiler notes alongside its binary')
    directory = tempfile.mkdtemp(prefix='v2-coverage-', dir=workdir)
    for path in notes:
        shutil.copy2(path, directory)
    env = os.environ.copy()
    env.update(GCOV_PREFIX=directory, GCOV_PREFIX_STRIP='99')
    return directory, env


def collect_v2(directory, names):
    """Get per-method counts and line coverage for recognized V2 callbacks."""
    data = sorted(glob.glob(os.path.join(directory, '*.gcda')))
    if not data:
        raise RuntimeError('coverage build produced no compiler counters')
    result = {}
    for i in range(0, len(data), 20):
        process = subprocess.run(['gcov', '--json-format', '--stdout', '--demangled-names'] + data[i:i + 20],
                                 capture_output=True, text=True, cwd=directory)
        if process.returncode:
            raise RuntimeError('gcov failed: ' + process.stderr[-2000:])
        for line in process.stdout.splitlines():
            if not line.startswith('{'):
                continue
            for source in json.loads(line).get('files', []):
                if not re.search(r'(?:^|/)stages/[^/]+/src/.*\.cpp$', source.get('file', '')):
                    continue
                for function in source.get('functions', []):
                    name = function.get('demangled_name', '').split('(', 1)[0]
                    if name not in names:
                        continue
                    start, end = function['start_line'], function['end_line']
                    lines = [entry for entry in source.get('lines', []) if start <= entry['line_number'] <= end]
                    result[name] = {'calls': function['execution_count'], 'lines_hit': sum(entry['count'] > 0 for entry in lines),
                                    'lines_total': len(lines)}
    return result


def clear_counters(obj_dir):
    for p in glob.glob(os.path.join(obj_dir, "**", "*.gcda"), recursive=True):
        os.remove(p)


def available(obj_dir):
    return bool(glob.glob(os.path.join(obj_dir, "modules", "**", "*.gcno"), recursive=True))


def collect(obj_dir, stage_src):
    """{relative source file: {function demangled name: (calls, lines_hit, lines_total, first line of the function)}} for every instrumented file with counters."""
    gcdas = sorted(glob.glob(os.path.join(obj_dir, "**", "*.gcda"), recursive=True))
    out = {}
    for i in range(0, len(gcdas), 40):
        p = subprocess.run(["gcov", "--json-format", "--stdout", "--demangled-names"] + gcdas[i:i + 40], capture_output=True, text=True, cwd=obj_dir)
        for line in p.stdout.splitlines():
            line = line.strip()
            if not line.startswith("{"):
                continue
            doc = json.loads(line)
            for f in doc.get("files", []):
                rel = os.path.relpath(f["file"], stage_src) if os.path.isabs(f["file"]) else f["file"]
                funcs = out.setdefault(rel, {})
                spans = [(fn["start_line"], fn["end_line"], fn.get("demangled_name") or fn["name"], fn["execution_count"]) for fn in f.get("functions", [])]
                for s, e, name, cnt in spans:
                    ls = [l for l in f.get("lines", []) if s <= l["line_number"] <= e]
                    total = len(ls); hit = sum(1 for l in ls if l["count"] > 0)
                    old = funcs.get(name, (0, 0, 0, s))
                    funcs[name] = (old[0] + cnt, max(old[1], hit), max(old[2], total), min(old[3], s))
    return out


def handler_rows(cov, rel):
    """Handler functions (on_*) of one module file with their counters."""
    rows = []
    for name, (calls, hit, total, line) in sorted(cov.get(rel, {}).items()):
        short = re.sub(r"\(.*", "", name).split("::")[-1]
        if short.startswith("on_"):
            rows.append({"function": name, "calls": calls, "lines_hit": hit, "lines_total": total, "line": line})
    return rows
