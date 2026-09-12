"""Verify the documentation snapshot matches live beads and revision provenance."""
import json
from pathlib import Path
import re
import subprocess

here = Path(__file__).resolve().parent
run = here.parent
wt = next(p for p in here.parents if (p / '.git').exists())
spec = run / '2026-09-11-per-device-temperature-loops-design.md'
text = subprocess.check_output(['git', '-C', str(wt), 'show', '4c2b42a:' + str(spec.relative_to(wt))], text=True)
count = 0
for name in ('rev4-control-replacements.json', 'rev4-sensor-replacements.json', 'rev4-refinement-replacements.json'):
    for p in json.loads((here / name).read_text()):
        assert text.count(p['old']) == 1, (name, p['old'][:80])
        text = text.replace(p['old'], p['new'], 1)
        count += 1
assert text == spec.read_text(), 'Replacement scripts do not reproduce final spec'
tree = (run / 'task-tree-settled.md').read_text()
original_tree = subprocess.check_output(['git', '-C', str(wt), 'show', '4c2b42a:' + str((run / 'task-tree-settled.md').relative_to(wt))], text=True)
ids = re.findall(r'^## (fw-fanctrl-loop-eb9(?:\.\d+)?) \[', tree, re.M)
expected = {'fw-fanctrl-loop-eb9'} | {f'fw-fanctrl-loop-eb9.{i}' for i in range(1,18)}
assert len(ids) == len(expected) and set(ids) == expected
for bid in ids:
    data = json.loads(subprocess.check_output(['bd', 'show', bid, '--json'], cwd=wt, text=True))
    issue = data[0] if isinstance(data, list) else data
    body = issue.get('description', '').rstrip()
    section = tree.split('## ' + bid + ' [', 1)[1].split('\n## ', 1)[0]
    assert body and body in section, f'{bid}: live body differs'
    deps = sorted(d['id'] for d in issue.get('dependencies', []) if d.get('dependency_type') == 'blocks')
    assert f'blocking deps: {deps}' in section, f'{bid}: dependency mismatch'
    original = original_tree.split('## ' + bid + ' [', 1)[1].split('\n## ', 1)[0]
    for line in original.splitlines():
        if line.startswith(('blocked-by ', 'Files:', 'blocking deps:')):
            assert line in section.splitlines(), f'{bid}: original ownership/dependency line changed: {line}'
    assert not re.search(r'\brev3\b|amendment', body, re.I), f'{bid}: stale amendment/revision'
assert 'revision 4' in tree and 'revision 4' in text
ledger = (run / 'per-device-temperature-loops-coverage-ledger.md').read_text()
for i in range(1,20):
    assert len(re.findall(rf'^- d3-{i:02d} · ', ledger, re.M)) == 1
subprocess.run(['git', '-C', str(wt), 'diff', '--check'], check=True)
print(f'PASS: {count} asserted replacements reproduce spec; 18 live bead bodies/dependencies match tree; 19 unique dispositions; diff whitespace clean')
