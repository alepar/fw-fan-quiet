"""Capture fix-pass refinements as unique asserted replacement hunks."""
import difflib
import json
from pathlib import Path
import subprocess

here = Path(__file__).resolve().parent
wt = next(p for p in here.parents if (p / '.git').exists())
spec = here.parent / '2026-09-11-per-device-temperature-loops-design.md'
old = subprocess.check_output(['git', '-C', str(wt), 'show', '4c2b42a:' + str(spec.relative_to(wt))], text=True)
for name in ('rev4-control-replacements.json', 'rev4-sensor-replacements.json'):
    for p in json.loads((here / name).read_text()):
        assert old.count(p['old']) == 1, p['old'][:100]
        old = old.replace(p['old'], p['new'], 1)
target = spec.read_text()
a, b = old.splitlines(keepends=True), target.splitlines(keepends=True)
pairs = []
for group in difflib.SequenceMatcher(a=a, b=b).get_grouped_opcodes(3):
    i, j = group[0][1], group[-1][2]
    x, y = group[0][3], group[-1][4]
    before, after = ''.join(a[i:j]), ''.join(b[x:y])
    assert old.count(before) == 1
    pairs.append({'old': before, 'new': after})
for p in pairs:
    assert old.count(p['old']) == 1
    old = old.replace(p['old'], p['new'], 1)
assert old == target
(here / 'rev4-refinement-replacements.json').write_text(json.dumps(pairs, indent=2, ensure_ascii=False) + '\n')
print(f'{len(pairs)} asserted refinement hunks; full revision reproducible from 4c2b42a')
