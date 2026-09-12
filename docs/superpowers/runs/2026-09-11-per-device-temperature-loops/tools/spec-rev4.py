"""Apply asserted revision-4 replacement pairs; never silently skip stale source text."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = HERE.parent / '2026-09-11-per-device-temperature-loops-design.md'
text = spec.read_text()
for name in ('rev4-control-replacements.json', 'rev4-sensor-replacements.json',
             'rev4-refinement-replacements.json'):
    pairs = json.loads((HERE / name).read_text())
    if isinstance(pairs, dict):
        pairs = pairs['replacements']
    for pair in pairs:
        old, new = (pair['old'], pair['new']) if isinstance(pair, dict) else pair
        assert old in text, f'{name}: missing replacement: {old[:160]!r}'
        assert text.count(old) == 1, f'{name}: ambiguous replacement: {old[:160]!r}'
        text = text.replace(old, new, 1)
spec.write_text(text)
print('Applied revision 4 replacement pairs')
