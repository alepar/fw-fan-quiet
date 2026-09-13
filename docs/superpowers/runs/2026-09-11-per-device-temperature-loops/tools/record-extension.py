"""Extract the manual review result and record the mandatory stop boundary."""
import json
from pathlib import Path
import re

here = Path(__file__).resolve().parent
run = here.parent
data = json.loads((here / 'extension-4/reporter.json').read_text())
report = data['reportMarkdown']
report = re.sub(r'(^## [^\n]+)\n(?=- )', r'\1\n\n', report, flags=re.M)
assert 'iteration: post-cap extension' in report
assert '[low coverage]' in data['verdict']
assert '[converged]' not in report
assert data['confirmedCount'] == 0
delta = next(line for line in report.splitlines() if line.startswith('delta vs prior:'))
name = '2026-09-11-per-device-temperature-loops-roast-design-4.md'
(run / name).write_text(report.rstrip() + '\n')
p = run / 'run.md'
text = p.read_text()
assert name not in text
marker = '\n- Extension authorization (2026-09-11):'
assert marker in text
bullet = f'\n- {name} ({data["verdict"]}; {delta.removeprefix("delta vs prior:").strip()}; 9/9 scouts completed, 0 raw/deduped candidates, no judge panels, no failed stages) — extension completed; low-coverage qualifier is required by reporter policy for zero raw findings on a non-trivial artifact, not by a dead scout or incomplete panel.\n'
# Keep review history grouped under roast-design, before extension metadata.
text = text.replace('roastDesignExtension:', bullet.lstrip('\n') + 'roastDesignExtension:', 1)
text += '\nroastDesignExtensionStatus: complete — clean (0 nits) [low coverage]; stopped after the authorized extension\n'
text += 'next: super-code (phase 3), after user disposition of the low-coverage qualifier; not started in this session\n'
text += 'stopReason: user explicitly required stopping after the extension report regardless of verdict; no further roast, implementation, merge, or hardware action\n'
p.write_text(text)
print(data['verdict'])
print(delta)
print(name)
