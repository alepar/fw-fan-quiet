"""Record the authorized fix pass and launch state, after bead regeneration."""
from pathlib import Path
import re

run = Path(__file__).resolve().parent.parent
ledger = run / 'per-device-temperature-loops-coverage-ledger.md'
text = ledger.read_text()
assert '## Design roast 3 dispositions' not in text
entries = [
('Blocking', '§2.3 hot draw dip tracking', 'one-time measured handover; no continuous draw overwrite; both shadow directions hold while hot; recovery replay', '3,4,11,14'),
('Blocking', '§2.2 group seed and view changes', 'own instantaneous group seeds; view changes preserve history and set interval only', '1,2,7'),
('Blocking', '§4 sim 4 dead-zone arithmetic', 'separate downward-knee crossing bound from post-crossing delay and 3λ settling', '3,10,11'),
('Blocking', '§2.5 verify_lock timing', 'pair sample with successfully completed command; bounded predecessor allowance; compliant and ignoring-card vectors', '7,14'),
('Should-fix', '§2.3 dt and resume', 'explicit bounded elapsed time, PI accumulator, resume hold/reset and dwell semantics', '2,3,7'),
('Should-fix', '§2.5 guard versus averaged group', 'guard clamps PI independent of group error; both branches tested; recovery bars start at eligibility', '3,7,14,15'),
('Should-fix', '§2.3/2.4 target-driven hot spell', 'measured-only handover, ΔT* error-history shift, explicit Curve upward reset and recovery assertions', '3,5,7,11,16'),
('Should-fix', '§2.8 last-good seed', 'strategy/target/timestamp qualification, six-hour age, migration and restart cases', '5,6,7,14,17'),
('Should-fix', '§2.4 Bypass transitions', 'seed shadow on entry and thermal on exit from commanded cap; suppress transition motion', '3,4,5,7,14'),
('Should-fix', '§2.1 plausibility/reconciliation', 'positive-only raw reconciliation stream separate from plausible control data; no filter-induced mismatch sim', '1,2,10,14'),
('Should-fix', '§2.4 DrawUnavailable anti-windup', 'remove from both directional blocking sets; live thermal-only must-not-hold vector', '3,16'),
('Should-fix', '§2.3 Bypass shadow gate', 'Bypass ignores frozen T* for both shadow directions; hot-group load-rise case', '3,4,14'),
('Should-fix', '§2.3 thermal-only plateau', 'explicit full max-to-knee bound and disabled/missing-draw sim legs; slew on transitions', '3,4,11,14'),
('Should-fix', '§2.3 bound tie identity', 'Floor then Max precedence; interior Thermal ties; reachable clamp diagnostics and smoke', '3,4,9,13,14,16'),
('Should-fix', '§2.3 GPU τ assumption', 'offline group-fit evidence; provisional τ15/Ti15/Kc2.1 and crossed τ8–50 uncertainty grid', '3,10,11'),
('Should-fix', '§2.4 stuck uncontrollable argmax', 'rolling unchanged dwell, visible quarantine, Held/Regulate backstop and recovery', '1,5,9,10,13,14,16'),
('Should-fix', '§2.1 unknown labels', 'exact plus prefix fallback; unknown/missing label diagnostics; unrelated Unknown routes Held', '1,5,9,13,14'),
('Should-fix', '§2.4 empty uncontrollable set', 'finite collapsed interval plus explicit diagnostic and recovery tests', '5,9,13,14,16'),
('Should-fix', '§2.8 CPU hot sanitizer', 'independent [82,94] range and relative exit/recovery thresholds', '6,7,14,15,16'),
]
text += '\n## Design roast 3 dispositions (d3-01..d3-19; 2026-09-11; spec revision 4)\n\n'
text += 'The authorized two-candidate alternative is retained with an edge-only measured thermal handover. The continuous draw-driven state overwrite is removed. All bead descriptions are rewritten wholesale. Offline arithmetic/trace evidence is documented; future implementation acceptance simulations are requirements, not claimed passes.\n\n'
for i, (sev, loc, change, beads) in enumerate(entries, 1):
    refs = ', '.join('eb9.' + n for n in beads.split(','))
    text += f'- d3-{i:02d} · {sev} · {loc} · applied · {change} → {refs}.\n'
ledger.write_text(text)
p = run / 'run.md'
text = p.read_text()
assert 'roastDesignRound: 3' in text
text = text.replace('roastDesignRound: 3', 'roastDesignRound: 4')
text = re.sub(r'^roastDesignCapped:.*$', 'roastDesignExtension: round 4 (capped-Blocking extension) launched 2026-09-11', text, flags=re.M)
text = text.replace('— NOT applied: cap round still Blocking; the user asked to pause before the capped-Blocking extension round', '— all 19 applied as spec revision 4 and wholesale bead descriptions (ledger d3-01..d3-19); authorized extension follows')
text += '\n- Extension authorization (2026-09-11): user adopted next-session-prompt.md; fix all round-3 confirmations, run exactly one post-cap extension, record verdict and stop before super-code regardless of verdict. Codex manual super-roast fallback uses fresh OpenAI GPT scouts and differentiated judges; no Workflow tool is available.\n'
p.write_text(text)
print('Recorded 19 dispositions and extension launch state')
