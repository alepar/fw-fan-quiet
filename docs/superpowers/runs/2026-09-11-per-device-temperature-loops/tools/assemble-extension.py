"""Render super-roast `args` for DESIGN mode (iteration N) from the plugin's prompt files,
then explode each prompt to a file and emit a pointer-style args JSON (small enough to pass
inline to the Workflow tool). Usage: assemble-design-roast.py <iteration> [prior_report_path]"""
import json, re, sys, pathlib, os

IT = int(sys.argv[1]); PRIOR = sys.argv[2] if len(sys.argv) > 2 else ''
SK = pathlib.Path('/var/home/alepar/.codex/plugins/cache/superpowers-alepar/superpowers/6.3.0-alepar3.8/skills/super-roast')
WT = '/var/home/alepar/AleCode/fw-fan-quiet/.claude/worktrees/fw-fanctrl-loop/.worktrees/super-auto-per-device-temperature-loops'
RUN = f'{WT}/docs/superpowers/runs/2026-09-11-per-device-temperature-loops'
SPEC = f'{RUN}/2026-09-11-per-device-temperature-loops-design.md'
TREE = f'{RUN}/task-tree-settled.md'
S = pathlib.Path(__file__).resolve().parent / 'extension-4'; S.mkdir(exist_ok=True)
OUTDIR = S / f'roast-prompts-design-{IT}'; OUTDIR.mkdir(exist_ok=True)
REMAINDER_CAP = 50

def fences(md): return re.findall(r'```(?:[a-z]*)\n(.*?)```', md, flags=re.S)
def section_fence(md, heading):
    i = md.index(heading); return fences(md[i:])[0]

shell = ('Shell discipline: absolute paths; `cd ' + WT + '` as the first statement of every call; plain '
         'separate commands (this harness refuses compound constructs and heredocs that mention git as '
         '"too complex to verify" — a re-spell-and-retry, never a permission denial). Do NOT modify any file.')
context = (
    f'The artifact is the design spec `{SPEC}` TOGETHER WITH its settled task tree `{TREE}` (18 beads: root epic '
    'fw-fanctrl-loop-eb9, 16 leaves, an integration sweep; every bead carries owns:/consumes:/blocked-by lines and an '
    'acceptance) — the tree is the decomposition the spec will be executed as, so a gap in the tree against the spec, '
    'or a spec decision the tree cannot deliver, is in scope. The spec supersedes §2.4/§2.5 of '
    f'`{WT}/docs/superpowers/specs/2026-09-07-fw-fanctrl-loop-design.md` (its §2.2/§2.6/§2.8/§2.9/§3.4 carry over '
    'and may be read for context); the seed that settled the user decisions is '
    f'`{WT}/docs/superpowers/specs/2026-09-11-per-device-temperature-loops-seed.md`; the code the tree will change is '
    f'the Rust crate at `{WT}` (branch super-auto/per-device-temperature-loops, base epic-fw-fanctrl-loop-6ma-integration). '
    'Field evidence the spec rests on: telemetry runs `/var/lib/fw-fan-quiet/telemetry/run-1789067819.jsonl` and '
    '`run-1789139478.jsonl` (readable). Decided by the user and NOT up for re-litigation unless you show them '
    'unworkable: budget/split deleted; per-device EC groups; GPU driven by clock lock (LUT deleted); shadow cap with '
    'override control; halt = hold; no trim with a live curve; per-device step test; the 30-min hardware acceptance '
    'and the VR/VRAM label spike are the user\'s and are parked. ' + shell)

# ---------- scouts (design lenses) ----------
md = (SK / 'scout-prompts-design.md').read_text()
core = section_fence(md, '## Shared core')
stance1 = section_fence(md, '**Iteration 1** (no prior report):').strip()
stance2 = section_fence(md, '**Iterations ≥ 2** (a prior report exists):').strip()
stance = stance1 if IT == 1 else stance2
def lens_block(name):
    return section_fence(md, f'## Lens: {name}').strip()
lenses = {'premortem': lens_block('premortem'), 'completeness': lens_block('completeness'), 'yagni': lens_block('yagni'),
          'failure-mode': lens_block('failure-mode'), 'feasibility': lens_block('feasibility')}
sec_maint = lens_block('security / maintainer')
lenses['security'] = 'Security lens. ' + sec_maint
lenses['maintainer'] = 'Maintainer lens (maintainability, operability, evolvability of the resulting code). ' + sec_maint
if IT >= 2:
    lenses['regression'] = lens_block('regression (iterations ≥ 2 only)')
domain_block = lens_block('domain:\\<name\\> — the `scoutDomainTemplate`')

cat_old = '- **category:** the lens name for this dispatch — same value used to fill `[LENS]` above\n  (required)'
assert cat_old in core
def render_scout(name, block):
    p = core.replace(cat_old, f'- **category:** `{name}` (required — this dispatch\'s lens)')
    p = p.replace('[LENS]', block).replace('[ITERATION_STANCE]', stance)
    p = p.replace('[SPEC_FILE_PATH — read it]', SPEC)
    p = p.replace('[REQUIREMENTS / EPIC]', context)
    return p
scouts = {k: render_scout(k, v) for k, v in lenses.items()}
domain_tpl = render_scout('domain:{{DOMAIN}}', domain_block)

# ---------- triage ----------
triage = fences((SK / 'triage-prompt.md').read_text())[0]
triage = triage.replace('[SPEC_FILE_PATH — read it, design mode] or [DIFF FILE LIST + STAT SUMMARY — PR mode]',
                        f'DESIGN MODE. Spec: `{SPEC}` (read it; its task tree is `{TREE}`). {shell}')

# ---------- seats ----------
seat_md = (SK / 'judge-seat-prompts.md').read_text()
score = section_fence(seat_md, '## Shared core')
blocks = {'reproduce': section_fence(seat_md, '## Seat 1 — reproduce'),
          'refute': section_fence(seat_md, '## Seat 2 — refute'),
          'ground': section_fence(seat_md, '## Seat 3 — ground')}
seat_desc = (f'DESIGN MODE. The spec is `{SPEC}`; its settled task tree is `{TREE}` (part of the artifact). '
             + context)
seats = {}
for n, b in blocks.items():
    s = score.replace('[SEAT PROCEDURE]', b.strip())
    s = s.replace('[SPEC_FILE_PATH — read the whole spec, not just the cited section]', seat_desc)
    seats[n] = s

# ---------- dedupe / reporter ----------
dedupe = fences((SK / 'dedupe-prompt.md').read_text())[0].replace('[REMAINDER_CAP]', str(REMAINDER_CAP))
rep_md = (SK / 'reporter-prompt.md').read_text()
rs = rep_md.index('```\n', rep_md.index('tokens: `{{PACKETS_JSON}}`')); re_ = rep_md.rindex('```')
reporter = rep_md[rs + 4:re_]
assert '## Output contract' in reporter and 'seat-agreement: panels N' in reporter
reporter = reporter.replace('{{INDEPENDENCE}}', 'same-family (OpenAI GPT) — seat-differentiated panel')

profile = ('Single-operator personal tooling with hardware side-effects: a Rust daemon on one Framework 16 laptop that '
           'writes CPU power limits via ryzenadj (sudo) and dGPU clock locks via NVML, and reads the EC through '
           'cros_ec/ectool. No network surface, no other users, no external data. Blast radius is the operator\'s own '
           'machine: a wrong cap or a stuck emergency release degrades performance or acoustics and, at worst, lets the '
           'hardware\'s own thermal protection (card slowdown 89 C / shutdown 92 C; EC trip points) take over. Rollback '
           'is a git revert and a daemon restart. Treat thermal-safety inversions, silent loss of control, control-loop '
           'instability (windup, chatter, starvation) and anything that defeats the hardware backstops as the material '
           'class; resilience/observability polish is low-value here.')

# ---------- explode to files + pointer args ----------
TOK = re.compile(r'\{\{[A-Z_]+\}\}')
def ptr(name, text, role):
    path = OUTDIR / (name + '.md'); path.write_text(text)
    toks = sorted(set(TOK.findall(text)) - {'{{DOMAIN}}'})
    lines = [f"You are the {role} of a super-roast run (DESIGN mode, iteration {IT}). Your FULL instructions are in the file `{path}` — Read it first, in full, and follow it exactly (including its output contract). The placeholder tokens inside that file are filled below; treat each value here as the literal substitution for that token."]
    for t in toks: lines.append(f"\n### {t}\n{t}")
    return "\n".join(lines)
prior_text = ''
if PRIOR:
    prior_text = (f'PRIOR REPORT (iteration {IT-1}) — the full text is in `{PRIOR}`; Read it in full before doing anything '
                  'else and treat its contents as the literal {{PRIOR_REPORT}} value. Its Rejected section stands unless '
                  'you cite materially new evidence.')
args = {
  'mode': 'design', 'iteration': IT,
  'inputs': f'{SPEC} + {TREE} (epic fw-fanctrl-loop-eb9, 18 beads)',
  'profile': profile, 'priorReport': prior_text, 'dryRun': False,
  'config': {'coreLenses': ['premortem', 'completeness', 'yagni', 'failure-mode', 'feasibility'] + (['regression'] if IT >= 2 else []),
             'widenLenses': ['security', 'maintainer'], 'remainderCap': REMAINDER_CAP, 'iterationCap': 3,
             'models': {'triage': 'gpt-5.6-sol', 'scout': 'gpt-6-astra', 'dedupe': 'gpt-5.6-sol', 'judge': 'gpt-5.6-sol', 'reporter': 'gpt-5.6-sol'}},
  'prompts': {
    'triage': ptr('triage', triage, 'triage'),
    'dedupe': ptr('dedupe', dedupe, 'dedupe-and-rank'),
    'reporter': ptr('reporter', reporter, 'reporter'),
    'seats': {k: ptr('seat-' + k, v, f'judge seat "{k}"') for k, v in seats.items()},
    'scouts': {k: ptr('scout-' + k, v, f'scout lens "{k}"') for k, v in scouts.items()},
    'scoutDomainTemplate': ptr('scout-domain', domain_tpl, 'scout lens "domain:{{DOMAIN}}"') + "\n\n### {{DOMAIN}}\n{{DOMAIN}}",
  }}
out = S / f'roast-args-design-{IT}.json'
args['prompts']['reporter'] = args['prompts']['reporter'].replace('{{ITERATION}}', 'post-cap extension')
args['priorReport'] += ' Earlier reports 1 and 2 are siblings of report 3; their Rejected dispositions also stand.'
out.write_text(json.dumps(args, ensure_ascii=False, separators=(',', ':')))
print(out, out.stat().st_size, 'bytes; scouts', sorted(scouts))
for tok in ('[LENS]', '[ITERATION_STANCE]', '[SEAT PROCEDURE]', '[SPEC_FILE_PATH', '[REMAINDER_CAP]', '[REQUIREMENTS / EPIC]'):
    hits = [p.name for p in OUTDIR.iterdir() if tok in p.read_text()]
    print(' leftover', tok, hits or 'none')
