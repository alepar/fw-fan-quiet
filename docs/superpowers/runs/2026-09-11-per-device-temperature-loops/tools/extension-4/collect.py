"""Validate manual-pipeline files and assemble the same packets/coverage as Workflow."""
import json
import hashlib
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
names = ['premortem', 'completeness', 'yagni', 'failure-mode', 'feasibility', 'regression',
         'domain-feedback-control-systems', 'domain-hardware-thermal-management', 'domain-real-time-systems']
severities = {'Blocking', 'Should-fix', 'Nit', 'FYI'}
def read(name):
    return json.loads((root / name).read_text())
def write(name, data):
    (root / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
def valid_finding(f):
    for field in ('claim', 'location', 'category', 'evidence'):
        assert isinstance(f[field], str) and f[field], (field, f)
    assert isinstance(f['external'], bool)

for filename, expected in read('artifact-hashes.json').items():
    artifact = root.parent.parent / filename
    assert hashlib.sha256(artifact.read_bytes()).hexdigest() == expected, f'artifact changed: {filename}'

if sys.argv[1] == 'scouts':
    findings = []
    for name in names:
        values = read('scout-' + name + '.json')['findings']
        for f in values:
            valid_finding(f)
        findings.extend(values)
    write('raw-findings.json', findings)
    print(f'{len(names)} valid scout outputs; {len(findings)} raw findings')
elif sys.argv[1] == 'dedupe':
    data = read('dedupe.json')
    assert isinstance(data['beyondCapCount'], int) and data['beyondCapCount'] >= 0
    findings = data['findings']
    for i, f in enumerate(findings, 1):
        valid_finding(f)
        assert f['suggestedSeverity'] in severities
        f['id'] = f'd4-{i:02d}'
    write('findings.json', findings)
    print(json.dumps([{'id': f['id'], 'severity': f['suggestedSeverity'], 'claim': f['claim']} for f in findings], ensure_ascii=False, indent=2))
elif sys.argv[1] == 'packets':
    findings = read('findings.json')
    packets = []
    for f in findings:
        severe = f['suggestedSeverity'] in {'Blocking', 'Should-fix'}
        spot = None if severe else read(f"{f['id']}-spot.json")
        promoted = not severe and spot['verdict'] == 'CONFIRM' and spot['severity'] in {'Blocking', 'Should-fix'}
        tier = 'panel' if severe else 'promoted' if promoted else 'spot'
        votes = [read(f"{f['id']}-{seat}.json") for seat in ('reproduce', 'refute', 'ground')] if severe or promoted else [spot]
        for vote in votes:
            if vote is None:
                continue
            assert vote['verdict'] in {'CONFIRM', 'REJECT', 'UNVERIFIED'}
            assert vote['severity'] in severities and isinstance(vote['evidence'], str)
        packets.append({'f': f, 'votes': votes, 'tier': tier, 'valid': sum(v is not None for v in votes)})
    total = sum(len(p['votes']) for p in packets)
    valid = sum(p['valid'] for p in packets)
    coverage = dict(triageDead=False, scoutsDispatched=len(names), scoutsDead=0,
                    rawFindings=len(read('raw-findings.json')), dedupedFindings=len(findings),
                    beyondCap=read('dedupe.json')['beyondCapCount'], beyondPanelCap=0,
                    dedupeDead=bool(read('raw-findings.json')) and not findings,
                    panelCount=sum(p['tier']=='panel' for p in packets),
                    spotCount=sum(p['tier']=='spot' for p in packets),
                    promotedCount=sum(p['tier']=='promoted' for p in packets),
                    judgeCompletionPct=round(100*valid/total) if total else 0)
    write('packets.json', packets)
    write('coverage.json', coverage)
    print(json.dumps(coverage, indent=2))
else:
    raise SystemExit('expected scouts | dedupe | packets')
