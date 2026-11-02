#!/usr/bin/env python3
"""Repository hygiene checks that need no MATLAB licence (run in CI and locally).

- No third-party or copyrighted material is tracked.
- No videos or stray large binaries are tracked.
- Every MATLAB file carries an H1 help line in the house format.
- Error identifiers follow rbsim:<package>:<tag>.
- The licence and the documentation index exist.
"""
import pathlib, re, subprocess, sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
errors = []

tracked = subprocess.run(['git', '-C', str(ROOT), 'ls-files'],
                         capture_output=True, text=True, check=True).stdout.split()

for t in tracked:
    if re.search(r'\.(mp4|avi|mov|mkv|pptx|docx|pdf)$', t, re.I):
        errors.append(f'third-party or heavy media tracked: {t}')
    blob = ROOT / t
    if blob.is_file() and blob.stat().st_size > 2_000_000:
        errors.append(f'file over 2 MB tracked: {t} ({blob.stat().st_size // 1024} kB)')

for required in ('LICENSE', 'README.md', 'setup.m', 'docs/README.md'):
    if required not in tracked:
        errors.append(f'missing from the repository: {required}')

h1 = re.compile(r'^\s*%[A-Z][A-Z0-9_]*  \S')
for t in tracked:
    if not t.endswith('.m'):
        continue
    lines = (ROOT / t).read_text(encoding='utf-8').split('\n')
    head = [l for l in lines[:4] if l.strip()]
    if not head:
        errors.append(f'{t}: empty file')
        continue
    if head[0].lstrip().startswith(('function', 'classdef')):
        if len(head) < 2 or not h1.match(head[1]):
            errors.append(f'{t}: no H1 help line "%NAME  description" (two spaces)')

bad_id = re.compile(r"\b(?:error|warning)\('(?!rbsim:)([A-Za-z]\w*):")
for t in tracked:
    if not t.startswith('matlab/functions/') or not t.endswith('.m'):
        continue
    for n, line in enumerate((ROOT / t).read_text(encoding='utf-8').split('\n'), 1):
        m = bad_id.search(line)
        if m and m.group(1) not in ('MATLAB',):
            errors.append(f'{t}:{n}: identifier should be rbsim:<package>:<tag>')

for e in errors:
    print('FAIL', e)
print(f'{len(tracked)} tracked files checked, {len(errors)} problem(s)')
sys.exit(1 if errors else 0)
