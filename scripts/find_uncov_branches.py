"""Print uncovered branch line numbers for a list of target files.

Usage: COV_JSON=target/cov-full.json python3 scripts/find_uncov_branches.py
"""
import json, os, sys

COV_PATH = os.environ.get('COV_JSON', 'target/cov-full.json')
with open(COV_PATH) as f:
    data = json.load(f)

TARGETS = sys.argv[1:] if len(sys.argv) > 1 else [
    'rp2040-emu/src/threaded/emulator.rs',
    'rp2350-emu/src/threaded/emulator.rs',
    'rp2350-emu/src/peripherals/i2c.rs',
    'picoem-common/src/threaded/barrier.rs',
]

def norm(p):
    return p.replace('\\', '/')

files = data['data'][0]['files']
for t in TARGETS:
    matches = [f for f in files if t in norm(f['filename'])]
    if not matches:
        print(f"!! no match for {t}")
        continue
    for f in matches:
        br = f.get('branches', [])
        # Branch entry shape varies — try common forms.
        # llvm-cov export gives entries like
        #   [line_start, col_start, line_end, col_end, count, count_false, file_id, ...]
        # or a dict with {line_start, count, ...}.
        uncov = []
        for b in br:
            if isinstance(b, list):
                # The list form: [ls, cs, le, ce, true_count, false_count, ...]
                if len(b) >= 6:
                    line_start = b[0]
                    true_count = b[4]
                    false_count = b[5]
                    if true_count == 0 or false_count == 0:
                        uncov.append((line_start, b[1], true_count, false_count))
            elif isinstance(b, dict):
                ls = b.get('line_start') or b.get('start', {}).get('line')
                if b.get('count', 0) == 0:
                    uncov.append((ls, None, b.get('count'), None))
        print(f"=== {t} ===")
        print(f"  total branch records: {len(br)}")
        print(f"  uncovered (true OR false 0): {len(uncov)}")
        for u in uncov[:50]:
            print(f"    line {u[0]} col {u[1]} : true_cnt={u[2]} false_cnt={u[3]}")
        print()
