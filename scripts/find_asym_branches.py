"""Print branches where one direction is 0 (asymmetric coverage)."""
import json, os, sys

COV_PATH = os.environ.get('COV_JSON', 'target/cov-full.json')
with open(COV_PATH) as f:
    data = json.load(f)

TARGETS = sys.argv[1:]

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
        asym = []
        unhit = []
        for b in br:
            if isinstance(b, list) and len(b) >= 6:
                line_start = b[0]
                col_start = b[1]
                tc = b[4]
                fc = b[5]
                if tc == 0 and fc == 0:
                    unhit.append((line_start, col_start, tc, fc))
                elif tc == 0 or fc == 0:
                    asym.append((line_start, col_start, tc, fc))
        print(f"=== {t} ===")
        print(f"  total branches: {len(br)}, asym(one zero): {len(asym)}, both zero: {len(unhit)}")
        asym.sort(key=lambda x: (x[0], x[1]))
        print(f"  -- ASYMMETRIC ({len(asym)}) --")
        for u in asym:
            print(f"    line {u[0]} col {u[1]} : true_cnt={u[2]} false_cnt={u[3]}")
        print()
