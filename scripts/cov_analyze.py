import json, re
from collections import defaultdict

with open('/tmp/cov-full.json') as f:
    data = json.load(f)

# Define the in-scope filter — exclude harness bins, hardware-oracle support libs,
# TUI app shells, third_party, registry, rustc internals.
EXCLUDE_PATTERNS = [
    r'/rustc/[0-9a-f]+/',
    r'\.cargo/registry/',
    r'\.cargo/git/',
    r'/rustup/toolchains/',
    r'mdpicoem/target/llvm-cov-target',
    r'mdpicoem/third_party/',
    r'mdpicoem-harness/src/bin/',
    r'mdpicoem-harness/src/(silicon_scenarios|isr_scenarios|isr_scenarios_rp2040|dualcore_cases|cycle_cases|bank_conflict_cases|silicon_oracle|gdb_client|ieee754_ref|onerom_(stress|serving_oracle|serving_oracle_cpu|cpu_speed_grade|trace|glue_dma|snapshot_fmt|sync)|picogus_pins)\.rs',
    r'mdrp2040app/src/(main|ui|sim|firmware|panels)',
    r'mdrp2350app/src/(main|ui|sim|firmware|panels)',
    r'mdpicoem/crates/[^/]+/tests/',
    r'mdpicoem/crates/[^/]+/src/(tests|tests_narrow|tests_stage3_thumb32|pio_tests)\.rs$',
    r'mdpicoem/crates/[^/]+/src/core_riscv/tests_(common|p1|p2|p3|p4|p5|p6)\.rs$',
]
exclude_re = re.compile('|'.join(EXCLUDE_PATTERNS))

def is_in_scope(path):
    return not exclude_re.search(path) and '/mdpicoem/' in path

def crate_for(path):
    m = re.search(r'/mdpicoem/crates/([^/]+)/', path)
    if not m:
        return None
    return m.group(1)

def file_key(path):
    m = re.search(r'/mdpicoem/(crates/[^/]+/src/[^?]+)', path)
    if m:
        return m.group(1)
    return path

per_crate = defaultdict(lambda: {'br_cov': 0, 'br_tot': 0, 'reg_cov': 0, 'reg_tot': 0, 'fn_cov': 0, 'fn_tot': 0, 'ln_cov': 0, 'ln_tot': 0})
per_file = defaultdict(lambda: {'br_cov': 0, 'br_tot': 0, 'reg_cov': 0, 'reg_tot': 0, 'fn_cov': 0, 'fn_tot': 0, 'ln_cov': 0, 'ln_tot': 0})

in_scope_total = {'br_cov': 0, 'br_tot': 0, 'reg_cov': 0, 'reg_tot': 0, 'fn_cov': 0, 'fn_tot': 0, 'ln_cov': 0, 'ln_tot': 0}
out_scope_total = {'br_cov': 0, 'br_tot': 0, 'reg_cov': 0, 'reg_tot': 0, 'fn_cov': 0, 'fn_tot': 0, 'ln_cov': 0, 'ln_tot': 0}

files = data['data'][0]['files']
print(f"Total files in coverage: {len(files)}")

for f in files:
    path = f['filename']
    s = f['summary']
    br = s['branches']
    rg = s['regions']
    fn = s['functions']
    ln = s['lines']
    bucket = in_scope_total if is_in_scope(path) else out_scope_total
    bucket['br_cov'] += br['covered']; bucket['br_tot'] += br['count']
    bucket['reg_cov'] += rg['covered']; bucket['reg_tot'] += rg['count']
    bucket['fn_cov'] += fn['covered']; bucket['fn_tot'] += fn['count']
    bucket['ln_cov'] += ln['covered']; bucket['ln_tot'] += ln['count']
    if is_in_scope(path):
        crate = crate_for(path)
        if crate:
            c = per_crate[crate]
            c['br_cov'] += br['covered']; c['br_tot'] += br['count']
            c['reg_cov'] += rg['covered']; c['reg_tot'] += rg['count']
            c['fn_cov'] += fn['covered']; c['fn_tot'] += fn['count']
            c['ln_cov'] += ln['covered']; c['ln_tot'] += ln['count']
        fk = file_key(path)
        f_ = per_file[fk]
        f_['br_cov'] += br['covered']; f_['br_tot'] += br['count']
        f_['reg_cov'] += rg['covered']; f_['reg_tot'] += rg['count']
        f_['fn_cov'] += fn['covered']; f_['fn_tot'] += fn['count']
        f_['ln_cov'] += ln['covered']; f_['ln_tot'] += ln['count']

def pct(cov, tot):
    return f"{cov*100/tot:.2f}%" if tot else "n/a"

def fmt_row(name, d):
    return f"{name:<60} {d['br_cov']:>5}/{d['br_tot']:<5} {pct(d['br_cov'], d['br_tot']):>7}  L:{pct(d['ln_cov'], d['ln_tot']):>7}  R:{pct(d['reg_cov'], d['reg_tot']):>7}  F:{pct(d['fn_cov'], d['fn_tot']):>7}"

print()
print("=== TOTALS ===")
print(fmt_row("WORKSPACE TOTAL (all)",     {k: data['data'][0]['totals'][m][k2] for (k, m, k2) in [('br_cov','branches','covered'),('br_tot','branches','count'),('reg_cov','regions','covered'),('reg_tot','regions','count'),('fn_cov','functions','covered'),('fn_tot','functions','count'),('ln_cov','lines','covered'),('ln_tot','lines','count')]}))
print(fmt_row("IN SCOPE", in_scope_total))
print(fmt_row("OUT OF SCOPE (harness bins, TUI, oracle libs)", out_scope_total))

print()
print("=== PER-CRATE (in-scope only) ===")
for crate, d in sorted(per_crate.items()):
    print(fmt_row(crate, d))

print()
print("=== TOP 30 IN-SCOPE FILES BY MISSED BRANCHES ===")
ranked = sorted(per_file.items(), key=lambda kv: -(kv[1]['br_tot'] - kv[1]['br_cov']))
for fk, d in ranked[:30]:
    miss = d['br_tot'] - d['br_cov']
    print(f"  {miss:>4} missed of {d['br_tot']:>4} ({pct(d['br_cov'], d['br_tot']):>7})  {fk}")

# Save detail to a file for later use.
with open('/tmp/cov-by-file.json','w') as f:
    out = {
        'totals_workspace': data['data'][0]['totals'],
        'totals_in_scope': in_scope_total,
        'totals_out_scope': out_scope_total,
        'per_crate': dict(per_crate),
        'per_file': {fk: d for fk, d in ranked},
    }
    json.dump(out, f, indent=2)
print()
print("Detail written to /tmp/cov-by-file.json")
