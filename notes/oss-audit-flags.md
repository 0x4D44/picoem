# Open-source audit flags

**Date:** 2026-04-26
**Scope:** publishable repository tree, using `rg` with generated/build output,
binary artefacts, vendored build directories, and gitignored upstream clones
excluded from the grep set.

## Grep battery

### Personal email

Rule: `martin@|tollens\.ai|martingdavidson@gmail\.com`

Match count: 15

Representative sample:

- `CODE_OF_CONDUCT.md:39` - `martin@tollens.ai` enforcement contact.
- `SECURITY.md:18` - `martin@tollens.ai` issue-report contact.
- `third_party/dosbox-x-picogus-tap.patch:1` - patch `From:` line uses `martingdavidson@gmail.com`.
- `wrk_journals/2026.04.17 - JRN - OneROM Stress Harness Implementation.md:43` - commit attribution note.
- `wrk_docs/2026.04.17 - HLD - Stage A.2 Silicon Oracle Residuals Handoff.md:89` - commit author convention.

### Ticket prefixes

Rule: `[A-Z]{3,}-\d{3,}`

Match count: 222

Representative sample:

- `CLAUDE.md:69` - `pre-2026-04-18`, not a ticket key.
- `wrk_journals/2026.04.13 - JRN - Phase 4 Implementation.md:31` - `SHA-256`.
- `wrk_journals/2026.04.15 - JRN - PicoGUS RP2040 Bootrom + Boot Smoke.md:27` - `cycle-8209`.
- `wrk_journals/2026.04.17 - JRN - OneROM Serving Oracle Fix Wave.md:323` - `OTP/TRNG/SHA-256`.
- `wrk_docs/2026.04.23 - HLD - OneROM CPU Serial Speed-Grade Oracle V2.md` - numeric threshold notation, not a ticket key.

### Credential words

Rule: `password|secret|token|api[_-]?key`

Match count: 193

Representative sample:

- `CLAUDE.md:123` - POWMAN password-gated register protocol.
- `NOTICE:160` - title of *The Secret of Monkey Island*.
- `tech_debt.md:463` - `token` as a launch-handshake state object.
- `wrk_journals/2026.04.17 - JRN - Coverage Gap Fill V11 Supervisor.md:191` - POWMAN password protocol.
- `wrk_docs/2026.04.26 - HLD - Open Source Release V1.md:551` - crates.io API token action item.

No actual credential value was visible in the representative sample.

### Chat and internal hostnames

Rule: `slack\.com|discord\.com|discordapp\.com|localhost|127\.0\.0\.1|\.local\b|\.internal\b|intranet`

Match count: 32

Representative sample:

- `wrk_journals/2026.04.14 - JRN - Overnight QEMU Fuzz Campaign.md:160` - local GDB port on `localhost:3334`.
- `wrk_journals/2026.04.22 - JRN - PicoGUS firmware rebuild with UART stdio.md:58` - Python package path under `~/.local`.
- `wrk_journals/2026.04.13 - JRN - QEMU Diff Harness LLD.md:30` - local GDB server.
- `wrk_journals/2026.04.13 - JRN - QEMU Diff Harness LLD.md:92` - Windows localhost latency note.
- `crates/mdpicoem-harness/tests/qemu_process_drop.rs:19` - `127.0.0.1:3333` test port.

No Slack, Discord, intranet, or private service hostname appeared in the sample.

### Named TODO markers

Rule: `TODO[(:][^)]*\b[A-Za-z]+\b`

Match count: 107

Representative sample:

- `tech_debt.md:737` - code TODO about SysTick behaviour.
- `tech_debt.md:1605` - `TODO(phase-7.1)` markers.
- `third_party/dosbox-x-picogus-tap.patch:305` - upstream DOSBox-X TODO context.
- `wrk_docs/2026.04.13 - LLD - Phase 4 Flash Boot V1.md:204` - `TODO: research exact picobin header format`.
- `wrk_journals/2026.04.14 - JRN - mdpicoem Workspace Restructure.md:253` - `TODO(Phase 4.B)` markers.

The sample did not show TODO markers naming a person.

### Co-author trailers

Rule: `Co-Authored-By:`

Match count: 4

Representative sample:

- `wrk_journals/2026.04.14 - JRN - PIO Idle Skip Implementation.md:94` - recorded Claude co-author trailer.
- `wrk_docs/2026.04.26 - HLD - Open Source Release V1.md:214` - audit rule table.
- `wrk_docs/2026.04.26 - HANDOFF - Session 2 Residual.md:56` - audit rule table.
- `CONTRIBUTING.md:72` - contribution guidance.

## Spot-read sample

Deterministic selection method: sort Markdown files by SHA-256 of
`oss-audit-seed-2026.04.26|<absolute path>` and take the first 10 per
directory.

### Journals read

- `wrk_journals/2026.04.15 - JRN - Fn-Pointer Dispatch Implementation.md`
- `wrk_journals/2026.04.17 - JRN - OneROM Serving Oracle Fix Wave.md`
- `wrk_journals/2026.04.14 - JRN - OneROM PIO Differential.md`
- `wrk_journals/2026.04.16 - JRN - ISR Tail-Chain Fast Path.md`
- `wrk_journals/2026.04.13 - JRN - Phase 2 Deferred Fixes.md`
- `wrk_journals/2026.04.16 - JRN - RP2040 Core 1 Multicore Launch Handshake.md`
- `wrk_journals/2026.04.24 - JRN - OneROM emulator perf results consolidation.md`
- `wrk_journals/2026.04.17 - JRN - Threaded Dual-Core Phase 3 V7 Implementation.md`
- `wrk_journals/2026.04.17 - JRN - RP2350 Peripheral Coverage Audit.md`
- `wrk_journals/2026.04.24 - JRN - Single-voice dump and FM trace.md`

Spot-read result: no secrets or private service references found. Tone is
candid and technical. The RP2350 peripheral coverage audit includes explicit
self-criticism of an agent mistake; that is publishable in my view, but Arthur
may want to decide whether that level of process candour belongs in public
journals.

### Design docs read

- `wrk_docs/2026.04.17 - HLD - RP2350 Coverage Gap Fill V4.md`
- `wrk_docs/2026.04.13 - LLD - Phase 5 Dual Core SIO V4.md`
- `wrk_docs/2026.04.23 - HLD - OneROM CPU Serial Speed-Grade Oracle V2.md`
- `wrk_docs/2026.04.13 - LLD - Phase 5 Dual Core SIO V2.md`
- `wrk_docs/2026.04.22 - HLD - Track A.1 RP2040 Attach Fix.md`
- `wrk_docs/2026.04.17 - HLD - Stage C.2 DMA Silicon Stall Investigation Handoff.md`
- `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V2.md`
- `wrk_docs/2026.04.14 - HLD - Phase 7 Coprocessors and FPU V1.md`
- `wrk_docs/2026.04.15 - HLD - RP2040 Peripheral Coverage V4.md`
- `wrk_docs/2026.04.13 - LLD - Phase 5 Dual Core SIO V1.md`

Spot-read result: no secrets found. Several docs contain hardware probe
serials, local debug hostnames, and implementation dead-end notes, all of
which match already-known project context.

## Triage

Probably needs Arthur's attention:

- `third_party/dosbox-x-picogus-tap.patch:1` exposes
  `martingdavidson@gmail.com` in patch metadata. This is not a secret, and it
  matches the requested commit identity, but it is the one personal email hit
  outside the intentional public contact address.
- `wrk_docs/` and `wrk_journals/` intentionally preserve blunt process notes,
  including self-criticism and failed hypotheses. I think this is acceptable
  and useful, but it is a publishing-choice call.
- `wrk_docs/2026.04.26 - HLD - Open Source Release V1.md:551` mentions a
  crates.io API token as an open action item. No token value appears, but the
  action item should remain private until Arthur is comfortable with it being
  public.

Clearly fine / no action:

- `martin@tollens.ai` appears as the intentional public contact in
  `SECURITY.md` and `CODE_OF_CONDUCT.md`.
- `password` hits are hardware register protocol terminology, mostly POWMAN
  password-gated writes.
- `secret` hits include *The Secret of Monkey Island* and are not credentials.
- `token` hits are state-machine or action-item terminology, not token
  values in the sampled matches.
- Ticket-prefix hits are false positives such as `SHA-256`, dated text, and
  cycle labels.
- `localhost` and `127.0.0.1` hits are local test/debug ports.
- `Co-Authored-By:` hits are expected transparency notes and contribution
  guidance.
