// Integration tests for QemuProcess child-cleanup on exit.
//
// Three layers per the HLD §6:
//
//   L1 (cross-platform): drop a `QemuProcess` wrapping a long-lived
//       no-op child, and assert the OS PID disappears within 500 ms.
//       Catches accidental field-reorder regressions and proves the
//       cooperative `Drop` path still kills + reaps the child.
//
//   L2 (Windows only): launch a helper binary (`test_helper_job_object`)
//       that wraps a child, prints its PID, then exits via
//       `std::process::abort()` — skipping `Drop`. Read the PID from
//       the helper's stdout, wait, then assert the PID is gone via
//       `OpenProcess`. Proves the Windows Job Object catches the
//       abnormal-exit path that `panic = "abort"` triggers in release.
//
//   L3 (Windows only, marked `#[ignore]`): launch the release
//       `qemu_diff_m33` binary, kill it externally, then bind to
//       127.0.0.1:3333. Bind-success means QEMU released the GDB port.
//       Marked `#[ignore]` because it requires the release binary to
//       have been built and a working `qemu-system-arm` install — both
//       outside the unit-test sandbox. Run manually as a smoke check.
//
// On non-Windows platforms only L1 runs; L2/L3 are gated `#[cfg(windows)]`.

use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use picoem_harness::gdb_client::{QemuProcess, QemuProfile};

// ===========================================================================
// Cross-platform helpers
// ===========================================================================

/// Spawn a long-lived no-op child. Lives ~60 seconds — far longer than
/// any test deadline.
fn spawn_long_lived_child() -> std::process::Child {
    #[cfg(windows)]
    {
        // `ping 127.0.0.1 -n 60` blocks for ~60 s with no output.
        Command::new("ping")
            .args(["127.0.0.1", "-n", "60"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .stdin(Stdio::null())
            .spawn()
            .expect("spawn ping (Windows long-lived stand-in child)")
    }
    #[cfg(not(windows))]
    {
        Command::new("sleep")
            .arg("60")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .stdin(Stdio::null())
            .spawn()
            .expect("spawn sleep (Unix long-lived stand-in child)")
    }
}

/// Returns `true` if a process with `pid` is still alive.
///
/// Polls using `OpenProcess` on Windows and `kill -0` semantics
/// (`libc::kill(pid, 0)`) on Unix. Conservative on errors: any failure
/// to query is treated as "alive" so the polling loop keeps waiting
/// rather than declaring a false success.
fn process_is_alive(pid: u32) -> bool {
    #[cfg(windows)]
    {
        use windows_sys::Win32::Foundation::{
            CloseHandle, ERROR_INVALID_PARAMETER, FALSE, GetLastError, WAIT_TIMEOUT,
        };
        use windows_sys::Win32::System::Threading::{
            GetExitCodeProcess, OpenProcess, PROCESS_QUERY_LIMITED_INFORMATION, WaitForSingleObject,
        };
        const STILL_ACTIVE: u32 = 259; // STATUS_PENDING; documented "process is still running"

        unsafe {
            let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
            if h.is_null() {
                // Most common reason after kill: ERROR_INVALID_PARAMETER
                // ("The process specified does not exist."). Treat as
                // dead. Any other error: be safe and report alive.
                let err = GetLastError();
                return err != ERROR_INVALID_PARAMETER;
            }
            // Wait briefly to distinguish "exiting but still in tables"
            // from "running". Returns WAIT_TIMEOUT if still running.
            let waited = WaitForSingleObject(h, 0);
            if waited == WAIT_TIMEOUT {
                CloseHandle(h);
                return true;
            }
            // Exited: confirm via exit code.
            let mut code: u32 = 0;
            let ok = GetExitCodeProcess(h, &mut code);
            CloseHandle(h);
            if ok == 0 {
                return true; // can't tell — be safe
            }
            code == STILL_ACTIVE
        }
    }
    #[cfg(unix)]
    {
        // `kill(pid, 0)` returns 0 if the process exists and we have
        // permission to signal it; `ESRCH` if it doesn't.
        unsafe {
            let r = libc::kill(pid as libc::pid_t, 0);
            if r == 0 {
                return true;
            }
            // SAFETY: errno is per-thread.
            let err = *libc::__errno_location();
            err != libc::ESRCH
        }
    }
    #[cfg(not(any(windows, unix)))]
    {
        // Unknown platform — assume alive so the test never falsely passes.
        let _ = pid;
        true
    }
}

/// Wait up to `timeout` for `pid` to die. Returns `true` if the process
/// was observed dead before the deadline.
fn wait_for_process_death(pid: u32, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if !process_is_alive(pid) {
            return true;
        }
        thread::sleep(Duration::from_millis(25));
    }
    !process_is_alive(pid)
}

// ===========================================================================
// L1 — cross-platform Drop / cooperative-kill test
// ===========================================================================

#[test]
fn drop_kills_child_within_500ms() {
    let child = spawn_long_lived_child();
    let qp =
        QemuProcess::from_child(child, QemuProfile::M33_RP2350).expect("wrap child in QemuProcess");
    let pid = qp.child_id();
    assert!(
        process_is_alive(pid),
        "stand-in child (pid {pid}) was already dead before drop"
    );

    drop(qp);

    let died = wait_for_process_death(pid, Duration::from_millis(500));
    assert!(
        died,
        "QemuProcess::Drop did not kill child pid {pid} within 500 ms — \
         field declaration order may have regressed"
    );
}

// ===========================================================================
// L2 — Windows Job Object enforcement (parent dies without Drop)
// ===========================================================================
//
// Drives the `test_helper_job_object` binary, which spawns a child via
// `QemuProcess::from_child` (creating + assigning a kill-on-close Job),
// prints the child PID, and aborts. `process::abort` skips `Drop` — only
// the kernel-enforced Job Object cleanup can kill the child.

#[cfg(windows)]
#[test]
fn job_object_kills_child_when_parent_aborts() {
    // Locate the built test_helper_job_object binary. Cargo builds bins
    // into the same target/<profile> directory as the tests themselves;
    // the easiest way to find it is via the `CARGO_BIN_EXE_<name>` env
    // var Cargo sets for every bin in the same package as the integration
    // test.
    let helper = env!("CARGO_BIN_EXE_test_helper_job_object");

    let mut cmd = Command::new(helper);
    cmd.stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .stdin(Stdio::null());
    let output = cmd.output().expect("run test_helper_job_object");

    // The helper aborts after printing the PID, so we expect a non-zero
    // exit code (Windows reports `STATUS_ACCESS_VIOLATION` /
    // `0xC0000409` etc. depending on libc; the *value* doesn't matter,
    // only that we got stdout).
    assert!(
        !output.status.success(),
        "helper exited cleanly (status={:?}); expected abort. stdout={:?} stderr={:?}",
        output.status,
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );

    let stdout = String::from_utf8(output.stdout).expect("helper stdout was not UTF-8");
    let pid_line = stdout
        .lines()
        .next()
        .expect("helper stdout was empty (no PID line)");
    let pid: u32 = pid_line
        .trim()
        .parse()
        .unwrap_or_else(|e| panic!("helper PID line {pid_line:?} was not a u32: {e}"));

    // Helper has already exited by the time `output()` returns. The
    // kernel reaps the parent's Job handle as part of process teardown,
    // which fires JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE. Give it 1 s.
    let died = wait_for_process_death(pid, Duration::from_secs(1));
    assert!(
        died,
        "Job Object did not kill child pid {pid} within 1 s after parent abort \
         (helper stdout={stdout:?}). The Job Object setup in QemuProcess::from_child \
         is not effective."
    );
}

// ===========================================================================
// L3 — Socket-leak regression (Windows only, manual smoke check)
// ===========================================================================
//
// Documented choice: `#[ignore]` rather than auto-run. Running this in
// `cargo test` would require:
//   1. The release binary `qemu_diff_m33` to have been built.
//   2. A working `qemu-system-arm.exe` install on PATH (or the standard
//      install location).
//   3. Spawning + killing real QEMU children, which is exactly the
//      kind of side-effecty integration the unit-test sandbox should
//      not have to depend on.
//
// L2 already proves the Job Object cleans up children when the parent
// dies abnormally — that's the primary correctness guarantee. L3 is a
// belt-and-braces "the visible socket really did get released" check
// that an operator can run by hand:
//
//     cargo build -p picoem-harness --release
//     cargo test  -p picoem-harness --test qemu_process_drop \
//                 -- --ignored socket_freed_after_external_kill
//
// (Mirror with `qemu_diff_m0plus` and port 3334 by editing the const.)

#[cfg(windows)]
#[test]
#[ignore = "requires release qemu_diff_m33 binary and working qemu-system-arm install; run manually"]
fn socket_freed_after_external_kill() {
    use std::net::TcpListener;

    use windows_sys::Win32::Foundation::{CloseHandle, FALSE};
    use windows_sys::Win32::System::Threading::{OpenProcess, PROCESS_TERMINATE, TerminateProcess};

    // Locate the release binary. We deliberately do NOT use
    // CARGO_BIN_EXE_* — that points at the dev profile under `cargo test`,
    // and L3 specifically wants the release binary because that's what
    // the workspace's panic-abort applies to.
    let exe = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("target").join("release").join("qemu_diff_m33.exe"))
        .expect("compute release binary path");
    if !exe.exists() {
        panic!(
            "release binary not built: {} — run `cargo build -p picoem-harness --release` first",
            exe.display()
        );
    }

    // Run with --fuzz 1 so the binary actually gets to the spawn-QEMU step.
    let mut child = Command::new(&exe)
        .args(["--fuzz", "1"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .stdin(Stdio::null())
        .spawn()
        .expect("spawn qemu_diff_m33 release binary");
    let pid = child.id();

    // Wait briefly so QEMU has had time to bind 3333.
    thread::sleep(Duration::from_millis(750));

    // External kill via TerminateProcess (mirrors `taskkill /F`).
    unsafe {
        let h = OpenProcess(PROCESS_TERMINATE, FALSE, pid);
        assert!(!h.is_null(), "OpenProcess on parent pid {pid} failed");
        let ok = TerminateProcess(h, 1);
        assert!(ok != 0, "TerminateProcess on parent pid {pid} failed");
        CloseHandle(h);
    }

    // Give the kernel a moment to reap the Job and tear down QEMU.
    let mut bound: Option<TcpListener> = None;
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        match TcpListener::bind("127.0.0.1:3333") {
            Ok(l) => {
                bound = Some(l);
                break;
            }
            Err(_) => thread::sleep(Duration::from_millis(50)),
        }
    }
    assert!(
        bound.is_some(),
        "GDB port 3333 was not freed within 2 s of TerminateProcess on parent pid {pid} — \
         a zombie qemu-system-arm.exe is still bound to it"
    );

    // Reap the externally-terminated child so its kernel handle is released
    // immediately rather than at end-of-test-process.
    let _ = child.wait();
}
