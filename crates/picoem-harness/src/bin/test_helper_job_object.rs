// Test-only helper binary for the L2 Job Object integration test.
//
// Verifies that when the parent process dies *abnormally* (skipping Drop),
// the Windows Job Object created by `QemuProcess::from_child` still cleans
// up the child process via `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE`.
//
// Sequence:
//   1. Spawn a long-lived stand-in child (`ping -n 60 127.0.0.1`).
//   2. Wrap it in a `QemuProcess` so the Job Object is created and the
//      child is assigned to it.
//   3. Print the child's OS PID to stdout (one decimal integer + newline).
//   4. Flush stdout so the test harness can read the PID.
//   5. Exit via `std::process::abort()` — skips Drop, mirrors the
//      panic-abort path the workspace uses in release builds.
//
// The test on the calling side reads the PID, waits for the kernel to
// reap the Job, and asserts the PID is gone via `OpenProcess`.
//
// Not gated `#[cfg(windows)]` because the test that drives it already is;
// on non-Windows platforms this binary builds and the helper just exits
// without doing anything useful (the corresponding test never invokes it).

use std::io::Write;

#[cfg(windows)]
use std::process::{Command, Stdio};

#[cfg(windows)]
use picoem_harness::gdb_client::{QemuProcess, QemuProfile};

fn main() {
    picoem_harness::harness_tracing_init();
    #[cfg(windows)]
    {
        // Stand-in child that lasts long enough for the test to observe.
        // `ping 127.0.0.1 -n 60` takes ~60 seconds; far longer than the
        // 1 s the test waits before asserting the kill.
        let child = Command::new("ping")
            .args(["127.0.0.1", "-n", "60"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .stdin(Stdio::null())
            .spawn()
            .expect("spawn ping");

        // Wrap in a QemuProcess so the Job Object is created and the
        // child is assigned to it. The profile is irrelevant for this
        // test — we never connect a GDB client.
        let qp = QemuProcess::from_child(child, QemuProfile::M33_RP2350)
            .expect("wrap child in QemuProcess");
        let pid = qp.child_id();

        // Tell the test harness which PID to watch.
        println!("{pid}");
        let _ = std::io::stdout().flush();

        // Forget the wrapper to ensure even Drop-on-the-stack (which
        // would otherwise run during `process::abort`'s teardown on some
        // libc implementations) cannot kill the child cooperatively.
        // We *want* the abnormal-exit path: only the kernel-enforced
        // Job Object cleanup should fire here.
        std::mem::forget(qp);

        // Skip Drop entirely — mimics `panic = "abort"` and external
        // TerminateProcess. Whatever happens to the ping child after
        // this point is the kernel's doing, via the Job Object.
        std::process::abort();
    }

    #[cfg(not(windows))]
    {
        // Non-Windows: nothing to test; emit a marker line and exit.
        // The L2 test that consumes this binary is `#[cfg(windows)]`,
        // so this branch is never invoked under the test harness.
        println!("0");
        let _ = std::io::stdout().flush();
    }
}
