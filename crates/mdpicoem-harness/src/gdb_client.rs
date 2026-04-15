// GDB RSP client and QEMU process manager for differential testing.
//
// Minimal GDB Remote Serial Protocol client implementing 5 packet types:
//   p/P (read/write register), m/M (read/write memory), s (single-step).
//
// Tested with QEMU 7.0–10.2:
//   * MPS2-AN505 machine, Cortex-M33 CPU (RP2350 oracle).
//   * Microbit machine, Cortex-M0 CPU (RP2040 oracle — QEMU 10.2 does not
//     ship a cortex-m0plus CPU model; the M0+ ISA is a strict superset of
//     the M0 ISA plus a 2-cycle MUL and the MOVS IR register pseudo-op).
//
// Note: QEMU's M-profile GDB stub omits EPSR.T (bit 24) from xPSR reads.
// The Thumb bit is implicit — Cortex-M always runs in Thumb mode.
// Indices 16-24 (legacy FPA) return E14 (unsupported) on QEMU 10.2.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use crate::REG_XPSR;

// ============================================================================
// QemuProfile — which QEMU machine/CPU/port to spawn
// ============================================================================

/// Selects the QEMU machine, CPU model and GDB port for [`QemuProcess::spawn_with`].
///
/// The two profiles in use by the harness correspond to the two chips modelled:
///   * [`QemuProfile::M33_RP2350`] — MPS2-AN505 + cortex-m33 on port 3333.
///   * [`QemuProfile::M0_PLUS_RP2040`] — microbit + cortex-m0 on port 3334.
///
/// QEMU 10.2 only exposes `cortex-m0` for -cpu; there is no `cortex-m0plus`
/// model. The M0+ is a strict superset of M0 for the ARMv6-M Thumb-16/Thumb-32
/// subset we care about (the Pico SDK uses the same binary for either chip),
/// so the M0 oracle is an acceptable M0+ reference for differential testing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QemuProfile {
    pub machine: &'static str,
    pub cpu: &'static str,
    pub gdb_port: u16,
}

impl QemuProfile {
    /// MPS2-AN505 + cortex-m33 on port 3333 (the mdrp2350 oracle).
    pub const M33_RP2350: Self = Self {
        machine: "mps2-an505",
        cpu: "cortex-m33",
        gdb_port: 3333,
    };
    /// Microbit + cortex-m0 on port 3334 (the mdrp2040 oracle).
    ///
    /// Port 3334 is used so the two harnesses can run concurrently on the
    /// same host without fighting for 3333.
    pub const M0_PLUS_RP2040: Self = Self {
        machine: "microbit",
        cpu: "cortex-m0",
        gdb_port: 3334,
    };

    /// Formatted `tcp::<port>` string for the `-gdb` argument.
    fn gdb_arg(&self) -> String {
        format!("tcp::{}", self.gdb_port)
    }

    /// Formatted `localhost:<port>` string for [`GdbClient::connect`].
    pub fn gdb_addr(&self) -> String {
        format!("localhost:{}", self.gdb_port)
    }
}

// ============================================================================
// QemuProcess — manages the QEMU child process lifetime
// ============================================================================
//
// Lifetime guarantee: the QEMU child must die when the parent (this) process
// exits — for *any* reason. The cooperative path is the explicit
// `child.kill()` + `child.wait()` in `Drop::drop` below; this fires on
// normal return, `?`-propagated errors, and panic-unwind.
//
// On Windows the workspace runs with `panic = "abort"` in release, which
// skips `Drop` entirely. External `TerminateProcess` (taskkill /F, sibling
// agents, Defender) bypasses userspace too. For both of these the safety
// net is a Windows Job Object created with
// `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` and assigned to the spawned QEMU:
// when the parent dies abnormally, the kernel closes the parent's handle
// to the Job, the Job tears down every process inside it, and QEMU is
// killed without any userspace involvement.
//
// Field declaration order matters: `_job` must drop *after* `child` so that
// in the cooperative path QEMU is already gone before the Job handle closes.

#[cfg(windows)]
mod job {
    //! Windows Job Object glue for `QemuProcess`.
    //!
    //! Best-effort: any failure here logs to stderr but does NOT fail the
    //! spawn — the cooperative `Drop` path remains the primary kill, and
    //! the Job is insurance for the abnormal-exit case.

    use std::process::Child;
    use windows_sys::Win32::Foundation::{
        CloseHandle, FALSE, HANDLE, INVALID_HANDLE_VALUE,
    };
    use windows_sys::Win32::System::JobObjects::{
        AssignProcessToJobObject, CreateJobObjectW, SetInformationJobObject,
        JobObjectExtendedLimitInformation, JOBOBJECT_BASIC_LIMIT_INFORMATION,
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION, JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE,
    };
    use windows_sys::Win32::System::Threading::{
        OpenProcess, PROCESS_SET_QUOTA, PROCESS_TERMINATE,
    };

    /// RAII wrapper around a Win32 Job Object handle.
    ///
    /// Closing the last handle to a Job created with
    /// `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` causes the kernel to terminate
    /// every process assigned to the Job.
    pub(super) struct JobHandle(HANDLE);

    impl Drop for JobHandle {
        fn drop(&mut self) {
            // SAFETY: `self.0` was returned by a successful
            // `CreateJobObjectW` and has not been closed yet.
            unsafe {
                CloseHandle(self.0);
            }
        }
    }

    /// Wrap `child` in a kill-on-close Job Object.
    ///
    /// Any failure is logged to stderr; we return `None` and let the
    /// caller proceed without the safety net — the cooperative kill in
    /// `Drop::drop` is still wired, so the only behaviour we lose is
    /// kernel-enforced cleanup on abnormal exit. We deliberately do NOT
    /// fail the spawn for a Win32 edge case that would never be the
    /// user's fault.
    pub(super) fn assign_to_kill_on_close_job(child: &Child) -> Option<JobHandle> {
        // SAFETY: see comments inline. All Win32 calls are documented to
        // accept the arguments we pass.
        unsafe {
            let job = CreateJobObjectW(std::ptr::null(), std::ptr::null());
            if job.is_null() || job == INVALID_HANDLE_VALUE {
                eprintln!(
                    "warning: CreateJobObjectW failed (errno {}); QEMU child \
                     will not be kernel-cleaned on abnormal parent exit",
                    std::io::Error::last_os_error()
                );
                return None;
            }
            let job = JobHandle(job);

            // Configure kill-on-close.
            let mut info: JOBOBJECT_EXTENDED_LIMIT_INFORMATION = std::mem::zeroed();
            info.BasicLimitInformation = JOBOBJECT_BASIC_LIMIT_INFORMATION {
                LimitFlags: JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE,
                ..std::mem::zeroed()
            };
            let info_size =
                std::mem::size_of::<JOBOBJECT_EXTENDED_LIMIT_INFORMATION>() as u32;
            let ok = SetInformationJobObject(
                job.0,
                JobObjectExtendedLimitInformation,
                &info as *const _ as *const _,
                info_size,
            );
            if ok == 0 {
                eprintln!(
                    "warning: SetInformationJobObject failed (errno {}); \
                     QEMU child will not be kernel-cleaned on abnormal parent exit",
                    std::io::Error::last_os_error()
                );
                return None;
            }

            // Open the child with the rights needed for AssignProcessToJobObject.
            let proc_handle = OpenProcess(
                PROCESS_SET_QUOTA | PROCESS_TERMINATE,
                FALSE,
                child.id(),
            );
            if proc_handle.is_null() {
                eprintln!(
                    "warning: OpenProcess(child={}) failed (errno {}); \
                     QEMU child will not be kernel-cleaned on abnormal parent exit",
                    child.id(),
                    std::io::Error::last_os_error()
                );
                return None;
            }

            let assigned = AssignProcessToJobObject(job.0, proc_handle);
            // We only needed the process handle long enough to call Assign;
            // the kernel keeps the assignment alive via the Job itself.
            CloseHandle(proc_handle);

            if assigned == 0 {
                eprintln!(
                    "warning: AssignProcessToJobObject failed (errno {}); \
                     QEMU child will not be kernel-cleaned on abnormal parent exit",
                    std::io::Error::last_os_error()
                );
                return None;
            }

            Some(job)
        }
    }
}

/// Owns a QEMU child process. Kills it on drop.
///
/// On Windows the spawned child is also assigned to a kill-on-close Job
/// Object — see the module-level comment above.
pub struct QemuProcess {
    child: Child,
    profile: QemuProfile,
    /// Windows-only. Field exists purely so its `Drop` runs (which closes
    /// the last handle to the Job, triggering kernel cleanup). Declared
    /// **last** so it drops *after* `child` in the abnormal-exit case
    /// where the explicit `Drop::drop` did not run cooperatively.
    #[cfg(windows)]
    _job: Option<job::JobHandle>,
}

impl QemuProcess {
    /// Standard Windows install path (winget / qemu.org installer).
    const WINDOWS_QEMU_PATH: &'static str =
        r"C:\Program Files\qemu\qemu-system-arm.exe";

    /// Spawn `qemu-system-arm` with the default M33 / RP2350 profile.
    ///
    /// Returns an error with a clear message if `qemu-system-arm` is not found.
    pub fn spawn() -> io::Result<Self> {
        Self::spawn_with(QemuProfile::M33_RP2350)
    }

    /// Spawn `qemu-system-arm` with the given profile.
    ///
    /// Used by the M0+ harness (`qemu_diff_m0plus`) to target a microbit /
    /// cortex-m0 oracle on a non-conflicting port.
    pub fn spawn_with(profile: QemuProfile) -> io::Result<Self> {
        let gdb_arg = profile.gdb_arg();
        let args = [
            "-machine",
            profile.machine,
            "-cpu",
            profile.cpu,
            "-nographic",
            "-S",
            "-gdb",
            gdb_arg.as_str(),
        ];

        // Try PATH first, then the standard Windows install location.
        let child = Command::new("qemu-system-arm")
            .args(&args)
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .stdin(Stdio::null())
            .spawn()
            .or_else(|e| {
                if e.kind() == io::ErrorKind::NotFound {
                    Command::new(Self::WINDOWS_QEMU_PATH)
                        .args(&args)
                        .stdout(Stdio::null())
                        .stderr(Stdio::piped())
                        .stdin(Stdio::null())
                        .spawn()
                } else {
                    Err(e)
                }
            })
            .map_err(|e| {
                if e.kind() == io::ErrorKind::NotFound {
                    io::Error::new(
                        io::ErrorKind::NotFound,
                        "qemu-system-arm not found on PATH or at \
                         C:\\Program Files\\qemu\\. \
                         Install QEMU >= 7.0 (winget install \
                         SoftwareFreedomConservancy.QEMU).",
                    )
                } else {
                    io::Error::new(
                        e.kind(),
                        format!("failed to spawn qemu-system-arm: {e}"),
                    )
                }
            })?;

        Self::from_child(child, profile)
    }

    /// Wrap an already-spawned child process in a `QemuProcess`.
    ///
    /// Public so integration tests can construct a `QemuProcess` around a
    /// stand-in child (e.g. `ping` / `sleep`) without going through the
    /// full QEMU command-line. Production callers should use [`spawn`] /
    /// [`spawn_with`] instead.
    #[doc(hidden)]
    pub fn from_child(child: Child, profile: QemuProfile) -> io::Result<Self> {
        #[cfg(windows)]
        {
            let _job = job::assign_to_kill_on_close_job(&child);
            Ok(Self { child, profile, _job })
        }
        #[cfg(not(windows))]
        {
            Ok(Self { child, profile })
        }
    }

    /// Returns the OS-level PID of the child for test assertions.
    ///
    /// Hidden from production callers; tests use this to verify the child
    /// disappears after `Drop`.
    #[doc(hidden)]
    pub fn child_id(&self) -> u32 {
        self.child.id()
    }

    /// Returns the profile used to spawn this QEMU instance. Useful for
    /// reconnecting the GDB client after a respawn.
    pub fn profile(&self) -> QemuProfile {
        self.profile
    }
}

impl Drop for QemuProcess {
    fn drop(&mut self) {
        // Cooperative fast path: kill QEMU immediately and reap the zombie.
        // On Windows the Job handle closes after this returns (field drop
        // order: child first, then `_job`); QEMU is already dead by then.
        // In abnormal-exit paths where this does not run (panic-abort,
        // external TerminateProcess), the kernel closes the parent's Job
        // handle as part of process teardown, which kills QEMU via
        // JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE.
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

// ============================================================================
// GdbClient — minimal GDB RSP client over TCP
// ============================================================================

/// Minimal GDB Remote Serial Protocol client.
///
/// Implements per-register read/write (`p`/`P`), memory read/write (`m`/`M`),
/// and single-step (`s`). Uses the ACK protocol (send `+` after receiving,
/// expect `+` after sending).
pub struct GdbClient {
    stream: TcpStream,
    buf: Vec<u8>,
}

impl GdbClient {
    /// Connect to a GDB server with retry loop.
    ///
    /// Retries every 100ms until `timeout` elapses. Sets `TCP_NODELAY`
    /// (critical on Windows to avoid Nagle delays) and a 10-second read
    /// timeout to catch QEMU hangs.
    pub fn connect(addr: &str, timeout: Duration) -> io::Result<Self> {
        let deadline = Instant::now() + timeout;
        let mut last_err = io::Error::new(io::ErrorKind::TimedOut, "connect timeout");

        while Instant::now() < deadline {
            match TcpStream::connect(addr) {
                Ok(stream) => {
                    stream.set_nodelay(true)?;
                    stream.set_read_timeout(Some(Duration::from_secs(10)))?;
                    return Ok(Self {
                        stream,
                        buf: Vec::with_capacity(1024),
                    });
                }
                Err(e) => {
                    last_err = e;
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        }

        Err(io::Error::new(
            last_err.kind(),
            format!("failed to connect to GDB server at {addr}: {last_err}"),
        ))
    }

    /// Query halt reason (`?` packet). Verifies QEMU is stopped.
    ///
    /// Expects a `T` (signal with info) or `S` (signal) stop reply.
    pub fn handshake(&mut self) -> io::Result<()> {
        let reply = self.send_recv("?")?;
        if reply.starts_with('T') || reply.starts_with('S') {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "unexpected handshake reply: expected T or S stop reply, got '{reply}'"
                ),
            ))
        }
    }

    /// Read one 32-bit register via `p` packet.
    ///
    /// `index` is the GDB register index (0-12 for R0-R12, 13=SP, 14=LR,
    /// 15=PC, 25=xPSR).
    pub fn read_reg(&mut self, index: u8) -> io::Result<u32> {
        let payload = format!("p{:x}", index);
        let reply = self.send_recv(&payload)?;

        if reply.len() != 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "read_reg({index}): expected 8 hex chars, got {} ('{reply}')",
                    reply.len()
                ),
            ));
        }

        decode_le_hex32(&reply).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("read_reg({index}): invalid hex in response '{reply}'"),
            )
        })
    }

    /// Write one 32-bit register via `P` packet.
    ///
    /// Value is encoded as little-endian hex (target byte order for ARM).
    pub fn write_reg(&mut self, index: u8, value: u32) -> io::Result<()> {
        let payload = format!("P{:x}={}", index, encode_le_hex32(value));
        let reply = self.send_recv(&payload)?;

        if reply != "OK" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("write_reg({index}, {value:#010x}): expected OK, got '{reply}'"),
            ));
        }
        Ok(())
    }

    /// Read memory via `m` packet. Returns raw bytes.
    pub fn read_mem(&mut self, addr: u32, len: usize) -> io::Result<Vec<u8>> {
        let payload = format!("m{:x},{:x}", addr, len);
        let reply = self.send_recv(&payload)?;

        if reply.len() != len * 2 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "read_mem({addr:#010x}, {len}): expected {} hex chars, got {} ('{reply}')",
                    len * 2,
                    reply.len()
                ),
            ));
        }

        decode_hex_bytes(&reply).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("read_mem({addr:#010x}, {len}): invalid hex in response '{reply}'"),
            )
        })
    }

    /// Write memory via `M` packet.
    pub fn write_mem(&mut self, addr: u32, data: &[u8]) -> io::Result<()> {
        let hex_data = encode_hex_bytes(data);
        let payload = format!("M{:x},{:x}:{}", addr, data.len(), hex_data);
        let reply = self.send_recv(&payload)?;

        if reply != "OK" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "write_mem({addr:#010x}, {} bytes): expected OK, got '{reply}'",
                    data.len()
                ),
            ));
        }
        Ok(())
    }

    /// Single-step via `s` packet. Blocks until QEMU stops.
    pub fn step(&mut self) -> io::Result<()> {
        let reply = self.send_recv("s")?;
        if reply.starts_with('T') || reply.starts_with('S') {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("step: expected stop reply (T/S), got '{reply}'"),
            ))
        }
    }

    /// Send kill packet. Best-effort — errors are ignored.
    pub fn kill(&mut self) {
        let _ = self.send_packet("k");
    }

    // ========================================================================
    // Internal packet framing
    // ========================================================================

    /// Send a GDB RSP packet: `$payload#XX` where XX is the checksum.
    fn send_packet(&mut self, payload: &str) -> io::Result<()> {
        let checksum = gdb_checksum(payload.as_bytes());
        // Format: $payload#XX
        write!(self.stream, "${}#{:02x}", payload, checksum)?;
        self.stream.flush()
    }

    /// Receive a GDB RSP packet. Reads the ACK (`+`), then `$response#XX`,
    /// verifies checksum, and sends ACK back.
    ///
    /// Returns the response payload (without framing).
    fn recv_packet(&mut self) -> io::Result<String> {
        // Read bytes until we get the full packet. The stream might deliver
        // data in chunks, so we accumulate in self.buf.
        self.buf.clear();
        let mut one = [0u8; 1];

        // Skip any leading `+` ACK bytes (response to our previous send).
        loop {
            self.stream.read_exact(&mut one)?;
            if one[0] != b'+' {
                self.buf.push(one[0]);
                break;
            }
        }

        // We should now have `$` as the first byte.
        if self.buf[0] != b'$' {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "expected '$' packet start, got {:#04x} ('{}')",
                    self.buf[0], self.buf[0] as char
                ),
            ));
        }

        // Read until `#XX` (hash + 2 hex digits).
        let mut found_hash = false;
        let mut checksum_chars = 0u8;

        loop {
            self.stream.read_exact(&mut one)?;
            self.buf.push(one[0]);

            if found_hash {
                checksum_chars += 1;
                if checksum_chars == 2 {
                    break;
                }
            } else if one[0] == b'#' {
                found_hash = true;
            }
        }

        // Parse: $payload#XX
        // buf[0] = '$', payload is buf[1..hash_pos], checksum is buf[hash_pos+1..hash_pos+3]
        let hash_pos = self.buf.iter().rposition(|&b| b == b'#').unwrap();
        let payload = &self.buf[1..hash_pos];
        let cksum_str = std::str::from_utf8(&self.buf[hash_pos + 1..])
            .map_err(|_| {
                io::Error::new(io::ErrorKind::InvalidData, "checksum is not valid UTF-8")
            })?;

        let received_cksum = u8::from_str_radix(cksum_str, 16).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid checksum hex: '{cksum_str}'"),
            )
        })?;

        let expected_cksum = gdb_checksum(payload);
        if received_cksum != expected_cksum {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "checksum mismatch: expected {expected_cksum:02x}, got {received_cksum:02x}"
                ),
            ));
        }

        let result = String::from_utf8(payload.to_vec()).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidData, "response payload is not valid UTF-8")
        })?;

        // Send ACK.
        self.stream.write_all(b"+")?;
        self.stream.flush()?;

        Ok(result)
    }

    /// Send a packet and receive the response.
    fn send_recv(&mut self, payload: &str) -> io::Result<String> {
        self.send_packet(payload)?;
        self.recv_packet()
    }
}

// ============================================================================
// Sanity check
// ============================================================================

/// Verify GDB register indices are correct.
///
/// Writes a known value to R0, reads it back, then confirms that the xPSR
/// register index returns a plausible status register value.
///
/// Note: QEMU's M-profile GDB stub omits EPSR.T (bit 24) from the xPSR
/// read — the Thumb bit is implicit (always 1 on Cortex-M). We cannot
/// check it here, so we verify the index is valid and no unexpected bits
/// are set in the reset-halt state.
pub fn sanity_check(gdb: &mut GdbClient) -> io::Result<()> {
    // 1. Round-trip a GP register to confirm the index mapping works.
    let probe: u32 = 0xDEAD_BEEF;
    gdb.write_reg(0, probe)?;
    let readback = gdb.read_reg(0)?;
    gdb.write_reg(0, 0)?; // restore
    if readback != probe {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "R0 round-trip failed: wrote {probe:#010x}, read {readback:#010x}. \
                 GDB register encoding may be wrong."
            ),
        ));
    }

    // 2. Read xPSR and verify the index is valid (returns 8 hex chars).
    //    At reset-halt, only condition flags (bits 31:27) may be set.
    //    QEMU omits EPSR.T from this read, so we don't check bit 24.
    let xpsr = gdb.read_reg(REG_XPSR)?;

    // Bits 26:25 (ICI/IT) and bits 15:10 (ICI/IT) should be zero at reset.
    // Exception number bits [8:0] are excluded: QEMU may boot into HardFault
    // if the vector table wasn't present during the reset sequence.
    let unexpected = xpsr & 0x0600_FC00;
    if unexpected != 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "xPSR has unexpected bits set at reset: {xpsr:#010x} \
                 (unexpected={unexpected:#010x}). Register index {REG_XPSR} may be wrong."
            ),
        ));
    }

    Ok(())
}

// ============================================================================
// Helpers — checksum, hex encoding/decoding
// ============================================================================

/// GDB RSP checksum: sum of all bytes modulo 256.
fn gdb_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

/// Encode a u32 as 8 little-endian hex characters.
///
/// Example: `0x12345678` -> `"78563412"` (LE byte order: 0x78, 0x56, 0x34, 0x12).
fn encode_le_hex32(value: u32) -> String {
    let bytes = value.to_le_bytes();
    format!("{:02x}{:02x}{:02x}{:02x}", bytes[0], bytes[1], bytes[2], bytes[3])
}

/// Decode 8 little-endian hex characters to a u32.
///
/// Example: `"78563412"` -> `Some(0x12345678)`.
fn decode_le_hex32(hex: &str) -> Option<u32> {
    if hex.len() != 8 {
        return None;
    }
    let b0 = u8::from_str_radix(&hex[0..2], 16).ok()?;
    let b1 = u8::from_str_radix(&hex[2..4], 16).ok()?;
    let b2 = u8::from_str_radix(&hex[4..6], 16).ok()?;
    let b3 = u8::from_str_radix(&hex[6..8], 16).ok()?;
    Some(u32::from_le_bytes([b0, b1, b2, b3]))
}

/// Encode a byte slice as hex string (2 chars per byte).
fn encode_hex_bytes(data: &[u8]) -> String {
    let mut s = String::with_capacity(data.len() * 2);
    for &b in data {
        use std::fmt::Write;
        let _ = write!(s, "{:02x}", b);
    }
    s
}

/// Decode a hex string to bytes (2 hex chars per byte).
fn decode_hex_bytes(hex: &str) -> Option<Vec<u8>> {
    if hex.len() % 2 != 0 {
        return None;
    }
    let mut bytes = Vec::with_capacity(hex.len() / 2);
    for i in (0..hex.len()).step_by(2) {
        bytes.push(u8::from_str_radix(&hex[i..i + 2], 16).ok()?);
    }
    Some(bytes)
}

// ============================================================================
// Unit tests — helpers only (GDB client requires a running QEMU instance)
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- gdb_checksum --

    #[test]
    fn checksum_empty() {
        assert_eq!(gdb_checksum(b""), 0);
    }

    #[test]
    fn checksum_question_mark() {
        // '?' = 0x3F
        assert_eq!(gdb_checksum(b"?"), 0x3F);
    }

    #[test]
    fn checksum_known_value() {
        // "g" = 0x67
        assert_eq!(gdb_checksum(b"g"), 0x67);
    }

    #[test]
    fn checksum_wraps_at_256() {
        // Two bytes that sum > 255: 0xFF + 0x01 = 0x100 -> wraps to 0x00
        assert_eq!(gdb_checksum(&[0xFF, 0x01]), 0x00);
    }

    #[test]
    fn checksum_multi_byte() {
        // "p0" = 0x70 + 0x30 = 0xA0
        assert_eq!(gdb_checksum(b"p0"), 0xA0);
    }

    // -- encode_le_hex32 / decode_le_hex32 --

    #[test]
    fn roundtrip_zero() {
        let hex = encode_le_hex32(0);
        assert_eq!(hex, "00000000");
        assert_eq!(decode_le_hex32(&hex), Some(0));
    }

    #[test]
    fn roundtrip_one() {
        let hex = encode_le_hex32(1);
        assert_eq!(hex, "01000000");
        assert_eq!(decode_le_hex32(&hex), Some(1));
    }

    #[test]
    fn roundtrip_max() {
        let hex = encode_le_hex32(0xFFFF_FFFF);
        assert_eq!(hex, "ffffffff");
        assert_eq!(decode_le_hex32(&hex), Some(0xFFFF_FFFF));
    }

    #[test]
    fn roundtrip_thumb_bit() {
        // xPSR with T bit set: 0x01000000
        let hex = encode_le_hex32(0x0100_0000);
        assert_eq!(hex, "00000001");
        assert_eq!(decode_le_hex32(&hex), Some(0x0100_0000));
    }

    #[test]
    fn roundtrip_mixed() {
        let hex = encode_le_hex32(0x12345678);
        assert_eq!(hex, "78563412");
        assert_eq!(decode_le_hex32(&hex), Some(0x12345678));
    }

    #[test]
    fn decode_wrong_length() {
        assert_eq!(decode_le_hex32("1234"), None);
        assert_eq!(decode_le_hex32("123456789"), None);
    }

    #[test]
    fn decode_invalid_hex() {
        assert_eq!(decode_le_hex32("ZZZZZZZZ"), None);
    }

    // -- encode_hex_bytes / decode_hex_bytes --

    #[test]
    fn hex_bytes_empty() {
        assert_eq!(encode_hex_bytes(&[]), "");
        assert_eq!(decode_hex_bytes(""), Some(vec![]));
    }

    #[test]
    fn hex_bytes_roundtrip() {
        let data = vec![0x00, 0xFF, 0xAB, 0x12];
        let hex = encode_hex_bytes(&data);
        assert_eq!(hex, "00ffab12");
        assert_eq!(decode_hex_bytes(&hex), Some(data));
    }

    #[test]
    fn hex_bytes_odd_length_fails() {
        assert_eq!(decode_hex_bytes("abc"), None);
    }

    #[test]
    fn hex_bytes_invalid_chars_fails() {
        assert_eq!(decode_hex_bytes("zz"), None);
    }

    // -- encode_le_hex32 byte order --

    #[test]
    fn le_hex_byte_order() {
        // 0xDEADBEEF in LE bytes: EF, BE, AD, DE
        let hex = encode_le_hex32(0xDEAD_BEEF);
        assert_eq!(hex, "efbeadde");
    }
}
