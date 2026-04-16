//! Full Test Runner for mdrp2350 (ARM Cortex-M33 emulator)
//!
//! Runs all tests in multiple phases:
//! 1. Code quality checks (cargo fmt, cargo clippy)
//! 2. Unit tests (cargo test --workspace)
//! 3. Emulator smoke test (differential test cases, in-process)
//!
//! Generates a markdown report in tests/results/.
//!
//! Usage:
//!   cargo run --release -p mdpicoem-harness --bin full_test_rp2350
//!
//! Options:
//!   --skip-quality    Skip code quality checks (fmt, clippy)
//!   --skip-unit       Skip cargo test unit tests
//!   --skip-smoke      Skip emulator smoke test
//!   --quick           Alias for --skip-quality
//!   --help, -h        Show help text

use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use chrono::Local;

const VERSION: &str = env!("CARGO_PKG_VERSION");

// ============================================================================
// Low priority
// ============================================================================

/// Set the process (and all future children) to low CPU scheduling priority.
fn set_low_priority() {
    #[cfg(unix)]
    {
        unsafe extern "C" {
            fn nice(inc: i32) -> i32;
        }
        // SAFETY: nice() is async-signal-safe and always succeeds for positive increments.
        unsafe {
            nice(10);
        }
    }
    #[cfg(windows)]
    {
        unsafe extern "system" {
            fn GetCurrentProcess() -> *mut std::ffi::c_void;
            fn SetPriorityClass(hProcess: *mut std::ffi::c_void, dwPriorityClass: u32) -> i32;
        }
        const BELOW_NORMAL_PRIORITY_CLASS: u32 = 0x0000_4000;
        // SAFETY: GetCurrentProcess() returns a pseudo-handle that never needs closing.
        unsafe {
            SetPriorityClass(GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS);
        }
    }
}

// ============================================================================
// Options
// ============================================================================

struct Options {
    skip_quality: bool,
    skip_unit: bool,
    skip_smoke: bool,
}

fn parse_options(args: &[String]) -> Option<Options> {
    let mut opts = Options {
        skip_quality: false,
        skip_unit: false,
        skip_smoke: false,
    };
    for arg in args.iter().skip(1) {
        match arg.as_str() {
            "--skip-quality" | "--quick" => opts.skip_quality = true,
            "--skip-unit" => opts.skip_unit = true,
            "--skip-smoke" => opts.skip_smoke = true,
            "--help" | "-h" => return None,
            other => {
                eprintln!("Unknown option: {other}");
                return None;
            }
        }
    }
    Some(opts)
}

fn print_help() {
    println!("full_test_rp2350 v{VERSION}");
    println!();
    println!("Full test runner for mdrp2350 ARM Cortex-M33 emulator.");
    println!();
    println!("USAGE:");
    println!("    cargo run --release -p mdpicoem-harness --bin full_test_rp2350 [OPTIONS]");
    println!();
    println!("OPTIONS:");
    println!("    --skip-quality    Skip code quality checks (fmt, clippy)");
    println!("    --skip-unit       Skip cargo test unit tests");
    println!("    --skip-smoke      Skip emulator smoke test");
    println!("    --quick           Alias for --skip-quality");
    println!("    --help, -h        Show this help message");
    println!();
    println!("PHASES:");
    println!("    1. Code quality checks (cargo fmt --check, cargo clippy)");
    println!("    2. Unit tests (cargo test --workspace --release)");
    println!("    3. Emulator smoke test (differential test cases, in-process)");
    println!();
    println!("OUTPUT:");
    println!("    Generates markdown report in tests/results/");
    println!();
    println!("EXAMPLES:");
    println!("    cargo run --release -p mdpicoem-harness --bin full_test_rp2350");
    println!("        Run full test suite");
    println!();
    println!("    cargo run --release -p mdpicoem-harness --bin full_test_rp2350 -- --quick");
    println!("        Skip code quality checks");
}

// ============================================================================
// Data types
// ============================================================================

#[derive(Debug, Clone)]
struct TestResult {
    name: String,
    passed: bool,
}

#[derive(Debug)]
struct SuiteResults {
    total: usize,
    passed: usize,
    failed: usize,
    skipped: usize,
    duration: Duration,
    tests: Vec<TestResult>,
}

#[derive(Debug)]
struct QualityResult {
    check_name: String,
    passed: bool,
    duration: Duration,
    issues: Vec<String>,
}

#[derive(Debug)]
struct EmuSmokeResults {
    total: usize,
    passed: usize,
    failed: usize,
    duration: Duration,
    failures: Vec<(String, String)>,
}

#[derive(Debug, Clone)]
struct CategoryStats {
    name: String,
    total: usize,
    passed: usize,
    failed: usize,
}

// ============================================================================
// Phase 1: Code quality
// ============================================================================

/// Parse issues from `cargo fmt -- --check` output.
fn parse_fmt_issues(stdout: &str, stderr: &str) -> Vec<String> {
    let mut issues = Vec::new();
    for line in stdout.lines().chain(stderr.lines()) {
        let trimmed = line.trim();
        if !trimmed.is_empty()
            && (trimmed.starts_with("Diff in")
                || trimmed.ends_with(".rs")
                || trimmed.contains("would reformat")
                || trimmed.contains("error")
                || trimmed.contains("warning:"))
        {
            issues.push(trimmed.to_string());
        }
    }
    issues
}

// NOTE: Assumes CWD is the workspace root (true when invoked via `cargo run`).
fn run_fmt_check() -> QualityResult {
    let start = Instant::now();

    let mut cmd = Command::new("cargo");
    cmd.args(["fmt", "--check", "--"]);
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let output = cmd.output().expect("Failed to execute cargo fmt");
    let duration = start.elapsed();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let passed = output.status.success();
    let issues = parse_fmt_issues(&stdout, &stderr);

    QualityResult {
        check_name: "cargo fmt".to_string(),
        passed,
        duration,
        issues,
    }
}

// NOTE: Assumes CWD is the workspace root (true when invoked via `cargo run`).
fn run_clippy() -> QualityResult {
    let start = Instant::now();

    let mut cmd = Command::new("cargo");
    cmd.args([
        "clippy",
        "--workspace",
        "--lib",
        "--tests",
        "--message-format=short",
        "--",
        "-D",
        "warnings",
    ]);
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let output = cmd.output().expect("Failed to execute cargo clippy");
    let duration = start.elapsed();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let passed = output.status.success();

    let mut issues = Vec::new();
    for line in stdout.lines().chain(stderr.lines()) {
        let trimmed = line.trim();
        if trimmed.contains("warning:") || trimmed.contains("error:") || trimmed.contains("error[")
        {
            issues.push(trimmed.to_string());
        }
    }

    QualityResult {
        check_name: "cargo clippy".to_string(),
        passed,
        duration,
        issues,
    }
}

// ============================================================================
// Phase 2: Unit tests
// ============================================================================

fn parse_test_output(stdout: &str, stderr: &str, duration: Duration) -> SuiteResults {
    let mut tests = Vec::new();
    let mut total = 0usize;
    let mut passed = 0usize;
    let mut failed = 0usize;
    let mut skipped = 0usize;

    for line in stdout.lines().chain(stderr.lines()) {
        if line.starts_with("test ")
            && (line.contains(" ... ok")
                || line.contains(" ... FAILED")
                || line.contains(" ... ignored"))
        {
            let parts: Vec<&str> = line.split(" ... ").collect();
            if parts.len() >= 2 {
                let name = parts[0].trim_start_matches("test ").to_string();
                let status = parts[1].trim();

                total += 1;
                let test_passed = match status {
                    "ok" => {
                        passed += 1;
                        true
                    }
                    "FAILED" => {
                        failed += 1;
                        false
                    }
                    "ignored" => {
                        skipped += 1;
                        continue;
                    }
                    _ => continue,
                };

                tests.push(TestResult {
                    name,
                    passed: test_passed,
                });
            }
        }
    }

    SuiteResults {
        total,
        passed,
        failed,
        skipped,
        duration,
        tests,
    }
}

fn run_cargo_test() -> SuiteResults {
    let start = Instant::now();

    let mut cmd = Command::new("cargo");
    cmd.args([
        "test",
        "--workspace",
        "--lib",
        "--tests",
        "--release",
        "--no-fail-fast",
        "--",
        "--test-threads=1",
    ]);
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let output = cmd.output().expect("Failed to execute cargo test");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let duration = start.elapsed();

    let mut results = parse_test_output(&stdout, &stderr, duration);

    if !output.status.success() {
        let status_label = output
            .status
            .code()
            .map_or_else(|| "terminated".to_string(), |c| format!("exit code {c}"));
        eprintln!("Warning: cargo test exited with {status_label}");
        for line in stderr.lines().take(10) {
            eprintln!("  {line}");
        }
        // If cargo test crashed without reporting individual test failures,
        // synthesize one so the report reflects the failure.
        if results.failed == 0 {
            results.total += 1;
            results.failed += 1;
            results.tests.push(TestResult {
                name: format!("[cargo test {status_label}]"),
                passed: false,
            });
        }
    }

    results
}

// ============================================================================
// Phase 3: Emulator smoke test
// ============================================================================

// ============================================================================
// Categorization helpers
// ============================================================================

/// Categorize a smoke test by instruction set and mnemonic.
///
/// Uses `hw1` to distinguish Thumb-32 from Thumb-16, and the first word of
/// the test name as the instruction mnemonic.
fn categorize_smoke_test(name: &str, is_thumb32: bool) -> String {
    let prefix = if is_thumb32 { "T32" } else { "T16" };
    let mnemonic = name.split_whitespace().next().unwrap_or("unknown");
    format!("{prefix} {mnemonic}")
}

/// Build per-category stats for smoke tests.
fn categorize_smoke_results(
    test_names: &[(String, bool, bool)], // (name, is_thumb32, passed)
) -> Vec<CategoryStats> {
    let mut categories: HashMap<String, CategoryStats> = HashMap::new();

    for (name, is_thumb32, test_passed) in test_names {
        let category = categorize_smoke_test(name, *is_thumb32);
        let stats = categories.entry(category.clone()).or_insert(CategoryStats {
            name: category,
            total: 0,
            passed: 0,
            failed: 0,
        });
        stats.total += 1;
        if *test_passed {
            stats.passed += 1;
        } else {
            stats.failed += 1;
        }
    }

    let mut result: Vec<CategoryStats> = categories.into_values().collect();
    result.sort_by(|a, b| b.failed.cmp(&a.failed).then_with(|| a.name.cmp(&b.name)));
    result
}

/// Run the smoke test with per-test metadata for categorization.
///
/// See `run_emu_smoke_test` for the panic=abort rationale.
fn run_emu_smoke_test_with_categories() -> (EmuSmokeResults, Vec<CategoryStats>) {
    use mdpicoem_harness::*;

    let tests = generate_all();
    let mut bus = Bus::new();
    let mut test_meta = Vec::new();
    let start = Instant::now();

    for tc in &tests {
        let is_thumb32 = tc.hw1.is_some();
        run_one_emu(tc, &mut bus);
        test_meta.push((tc.name.clone(), is_thumb32, true));
    }

    let duration = start.elapsed();
    let categories = categorize_smoke_results(&test_meta);

    let results = EmuSmokeResults {
        total: tests.len(),
        passed: tests.len(),
        failed: 0,
        duration,
        failures: Vec::new(),
    };

    (results, categories)
}

/// Extract category from unit test name by taking the first 2 path components.
/// e.g., "core::thumb16::tests::test_lsls" -> "core::thumb16"
fn extract_unit_category(test_name: &str) -> String {
    let parts: Vec<&str> = test_name.split("::").collect();
    if parts.len() >= 2 {
        format!("{}::{}", parts[0], parts[1])
    } else if !parts.is_empty() && !parts[0].is_empty() {
        parts[0].to_string()
    } else {
        "uncategorized".to_string()
    }
}

/// Build per-category stats for unit tests.
fn categorize_unit_tests(tests: &[TestResult]) -> Vec<CategoryStats> {
    let mut categories: HashMap<String, CategoryStats> = HashMap::new();

    for test in tests {
        let category = extract_unit_category(&test.name);
        let stats = categories.entry(category.clone()).or_insert(CategoryStats {
            name: category,
            total: 0,
            passed: 0,
            failed: 0,
        });
        stats.total += 1;
        if test.passed {
            stats.passed += 1;
        } else {
            stats.failed += 1;
        }
    }

    let mut result: Vec<CategoryStats> = categories.into_values().collect();
    result.sort_by(|a, b| b.failed.cmp(&a.failed).then_with(|| a.name.cmp(&b.name)));
    result
}

// ============================================================================
// Report generation
// ============================================================================

fn build_report_path(now: chrono::DateTime<Local>) -> PathBuf {
    let date = now.format("%Y.%m.%d");
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("results");

    let base = dir.join(format!("{date} - full test report.md"));
    if !base.exists() {
        return base;
    }

    for n in 2.. {
        let path = dir.join(format!("{date} - full test report ({n}).md"));
        if !path.exists() {
            return path;
        }
    }
    unreachable!()
}

fn write_report(report_path: &PathBuf, report: &str) -> std::io::Result<()> {
    if let Some(parent) = report_path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let mut file = std::fs::File::create(report_path)?;
    file.write_all(report.as_bytes())?;
    Ok(())
}

fn generate_report(
    unit_results: Option<&SuiteResults>,
    smoke_results: Option<&EmuSmokeResults>,
    smoke_categories: Option<&[CategoryStats]>,
    quality_results: &[QualityResult],
    total_duration: Duration,
) -> String {
    let now = Local::now();
    let mut report = String::new();

    // Title
    report.push_str(&format!(
        "# Full Test Report - {}\n\n",
        now.format("%Y.%m.%d %H:%M:%S")
    ));

    // Summary table
    report.push_str("## Summary\n\n");
    report.push_str("| Phase | Total | Passed | Failed | Skipped | Duration |\n");
    report.push_str("|-------|-------|--------|--------|---------|----------|\n");

    if let Some(r) = unit_results {
        report.push_str(&format!(
            "| Unit Tests | {} | {} | {} | {} | {:.2}s |\n",
            r.total,
            r.passed,
            r.failed,
            r.skipped,
            r.duration.as_secs_f64()
        ));
    }

    if let Some(r) = smoke_results {
        report.push_str(&format!(
            "| Emulator Smoke | {} | {} | {} | 0 | {:.2}s |\n",
            r.total,
            r.passed,
            r.failed,
            r.duration.as_secs_f64()
        ));
    }

    report.push('\n');

    // Code Quality section
    if !quality_results.is_empty() {
        report.push_str("## Code Quality\n\n");
        report.push_str("| Check | Status | Duration | Issues |\n");
        report.push_str("|-------|--------|----------|--------|\n");

        for result in quality_results {
            let status = if result.passed { "PASS" } else { "FAIL" };
            report.push_str(&format!(
                "| {} | {} | {:.2}s | {} |\n",
                result.check_name,
                status,
                result.duration.as_secs_f64(),
                result.issues.len()
            ));
        }
        report.push('\n');

        // Show issues for failing checks
        for result in quality_results {
            if !result.passed && !result.issues.is_empty() {
                report.push_str(&format!("### {} Issues\n\n", result.check_name));
                for issue in result.issues.iter().take(20) {
                    report.push_str(&format!("- {issue}\n"));
                }
                if result.issues.len() > 20 {
                    report.push_str(&format!("- ... and {} more\n", result.issues.len() - 20));
                }
                report.push('\n');
            }
        }
    }

    // Emulator Smoke Test section
    if let Some(r) = smoke_results {
        report.push_str("## Emulator Smoke Test\n\n");
        report.push_str(&format!(
            "Executed {} differential test cases through the emulator (no QEMU).\n\n",
            r.total
        ));

        if let Some(cats) = smoke_categories {
            report.push_str("| Category | Total | Passed | Failed |\n");
            report.push_str("|----------|-------|--------|--------|\n");
            for cat in cats {
                report.push_str(&format!(
                    "| {} | {} | {} | {} |\n",
                    cat.name, cat.total, cat.passed, cat.failed
                ));
            }
            report.push('\n');
        }

        if !r.failures.is_empty() {
            report.push_str("### Failures\n\n");
            for (name, msg) in &r.failures {
                report.push_str(&format!("- **{name}**: {msg}\n"));
            }
            report.push('\n');
        }
    }

    // Unit Test Breakdown section
    if let Some(r) = unit_results {
        let categories = categorize_unit_tests(&r.tests);
        if !categories.is_empty() {
            report.push_str("## Unit Test Breakdown\n\n");
            report.push_str("| Category | Total | Passed | Failed |\n");
            report.push_str("|----------|-------|--------|--------|\n");
            for cat in &categories {
                report.push_str(&format!(
                    "| {} | {} | {} | {} |\n",
                    cat.name, cat.total, cat.passed, cat.failed
                ));
            }
            report.push('\n');
        }

        let failed_tests: Vec<&TestResult> = r.tests.iter().filter(|t| !t.passed).collect();
        if !failed_tests.is_empty() {
            report.push_str("### Failed Tests\n\n");
            for t in &failed_tests {
                report.push_str(&format!("- {}\n", t.name));
            }
            report.push('\n');
        }
    }

    report.push_str(&format!(
        "---\n\nTotal duration: {:.2}s\n",
        total_duration.as_secs_f64()
    ));

    report
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    mdpicoem_harness::harness_tracing_init();
    set_low_priority();

    let args: Vec<String> = std::env::args().collect();
    let options = match parse_options(&args) {
        Some(opts) => opts,
        None => {
            print_help();
            return;
        }
    };

    let now = Local::now();
    let overall_start = Instant::now();

    println!("mdrp2350 full test runner v{VERSION}");
    println!();

    // Phase 1: Code quality
    let quality_results = if !options.skip_quality {
        println!("[1/4] Code quality checks...");
        let fmt = run_fmt_check();
        println!(
            "  cargo fmt:    {} ({:.1}s)",
            if fmt.passed { "PASS" } else { "FAIL" },
            fmt.duration.as_secs_f64()
        );
        let clippy = run_clippy();
        println!(
            "  cargo clippy: {} ({:.1}s)",
            if clippy.passed { "PASS" } else { "FAIL" },
            clippy.duration.as_secs_f64()
        );
        println!();
        vec![fmt, clippy]
    } else {
        println!("[1/4] Code quality checks... SKIPPED");
        println!();
        Vec::new()
    };

    // Phase 2: Unit tests
    let unit_results = if !options.skip_unit {
        println!("[2/4] Unit tests...");
        println!("  Running cargo test --workspace --release...");
        let results = run_cargo_test();
        println!(
            "  {}/{} passed ({:.1}s)",
            results.passed,
            results.total,
            results.duration.as_secs_f64()
        );
        println!();
        Some(results)
    } else {
        println!("[2/4] Unit tests... SKIPPED");
        println!();
        None
    };

    // Phase 3: Emulator smoke test
    let (smoke_results, smoke_categories) = if !options.skip_smoke {
        println!("[3/4] Emulator smoke test...");
        println!("  Running differential test cases...");
        let (results, cats) = run_emu_smoke_test_with_categories();
        println!(
            "  {}/{} passed ({:.1}s)",
            results.passed,
            results.total,
            results.duration.as_secs_f64()
        );
        println!();
        (Some(results), Some(cats))
    } else {
        println!("[3/4] Emulator smoke test... SKIPPED");
        println!();
        (None, None)
    };

    // Phase 4: Report
    let total_duration = overall_start.elapsed();
    let report = generate_report(
        unit_results.as_ref(),
        smoke_results.as_ref(),
        smoke_categories.as_deref(),
        &quality_results,
        total_duration,
    );

    let report_path = build_report_path(now);
    println!("[4/4] Generating report...");
    write_report(&report_path, &report).expect("Failed to write report");
    println!("  Saved to {}", report_path.display());
    println!();

    // Final summary
    let unit_failed = unit_results.as_ref().map_or(0, |r| r.failed);
    let smoke_failed = smoke_results.as_ref().map_or(0, |r| r.failed);
    let quality_failed = quality_results.iter().filter(|q| !q.passed).count();
    let any_failed = unit_failed > 0 || smoke_failed > 0 || quality_failed > 0;

    let quality_total = quality_results.len();
    let quality_passed = quality_total - quality_failed;

    if any_failed {
        println!("=== RESULT: FAILURES DETECTED ===");
    } else {
        println!("=== RESULT: ALL PASSED ===");
    }

    if let Some(r) = &unit_results {
        println!("  Unit tests:     {}/{}", r.passed, r.total);
    }
    if let Some(r) = &smoke_results {
        println!("  Emulator smoke: {}/{}", r.passed, r.total);
    }
    if !quality_results.is_empty() {
        println!(
            "  Code quality:   {}/{} checks passed",
            quality_passed, quality_total
        );
    }
    println!("  Duration:       {:.1}s", total_duration.as_secs_f64());

    if any_failed {
        std::process::exit(1);
    }
}
