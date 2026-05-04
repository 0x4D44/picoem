//! OneROM fixture pin-map and capacity descriptor — Stage 1 of the fixture
//! generalization.
//!
//! Single value parsed from a loaded OneROM `.bin` that captures everything
//! the byte-correctness sweep + the two serving oracles need to know about
//! the hardware variant: which GPIOs carry data, address, and chip-select
//! lines, what to drive (or deassert) during reads, which pin patterns are
//! unservable, and how big the per-set shadow is.
//!
//! Design: `wrk_docs/2026.05.04 - HLD - OneROM Serving Oracle Fixture
//! Generalization.md` §4.1, §4.2.
//!
//! Struct offsets (sdrr_info_t, onerom_metadata_header_t, sdrr_rom_set_t,
//! sdrr_pins_t) mirror the upstream OneROM header at
//! `sdrr/include/config_base.h` from <https://github.com/piersfinlayson/one-rom>.
//! Pin a specific commit when the upstream layout drifts; the fields we
//! consume have been stable since v0.6.2 (the `extended` byte gates the
//! 256-byte struct shape).
//!
//! `addr_pins` reflects the chip's *native* address width (read from
//! `sdrr_pins_t.addr[]` + `addr2[]`), not the socket's wired width. A
//! 27C010 (17 native pins) on the same fire-32-a socket would yield
//! `addr_pins.len() == 17` even though the socket exposes 19 lines.
//!
//! Stage 1 introduced [`FixtureSpec`] + [`FixtureSpec::from_flash`] and
//! consolidated the SDRR metadata helpers ([`parse_rom_set_layout`],
//! [`RomSetSlot`], the `SDRR_INFO_*` / `METADATA_HEADER_*` / `ROM_SET_*`
//! constants, [`FLASH_BASE`]) here. Stage 2 widened the serving oracles
//! to consume `FixtureSpec` directly and collapsed the legacy
//! `onerom_serving_oracle.rs` re-export shim; this module now owns the
//! metadata helpers outright.

use std::error::Error;
use std::fmt;

// ---------------------------------------------------------------------------
// Public constants
// ---------------------------------------------------------------------------

/// Upper bound on valid GPIO pin numbers across the OneROM target family.
///
/// RP2350A bondout exposes 30 GPIOs, RP2350B exposes 48. We use the wider
/// limit so the same parser accepts both fire-24-a (RP2350A; max GPIO in
/// the pin map = 29) and fire-32-a (RP2350B; max GPIO in the pin map = 47:
/// `external_flash.cs_pin = 47`). Pins at or above this value are rejected
/// as invalid. This mirrors the firmware's own `MAX_USED_GPIOS` constants
/// (`reg-rp235x.h`: `MAX_USED_GPIOS_RP2350B = 48`).
///
/// Trade-off: this looser bound accepts an RP2350B-targeted fixture even
/// when run against an RP2350A. We accept that to support both MCUs in
/// one parser; per-MCU narrowing (rejecting GPIOs ≥ 30 when targeting
/// RP2350A) is a follow-up if a fixture-validation failure ever traces
/// back to a wrong-MCU fixture binary.
pub const MAX_USED_GPIOS: u8 = 48;

/// Legacy constant kept for the existing fire-24-a CPU-serve oracle's
/// `parse_sel_pins` helper. It happens to be the correct flash offset for
/// `sdrr_pins_t` in the fire-24-a SDRR firmware build but **not** for
/// fire-32-a, where the struct lives at 0x9460 (per the `sdrr_info_t.pins`
/// pointer chain — see [`FixtureSpec::from_flash`]). New code should walk
/// the pointer chain via [`FixtureSpec::from_flash`] rather than hardcoding
/// either offset.
pub const SDRR_PINS_FLASH_OFFSET: usize = 0x80FC;

// ---------------------------------------------------------------------------
// SDRR struct chain layout — moved here from `onerom_serving_oracle.rs`.
// Keep these `pub`: the legacy module re-exports them so existing call sites
// (`build_seabios_fixture`, the in-crate tests) continue to compile while
// Stage 1 lands.
// ---------------------------------------------------------------------------

/// SDRR structs are all expressed as XIP addresses; subtract this to
/// get a byte offset into the loaded `.bin`.
pub const FLASH_BASE: u32 = 0x1000_0000;

/// Offset of `sdrr_info_t` within flash (per `sdrr/link/common.ld`:
/// `flash_isr_vector` + boot block ends at `0x200`, `sdrr_info_t`
/// follows).
pub const SDRR_INFO_OFFSET: usize = 0x0200;

/// Field offset of `metadata_header` pointer within `sdrr_info_t`
/// (see `sdrr_info_t` comments in `sdrr/include/config_base.h`).
pub const SDRR_INFO_METADATA_PTR_OFFSET: usize = 44;

/// Field offset of `pins` pointer (`sdrr_pins_t *`) within `sdrr_info_t`.
/// See `sdrr/include/config_base.h`: `pins` lives at offset 48 immediately
/// after `metadata_header`. Following this pointer is the **only**
/// authoritative way to locate `sdrr_pins_t`; the `0x80FC` constant we
/// used pre-Stage-1 was an accident of the fire-24-a build's link layout.
pub const SDRR_INFO_PINS_PTR_OFFSET: usize = 48;

/// Field offset of `rom_sets` pointer within `onerom_metadata_header_t`.
pub const METADATA_HEADER_ROM_SETS_PTR_OFFSET: usize = 24;

/// Field offset of `rom_set_count` within `onerom_metadata_header_t`.
pub const METADATA_HEADER_ROM_SET_COUNT_OFFSET: usize = 20;

/// Stride of `sdrr_rom_set_t` in the `rom_sets` array. The struct is
/// padded to 64 bytes (see `pad2[40]` in `config_base.h`).
pub const ROM_SET_STRIDE: usize = 64;

/// Field offset of `data` pointer within `sdrr_rom_set_t`.
pub const ROM_SET_DATA_PTR_OFFSET: usize = 0;

/// Field offset of `size` within `sdrr_rom_set_t`.
pub const ROM_SET_SIZE_OFFSET: usize = 4;

// ---------------------------------------------------------------------------
// `sdrr_pins_t` field offsets within the struct (relative to its base in
// flash). The struct is 256 bytes from v0.6.2 onward; layout pinned in
// `sdrr/include/config_base.h`. We only call out the fields the
// byte-correctness sweep + oracles care about; the rest are skipped.
// ---------------------------------------------------------------------------

const SDRR_PINS_DATA_OFFSET: usize = 8; // data[8]
const SDRR_PINS_ADDR_OFFSET: usize = 16; // addr[16]
const SDRR_PINS_CS1_OFFSET: usize = 36;
const SDRR_PINS_CS2_OFFSET: usize = 37;
const SDRR_PINS_CS3_OFFSET: usize = 38;
const SDRR_PINS_CE_OFFSET: usize = 44;
const SDRR_PINS_OE_OFFSET: usize = 45;
const SDRR_PINS_EXTENDED_OFFSET: usize = 63;
const SDRR_PINS_ADDR2_OFFSET: usize = 72; // addr2[16] — pins beyond A15
const SDRR_PINS_STRUCT_SIZE: usize = 256;

/// Sentinel value for an unused pin slot. Mirrors `INVALID_PIN` in
/// `sdrr/include/config_base.h`.
const INVALID_PIN: u8 = 0xFF;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Hardware-variant pin map and capacity for one OneROM fixture.
///
/// Parsed from the loaded `.bin` by [`FixtureSpec::from_flash`].
#[derive(Clone, Debug)]
pub struct FixtureSpec {
    /// Variant tag, for diagnostics. e.g. `"24pin/0x10000-shadow"`.
    pub label: &'static str,

    /// Chip footprint in pin count: 24 for the fire-24-a 27C-series
    /// socket, 32 for the fire-32-a 27C020/27C040 socket. Read straight
    /// from `sdrr_pins_t` offset +5. Stage 2 oracles will use this to
    /// pick stim modes (e.g. fire-24-a CS1-gated reads vs fire-32-a
    /// CS2-aliased-A16 reads).
    pub chip_pins: u8,

    /// GPIO pin per address line, low-to-high (A0 first). The vector
    /// length is the address width in bits — 13 for fire-24-a/27C256,
    /// 19 for fire-32-a/27C020. Built from `addr[16]` followed by
    /// `addr2[16]` (the ">16-bit" extra slots), stopping at the first
    /// `0xFF` sentinel.
    pub addr_pins: Vec<u8>,

    /// GPIO pin per data line, D0..D7. Always 8 entries.
    /// fire-24-a: 16..=23. fire-32-a: 0..=7.
    pub data_pins: [u8; 8],

    /// Primary CS pin slot.
    ///
    /// On fire-24-a (`chip_pins == 24`) this is the active-low gate
    /// that controls whether D0..D7 are driven (e.g. /CS on a 27C256
    /// at GPIO13).
    ///
    /// On fire-32-a (`chip_pins == 32`) the value may be a generator-
    /// emitted placeholder — the 27C020 chip type doesn't use CS1, but
    /// `sdrr-gen` unconditionally writes `board.pin_cs1(Chip27C080)`
    /// for 32-pin boards. **Don't drive `cs1` LOW during reads on
    /// 32-pin fixtures unless you've verified the chip type uses it.**
    /// On fire-32-a/27C020 the actual gate is CS2 (aliased to A16);
    /// see [`Self::unservable_when_high`].
    pub cs1: u8,

    /// Pins that must be driven LOW during reads to enable serving.
    /// fire-32-a uses this for /CE (P0:15) and /OE (P0:14). fire-24-a
    /// has no such pins (CS1 is the read-active gate; no separate CE/OE
    /// signals on the 24-pin board) — the empty vec is the right
    /// answer.
    ///
    /// CE/OE entries that alias any of `cs1/cs2/cs3` are filtered out:
    /// on fire-24-a the firmware writes `ce=cs3` and `oe=cs1` as
    /// firmware-level aliases (no real CE/OE pins on the 24-pin board),
    /// and surfacing those duplicates here would double-write the
    /// chip-select levels.
    pub asserted_low_during_read: Vec<u8>,

    /// Pins that must be driven HIGH during reads (deasserted, since
    /// the chip-select sense is active-low).
    ///
    /// fire-24-a: CS2 (P0:12) and CS3 (P0:15). Both alias address pins
    /// in the 24-pin pin map but are still listed — HLD §4.4 stim
    /// composition is order-tolerant: `deasserted_high` writes HIGH,
    /// then `pin_pattern` ORs in, so the bit stays HIGH whether or not
    /// the address pattern would have set it.
    ///
    /// fire-32-a in our 27C020 fixture: empty. fire-32-a has no CS3
    /// pin, and its CS2 (P0:16) is the active-low gate aliased to A16
    /// — captured by [`Self::unservable_when_high`] rather than driven
    /// to a fixed deasserted level (HLD §4.1, §5.1). The gate CS is
    /// excluded from this list to avoid double-counting.
    pub deasserted_high_during_read: Vec<u8>,

    /// Pin pattern bits that, when high, make the chip un-servable
    /// (data lines tristated by the firmware). The byte-correctness
    /// sweep skips any pin pattern P where `(P & unservable_when_high)
    /// != 0`. This is uniform across fixtures regardless of whether
    /// the unservable bit is a "real" CS GPIO outside `addr_pins` or
    /// an address-pin-aliased CS.
    ///
    /// fire-24-a: `1 << 13` — CS1 at GPIO13 is the active-low gate;
    /// CS1=high tristates D0..D7 (the existing fire-24-a SeaBIOS
    /// path's `CS1_BIT` filter becomes this).
    ///
    /// fire-32-a/27C020: `1 << 16` — CS2 is at GPIO16, which IS A16
    /// in the address bus. Setting A16 high also raises CS2, which
    /// deselects the chip (HLD §5.1). 2^18 of 2^19 patterns are
    /// skipped.
    ///
    /// Discriminated by `chip_pins`: 24 → CS1-gate, 32 → CS2-gate.
    /// Other values are rejected at parse time (no other footprints
    /// are known today).
    pub unservable_when_high: u64,

    /// Shadow size in bytes — comes from the per-set `sdrr_rom_set_t.size`
    /// field of the **first** ROM set. fire-24-a/27C256: 65 536.
    /// fire-32-a/27C020: 524 288 (firmware-baked permutation table is
    /// 27C040-capacity-sized even for 27C020 content; HLD §5.2).
    pub shadow_size: usize,
}

/// Layout descriptor for one ROM set in an SDRR fixture image. Pairs
/// the byte offset within the flash where the set's data lives with
/// the declared size (in bytes). Returned by [`parse_rom_set_layout`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RomSetSlot {
    /// Byte offset within the flash image of the `sdrr_rom_set_t`
    /// descriptor itself (the 64-byte struct entry in the `rom_sets[]`
    /// array). Useful for fixture authors that need to patch fields of
    /// the descriptor (e.g. the `roms[]` pointer at `+0x08`).
    pub descriptor_offset: usize,
    /// Byte offset within the flash image of the start of this set's
    /// pre-processed ROM data.
    pub data_offset: usize,
    /// Declared `size` field of the `sdrr_rom_set_t` entry, in bytes.
    pub size: usize,
}

/// Errors surfaced by [`FixtureSpec::from_flash`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FixtureError {
    /// Flash image too short to contain the SDRR struct chain or the
    /// `sdrr_pins_t` it points to.
    Truncated,

    /// A pin field carried a value that's not a valid GPIO number on
    /// any RP2350 variant we support (>= [`MAX_USED_GPIOS`]).
    BadPin {
        /// Field that carried the bad value (e.g. `"data[0]"`,
        /// `"cs1"`, `"addr[5]"`).
        field: &'static str,
        /// The offending value as read from the firmware.
        value: u8,
    },

    /// `sdrr_pins_t.extended` was 0 — pre-v0.6.2 fixtures with a 64-byte
    /// (non-extended) pins struct. We don't support those: they lack
    /// `data2`/`addr2`, so >16-bit address layouts (fire-32-a) can't be
    /// expressed.
    UnknownLayout {
        /// Why we couldn't parse it.
        reason: &'static str,
    },

    /// SDRR metadata declared zero ROM sets, or [`parse_rom_set_layout`]
    /// otherwise failed to walk the chain. We need at least the first
    /// set to derive `shadow_size`.
    MissingRomSet,

    /// The first ROM set's declared size is zero (or otherwise nonsense
    /// for a shadow buffer).
    InvalidShadowSize {
        /// The offending size as read from `sdrr_rom_set_t.size`.
        size: usize,
    },
}

impl fmt::Display for FixtureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated => write!(f, "OneROM fixture: flash image truncated"),
            Self::BadPin { field, value } => write!(
                f,
                "OneROM fixture: field {field} carries pin {value}, which is not a valid GPIO (must be < {MAX_USED_GPIOS})"
            ),
            Self::UnknownLayout { reason } => {
                write!(f, "OneROM fixture: unknown sdrr_pins_t layout — {reason}")
            }
            Self::MissingRomSet => write!(
                f,
                "OneROM fixture: no parseable ROM set; cannot derive shadow_size"
            ),
            Self::InvalidShadowSize { size } => write!(
                f,
                "OneROM fixture: rom_set[0].size = {size}, which is not a valid shadow size"
            ),
        }
    }
}

impl Error for FixtureError {}

impl FixtureSpec {
    /// Parse a [`FixtureSpec`] out of a loaded OneROM `.bin`.
    ///
    /// Walks the SDRR struct chain (`sdrr_info_t` at `flash + 0x200` →
    /// `sdrr_pins_t` via the pointer at `sdrr_info_t + 48`) plus the
    /// metadata pointer chain via [`parse_rom_set_layout`]. Validates
    /// every pin number against [`MAX_USED_GPIOS`]; rejects pre-v0.6.2
    /// (non-extended) `sdrr_pins_t` layouts.
    ///
    /// Returns a [`FixtureError`] on any parse failure. Callers in the
    /// test/diag binaries `unwrap()`; production code (no production
    /// code today consumes this — only test binaries) should propagate
    /// or surface the typed error.
    ///
    /// **Memory note:** the [`FixtureSpec::label`] field is allocated
    /// via [`Box::leak`] to satisfy the `&'static str` type (one leak
    /// per parse). Acceptable on test/diag binaries that call this
    /// once at startup; **do not call in a tight hot loop** — the
    /// leaked label strings would accumulate.
    pub fn from_flash(flash: &[u8]) -> Result<Self, FixtureError> {
        // 0. Upfront truncation check — the flash must be large enough
        //    to contain `sdrr_info_t` (which holds the metadata + pins
        //    pointers we walk below). Without this, a too-small flash
        //    can fail later in `parse_rom_set_layout` and surface the
        //    less-specific `MissingRomSet` instead of the truthful
        //    `Truncated`. The minimum needed is `SDRR_INFO_OFFSET +
        //    SDRR_INFO_PINS_PTR_OFFSET + 4` (to read both the metadata
        //    pointer at +44 and the pins pointer at +48).
        let min_info_bytes = SDRR_INFO_OFFSET
            .checked_add(SDRR_INFO_PINS_PTR_OFFSET)
            .and_then(|x| x.checked_add(4))
            .ok_or(FixtureError::Truncated)?;
        if flash.len() < min_info_bytes {
            return Err(FixtureError::Truncated);
        }

        // 1. Walk the metadata chain to derive shadow_size from the
        //    first ROM set. Doing this first lets the label string
        //    include the shadow size (and surfaces metadata-shape errors
        //    before we touch sdrr_pins_t).
        let layout = parse_rom_set_layout(flash).ok_or(FixtureError::MissingRomSet)?;
        let first_set = layout.first().ok_or(FixtureError::MissingRomSet)?;
        let shadow_size = first_set.size;
        if shadow_size == 0 {
            return Err(FixtureError::InvalidShadowSize { size: shadow_size });
        }

        // 2. Locate sdrr_pins_t via the pointer at sdrr_info_t + 48.
        //    HLD §4.2 says "0x80FC for both fixtures" — this is wrong;
        //    fire-32-a places sdrr_pins_t at 0x9460. Following the
        //    pointer is the only authoritative method.
        let pins_ptr_off = SDRR_INFO_OFFSET + SDRR_INFO_PINS_PTR_OFFSET;
        let pins_ptr = u32::from_le_bytes(
            flash[pins_ptr_off..pins_ptr_off + 4]
                .try_into()
                .map_err(|_| FixtureError::Truncated)?,
        );
        let pins_off = (pins_ptr.checked_sub(FLASH_BASE).ok_or(FixtureError::Truncated)?) as usize;
        if flash.len() < pins_off + SDRR_PINS_STRUCT_SIZE {
            return Err(FixtureError::Truncated);
        }
        let pins = &flash[pins_off..pins_off + SDRR_PINS_STRUCT_SIZE];

        // 3. extended must be 1 (256-byte struct, v0.6.2+). Pre-extended
        //    fixtures have neither data2 nor addr2 and we can't model
        //    >16-bit-address fixtures without addr2.
        if pins[SDRR_PINS_EXTENDED_OFFSET] != 1 {
            return Err(FixtureError::UnknownLayout {
                reason: "non-extended sdrr_pins_t (pre-v0.6.2)",
            });
        }

        // 4. data[8] — eight required pins, all valid.
        let mut data_pins = [0u8; 8];
        for ii in 0..8 {
            let pin = pins[SDRR_PINS_DATA_OFFSET + ii];
            if pin >= MAX_USED_GPIOS {
                return Err(FixtureError::BadPin {
                    field: data_field_name(ii),
                    value: pin,
                });
            }
            data_pins[ii] = pin;
        }

        // 4a. Reject non-contiguous data layouts. Both serving oracles
        // compute the served byte as `(gpio_in >> data_pins[0]) & 0xFF`,
        // which only holds when D0..D7 land on consecutive GPIOs. Both
        // known fixtures satisfy this (fire-24-a: 16..=23; fire-32-a:
        // 0..=7), so the check is non-breaking; it traps any future
        // fixture variant that splits the byte across non-adjacent pins
        // before the silent miscompose reaches the byte-correct sweep.
        for ii in 1..8 {
            if data_pins[ii] != data_pins[0].wrapping_add(ii as u8) {
                return Err(FixtureError::UnknownLayout {
                    reason: "data pins must be contiguous (D0..D7 on consecutive GPIOs)",
                });
            }
        }

        // 5. addr[16] then addr2[16], scanning up to the first 0xFF
        //    sentinel. Each entry validated against MAX_USED_GPIOS.
        //
        //    Per HLD §4.1, fire-32-a needs 19 address pins which spill
        //    into addr2[0..3]. The original spec sheet only mentioned
        //    addr[16]; we also walk addr2 because empirically that's
        //    where the high pins of fire-32-a live (matching the
        //    canonical generator output and `sdrr-info -d`).
        let mut addr_pins: Vec<u8> = Vec::with_capacity(32);
        let scan = |slice_off: usize, len: usize, label: fn(usize) -> &'static str, sink: &mut Vec<u8>|
            -> Result<bool, FixtureError>
        {
            for ii in 0..len {
                let pin = pins[slice_off + ii];
                if pin == INVALID_PIN {
                    return Ok(false); // sentinel reached → stop scanning further slots
                }
                if pin >= MAX_USED_GPIOS {
                    return Err(FixtureError::BadPin {
                        field: label(ii),
                        value: pin,
                    });
                }
                sink.push(pin);
            }
            Ok(true) // ran to end; addr2 may continue
        };
        let saw_full_addr = scan(SDRR_PINS_ADDR_OFFSET, 16, addr_field_name, &mut addr_pins)?;
        if saw_full_addr {
            scan(SDRR_PINS_ADDR2_OFFSET, 16, addr2_field_name, &mut addr_pins)?;
        }

        // 6. cs1 — required, must be valid.
        let cs1 = pins[SDRR_PINS_CS1_OFFSET];
        if cs1 >= MAX_USED_GPIOS {
            return Err(FixtureError::BadPin {
                field: "cs1",
                value: cs1,
            });
        }

        // 7. cs2, cs3, ce, oe — all optional (0xFF sentinel = absent).
        let cs2 = pins[SDRR_PINS_CS2_OFFSET];
        let cs3 = pins[SDRR_PINS_CS3_OFFSET];
        let ce = pins[SDRR_PINS_CE_OFFSET];
        let oe = pins[SDRR_PINS_OE_OFFSET];
        for &(field, value) in &[("cs2", cs2), ("cs3", cs3), ("ce", ce), ("oe", oe)] {
            if value != INVALID_PIN && value >= MAX_USED_GPIOS {
                return Err(FixtureError::BadPin { field, value });
            }
        }

        // 8. chip_pins (offset +5) — the discriminator that picks the
        //    unservable_when_high gate (CS1 vs CS2-aliased-A16) and the
        //    label format. Read it before composing the CS-derived
        //    fields below.
        let chip_pins = pins[5];

        // 9. Compute unservable_when_high (HLD §5.1) — discriminated by
        //    `chip_pins`:
        //
        //    - 24-pin (fire-24-a 27C-family): CS1 is the active-low
        //      gate; CS1=high tristates D0..D7. Mask = `1 << cs1`.
        //    - 32-pin (fire-32-a 27C020): CS2 is the active-low gate
        //      AND aliases A16 in the address bus. Setting A16 high
        //      raises CS2 which deselects the chip. Mask = `1 << cs2`.
        //
        //    Other footprints aren't modelled; reject explicitly so a
        //    new chip type fails loudly here rather than silently
        //    producing a wrong mask.
        let unservable_when_high: u64 = match chip_pins {
            24 => {
                // Fire-24-a 27C-family on 24-pin socket: CS1 is the
                // active-low gate.
                1u64 << cs1
            }
            32 => {
                // Fire-32-a 27C020 on 32-pin socket: CS2 is the
                // active-low gate and aliases A16 in the address bus
                // (HLD §5.1).
                if cs2 == INVALID_PIN || !addr_pins.contains(&cs2) {
                    return Err(FixtureError::UnknownLayout {
                        reason: "32-pin fixture: CS2 must be present and alias an address pin (27C020 layout)",
                    });
                }
                1u64 << cs2
            }
            _ => {
                return Err(FixtureError::UnknownLayout {
                    reason: "unsupported chip_pins value (only 24 and 32 are known)",
                });
            }
        };

        // 10. Compose asserted_low_during_read = [ce, oe] (in that order),
        //     skipping 0xFF sentinels AND any entry that aliases one of
        //     cs1/cs2/cs3 (those are firmware-level placeholders, not
        //     real CE/OE pins on the 24-pin board — surfacing them here
        //     would double-write the chip-select level). HLD §4.4.
        let cs_pins = [cs1, cs2, cs3];
        let mut asserted_low_during_read: Vec<u8> = Vec::with_capacity(2);
        if ce != INVALID_PIN && !cs_pins.contains(&ce) {
            asserted_low_during_read.push(ce);
        }
        if oe != INVALID_PIN && !cs_pins.contains(&oe) {
            asserted_low_during_read.push(oe);
        }

        // 11. Compose deasserted_high_during_read = cs2/cs3 IFF they are
        //     valid pins AND not the gate captured by
        //     `unservable_when_high`. Per HLD §4.4 the stim composition
        //     is order-tolerant (deasserted-high writes HIGH, then
        //     pin_pattern ORs in — the bit stays HIGH either way), so
        //     we DON'T filter out address-pin aliases. We only filter
        //     out the gate CS so its semantics aren't double-counted.
        let mut deasserted_high_during_read: Vec<u8> = Vec::with_capacity(2);
        for &cs in &[cs2, cs3] {
            if cs == INVALID_PIN {
                continue;
            }
            // Don't include the gate CS — its semantics are captured
            // by unservable_when_high.
            if (1u64 << cs) & unservable_when_high != 0 {
                continue;
            }
            deasserted_high_during_read.push(cs);
        }

        // 12. Format the human-readable label as "{N}pin/{0xSIZE}-shadow".
        //     Leak the Box<str> for the &'static str type. One leak per
        //     parse; see the `from_flash` doc-comment for the no-hot-loop
        //     warning.
        let label_owned = format!("{chip_pins}pin/{shadow_size:#x}-shadow");
        let label: &'static str = Box::leak(label_owned.into_boxed_str());

        Ok(FixtureSpec {
            label,
            chip_pins,
            addr_pins,
            data_pins,
            cs1,
            asserted_low_during_read,
            deasserted_high_during_read,
            unservable_when_high,
            shadow_size,
        })
    }
}

// Compile-time-ish field labels for `FixtureError::BadPin`. Returning
// `&'static str` keeps the error variant cheap and Eq-comparable.
fn data_field_name(ii: usize) -> &'static str {
    const NAMES: [&str; 8] = [
        "data[0]", "data[1]", "data[2]", "data[3]", "data[4]", "data[5]", "data[6]", "data[7]",
    ];
    NAMES[ii]
}

fn addr_field_name(ii: usize) -> &'static str {
    const NAMES: [&str; 16] = [
        "addr[0]", "addr[1]", "addr[2]", "addr[3]", "addr[4]", "addr[5]", "addr[6]", "addr[7]",
        "addr[8]", "addr[9]", "addr[10]", "addr[11]", "addr[12]", "addr[13]", "addr[14]",
        "addr[15]",
    ];
    NAMES[ii]
}

fn addr2_field_name(ii: usize) -> &'static str {
    const NAMES: [&str; 16] = [
        "addr2[0]", "addr2[1]", "addr2[2]", "addr2[3]", "addr2[4]", "addr2[5]", "addr2[6]",
        "addr2[7]", "addr2[8]", "addr2[9]", "addr2[10]", "addr2[11]", "addr2[12]", "addr2[13]",
        "addr2[14]", "addr2[15]",
    ];
    NAMES[ii]
}

// ---------------------------------------------------------------------------
// SDRR struct chain helpers — moved verbatim from `onerom_serving_oracle.rs`.
// ---------------------------------------------------------------------------

/// Lift the per-set shadow from a loaded SDRR `.bin`.
///
/// Walks the SDRR struct chain (`sdrr_info_t` at `0x200` →
/// `onerom_metadata_header_t` → `sdrr_rom_set_t[rom_set_index]`) to
/// locate the selected ROM set's pre-processed image, then copies
/// `spec.shadow_size` bytes into a heap-allocated buffer. This is the
/// exact byte sequence `preload_rom_image` would have copied from
/// flash to `rom_table` in SRAM — reading it from flash directly
/// sidesteps the preload-not-done-at-sync problem (the DMA program
/// never fires on our emulator).
///
/// Returns `None` on any parse failure (malformed struct pointer,
/// `rom_set_index` out of range, source truncated). Callers fall back
/// to a zero-filled shadow; the binary-level tripwire then surfaces
/// the all-zero result via the `unique bytes == 1` warning.
///
/// Buffer length is exactly `spec.shadow_size`. Pre-Stage-2 the helper
/// returned a fixed-size `Box<[u8; SHADOW_SIZE]>` clamped via
/// `size.min(SHADOW_SIZE)`; Stage 2 honours the per-fixture shadow
/// size verbatim so 27C020-class (512 KiB) fixtures can be lifted in
/// full.
pub fn lift_shadow_from_flash(
    flash: &[u8],
    rom_set_index: u8,
    spec: &FixtureSpec,
) -> Option<Box<[u8]>> {
    let ptr_to_off = |ptr: u32| -> Option<usize> {
        let off = (ptr.checked_sub(FLASH_BASE)?) as usize;
        if off >= flash.len() { None } else { Some(off) }
    };
    let read_u32 = |off: usize| -> Option<u32> {
        let bytes = flash.get(off..off + 4)?;
        Some(u32::from_le_bytes(bytes.try_into().ok()?))
    };

    // sdrr_info_t at flash+0x200 → metadata_header pointer at +44.
    let metadata_ptr = read_u32(SDRR_INFO_OFFSET + SDRR_INFO_METADATA_PTR_OFFSET)?;
    let metadata_off = ptr_to_off(metadata_ptr)?;

    // onerom_metadata_header_t: rom_set_count at +20, rom_sets ptr at +24.
    let rom_set_count = *flash.get(metadata_off + METADATA_HEADER_ROM_SET_COUNT_OFFSET)?;
    if rom_set_index >= rom_set_count {
        return None;
    }
    let rom_sets_ptr = read_u32(metadata_off + METADATA_HEADER_ROM_SETS_PTR_OFFSET)?;
    let rom_sets_off = ptr_to_off(rom_sets_ptr)?;

    // sdrr_rom_set_t[rom_set_index]: data ptr at +0, size at +4.
    let set_off = rom_sets_off + (rom_set_index as usize) * ROM_SET_STRIDE;
    let data_ptr = read_u32(set_off + ROM_SET_DATA_PTR_OFFSET)?;
    let size = read_u32(set_off + ROM_SET_SIZE_OFFSET)? as usize;
    let data_off = ptr_to_off(data_ptr)?;

    // Allocate exactly `spec.shadow_size` bytes (no clamp). Copy up to
    // the per-set declared `size`; zero-pad the tail if `size <
    // spec.shadow_size` (defensive — not expected on conformant fixtures).
    let copy_len = size.min(spec.shadow_size);
    let src = flash.get(data_off..data_off + copy_len)?;
    let mut shadow: Box<[u8]> = vec![0u8; spec.shadow_size].into_boxed_slice();
    shadow[..copy_len].copy_from_slice(src);
    Some(shadow)
}

/// Walk the SDRR struct chain (`sdrr_info_t` → `onerom_metadata_header_t`
/// → `sdrr_rom_set_t[]`) and return one [`RomSetSlot`] per ROM set in
/// the fixture, in declaration order. Inverse-of-author for
/// `lift_shadow_from_flash`: callers (e.g. the SeaBIOS fixture builder)
/// can use the returned offsets to overwrite the per-set shadow bytes
/// in a flash image without rebuilding the whole envelope.
///
/// Returns `None` on any parse failure (truncated image, out-of-flash
/// pointer, etc.).
pub fn parse_rom_set_layout(flash: &[u8]) -> Option<Vec<RomSetSlot>> {
    let ptr_to_off = |ptr: u32| -> Option<usize> {
        let off = (ptr.checked_sub(FLASH_BASE)?) as usize;
        if off >= flash.len() { None } else { Some(off) }
    };
    let read_u32 = |off: usize| -> Option<u32> {
        let bytes = flash.get(off..off + 4)?;
        Some(u32::from_le_bytes(bytes.try_into().ok()?))
    };

    let metadata_ptr = read_u32(SDRR_INFO_OFFSET + SDRR_INFO_METADATA_PTR_OFFSET)?;
    let metadata_off = ptr_to_off(metadata_ptr)?;

    let rom_set_count = *flash.get(metadata_off + METADATA_HEADER_ROM_SET_COUNT_OFFSET)? as usize;
    if rom_set_count == 0 {
        // A zero-count fixture is malformed; the firmware would never
        // dereference rom_sets[0]. Report it via the function's standard
        // "None on parse failure" channel rather than returning Some(vec![]).
        return None;
    }
    let rom_sets_ptr = read_u32(metadata_off + METADATA_HEADER_ROM_SETS_PTR_OFFSET)?;
    let rom_sets_off = ptr_to_off(rom_sets_ptr)?;

    let mut out = Vec::with_capacity(rom_set_count);
    for k in 0..rom_set_count {
        let set_off = rom_sets_off + k * ROM_SET_STRIDE;
        let data_ptr = read_u32(set_off + ROM_SET_DATA_PTR_OFFSET)?;
        let size = read_u32(set_off + ROM_SET_SIZE_OFFSET)? as usize;
        let data_off = ptr_to_off(data_ptr)?;
        // Bounds check: ensure the declared size fits inside the flash
        // image. A bad size here would otherwise let the caller copy
        // off the end.
        if data_off
            .checked_add(size)
            .is_none_or(|end| end > flash.len())
        {
            return None;
        }
        out.push(RomSetSlot {
            descriptor_offset: set_off,
            data_offset: data_off,
            size,
        });
    }
    Some(out)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixture_path(name: &str) -> PathBuf {
        let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        p.push("fixtures");
        p.push(name);
        p
    }

    fn read_fixture(name: &str) -> Vec<u8> {
        let path = fixture_path(name);
        std::fs::read(&path).unwrap_or_else(|e| panic!("read {} failed: {e}", path.display()))
    }

    #[test]
    fn from_flash_fire24a_seabios() {
        let flash = read_fixture("onerom-fire-24-a-rp2350-seabios-cpu.bin");
        let spec = FixtureSpec::from_flash(&flash).expect("fire-24-a parse must succeed");

        // chip_pins: 24-pin socket.
        assert_eq!(spec.chip_pins, 24);

        // Address pin map: 13 entries, walking-1s order from the
        // canonical fire-24-a JSON. HLD §4.1 example values.
        assert_eq!(
            spec.addr_pins,
            vec![7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 14, 15, 12]
        );

        // Data pins on GPIO 16..23.
        assert_eq!(spec.data_pins, [16, 17, 18, 19, 20, 21, 22, 23]);

        // CS1 at GPIO13. Per the chip_pins=24 discriminator, CS1 is the
        // active-low gate; `unservable_when_high` captures only that.
        assert_eq!(spec.cs1, 13);
        assert_eq!(
            spec.unservable_when_high,
            1u64 << 13,
            "fire-24-a: CS1 (GPIO13) is the gate — unservable_when_high = 1<<13"
        );

        // CE/OE are firmware-level placeholders aliasing CS3/CS1 on the
        // 24-pin board (no real CE/OE pins). The cs_pins-aware filter
        // drops them; the asserted-low list is empty.
        assert!(
            spec.asserted_low_during_read.is_empty(),
            "fire-24-a: CE/OE alias CS pins; asserted_low must be empty: {:?}",
            spec.asserted_low_during_read
        );

        // CS2 (GPIO12) and CS3 (GPIO15) are deasserted-high during reads.
        // Both alias address pins (addr_pins[12]==12 and addr_pins[11]==15)
        // but per HLD §4.4 the stim composition is order-tolerant so
        // they're still listed (the deasserted-high write goes first,
        // then pin_pattern ORs in — the bit stays HIGH either way).
        assert_eq!(
            spec.deasserted_high_during_read,
            vec![12, 15],
            "fire-24-a: CS2 (12) and CS3 (15) deasserted high during reads"
        );

        // Shadow size: one ROM set image is 64 KiB on this fixture.
        assert_eq!(spec.shadow_size, 0x1_0000);

        // Sanity: label includes both fields.
        assert!(spec.label.contains("24pin"), "label = {}", spec.label);
        assert!(spec.label.contains("0x10000"), "label = {}", spec.label);
    }

    #[test]
    fn from_flash_fire32a_seabios() {
        let flash = read_fixture("onerom-fire-32-a-rp2350-seabios.bin");
        let spec = FixtureSpec::from_flash(&flash).expect("fire-32-a parse must succeed");

        // chip_pins: 32-pin socket.
        assert_eq!(spec.chip_pins, 32);

        // Address pin map: 19 entries (16 from addr[16] plus 3 from
        // addr2[16]). HLD §4.1 example length.
        assert_eq!(spec.addr_pins.len(), 19, "fire-32-a address width");

        // Data pins on GPIO 0..7. HLD §4.1 example values.
        assert_eq!(spec.data_pins, [0, 1, 2, 3, 4, 5, 6, 7]);

        // CS1 reported as 13 — generator-emitted placeholder for 32-pin
        // boards (the 27C020 chip type doesn't use CS1; sdrr-gen
        // unconditionally writes `board.pin_cs1(Chip27C080)`). Surfaced
        // verbatim per the FixtureSpec doc-comment caveat.
        assert_eq!(spec.cs1, 13);

        // Unservable mask: CS2 at GPIO16 aliases A16 in the address bus
        // (HLD §5.1). The chip_pins=32 discriminator captures CS2
        // exclusively — no CS1 contribution.
        assert_eq!(
            spec.unservable_when_high,
            1u64 << 16,
            "fire-32-a: CS2 (GPIO16, aliased to A16) is the gate — unservable_when_high = 1<<16"
        );

        // ce=15, oe=14 — separate /CE and /OE pins for 27C020, both
        // active-low during reads. Neither aliases a CS pin, so both
        // surface here in field order (ce, then oe).
        assert_eq!(
            spec.asserted_low_during_read,
            vec![15, 14],
            "fire-32-a: real CE (GPIO15) and OE (GPIO14) pins"
        );

        // No CS3 (cs3 = 0xFF in this fixture); CS2 IS the gate captured
        // by `unservable_when_high`, so it's excluded from this list to
        // avoid double-counting. Result: empty.
        assert!(
            spec.deasserted_high_during_read.is_empty(),
            "fire-32-a: CS2 is the gate (in unservable_when_high); deasserted set must be empty: {:?}",
            spec.deasserted_high_during_read
        );

        // Shadow size: 27C020 baked into a 27C040-capacity table = 512 KiB.
        assert_eq!(spec.shadow_size, 524_288);

        assert!(spec.label.contains("32pin"), "label = {}", spec.label);
        assert!(spec.label.contains("0x80000"), "label = {}", spec.label);
    }

    #[test]
    fn from_flash_truncated() {
        // 100 bytes is well under the upfront sdrr_info_t size requirement
        // (SDRR_INFO_OFFSET + SDRR_INFO_PINS_PTR_OFFSET + 4 = 0x238).
        // Must surface as `Truncated` exactly — `MissingRomSet` would hide
        // which path fired and would be the wrong diagnosis (a 100-byte
        // flash isn't missing a ROM set; it's truncated).
        let flash = vec![0u8; 100];
        match FixtureSpec::from_flash(&flash) {
            Err(FixtureError::Truncated) => {}
            other => panic!("expected Truncated, got {other:?}"),
        }
    }

    /// Build a minimal mock flash containing a valid SDRR_INFO + metadata
    /// chain + an `sdrr_pins_t` at a follow-the-pointer location. Used
    /// to construct targeted error-path tests without hand-crafting
    /// bytes in every test.
    fn synth_fixture_flash() -> Vec<u8> {
        // Layout we'll synthesise (XIP base = 0x1000_0000):
        //   0x0200  sdrr_info_t        (64 bytes)
        //   0x1000  sdrr_pins_t        (256 bytes)
        //   0xC000  metadata_header    (256 bytes; rom_set_count=1, rom_sets_ptr=0xC100)
        //   0xC100  rom_sets[0]        (64 bytes; data_ptr=0x1_0000, size=0x1_0000)
        //   0x1_0000 ROM data          (0x1_0000 bytes)
        let mut flash = vec![0u8; 0x2_0000];

        // sdrr_info_t at +0x200
        flash[0x200..0x204].copy_from_slice(b"SDRR");
        // metadata_header pointer at +0x200 + 44 = 0x22C → 0x1000_C000
        let metadata_ptr = 0x1000_C000u32;
        flash[0x22C..0x230].copy_from_slice(&metadata_ptr.to_le_bytes());
        // pins pointer at +0x200 + 48 = 0x230 → 0x1000_1000
        let pins_ptr = 0x1000_1000u32;
        flash[0x230..0x234].copy_from_slice(&pins_ptr.to_le_bytes());

        // sdrr_pins_t at 0x1000. Zero-init most fields; set a sane data[],
        // a 1-pin addr[] (so addr_pins.len() == 1), cs1 = 5, no other CS,
        // extended = 1, chip_pins = 24 (so the chip_pins=24 discriminator
        // path applies — CS1-as-gate, mask = 1 << cs1).
        let pins_off = 0x1000;
        // chip_pins (offset +5) — pretend this is a fire-24-a-shaped synth.
        flash[pins_off + 5] = 24;
        // data[8] = [10, 11, 12, 13, 14, 15, 16, 17]
        for ii in 0..8 {
            flash[pins_off + 8 + ii] = (10 + ii) as u8;
        }
        // addr[0] = 9, rest 0xFF
        for ii in 0..16 {
            flash[pins_off + 16 + ii] = INVALID_PIN;
        }
        flash[pins_off + 16] = 9;
        // cs1 = 5
        flash[pins_off + 36] = 5;
        // cs2/cs3/ce/oe = 0xFF (absent)
        flash[pins_off + 37] = INVALID_PIN;
        flash[pins_off + 38] = INVALID_PIN;
        flash[pins_off + 44] = INVALID_PIN;
        flash[pins_off + 45] = INVALID_PIN;
        // extended = 1
        flash[pins_off + 63] = 1;
        // addr2[..] = 0xFF
        for ii in 0..16 {
            flash[pins_off + 72 + ii] = INVALID_PIN;
        }

        // metadata_header at +0xC000
        let md_off = 0xC000;
        flash[md_off + METADATA_HEADER_ROM_SET_COUNT_OFFSET] = 1;
        let rom_sets_ptr = 0x1000_C100u32;
        flash[md_off + METADATA_HEADER_ROM_SETS_PTR_OFFSET
            ..md_off + METADATA_HEADER_ROM_SETS_PTR_OFFSET + 4]
            .copy_from_slice(&rom_sets_ptr.to_le_bytes());

        // rom_sets[0] at +0xC100
        let set_off = 0xC100;
        let data_ptr = 0x1001_0000u32; // → flash off 0x1_0000
        flash[set_off + ROM_SET_DATA_PTR_OFFSET..set_off + ROM_SET_DATA_PTR_OFFSET + 4]
            .copy_from_slice(&data_ptr.to_le_bytes());
        let set_size = 0x1_0000u32;
        flash[set_off + ROM_SET_SIZE_OFFSET..set_off + ROM_SET_SIZE_OFFSET + 4]
            .copy_from_slice(&set_size.to_le_bytes());

        flash
    }

    #[test]
    fn from_flash_synth_fixture_smoke() {
        // Sanity-check the synth helper before using it in error-path
        // tests: a clean synth blob must parse without complaint.
        let flash = synth_fixture_flash();
        let spec = FixtureSpec::from_flash(&flash).expect("synth fixture must parse cleanly");
        assert_eq!(spec.chip_pins, 24);
        assert_eq!(spec.cs1, 5);
        assert_eq!(spec.addr_pins, vec![9]);
        assert_eq!(spec.data_pins, [10, 11, 12, 13, 14, 15, 16, 17]);
        assert_eq!(spec.shadow_size, 0x1_0000);
        // chip_pins=24 → CS1 (GPIO5) is the gate.
        assert_eq!(spec.unservable_when_high, 1u64 << 5);
    }

    #[test]
    fn from_flash_bad_pin() {
        let mut flash = synth_fixture_flash();
        flash[0x1000 + 8] = 200; // data[0] = 200 (>= MAX_USED_GPIOS)
        match FixtureSpec::from_flash(&flash) {
            Err(FixtureError::BadPin {
                field: "data[0]",
                value: 200,
            }) => {}
            other => panic!("expected BadPin {{ field: data[0], value: 200 }}, got {other:?}"),
        }
    }

    #[test]
    fn from_flash_pre_v062() {
        let mut flash = synth_fixture_flash();
        flash[0x1000 + SDRR_PINS_EXTENDED_OFFSET] = 0;
        match FixtureSpec::from_flash(&flash) {
            Err(FixtureError::UnknownLayout { .. }) => {}
            other => panic!("expected UnknownLayout, got {other:?}"),
        }
    }

    #[test]
    fn from_flash_unknown_chip_pins() {
        // chip_pins must be 24 or 32 — anything else is rejected by the
        // unservable_when_high discriminator. Use 28 (a real DIP footprint
        // we don't model) to confirm the error path.
        let mut flash = synth_fixture_flash();
        flash[0x1000 + 5] = 28;
        match FixtureSpec::from_flash(&flash) {
            Err(FixtureError::UnknownLayout { .. }) => {}
            other => panic!("expected UnknownLayout for chip_pins=28, got {other:?}"),
        }
    }

    #[test]
    fn from_flash_rejects_non_contiguous_data_pins() {
        // Both serving oracles compose the served byte by shifting
        // `gpio_in >> data_pins[0]` and masking to 8 bits — that's only
        // correct when D0..D7 are consecutive. The synth helper sets
        // data[] to 10..=17; perturb data[3] off the consecutive line
        // and expect UnknownLayout.
        let mut flash = synth_fixture_flash();
        flash[0x1000 + SDRR_PINS_DATA_OFFSET + 3] = 25;
        match FixtureSpec::from_flash(&flash) {
            Err(FixtureError::UnknownLayout { reason }) => {
                assert!(
                    reason.contains("contiguous"),
                    "reason should mention contiguity; got {reason:?}"
                );
            }
            other => panic!("expected UnknownLayout for non-contiguous data pins, got {other:?}"),
        }
    }

    // -------------------------------------------------------------------
    // Tests for lift_shadow_from_flash + parse_rom_set_layout, ported
    // from the pre-Stage-2 onerom_serving_oracle.rs (commit de64969). The
    // helpers moved into this module during Stage 1; these tests guard
    // the move.
    // -------------------------------------------------------------------

    /// Per-set shadow size used by the multi-set synth helper. Distinct
    /// from `synth_fixture_flash`'s 0x1_0000-byte single-set layout (the
    /// two-set helper places data at 0x1_0000 and 0x2_0000, so the
    /// 192 KiB blob has room for both).
    const SYNTH_TWO_SET_SHADOW_SIZE: usize = 0x1_0000;

    /// Build a synthetic flash blob with `rom_set_count` SDRR ROM sets.
    /// Mirrors the `synth_flash` helper that lived in the pre-Stage-2
    /// `onerom_serving_oracle.rs`. Set 0's data lands at flash offset
    /// 0x2_0000 with byte pattern `j as u8`; set 1's data lands at
    /// 0x1_0000 with pattern `(j as u8) + 0x80`. The deliberate
    /// out-of-order layout (set 0 beyond set 1) confirms the parser
    /// follows the per-set `data` pointer rather than assuming
    /// contiguous packing.
    fn synth_flash_two_set(rom_set_count: u8) -> Vec<u8> {
        let mut flash = vec![0u8; 0x3_0000]; // 192 KiB, room for 2 sets
        // sdrr_info_t.metadata_header at flash+0x200+44 → 0x1000_C000.
        flash[SDRR_INFO_OFFSET + SDRR_INFO_METADATA_PTR_OFFSET
            ..SDRR_INFO_OFFSET + SDRR_INFO_METADATA_PTR_OFFSET + 4]
            .copy_from_slice(&(0x1000_C000u32).to_le_bytes());
        // metadata_header.rom_set_count at 0xC000 + 20.
        flash[0xC000 + METADATA_HEADER_ROM_SET_COUNT_OFFSET] = rom_set_count;
        // metadata_header.rom_sets at 0xC000 + 24 → 0x1000_C100.
        flash[0xC000 + METADATA_HEADER_ROM_SETS_PTR_OFFSET
            ..0xC000 + METADATA_HEADER_ROM_SETS_PTR_OFFSET + 4]
            .copy_from_slice(&(0x1000_C100u32).to_le_bytes());
        // Two sdrr_rom_set_t entries, stride 64 bytes.
        for i in 0..2 {
            let entry = 0xC100 + i * ROM_SET_STRIDE;
            // data ptr: set 0 → 0x1002_0000, set 1 → 0x1001_0000.
            let data_ptr = if i == 0 {
                0x1002_0000u32
            } else {
                0x1001_0000u32
            };
            flash[entry + ROM_SET_DATA_PTR_OFFSET..entry + ROM_SET_DATA_PTR_OFFSET + 4]
                .copy_from_slice(&data_ptr.to_le_bytes());
            // size = SYNTH_TWO_SET_SHADOW_SIZE.
            flash[entry + ROM_SET_SIZE_OFFSET..entry + ROM_SET_SIZE_OFFSET + 4]
                .copy_from_slice(&(SYNTH_TWO_SET_SHADOW_SIZE as u32).to_le_bytes());
        }
        // Per-set ROM image: walking byte keyed on set index.
        for j in 0..SYNTH_TWO_SET_SHADOW_SIZE {
            flash[0x2_0000 + j] = j as u8; // set 0
            flash[0x1_0000 + j] = (j as u8).wrapping_add(0x80); // set 1
        }
        flash
    }

    /// Build a `FixtureSpec` with the same `shadow_size` the multi-set
    /// helper generates. `lift_shadow_from_flash` only consumes
    /// `spec.shadow_size`, so other fields can be defaulted.
    fn synth_two_set_spec() -> FixtureSpec {
        FixtureSpec {
            label: "synth/0x10000",
            chip_pins: 24,
            addr_pins: vec![],
            data_pins: [0; 8],
            cs1: 0,
            asserted_low_during_read: vec![],
            deasserted_high_during_read: vec![],
            unservable_when_high: 0,
            shadow_size: SYNTH_TWO_SET_SHADOW_SIZE,
        }
    }

    /// `lift_shadow_from_flash` follows the SDRR struct chain and
    /// returns the selected set's bytes. Exercises the parser with a
    /// synthetic two-set flash blob — no emulator in the loop.
    #[test]
    fn lift_shadow_from_flash_happy_path() {
        let flash = synth_flash_two_set(2);
        let spec = synth_two_set_spec();

        // Set 0 → pattern (j as u8).
        let s0 = lift_shadow_from_flash(&flash, 0, &spec).expect("set 0");
        for i in 0..SYNTH_TWO_SET_SHADOW_SIZE {
            assert_eq!(
                s0[i], i as u8,
                "set 0 shadow[{}] = 0x{:02X}, expected 0x{:02X}",
                i, s0[i], i as u8
            );
        }

        // Set 1 → pattern (j as u8) + 0x80.
        let s1 = lift_shadow_from_flash(&flash, 1, &spec).expect("set 1");
        for i in 0..SYNTH_TWO_SET_SHADOW_SIZE {
            let want = (i as u8).wrapping_add(0x80);
            assert_eq!(
                s1[i], want,
                "set 1 shadow[{}] = 0x{:02X}, expected 0x{:02X}",
                i, s1[i], want
            );
        }
    }

    /// `lift_shadow_from_flash` returns `None` when `rom_set_index`
    /// is out of range. Protects against the firmware-not-yet-initialised
    /// case where `rom_set_index == 0xFF` and naively indexing would
    /// walk off the end of the array.
    #[test]
    fn lift_shadow_rejects_out_of_range_index() {
        let flash = synth_flash_two_set(2);
        let spec = synth_two_set_spec();
        assert!(
            lift_shadow_from_flash(&flash, 2, &spec).is_none(),
            "index 2 must be rejected (count = 2)"
        );
        assert!(
            lift_shadow_from_flash(&flash, 0xFF, &spec).is_none(),
            "index 0xFF must be rejected"
        );
    }

    /// `lift_shadow_from_flash` returns `None` on a malformed blob
    /// (here: truncated so the metadata_header pointer reads past EOF).
    /// Callers must never panic on a bad fixture.
    #[test]
    fn lift_shadow_rejects_truncated_flash() {
        let flash = vec![0u8; 0x300]; // only ~sdrr_info_t bytes; no metadata.
        let spec = synth_two_set_spec();
        assert!(lift_shadow_from_flash(&flash, 0, &spec).is_none());
    }

    /// `parse_rom_set_layout` walks the SDRR struct chain and returns one
    /// `RomSetSlot` per declared ROM set, with descriptor / data offsets
    /// matching what `lift_shadow_from_flash` would resolve internally.
    #[test]
    fn parse_rom_set_layout_happy_path() {
        let flash = synth_flash_two_set(2);
        let layout = parse_rom_set_layout(&flash).expect("layout must parse");
        assert_eq!(layout.len(), 2);

        // Set 0: data ptr = 0x1002_0000 → off 0x2_0000.
        // Set 1: data ptr = 0x1001_0000 → off 0x1_0000.
        // Stride = 64; rom_sets array starts at 0xC100 in the synth blob.
        assert_eq!(layout[0].descriptor_offset, 0xC100);
        assert_eq!(layout[0].data_offset, 0x2_0000);
        assert_eq!(layout[0].size, SYNTH_TWO_SET_SHADOW_SIZE);

        assert_eq!(layout[1].descriptor_offset, 0xC100 + ROM_SET_STRIDE);
        assert_eq!(layout[1].data_offset, 0x1_0000);
        assert_eq!(layout[1].size, SYNTH_TWO_SET_SHADOW_SIZE);
    }

    /// Truncated flash (smaller than the metadata pointer target) must
    /// return `None` — same conservative behaviour as
    /// `lift_shadow_from_flash`.
    #[test]
    fn parse_rom_set_layout_rejects_truncated_flash() {
        let flash = vec![0u8; 512];
        assert!(parse_rom_set_layout(&flash).is_none());
    }

    /// A `rom_set_count` of zero is malformed (the firmware would
    /// dereference rom_sets[0] regardless), so the parser must
    /// surface it as a parse failure rather than `Some(vec![])`.
    #[test]
    fn parse_rom_set_layout_rejects_zero_count() {
        let mut flash = synth_flash_two_set(2);
        flash[0xC000 + METADATA_HEADER_ROM_SET_COUNT_OFFSET] = 0;
        assert!(parse_rom_set_layout(&flash).is_none());
    }

    /// A `size` field that overruns the flash image must be rejected
    /// (otherwise a downstream copy could walk off the end).
    #[test]
    fn parse_rom_set_layout_rejects_oversize_size_field() {
        let mut flash = synth_flash_two_set(2);
        let entry_off = 0xC100; // set 0
        let bad_size = (flash.len() as u32) + 1;
        flash[entry_off + ROM_SET_SIZE_OFFSET..entry_off + ROM_SET_SIZE_OFFSET + 4]
            .copy_from_slice(&bad_size.to_le_bytes());
        assert!(parse_rom_set_layout(&flash).is_none());
    }
}
