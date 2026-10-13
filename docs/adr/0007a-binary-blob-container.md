# ADR-0007a: Binary blob container for the native format

## Status
Accepted (2026-10-13) — companion to [ADR-0007](0007-native-format-versioning.md).
Required by [ADR-0003](0003-third-party-policy.md): every format change of
this weight gets a written decision record.

## Context
ADR-0007 specifies "JSON for structure + binary blob (zstd-compressed) for
any bulk data (meshes, voxel grids)". Phase A shipped the JSON half only:
the scene was fully parametric, so cached meshes were re-tessellated on
load and no bulk data needed persisting. STL/OBJ import changes that —
imported meshes have no parametric description to re-tessellate, so their
geometry MUST live in the file.

Constraints:
- No new third-party dependencies (ADR-0003) → zstd is deferred; the blob
  payload ships uncompressed ("raw") today.
- Cross-platform byte-exactness (ADR-0014): a save→load→save cycle must
  reproduce identical bytes, on every compiler/OS pair we support.
- Old files (v0.1.0) must keep opening (they exist in the wild and in the
  CI golden suite).

## Decision

### Container layout (v0.2.0)
A `.CAD_0` file is now a framed binary container:

```
offset  size  content
0       8     magic: "CAD0BLOB" (ASCII)
8       4     json_len      — uint32, little-endian
12      4     blob_len      — uint32, little-endian
16      j     JSON document — UTF-8, pretty-printed (the ADR-0007 JSON,
                unchanged structure: format/version/scene/camera [+ meshes])
16+j    b     blob section — concatenated mesh blob payloads
```

All multi-byte integers in the container (lengths and blob payload) are
explicitly little-endian; the codec reads and writes them byte-by-byte, so
it is endian-independent by construction. Floats travel as IEEE-754
float32 bit patterns (via `std::bit_cast`, C++20) — no arithmetic happens
on the encode/decode path, guaranteeing bit-exact round-trips including
`-0.0`, denormals and NaNs.

Reader auto-detection, checked BEFORE any parsing:
1. first 8 bytes == `"CAD0BLOB"` → binary container;
2. first byte == `'{'` → legacy pure-JSON v0.1.0 document (the Phase A
   path, intact);
3. anything else → explicit `std::runtime_error` ("not a CAD_0 native
   document: expected the CAD0BLOB container or a legacy JSON document").

Writers always emit the CURRENT version (0.2.0) in the container form —
including for scenes with no meshes, where the blob section is simply
empty (`blob_len = 0`). This keeps exactly one writer path.

### Mesh blob encoding "raw"
The JSON document gains an optional top-level `"meshes"` array (omitted
when empty, so a mesh-free v0.2.0 document differs from its v0.1.0
counterpart ONLY in the version string). Each entry is the blob header:

```json
{ "id": "mesh_1", "encoding": "raw", "offset": 0, "size": 240,
  "vertex_count": 6, "triangle_count": 8, "has_normals": true }
```

- `offset` is relative to the START OF THE BLOB SECTION (not the file);
  `size` is the exact payload length. Both are validated on load
  (`offset + size <= blob_len`, `size == expected_from_counts`) — any
  mismatch is an explicit error, so a truncated or hand-edited file fails
  loudly instead of yielding garbage geometry.
- `encoding` selects the payload codec. Today only `"raw"` exists; the
  field exists precisely so that adding zstd later is a *codec bump*
  inside a new format version (with its own companion ADR), not a layout
  change.
- `id` is a stable string referenced by scene nodes ("mesh" nodes carry
  `"mesh": {"blob": "<id>"}` — wired in the scene layer, see
  `core/scene/scene.hpp`). Ids are unique within a document; duplicates
  are a load error.

Payload layout for `"raw"` (little-endian, tightly packed, in order):

```
positions : float32 ×3 × vertex_count        (12 bytes/vertex)
normals   : float32 ×3 × vertex_count        (12 bytes/vertex, iff has_normals)
indices   : uint32  ×3 × triangle_count      (12 bytes/triangle)
```

Triangle indices are validated against `vertex_count` on decode.

### Version 0.2.0 and the first real migrator
`kNativeVersion` moves 0.1.0 → 0.2.0. The migrator registry gains its
first real entry: **0.1.0 → 0.2.0, a no-op** — v0.1.0 documents carry no
blob payloads, and a missing `"meshes"` member already parses as "no
meshes" (the same tolerance PATCH files rely on). The migrator exists so
the upgrade path is explicit and testable: the checked-in v0.1.0 golden
file now exercises the migrator on every CI run.

The registry lookup normalizes the file's PATCH component to zero: a
migrator registered from (M.m.0) upgrades **every** M.m.x — because
PATCH differences are compatible in both directions (ADR-0007), a 0.1.3
document opens through the 0.1.0 → 0.2.0 migrator without needing its own
registration. Migrators run forward (upgrade) only; there is no
downgrade path.

### Honest reconciliation with ADR-0007's compatibility table
ADR-0007 states: "MINOR bump: backward-compatible. Old readers open new
files with feature flags set to defaults." **With a binary container that
sentence is no longer true and we do not pretend otherwise.** The real
semantics, stated explicitly:

- **New readers open old files.** Always. Via the migrator chain
  (upgrade-only; e.g. 0.1.0 → 0.2.0). This is the direction that matters
  for users: their saved projects keep opening forever.
- **Old readers do NOT open new files** once the MINOR bump changes the
  framing. A v0.1.0 reader handed a v0.2.0 file fails at its very first
  step (the text no longer starts with `'{'`) — it fails LOUDLY with a
  parse error, not silently, which is the part of ADR-0007's spirit we
  preserve. It cannot "open with feature flags defaulted" because the
  blob section is opaque to it.
- **Writers always write the current version.** There is no
  "save-as-older-format" mode.
- PATCH bumps keep both-direction compatibility (unknown JSON fields are
  ignored — unchanged). MAJOR bumps keep the migrator-chain semantics
  with one release cycle of support (unchanged).

A future MINOR bump that does NOT change the container framing (pure
JSON-level addition, no new blob codec) would restore the old-reader
property *at the JSON level* — but a v0.2.0 reader is still required to
make sense of the framing, so the project's promise remains the
one-directional rule above. Migration chains run forward only.

## Consequences
- `io::native::load_file` auto-detects the framing; `parse_document`
  (text-only entry point) accepts the JSON block of either version, and
  rejects documents that carry mesh blobs with an explicit "use
  load_file" error — the geometry is not in the text.
- Golden suite: `tests/golden/native_v0_1_0.CAD_0` is FROZEN as a legacy
  fixture (it exercises the migrator forever); `native_v0_2_0.CAD_0` is
  the new drift detector (container + one embedded mesh, byte-identical
  re-serialization). The regen variable `CAD_0_GENERATE_GOLDEN=1`
  regenerates ONLY the v0.2.0 golden — the v0.1.0 file must never be
  rewritten.
- SDF-only documents round-trip bit-identically as before (the JSON block
  is unchanged apart from the version string); the existing round-trip
  tests are the safety net for this.
- `blob_len`/`json_len` are uint32 → the JSON block and the blob section
  are each capped at 4 GiB. Far beyond anything the kernel produces; a
  document that size would fail loudly at write time.
- zstd compression: deferred until profiling justifies it (ADR-0003).
  When it lands: new `encoding: "zstd"` value inside a new MINOR version
  + companion ADR + migrator, without touching the container layout.
