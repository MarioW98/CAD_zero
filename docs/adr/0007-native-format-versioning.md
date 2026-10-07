# ADR-0007: Native format versioning

## Status
Accepted (2025-10-07)

## Context
The native project format (`.cadforge`) is a structured file containing
feature tree, parameters, scripts, and any embedded meshes/voxels.
The format will evolve. We need a versioning strategy that:

- Lets old files open on new versions.
- Surfaces unsupported features rather than silently dropping them.
- Allows parallel-side migrations in CI (golden round-trip tests).

## Decision

### Format
JSON for structure + binary blob (zstd-compressed) for any bulk data
(meshes, voxel grids). The top-level JSON has a `version` field with
`MAJOR.MINOR.PATCH` semantics.

### Migration rules
- **PATCH bump**: forward-compatible. Old readers open new files.
- **MINOR bump**: backward-compatible. Old readers open new files
  with feature flags set to defaults.
- **MAJOR bump**: incompatible. New readers support a migration
  script from the previous major; older majors are dropped after
  one release cycle.

### Implementation
- `core/io/native.hpp` exposes `serialize(FeatureTree, version)`.
- `core/io/native.hpp` exposes `deserialize(stream)` that dispatches
  by version into a `Migrator` chain.
- Each `Migrator` is a function `(Document& doc) -> Status`. Migrators
  are registered in a chain indexed by `(from_version, to_version)`.

## Consequences
- Phase A includes a minimal `Migrator` registry (empty) and a
  `version = 0.1.0` constant in `native.hpp`.
- Each MINOR or MAJOR format change requires a new ADR (0007b, 0007c, …)
  describing the migration steps.
- The CI golden-test suite includes files at every historical version.
