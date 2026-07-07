# ADR-017: Staged Wire-Contract Governance (contracts-lite now, schema-first when a consumer exists)

| Field           | Value |
|-----------------|-------|
| **Status**      | Accepted |
| **Date**        | 2026-07-07 |
| **Author**      | Team |
| **Deciders**    | Project leads |
| **Supersedes**  | — |
| **Related**     | Issue [#806](https://github.com/nmohamaya/companion_software_stack/issues/806), [ADR-001](ADR-001-ipc-framework-selection.md) (Zenoh + POD wire format), [ADR-002](ADR-002-modular-ipc-backend-architecture.md), [`contracts/`](../../contracts/), [`common/ipc/include/ipc/wire_format.h`](../../common/ipc/include/ipc/wire_format.h), [`docs/SAFETY_ENGINEERING.md`](../SAFETY_ENGINEERING.md) §2.8 |

---

## 1. Context

The IPC wire contract is trivially-copyable C++ structs over Zenoh (ADR-001). The mechanics that exist today are good but **implicit**, spread across two headers:

- `ipc_types.h` — the struct SSOT: trivially-copyable + standard-layout `static_assert`s on every wire struct, `validate()` boundary checks, and *partial* size guards (`SemanticVoxel`/`SemanticVoxelBatch` carry explicit `sizeof` asserts; most structs do not).
- `wire_format.h` — a single global `kWireVersion` byte carried in every wire header; receivers reject unknown/future versions; the flight recorder stamps records and the replay tool validates per record.

Three gaps, one of them already demonstrated:

1. **No written compatibility policy.** What a version bump means, what counts as breaking, and who must update when a struct changes exist only as tribal knowledge.
2. **No machine-readable contract.** There is no artifact stating "topic X carries struct Y at size Z under version V" that a tool — or a reviewer — can check the code against.
3. **Implicit governance decays — proven.** The `FCState` ABI comment promised a size guard that was never written, and cited issue #718 for "cross-version FCState deserialisation" when #718 was actually the (closed) preflight-timeout issue. Comment-based contracts drift; nothing failed when they did.

At the same time, the *expensive* end of contract tooling has no consumer today: every wire-format consumer is C++, compiled from the same commit, deployed in lockstep. The only artifact that genuinely crosses versions is recorded flight data — which is exactly where version machinery already exists (recorder stamps + replay validation).

## 2. Decision

Adopt **staged governance**, spending only where a consumer exists at each stage:

**Phase 1 — contracts-lite (this ADR, implemented with it):**

- `contracts/` directory: [`README.md`](../../contracts/README.md) (message set), [`VERSIONING.md`](../../contracts/VERSIONING.md) (compatibility policy), [`topics.json`](../../contracts/topics.json) (machine-readable manifest: topic ↔ struct ↔ expected `sizeof` ↔ wire version).
- A CI drift gate: `tests/test_contracts_manifest.cpp` compares the manifest against the *compiled* structs and `kWireVersion`. Any struct size change, topic rename, or version bump that doesn't consciously touch `contracts/` fails the build. This generalises the per-struct `sizeof` asserts into full coverage — the manifest is now the size guard the `FCState` comment promised.
- `ipc_types.h` remains the struct SSOT. The manifest **mirrors** it and is enforced against it; it does not generate it.

**Phase 2 — schema-first codegen. Trigger: the first non-C++ consumer** (Python ground tooling, web GCS, a second vehicle language). Schema definitions become the SSOT and emit `ipc_types.h` at its current path (so the ~100 including files don't churn).

**Phase 3 — cross-version deserialisation + replay migration shims. Trigger: the first field-forced incompatible bump, or multi-vehicle rolling upgrades.** Until then, `sizeof(struct)` remains part of the public IPC ABI by policy, and same-commit lockstep deployment is the compatibility mechanism.

## 3. Alternatives considered

- **A. Adopt an IDL now (protobuf / FlatBuffers / Cap'n Proto).** Rejected for now: buys polyglot support and schema evolution nobody consumes yet, at the cost of an IDL toolchain in every build, serialization on paths that are currently zero-copy POD, and a rewrite of every producer/consumer. Revisit at the Phase 2 trigger — that decision would get its own ADR (format choice, cert implications).
- **B. Schema-first codegen immediately (custom schema → generated `ipc_types.h`).** Rejected for now: inverts the SSOT and adds a generator to maintain, with zero consumers of the schema beyond the C++ it would regenerate. Kept as the Phase 2 design.
- **C. Status quo (comments + partial asserts).** Rejected: gap 3 above is the counter-evidence — the governance we believed we had was already rotting, silently.
- **D. Staged contracts-lite (chosen).** ~200 lines of docs + one test buys: written policy, machine-checked drift detection, full size-guard coverage, and an explicit, triggered path to the expensive stages.

## 4. Consequences

**Positive:**

- A wire-struct change that forgets the contract now fails CI instead of shipping a silent ABI drift.
- The compatibility policy is written, reviewable, and linked from the safety plan (`SAFETY_ENGINEERING.md` §2.8 / G5).
- Phase 2/3 spend is deferred until their consumers exist, with the triggers recorded here rather than re-litigated.

**Negative / residual risks (accepted):**

- The test's topic↔struct binding table is maintained by hand, colocated with the test. A new topic added to `topics::` without a manifest entry is caught only if the author also updates the binding table — the completeness checks catch manifest/binding mismatches in both directions, but a topic added to *neither* is invisible to the gate. Mitigation: `VERSIONING.md` makes "wire change ⇒ `contracts/` change in the same commit" a review rule; full automation is exactly Phase 2.
- `sizeof` equality does not detect same-size field reordering. The existing standard-layout asserts plus review remain the guard; `offsetof` layout snapshots are a possible hardening noted in #806.
- One more artifact to keep honest — accepted precisely because the keeping-honest is now mechanical, not archaeological.
