# Wire-contract versioning policy

The mechanism today (see [ADR-017](../docs/adr/ADR-017-staged-wire-contract-governance.md) for why this shape):

- One global **`kWireVersion`** byte ([`wire_format.h`](../common/ipc/include/ipc/wire_format.h)), carried in every wire header. Receivers **reject** messages with version `0` or `> kWireVersion`. The flight recorder stamps every record; the replay tool validates per record, so old recordings stay readable across evolution.
- All wire structs are trivially-copyable, standard-layout PODs; **`sizeof` is part of the public IPC ABI** until cross-version deserialisation lands (#806 Phase 3). Same-commit lockstep deployment of all processes is the compatibility mechanism between peers.

## Rules for changing the wire format

1. **Any change to any wire struct's layout, size, or field semantics** ⇒ bump `kWireVersion` **and** update [`topics.json`](topics.json) **in the same commit**. `tests/test_contracts_manifest.cpp` turns a forgotten manifest update into a build failure; review enforces the rest.
2. **Additive evolution**: new fields append at the end of the struct; never reorder, retype, or reuse existing fields. Update the struct's `validate()` and default member initializers with it.
3. **Breaking changes** (remove/rename/retype a field, change units or semantics): bump `kWireVersion`, add a migration note to [`docs/design/API.md`](../docs/design/API.md), and update every producer and consumer in the same commit — there is no cross-version path yet, so a split-brain deploy is a wire-corruption bug by definition.
4. **Safety-relevant messages** — `FCCommand`, `GCSCommand`, `FCState`, `TrajectoryCmd`, `FaultOverrides` — sit on the command-authority path ([`docs/SAFETY_ENGINEERING.md`](../docs/SAFETY_ENGINEERING.md) §2.3/§2.8). Changes to them get safety-focused review (fault-recovery + security reviewers in the two-pass pipeline), not just wire-format review.
5. **New topics** register in three places in one commit: `topics::` constant in `ipc_types.h`, a row in [`topics.json`](topics.json), and the binding table in `tests/test_contracts_manifest.cpp` (the completeness checks then hold the three mutually consistent).

## Deferred (triggers recorded in ADR-017)

- Per-message version constants, schema-first codegen ⇒ first non-C++ consumer.
- Cross-version deserialisation + replay migration shims ⇒ first forced incompatible bump or multi-vehicle rolling upgrade.
