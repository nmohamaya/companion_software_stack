# AI / ML Training — Opt-Out Posture

This repository explicitly **opts out** of being used for AI/ML training, fine-tuning, distillation, dataset construction, or retrieval-augmented generation corpora. This doc explains how the opt-out is communicated, why it has the form it has, and what contributors and downstream consumers need to know.

> **Bottom line:** the AGPL/COMMERCIAL/PIPELINE licences continue to apply to code and humans. For automated AI/ML training pipelines, the rights are **expressly reserved** — no licence is granted.

---

## 1. What's covered

Three machine-readable signals plus one human-readable legal notice, all at the repository root:

| File | Audience | What it does |
|---|---|---|
| [`NOTICE`](../../NOTICE) | Humans + scrapers that read NOTICE files | The canonical legal reservation. Cites EU CDSM Article 4(3), US 17 U.S.C. § 106, and UK CDPA 1988. Includes contact for licensing inquiries. |
| [`ai.robots.txt`](../../ai.robots.txt) | AI-specific crawlers that respect the community `ai.robots.txt` convention | Disallows specific known AI training user-agents (OpenAI's GPTBot, Google-Extended, ClaudeBot, anthropic-ai, CCBot, FacebookBot, PerplexityBot, etc.) |
| [`.github/robots.txt`](../../.github/robots.txt) | General web crawlers via GitHub's repo browser | Standard robots.txt also used to signal AI/training disallow for crawlers that don't honour `ai.robots.txt` |
| `LICENSE` + `COMMERCIAL_LICENSE.md` + `PIPELINE_LICENSE.md` | Legal | The underlying licences. The NOTICE file's reservation sits *on top of* these; it does not replace them. |

## 2. Why this form

A simple "© All rights reserved" line is no longer enough. Modern AI training pipelines harvest code from public Git hosts indiscriminately, and several jurisdictions have introduced or are introducing text-and-data-mining (TDM) exceptions that **default to permitted unless the rightsholder opts out**. The opt-out must be machine-readable to be enforceable under those laws.

The three signals serve different mechanisms:

- **EU CDSM Article 4(3) opt-out** — EU law makes TDM lawful by default for commercial purposes, **unless** the rightsholder reserves the right in an "appropriate machine-readable format". `NOTICE`, `ai.robots.txt`, and `.github/robots.txt` are the recognised forms. The NOTICE file states this explicitly so the reservation is unambiguous.
- **US 17 U.S.C. § 106 reservation** — US copyright already reserves reproduction, derivative works, and distribution. The NOTICE file makes it explicit that no licence is granted for AI training, foreclosing a "we assumed it was permitted" defence.
- **UK CDPA 1988 reservation** — the UK has been considering a TDM exception. The NOTICE file pre-emptively reserves rights against any such exception.
- **`ai.robots.txt`** — a community convention (see [ai.robots.txt](https://github.com/ai-robots-txt/ai.robots.txt)) that AI-specific crawlers have started honouring. Targets *known* AI training user-agents by name.
- **`.github/robots.txt`** — fallback for general crawlers and for AI services that don't honour the AI-specific file but do honour the standard robots.txt.

## 3. What this does NOT do

- **Does not** revoke the AGPL or commercial licences. Humans (and their tools) can still read, fork, run, contribute, and distribute under the stated licence terms.
- **Does not** prevent AI tools from reading the code at *inference time* when a human directly pastes a snippet — the opt-out targets training-set construction and retrieval-augmented corpora, not interactive use by a human's licensed tooling.
- **Does not** prevent any AI training that's done with **express prior written permission** from the copyright holder. Contact details are in `NOTICE`.
- **Does not** affect the third-party dependencies. Zenoh, Eigen, OpenCV, etc. are each licensed under their own terms (see `NOTICE` § Third-Party Attributions).

## 4. What contributors need to know

If you contribute a PR to this repository:

- Your contribution becomes covered by the same opt-out reservation as the rest of the repo (you grant the licence that lets the project use your contribution; the project's AI/ML reservation then applies to the combined work).
- You do not need to add a separate NOTICE entry for your file.
- If you copy code *from* this repo into another project, the destination project should respect the same opt-out (i.e. carry forward the NOTICE entry or equivalent). If you can't preserve the reservation downstream, contact the copyright holder before redistributing.
- **Never strip or weaken** the NOTICE, `ai.robots.txt`, or `.github/robots.txt` files. They are load-bearing for the legal posture.

## 5. What downstream consumers need to know

If you're a downstream consumer — a customer using the proprietary pipeline, an academic citing the AGPL stack, a vendor evaluating the project for licensing:

- The licence file you signed (commercial or AGPL) determines what you can do with the code.
- **None** of those licences grant AI/ML training rights. If your use case involves training a model on this code, you need a separate written agreement.
- The same applies to RAG / vector-database / agent-context-window use — inclusion of this repo's code in an inference-time corpus that isn't your own interactive session needs separate permission.

## 6. Updating the opt-out

If new AI training user-agents emerge, update `ai.robots.txt` and `.github/robots.txt`. If new TDM legislation lands in a relevant jurisdiction, add a reservation clause to `NOTICE` with a citation to the statute. Track the document history at the bottom of `NOTICE`.

The opt-out is a living posture; the codebase's terms can stiffen but should not be silently loosened.

## 7. Related

- [`NOTICE`](../../NOTICE) — the canonical legal text
- [`LICENSE`](../../LICENSE) — AGPL v3.0
- [`COMMERCIAL_LICENSE.md`](../../COMMERCIAL_LICENSE.md) — commercial terms
- [`PIPELINE_LICENSE.md`](../../PIPELINE_LICENSE.md) — proprietary subset
- [`ai.robots.txt`](../../ai.robots.txt) and [`.github/robots.txt`](../../.github/robots.txt) — machine-readable signals
- [EU Directive 2019/790](https://eur-lex.europa.eu/eli/dir/2019/790/oj) — the CDSM Directive whose Article 4(3) governs the EU TDM opt-out
- [ai.robots.txt project](https://github.com/ai-robots-txt/ai.robots.txt) — community convention this repo participates in
