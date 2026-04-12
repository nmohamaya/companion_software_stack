# Commercial License

## Licensing Structure

This repository contains two separately licensed components:

### Drone Software Stack (AGPL 3.0)

All C++ source code, configuration, deployment scripts, tests, and documentation.
See [LICENSE](LICENSE) for full terms.

You are free to use, modify, and distribute this software under the terms of the AGPL. The key requirement is that if you deploy a modified version of this software (including over a network), you must make your modifications available under the same AGPL 3.0 license.

This is ideal for:
- Academic research and education
- Open-source drone projects
- Hobbyist and personal use
- Internal evaluation and prototyping

### Agent Pipeline (Proprietary)

The 17-agent development pipeline, skills, orchestrator, and shared context.
See [PIPELINE_LICENSE.md](PIPELINE_LICENSE.md) for terms.

The pipeline is viewable for reference but requires a commercial license to use, modify, or redistribute. This includes:
- `.claude/agents/` — Agent role definitions
- `.claude/skills/` — Skill specifications
- `.claude/shared-context/` — Shared agent context
- `scripts/orchestrator/` — Pipeline orchestrator (Python)
- `tasks/` — Work tracking files

## Commercial License

For organizations that need to use this software **without the AGPL 3.0 copyleft obligations** or that want access to the agent pipeline, a commercial license is available.

### Commercial License Tiers

| Tier | Drone Stack | Agent Pipeline |
|------|-------------|----------------|
| **Source License** | Proprietary use rights for the public codebase | Not included |
| **Enterprise** | + full test suite (1259+ tests) + design documentation | Not included |
| **Integration** | + dedicated integration support | **Included: full pipeline, skills, orchestrator** |

### What Each Tier Enables

**Source License:**
- Integrate the drone software into proprietary products without disclosing your source code
- Deploy modified versions without open-sourcing your changes
- Use the software in commercial drone platforms and services

**Enterprise:**
- Everything in Source License
- Access to the full test suite (1259+ unit and integration tests)
- Design documentation and architecture guides

**Integration:**
- Everything in Enterprise
- Dedicated integration support
- Full 17-agent development pipeline (orchestrator, skills, agent profiles, shared context)
- License to use, modify, and deploy the pipeline for your own development workflow

### Contact

For commercial licensing inquiries, please contact:

- **LinkedIn:** [Naveen Mohanan](https://www.linkedin.com/in/naveen-mohanan)
- **GitHub:** [@nmohamaya](https://github.com/nmohamaya)
- **Repository:** [companion_software_stack](https://github.com/nmohamaya/companion_software_stack)

## FAQ

**Q: Can I use the drone stack for a university research project?**
A: Yes. Academic use under AGPL 3.0 is free. If you publish modifications, they must also be AGPL 3.0.

**Q: Can I use this in a commercial drone product?**
A: Yes, under AGPL 3.0 if you open-source your modifications. If you need proprietary modifications, contact us for a commercial license.

**Q: Does the AGPL apply if I only use the software internally?**
A: AGPL applies when you deploy the software to users over a network or distribute it. Purely internal use (not exposed to external users) does not trigger the copyleft requirement. However, if your drone provides a network service using this software, AGPL applies.

**Q: Can I read and learn from the agent pipeline code?**
A: Yes. The pipeline is visible in the repo for educational and reference purposes. You may not copy, modify, or use the pipeline files without a commercial Integration license.

**Q: What about the YOLOv8 model?**
A: The optional YOLOv8n model is separately licensed under AGPL 3.0 by Ultralytics. Our AGPL 3.0 license is fully compatible. For commercial use of YOLOv8, contact Ultralytics for their commercial license.
