# Contributing to mrs-airborne-tracker

Thank you for your interest in contributing to this project.

## Before You Submit Anything

This repository is a portfolio and commercial-licensing artifact maintained by
**MicroRobo Systems LLC**. Before submitting a pull request, you must read and
agree to the terms below. Submitting a PR constitutes your agreement.

## Contributor License Agreement (CLA)

Because this software is released under the
[PolyForm Noncommercial 1.0.0](LICENSE) license and MicroRobo Systems LLC
offers separate commercial licenses to third parties, we require contributors
to assign sufficient rights to MicroRobo Systems LLC to sublicense contributed
code under commercial terms.

**By submitting a pull request to this repository, you agree that:**

1. You are the sole author of the contributed code, or you have the authority
   to grant the rights described here on behalf of any co-authors or your
   employer.

2. You grant MicroRobo Systems LLC a perpetual, worldwide, non-exclusive,
   royalty-free, irrevocable copyright license to reproduce, prepare derivative
   works of, publicly display, publicly perform, sublicense, and distribute
   your contributions and such derivative works under any license terms,
   including commercial terms.

3. You grant MicroRobo Systems LLC a perpetual, worldwide, non-exclusive,
   royalty-free, irrevocable patent license to make, have made, use, offer to
   sell, sell, import, and otherwise transfer your contributions, where such
   license applies only to those patent claims licensable by you that are
   necessarily infringed by your contributions alone or in combination with
   this project.

4. Your contribution does not, to the best of your knowledge, infringe the
   intellectual property rights of any third party.

5. If your employer has rights to intellectual property you create (e.g., as
   part of your employment), you have received permission from your employer
   to make the contribution on behalf of that employer, or your employer has
   waived such rights for this contribution.

A formal, digitally-signed CLA process may be introduced in the future. Until
then, this in-band agreement via PR submission is the operative mechanism.

## What We Welcome

- Bug fixes with accompanying description of the defect and reproduction steps
- Documentation corrections (typos, factual errors, clarity improvements)
- Compatibility notes for hardware variants not listed in the README

## What We Are Not Looking For (Right Now)

- Architectural refactors — the codebase structure is intentional
- New features that are not discussed in an issue first
- Changes to the telemetry frame format without a coordinated update across
  all three system repositories

## How to Submit

1. Open an issue describing the bug or improvement before writing code.
2. Fork the repository and make your changes in a feature branch.
3. Submit a pull request against `main` with a clear description of what
   changed and why.
4. Reference the issue number in your PR description.

## Code Style

- Match the style of the surrounding code
- Comment any non-obvious hardware timing or protocol decisions
- Do not remove or weaken existing known-limitation comments

## Contact

For questions about commercial licensing, reach out via
[microrobosys.com](https://microrobosys.com).
