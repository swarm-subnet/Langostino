# Contributing to Langostino 🦐

Thanks for your interest in contributing to **Langostino** — an open-source reference drone for real-world autonomous flight, built with a **global** community.

**Docs are part of the product.** If something is unclear, outdated, or missing: that’s a great first contribution.

---

## Quick links

- 💬 Discord (help + coordination): https://discord.com/invite/bittensor
- 🐛 Issues: https://github.com/swarm-subnet/Langostino/issues
- 🟢 Good first issues:
  https://github.com/swarm-subnet/Langostino/issues?q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22

## Documentation index (in this repo)

- Setup: `docs/SETUP_GUIDE.md`
- Commands: `docs/COMMANDS_GUIDE.md`
- INAV: `docs/INAV_GUIDE.md`
- Params: `docs/CONFIG_PARAMS_GUIDE.md`
- Troubleshooting: `docs/TROUBLESHOOTING_GUIDE.md`
- Assembly (build from scratch): `docs/assembly/README.md`

---

## Ways to contribute

### 1) Documentation (high impact)

- Fix unclear steps, missing prerequisites, broken links
- Add screenshots / wiring diagrams / photos
- Improve “first-time user” experience (shorter, clearer, safer)

### 2) Issues & bug reports

- Report reproducible bugs (include logs and environment details)
- Improve error messages and troubleshooting steps

### 3) Hardware & build improvements

- BOM improvements (availability, alternatives, pricing)
- Assembly steps, safety checklists, calibration tips

### 4) Code contributions

- Small fixes, refactors, and automation improvements

### 5) Community contributions

- Share build logs & short flight clips (we feature the best builds)
- Help answer questions in Discord

---

## Getting started (recommended workflow)

1. Fork the repo and clone your fork
2. Create a branch:
   - `docs/<topic>` (docs-only)
   - `fix/<bug>` (bugfix)
   - `feat/<feature>` (feature)

Example:

```bash
git checkout -b docs/quickstart-clarify
```

3. Make your changes
4. Open a Pull Request (PR)

---

## Build & run (development setup)

> The system is currently documented for **Ubuntu 22.04**.

### Quick setup (recommended)

Follow:

- `docs/SETUP_GUIDE.md#quick-setup`

In most cases you’ll run:

```bash
sudo ./setup.sh
./verify_setup.sh
./launch.sh
```

### Useful scripts

- `setup.sh` — automated install & configuration
- `verify_setup.sh` — verification checks
- `launch.sh` — system launcher

### Where configuration lives

- Main parameters: `src/swarm_ai_integration/config/swarm_params.yaml`
- INAV CLI commands: `docs/INAV_GUIDE.md#complete-configuration-script-inav-cli-commands`

---

## Reporting issues (bug reports)

Before opening a new issue:

1. Check `docs/TROUBLESHOOTING_GUIDE.md`
2. Run:

```bash
./verify_setup.sh
```

When reporting, include:

- OS + hardware (e.g., Ubuntu 22.04 / Raspberry Pi 4)
- What you expected vs what happened
- Steps to reproduce
- Relevant logs (snippets are fine)

Helpful logs/locations:

- PM2 logs: `~/.pm2/logs/`
- Flight logs: `~/swarm-ros/flight-logs/` (if applicable)

---

## Pull request (PR) guidelines

### Keep PRs small and reviewable

- Prefer small PRs that do one thing well
- For large changes, open an issue first and discuss in Discord

### PR checklist

- [ ] Clear title and description
- [ ] Linked issue (if applicable)
- [ ] Docs updated if behavior changed
- [ ] No secrets/credentials committed
- [ ] You tested the steps you changed (at least once)

### Commit messages

Use simple, readable messages:

- `docs: clarify quick setup`
- `fix: handle missing device path`
- `feat: add verification step`

---

## Flight controllers (INAV / ArduPilot)

Current docs focus on **INAV**:

- `docs/INAV_GUIDE.md`

If you’re working on **ArduPilot** compatibility/integration:

- Open an issue describing the approach and what you need
- We’ll label it and coordinate in Discord

---

## Safety

Langostino is a real flying machine.

- Follow local regulations
- Test in safe environments
- Use prop guards and appropriate failsafes
- Props off for bench testing

---

## License

By contributing, you agree that your contributions will be licensed under the project’s MIT License (see `LICENSE.md`).
