# PX4 research backup manifest — 2026-10-19

This manifest identifies GitHub recovery points preserved from the external-drive workspace.

## Current integrated project
- Repository: `XpiredRuby/PX4_project_improved`
- Default branch: `main`
- Integrated source checkpoint: `de7397b1d0d003d0e514ef7c0d3a0e3f0e5722ca`
- Ancestry: main fast-forwarded by 27 commits from `e0968f2d3447c2ba130aa9271a0500dec9517ddf`.
- Development branch retained: `codex/trajectory-research-and-offline-studio`.
- Scope of latest integrated work: trajectory studio, planning and aircraft-specific inputs, simulation verification, and software-side flight-release safeguards.

## Preserved independent worktrees
1. `XpiredRuby/PX4_project_improved`, branch `backup/legacy-worktree-2026-10-19`, commit `11f9c794ba387cc3e814d8b040b108126d46ab99`. This is a recovery snapshot of 41 modified or new legacy source, test, configuration, and documentation files based on `4652b13747e19833df4e8c5f807789701235e840`. It is **not** automatically merged into main.
2. `XpiredRuby/Final-Code`, branch `backup/working-copy-2026-10-19`, commit `066bef04a829e5e86c06d86e098653dcac6427d3`. This captures four modified controller/test files without rewriting its main branch.

## Recovery
```bash
git clone https://github.com/XpiredRuby/PX4_project_improved.git
cd PX4_project_improved
git switch main
# To inspect older alternative work without replacing main:
git fetch origin backup/legacy-worktree-2026-10-19
git switch -c inspect-legacy FETCH_HEAD
```

## Deliberate exclusions / limits
- Untracked bulk local validation logs, generated simulation captures, browser profiles, caches, WSL runtime disk images, and scratch files were **not** included in the legacy snapshot. Such files may still only exist on the flash drive.
- The GitHub snapshots preserve source progress, not an executable copy of every local dependency or simulator installation.
- Software-side SITL or flight-release checks do **not** certify readiness for physical flight. Vehicle-specific safety validation is still required.
- Before merging a legacy backup into the integrated project, review conflicts and run the relevant test suite.
