# Gitignore Review

## Original State

The original `.gitignore` covered only a narrow set of build and binary patterns:

- build directories
- Python caches
- logs
- a few binary extensions
- CMake cache directories

## Problems Found

- Did not ignore `.env`
- Did not protect runtime databases
- Did not cover IDE/editor state
- Did not cover crash dumps, coverage, virtual environments, or many native build outputs
- Did not cover cross-platform OS noise files

## Improvements Applied

Added protection for:

- `.env` and other local environment files while preserving `.env.example`
- `data/dashboard/*.sqlite3`
- logs, caches, and dump files
- Python caches and virtual environments
- Go test/coverage output
- native build directories and intermediates
- Windows, macOS, and editor-generated noise
- certificate/key material

## Residual Note

The ignore file now preserves `data/dashboard/.gitkeep` so the runtime state directory can exist without committing the generated SQLite database.

## Verdict

`PASS`
