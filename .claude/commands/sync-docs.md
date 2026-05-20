# /sync-docs — Update project documentation

You are updating ARCHITECTURE.md and CLAUDE.md to reflect work done in this session.

## Steps

1. Run `git diff HEAD` and `git status` to see all changed files in this session.
2. Read the current `ARCHITECTURE.md` and `CLAUDE.md` from the project root.
3. For each significant change (new feature, changed config constant, modified state machine, new hardware behavior, new key binding, new module), update the relevant section in both files.

## What to update in CLAUDE.md
- Key config constants table — add/update any constant that was changed or added
- Controls table — add any new key bindings
- Architecture decisions — add any new non-obvious design decisions made this session
- Common pitfalls — add any new gotcha discovered this session

## What to update in ARCHITECTURE.md
- Module map — add any new files or modules
- Control loop diagram — update if the per-frame flow changed
- Approach state machine — update if new states or transitions were added
- VL53 section — update if sensor logic changed
- Motor convention table — update if motor behavior changed
- Key files table — add any new important files

## Rules
- Do not remove existing correct information — only add or correct.
- Keep entries concise (one line per item where possible).
- If nothing changed that affects the docs, say so explicitly rather than making empty edits.
- After updating, print a brief summary of what you changed in each file.
