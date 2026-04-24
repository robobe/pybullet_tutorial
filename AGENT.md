# Agent Instructions

- Prefer `pathlib.Path` for filesystem path handling in Python code.
- Avoid new `os.path` usage unless the surrounding code already depends on it or a library requires a string path.
- When changing existing path code, keep edits scoped and do not refactor unrelated logic just to use `pathlib`.
