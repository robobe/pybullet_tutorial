# Agent Instructions

- Prefer `pathlib.Path` for filesystem path handling in Python code.
- Avoid new `os.path` usage unless the surrounding code already depends on it or a library requires a string path.
- When changing existing path code, keep edits scoped and do not refactor unrelated logic just to use `pathlib`.

## Python Style

- Prefer `Enum` for function or constructor arguments that accept a fixed set of named options.
- Avoid raw string option values such as `"wall"` or `"relative"` when the valid choices are known in advance.
- Use raw strings only for open-ended user data, external protocol values, or when matching an existing library/API.
- Name constants with uppercase `SNAKE_CASE`.
