# Contributing

Thanks for helping make Mochi Sprout's experiments easier to understand and reproduce.

1. Open an issue to discuss substantial changes before implementing them. For a small correction, a focused pull request is fine.
2. Use Python 3.11 and `uv sync --locked --python 3.11`.
3. Run `uv run --locked --python 3.11 pytest -q` before proposing a change.
4. For an experiment change, document the model, initial conditions, commands, measured results, and limitations. Distinguish observations from expectations.
5. Keep changes within the current milestone and avoid adding future features without an agreed learning gate.

By contributing, you agree that your contributions are licensed under Apache-2.0. Do not include credentials, private information, or creative assets you cannot license for this project. See the [README](README.md) for the distinction between code and future character assets.
