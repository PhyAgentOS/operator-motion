# Contributing

Thank you for contributing to Forge Motion.

## Development

Use Python 3.12 and `uv`:

```bash
uv sync --frozen --all-groups
PYTHONPATH=src uv run pytest -q
uv run forge-motion-server --help
uv run forge-motion-action-policy --help
```

The vendored `forge-tool` protocol under `src/forge_tool` is the reason tests
need `PYTHONPATH=src`. Keep changes focused and add tests for behavior changes.
Keep the deterministic action-policy core separate from its Dora transport
adapter, and document any change to action handling, configuration schema, or
message handling.

## Security and generated data

- Report vulnerabilities privately as described in `SECURITY.md`.
- Never commit credentials, private repository URLs, recordings, runtime state,
  machine-specific paths, or personal data.
- Do not run tests that command physical hardware without independent safety
  controls and an operator-accessible emergency stop.

By submitting a contribution, you agree that it is licensed under Apache
License 2.0.
