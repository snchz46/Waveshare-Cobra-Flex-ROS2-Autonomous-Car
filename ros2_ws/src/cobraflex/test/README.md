# Tests

Pytest/ament tests that exercise the Cobra Flex package live here:

- `test_cobraflex.py`: Basic health check to ensure the package imports and nodes initialize without errors.
- `test_flake8.py`: Style gate leveraging `flake8` to enforce linting rules.
- `test_pep257.py`: Docstring/style validation for public modules.
- `test_copyright.py`: Verifies package metadata and copyright headers.

Run the suite from the workspace root after building:

```bash
colcon test --packages-select cobraflex
colcon test-result --verbose
```
