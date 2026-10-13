#!/usr/bin/env python3
"""
| File: run_physics_tests.py
| License: BSD-3-Clause. Copyright (c) 2026. All rights reserved.
| Description: Standalone runner for the rotating-wing physics tests, with or without pytest.

Prefer pytest when it is installed -- it gives better failure output::

    python3 -m pytest examples/controller_tests/physics_tests -q

This script exists so the suite is still runnable where pytest is not available, which
includes Isaac Sim's bundled interpreter. It collects every ``test_*`` function from every
``test_*.py`` module in this directory, runs it, and reports a summary. Parametrised tests
(``@pytest.mark.parametrize``) are expanded manually.

Exit status is 0 if everything passed and 1 otherwise, so it can be used in CI.
"""

from __future__ import annotations

import importlib.util
import inspect
import os
import sys
import traceback
from typing import Any, Callable, Iterator, List, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))

# The test modules import ``_context`` by bare name, so this directory must be importable.
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def _load_module(path: str):
    """Import a test module from its file path."""
    name = os.path.splitext(os.path.basename(path))[0]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _expand_parametrize(func: Callable) -> Iterator[Tuple[str, Callable]]:
    """Yield (label, callable) for a test, expanding pytest parametrize marks.

    Only the single-mark, positional-argnames form used in this suite is supported. Any
    other form is reported rather than silently skipped, so the runner cannot quietly
    under-test the model.
    """
    marks = getattr(func, "pytestmark", [])
    param_marks = [m for m in marks if getattr(m, "name", "") == "parametrize"]

    if not param_marks:
        yield func.__name__, func
        return

    if len(param_marks) > 1:
        raise RuntimeError(
            f"{func.__name__}: stacked parametrize marks are not supported by this "
            "runner; run the suite under pytest instead"
        )

    argnames, argvalues = param_marks[0].args[0], param_marks[0].args[1]
    names = [n.strip() for n in argnames.split(",")] if isinstance(argnames, str) else list(argnames)

    for index, values in enumerate(argvalues):
        if len(names) == 1:
            values = (values,)
        kwargs = dict(zip(names, values))
        label = f"{func.__name__}[{index}]"
        yield label, (lambda _f=func, _k=kwargs: _f(**_k))


def _call_fixture(name: str, module: Any, stack: Tuple[str, ...] = ()) -> Any:
    """Instantiate one fixture, resolving its own fixture dependencies recursively.

    The suite's fixtures form a shallow dependency graph (``design_loads`` depends on
    ``wing``), which pytest would resolve by injection. This reproduces just enough of
    that to run the suite standalone.

    Raises:
        RuntimeError: If a fixture is missing or the graph contains a cycle.
    """
    if name in stack:
        raise RuntimeError(f"fixture cycle: {' -> '.join(stack + (name,))}")

    fixture = getattr(module, name, None)
    if fixture is None:
        raise RuntimeError(f"no fixture named {name!r}")

    # pytest.fixture wraps the function; unwrap to get the real callable.
    target = getattr(fixture, "__wrapped__", fixture)

    kwargs = {
        dep: _call_fixture(dep, module, stack + (name,))
        for dep in inspect.signature(target).parameters
    }
    return target(**kwargs)


def _resolve_fixtures(func: Callable, module: Any) -> dict:
    """Instantiate every fixture a test function asks for.

    Each test gets freshly-built fixtures, matching pytest's default function scope. That
    matters here: the wing model carries an induced-velocity warm start, so sharing one
    instance between tests would let results leak across them.
    """
    try:
        signature = inspect.signature(func)
    except (TypeError, ValueError):
        return {}

    return {
        name: _call_fixture(name, module) for name in signature.parameters
    }


def main() -> int:
    """Run every test in this directory. Returns a process exit status."""
    test_files = sorted(
        os.path.join(HERE, f)
        for f in os.listdir(HERE)
        if f.startswith("test_") and f.endswith(".py")
    )

    if not test_files:
        print("no test modules found")
        return 1

    passed: List[str] = []
    failures: List[Tuple[str, str]] = []

    for path in test_files:
        module = _load_module(path)
        module_name = os.path.basename(path)
        print(f"\n{module_name}")

        functions = [
            obj
            for name, obj in sorted(vars(module).items())
            if name.startswith("test_") and callable(obj)
        ]

        for func in functions:
            for label, call in _expand_parametrize(func):
                try:
                    if call is func:
                        call(**_resolve_fixtures(func, module))
                    else:
                        # A parametrised closure already binds its arguments; resolving
                        # fixtures for it would try to look the parameters up as fixtures.
                        call()
                except AssertionError as exc:
                    failures.append((f"{module_name}::{label}", str(exc)))
                    print(f"  FAIL  {label}")
                except Exception:  # noqa: BLE001 - a crash is a failure, and we want the trace
                    failures.append((f"{module_name}::{label}", traceback.format_exc()))
                    print(f"  ERROR {label}")
                else:
                    passed.append(label)
                    print(f"  ok    {label}")

    print("\n" + "=" * 72)
    if failures:
        print(f"{len(passed)} passed, {len(failures)} FAILED\n")
        for name, detail in failures:
            print(f"--- {name}")
            print(detail.strip()[:1500])
            print()
        return 1

    print(f"{len(passed)} passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
