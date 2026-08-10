"""Shared test setup.

The one thing this does is turn a missing model into a SKIP rather than an
ERROR.

The distinction matters. `pursuit/onnxbase.py` raises `FileNotFoundError` when
a graph is absent, and that is correct behaviour for the pipeline: someone
trying to fly the aircraft with no detector should be told loudly and pointed
at `fetch_models.py`. But the weights are ~295 MB, they are deliberately not in
the repository, and two of them are over GitHub's file-size limit — so the
normal state of a fresh clone, and of every CI runner, is that they are not
there yet.

Without this, `pytest vision/tests` on a fresh clone reports 14 errors, which
reads as "this project is broken" when it actually means "you have not
downloaded 295 MB yet". The suite is designed so that everything not requiring
weights — the protocol, the link, the lock machine, the arbiter, the control
maths, the geometry — runs and passes with nothing downloaded at all, and that
is the majority of it.

Scoped deliberately narrowly: only a FileNotFoundError naming a missing model,
and only during fixture setup. Any other error still fails.
"""
from __future__ import annotations

import pytest


def _is_missing_model(excinfo) -> bool:
    if excinfo is None:
        return False
    exc = excinfo.value
    return (isinstance(exc, FileNotFoundError)
            and "model not found" in str(exc))


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()

    if report.when == "setup" and report.failed and _is_missing_model(call.excinfo):
        report.outcome = "skipped"
        missing = str(call.excinfo.value).splitlines()[0]
        # The wantonly specific tuple shape is what pytest expects for a skip:
        # (path, lineno, reason).
        report.longrepr = (str(item.fspath), 0,
                           f"Skipped: {missing} — run vision/fetch_models.py")
