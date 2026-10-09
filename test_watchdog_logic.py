"""Hardware-free tests for the safety-watchdog logic in main.py.

Extracts the pure functions via AST so they can be tested without flask,
pigpio, or the camera (this file runs on the dev box and on the Pi):

    python test_watchdog_logic.py
"""
import ast
import contextlib
import io
import os
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "main.py")

WANTED = {"read_meminfo", "read_self_rss_mb", "wifi_link_state", "_dump_mem_history"}


def load_functions():
    with open(MAIN, "r", encoding="utf-8") as handle:
        tree = ast.parse(handle.read(), MAIN)
    funcs = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in WANTED
    ]
    found = {node.name for node in funcs}
    missing = WANTED - found
    assert not missing, f"functions missing from main.py: {sorted(missing)}"
    module = ast.Module(body=funcs, type_ignores=[])
    namespace = {
        "os": os,
        "time": __import__("time"),
        "MEMINFO_PATH": "/proc/meminfo",
        "WIFI_OPERSTATE_PATH": "/sys/class/net/wlan0/operstate",
        "WIFI_ROUTE_PATH": "/proc/net/route",
        "_mem_history": [],
    }
    exec(compile(module, MAIN, "exec"), namespace)
    return namespace


def test_read_meminfo():
    ns = load_functions()
    content = (
        "MemTotal:        437248 kB\n"
        "MemFree:         221000 kB\n"
        "MemAvailable:    330752 kB\n"
        "SwapTotal:       523260 kB\n"
        "SwapFree:        523260 kB\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix="meminfo", delete=False) as handle:
        handle.write(content)
        path = handle.name
    try:
        mem = ns["read_meminfo"](path)
    finally:
        os.unlink(path)
    assert mem["mem_total_mb"] == 437248 / 1024, mem
    assert mem["mem_available_mb"] == 330752 / 1024, mem
    assert mem["swap_total_mb"] == 523260 / 1024, mem
    assert mem["swap_used_mb"] == 0.0, mem

    used = ns["read_meminfo"]  # now simulate swap in use
    with tempfile.NamedTemporaryFile("w", suffix="meminfo", delete=False) as handle:
        handle.write(
            "MemTotal: 437248 kB\nMemAvailable: 102400 kB\n"
            "SwapTotal: 523260 kB\nSwapFree: 400000 kB\n"
        )
        path = handle.name
    try:
        mem = used(path)
    finally:
        os.unlink(path)
    assert abs(mem["swap_used_mb"] - (523260 - 400000) / 1024) < 0.01, mem

    assert ns["read_meminfo"](os.path.join(HERE, "no-such-file")) is None
    print("PASS read_meminfo (MB conversion, swap used, missing file -> None)")


def test_read_self_rss_mb():
    ns = load_functions()
    rss = ns["read_self_rss_mb"]()
    if os.path.exists("/proc/self/status"):
        assert rss is not None and rss > 0, rss
        print(f"PASS read_self_rss_mb ({rss:.1f} MB on this host)")
    else:
        assert rss is None
        print("PASS read_self_rss_mb (no /proc on this OS -> graceful None)")


def _write(name, content, folder):
    path = os.path.join(folder, name)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(content)
    return path


def test_wifi_link_state():
    ns = load_functions()
    check = ns["wifi_link_state"]
    header = "Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\tMTU\tWindow\tIRTT\n"
    default = "wlan0\t00000000\t0101A8C0\t0003\t0\t0\t0\t00000000\t0\t0\t0\n"
    subnet = "wlan0\t00B8A8C0\t00000000\t0001\t0\t0\t0\t00FFFFFF\t0\t0\t0\n"

    with tempfile.TemporaryDirectory() as folder:
        oper = _write("operstate", "up\n", folder)
        routes_ok = _write("route_ok", header + default + subnet, folder)
        routes_none = _write("route_none", header + subnet, folder)
        routes_missing = os.path.join(folder, "route_absent")
        oper_missing = os.path.join(folder, "operstate_absent")

        assert check(oper, routes_ok) == "up", "default route must count as up"
        assert check(oper, routes_none) == "down", "no default route -> down"
        assert check(oper_missing, routes_none) == "absent", "no iface -> absent"
        assert check(oper_missing, routes_missing) == "absent", "no route file -> absent"
    print("PASS wifi_link_state (up / down / absent)")


def test_dump_mem_history():
    ns = load_functions()
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ns["_dump_mem_history"]()
    assert out.getvalue() == "", "empty history must print nothing"

    ns["_mem_history"].append("12:00:00 avail=300MB rss=31MB swap=0MB")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ns["_dump_mem_history"]()
    text = out.getvalue()
    assert "Memory history (1 samples" in text and "avail=300MB" in text, text
    print("PASS _dump_mem_history (empty silent, populated dumps)")


if __name__ == "__main__":
    test_read_meminfo()
    test_read_self_rss_mb()
    test_wifi_link_state()
    test_dump_mem_history()
    print("ALL WATCHDOG LOGIC TESTS PASSED")
