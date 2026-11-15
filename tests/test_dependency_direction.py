"""依存の向きの検査: policy / perception が eval を(推移込みで)import しないことを保証する(SGT-002)。

【機能概要】
`so101_vlm.policy*` と `so101_vlm.perception*` の全モジュールから、`so101_vlm.eval`
(`eval.scene_spec` を含む)へ import をたどって届かないことを、`ast` による静的走査で検査する。
要件: FR-106 / FR-401 / NFR-202。新規依存なし(CON-001。import-linter は使わない)。

【検査の仕組み】
1. `imported_modules`: ソースを `ast` で走査し、import を絶対名に解決して返す。
   `from x import y` は `x` と `x.y` の両方を返す(y が部分モジュールの場合に備える)。
   `if TYPE_CHECKING:` 内・関数内の遅延 import も `ast.walk` で拾う(型の上でも依存させない方針)。
   `importlib.import_module("...")` / `import_module("...")` / `__import__("...")` の文字列定数も拾う。
2. `build_graph`: src 配下の全 .py をモジュール名に対応づけ、実在するモジュール同士の辺を張る。
   import 先の祖先パッケージの `__init__` も辺にする(実行時に必ず実行されるため)。
3. `find_violation`: 開始モジュールから幅優先で探索し、禁止名に一致する import が出たら経路を返す。
   判定は raw の import 名で行うので、eval 側のモジュールが実在しなくても検出できる。

【限界】🟡
- 推移先のモジュールにある「引数が定数でない動的 import」(f-string 等)は静的に追えず、検査対象外。
- ガード対象(policy / perception)自身にある非定数の動的 import は、検査不能として違反にする。
- 実行時に sys.modules や exec 経由で組み立てる import、外部パッケージ経由の逆流は追わない。
"""

from __future__ import annotations

import ast
from collections import deque
from pathlib import Path

import pytest

FORBIDDEN_PREFIXES = ("so101_vlm.eval",)  # 🔵 FR-106 / FR-401
GUARDED_PACKAGES = ("so101_vlm.policy", "so101_vlm.perception")  # 🔵 検査対象はこの 2 つのみ
ROOT_PACKAGE = "so101_vlm"


# === ヘルパ ===
def _resolve_relative(current_module: str, is_package: bool, level: int) -> str:
    """相対 import の基準パッケージ名を返す。

    `__init__.py` なら自分自身のパッケージ、通常のモジュールなら親パッケージが基準。
    そこから `level - 1` 段だけ上がる。トップより上へ出るなら ValueError。
    """
    parts = current_module.split(".")
    base = parts if is_package else parts[:-1]
    up = level - 1
    if up >= len(base):
        raise ValueError(f"相対 import がトップパッケージより上へ出ます: {current_module} (level={level})")
    return ".".join(base[: len(base) - up])


def _dynamic_kind(node: ast.Call) -> str | None:
    """importlib.import_module / import_module / __import__ の呼び出しなら種類を返す。"""
    f = node.func
    if isinstance(f, ast.Name) and f.id in ("import_module", "__import__"):
        return f.id
    if (
        isinstance(f, ast.Attribute)
        and f.attr == "import_module"
        and isinstance(f.value, ast.Name)
        and f.value.id == "importlib"
    ):
        return "import_module"
    return None


def _scan(source: str, current_module: str, is_package: bool) -> tuple[set[str], list[int]]:
    """import 名(絶対)と、解決できない動的 import の行番号を返す。"""
    tree = ast.parse(source)
    names: set[str] = set()
    unresolved: list[int] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base = _resolve_relative(current_module, is_package, node.level)
                resolved = f"{base}.{node.module}" if node.module else base
            else:
                resolved = node.module or ""
            names.add(resolved)
            names.update(f"{resolved}.{a.name}" for a in node.names if a.name != "*")
        elif isinstance(node, ast.Call) and _dynamic_kind(node):
            arg = node.args[0] if node.args else None
            if not (isinstance(arg, ast.Constant) and isinstance(arg.value, str)):
                unresolved.append(node.lineno)
                continue
            name = arg.value
            if name.startswith("."):
                pkg = None
                if len(node.args) > 1 and isinstance(node.args[1], ast.Constant):
                    pkg = node.args[1].value
                for kw in node.keywords:
                    if kw.arg == "package" and isinstance(kw.value, ast.Constant):
                        pkg = kw.value.value
                if not isinstance(pkg, str):
                    unresolved.append(node.lineno)
                    continue
                level = len(name) - len(name.lstrip("."))
                pkg_parts = pkg.split(".")
                if level - 1 >= len(pkg_parts):
                    unresolved.append(node.lineno)
                    continue
                name = ".".join(pkg_parts[: len(pkg_parts) - (level - 1)] + [name[level:]])
            names.add(name)
    inside = {n for n in names if n == ROOT_PACKAGE or n.startswith(ROOT_PACKAGE + ".")}
    return inside, unresolved


def imported_modules(source: str, current_module: str, is_package: bool) -> set[str]:
    """ソースの import を絶対名に解決して返す(`so101_vlm` 配下のみ)。🟡"""
    return _scan(source, current_module, is_package)[0]


def unresolved_dynamic_imports(source: str, current_module: str, is_package: bool) -> list[int]:
    """引数が定数でない(静的に追えない)動的 import の行番号を返す。🟡"""
    return _scan(source, current_module, is_package)[1]


def _module_files(src_root: Path) -> dict[str, tuple[Path, bool]]:
    """{モジュール名: (ファイル, パッケージか)}。`__init__.py` はパッケージ名に対応づける。"""
    out: dict[str, tuple[Path, bool]] = {}
    for path in sorted((src_root / ROOT_PACKAGE).rglob("*.py")):
        parts = list(path.relative_to(src_root).with_suffix("").parts)
        is_pkg = parts[-1] == "__init__"
        if is_pkg:
            parts.pop()
        out[".".join(parts)] = (path, is_pkg)
    return out


def _read_source(path: Path) -> str:
    # BOM 付きでも ast.parse が落ちないよう utf-8-sig で読む。CRLF は ast が通す。
    return path.read_text(encoding="utf-8-sig")


def _scan_file(path: Path, module: str, is_pkg: bool) -> tuple[set[str], list[int]]:
    try:
        return _scan(_read_source(path), module, is_pkg)
    except SyntaxError as e:
        raise ValueError(f"{path}: 構文エラーで import を解析できません: {e}") from e
    except ValueError as e:
        raise ValueError(f"{path}: {e}") from e


def collect_raw_imports(src_root: Path) -> dict[str, set[str]]:
    """全モジュールの raw の import 名(実在しない名前も含む)。"""
    return {m: _scan_file(p, m, k)[0] for m, (p, k) in _module_files(src_root).items()}


def build_graph(
    src_root: Path, raw_imports: dict[str, set[str]] | None = None
) -> dict[str, set[str]]:
    """実在するモジュール同士の import グラフ。祖先パッケージの `__init__` も辺に含める。"""
    modules = _module_files(src_root)
    raw = raw_imports if raw_imports is not None else collect_raw_imports(src_root)
    graph: dict[str, set[str]] = {}
    for module in modules:
        targets = [module, *raw.get(module, ())]
        edges: set[str] = set()
        for name in targets:
            parts = name.split(".")
            for i in range(1, len(parts) + 1):
                cand = ".".join(parts[:i])
                if cand in modules:
                    edges.add(cand)
        edges.discard(module)
        graph[module] = edges
    return graph


def _is_forbidden(name: str, prefixes: tuple[str, ...]) -> bool:
    # 前方一致の罠(evaluation_x など)を避けるため、完全一致か「prefix + .」で判定する。
    return any(name == p or name.startswith(p + ".") for p in prefixes)


def find_violation(
    graph: dict[str, set[str]],
    raw_imports: dict[str, set[str]],
    start: str,
    forbidden_prefixes: tuple[str, ...] = FORBIDDEN_PREFIXES,
) -> list[str] | None:
    """start から幅優先で探索し、禁止名への import に至る経路を返す。無ければ None。"""
    seen = {start}
    queue: deque[list[str]] = deque([[start]])
    while queue:
        path = queue.popleft()
        node = path[-1]
        for name in sorted(raw_imports.get(node, ())):
            if _is_forbidden(name, forbidden_prefixes):
                return [*path, name]
        for nxt in sorted(graph.get(node, ())):
            if nxt not in seen:
                seen.add(nxt)
                queue.append([*path, nxt])
    return None


def is_guarded(module: str) -> bool:
    return any(module == p or module.startswith(p + ".") for p in GUARDED_PACKAGES)


def guarded_unresolved(src_root: Path) -> dict[str, list[int]]:
    """ガード対象モジュール自身にある、静的に追えない動的 import(行番号つき)。"""
    out: dict[str, list[int]] = {}
    for m, (p, k) in _module_files(src_root).items():
        if is_guarded(m):
            lines = _scan_file(p, m, k)[1]
            if lines:
                out[m] = lines
    return out


# === テスト用の合成パッケージ ===
def make_pkg(tmp_path: Path, files: dict[str, str | bytes]) -> Path:
    """{src からの相対パス: ソース} を書き出し、足りない __init__.py を空で補う。src_root を返す。"""
    src = tmp_path / "src"
    for rel, content in files.items():
        p = src / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, bytes):
            p.write_bytes(content)
        else:
            p.write_text(content, encoding="utf-8", newline="\n")
    for d in {q.parent for q in src.rglob("*.py")} | {src / ROOT_PACKAGE}:
        d.mkdir(parents=True, exist_ok=True)
        while d != src:
            init = d / "__init__.py"
            if not init.exists():
                init.write_text("", encoding="utf-8")
            d = d.parent
    return src


def check(tmp_path: Path, files: dict[str, str | bytes], start: str) -> list[str] | None:
    src = make_pkg(tmp_path, files)
    raw = collect_raw_imports(src)
    return find_violation(build_graph(src, raw), raw, start)


BASE = "so101_vlm.policy.base"

# 違反として検出されるべき合成パターン(id, files)
VIOLATION_CASES = [
    ("import_直接", {"so101_vlm/policy/base.py": "import so101_vlm.eval.x\n"}),
    ("from_eval_import", {"so101_vlm/policy/base.py": "from so101_vlm.eval import x\n"}),
    (
        "from_scene_spec",
        {"so101_vlm/policy/base.py": "from so101_vlm.eval.scene_spec import S\n"},
    ),
    ("import_as", {"so101_vlm/policy/base.py": "import so101_vlm.eval.x as y\n"}),
    ("相対_from_dotdot_import_eval", {"so101_vlm/policy/base.py": "from .. import eval\n"}),
    ("相対_from_dotdot_eval", {"so101_vlm/policy/base.py": "from ..eval import x\n"}),
    (
        "相対_from_dot_import_兄弟",
        {
            "so101_vlm/policy/base.py": "from . import helper\n",
            "so101_vlm/policy/helper.py": "import so101_vlm.eval.x\n",
        },
    ),
    (
        "TYPE_CHECKING内",
        {
            "so101_vlm/policy/base.py": (
                "from typing import TYPE_CHECKING\n"
                "if TYPE_CHECKING:\n"
                "    from so101_vlm.eval.scene_spec import S\n"
            )
        },
    ),
    (
        "関数内の遅延import",
        {"so101_vlm/policy/base.py": "def f():\n    from so101_vlm.eval import x\n    return x\n"},
    ),
    (
        "推移_中間モジュール経由",
        {
            "so101_vlm/policy/base.py": "from so101_vlm.safety.config import C\n",
            "so101_vlm/safety/config.py": "from so101_vlm.eval.validation import f\n",
        },
    ),
    (
        "親パッケージ__init__経由",
        {
            "so101_vlm/policy/__init__.py": "import so101_vlm.eval.x\n",
            "so101_vlm/policy/base.py": "x = 1\n",
        },
    ),
    (
        "importlib.import_module",
        {
            "so101_vlm/policy/base.py": (
                'import importlib\nm = importlib.import_module("so101_vlm.eval.x")\n'
            )
        },
    ),
    ("__import__", {"so101_vlm/policy/base.py": '__import__("so101_vlm.eval.x")\n'}),
    (
        "素のimport_module",
        {
            "so101_vlm/policy/base.py": (
                'from importlib import import_module\nimport_module("so101_vlm.eval.x")\n'
            )
        },
    ),
    (
        "相対import_module",
        {
            "so101_vlm/policy/base.py": (
                "from importlib import import_module\n"
                'import_module(".x", package="so101_vlm.eval")\n'
            )
        },
    ),
    (
        "eval実在",
        {
            "so101_vlm/policy/base.py": "from so101_vlm.eval.scene_spec import S\n",
            "so101_vlm/eval/scene_spec.py": "S = 1\n",
        },
    ),
]


@pytest.mark.parametrize(
    "files", [c[1] for c in VIOLATION_CASES], ids=[c[0] for c in VIOLATION_CASES]
)
def test_違反パターンを検出する(tmp_path, files):
    # 🔵 完了条件: 直接・相対・TYPE_CHECKING・遅延・推移・動的 import を違反とする
    path = check(tmp_path, files, BASE)
    assert path is not None
    assert path[0] == BASE
    assert path[-1].startswith("so101_vlm.eval")


def test_perceptionからの違反も検出する(tmp_path):
    files = {"so101_vlm/perception/calibration.py": "from so101_vlm.eval.scene_spec import S\n"}
    assert check(tmp_path, files, "so101_vlm.perception.calibration") is not None


# === 偽陽性が無いこと ===
CLEAN_CASES = [
    ("eval不使用", {"so101_vlm/policy/base.py": "x = 1\n"}),
    (
        "標準ライブラリと外部",
        {"so101_vlm/policy/base.py": "import os\nimport numpy as np\nfrom pathlib import Path\n"},
    ),
    (
        "eval前方一致の別名",
        {
            "so101_vlm/policy/base.py": (
                "import so101_vlm.evaluation_x\n"
                "from so101_vlm import evaluation_x\n"
                "from so101_vlm.evaluation_x import y\n"
            ),
            "so101_vlm/evaluation_x.py": "y = 1\n",
        },
    ),
    (
        "安全な推移",
        {
            "so101_vlm/policy/base.py": "from so101_vlm.safety.config import C\n",
            "so101_vlm/safety/config.py": "C = 1\n",
        },
    ),
    (
        "相対importで兄弟",
        {"so101_vlm/policy/base.py": "from . import helper\n", "so101_vlm/policy/helper.py": ""},
    ),
]


@pytest.mark.parametrize("files", [c[1] for c in CLEAN_CASES], ids=[c[0] for c in CLEAN_CASES])
def test_違反でないものは検出しない(tmp_path, files):
    assert check(tmp_path, files, BASE) is None


# === 失敗メッセージに経路が出る ===
def test_違反の経路を返す(tmp_path):
    files = {
        "so101_vlm/policy/base.py": "from so101_vlm.safety.config import C\n",
        "so101_vlm/safety/config.py": "from so101_vlm.eval.validation import f\n",
    }
    path = check(tmp_path, files, BASE)
    assert " -> ".join(path) == (
        "so101_vlm.policy.base -> so101_vlm.safety.config -> so101_vlm.eval.validation"
    )


# === 文字コードと改行 ===
def test_BOM付きでも検出する(tmp_path):
    src = b"\xef\xbb\xbfimport so101_vlm.eval.x\n"
    assert check(tmp_path, {"so101_vlm/policy/base.py": src}, BASE) is not None


def test_CRLFでも検出する(tmp_path):
    src = b"import os\r\nfrom so101_vlm.eval import x\r\n"
    assert check(tmp_path, {"so101_vlm/policy/base.py": src}, BASE) is not None


def test_構文エラーはファイル名入りで失敗する(tmp_path):
    src = make_pkg(tmp_path, {"so101_vlm/policy/broken.py": "def (:\n"})
    with pytest.raises(ValueError, match="broken.py"):
        build_graph(src)


# === 相対 import の解決規則 ===
def test_通常モジュールの相対import():
    got = imported_modules("from . import x\nfrom .y import z\n", "so101_vlm.policy.base", False)
    assert got == {
        "so101_vlm.policy",
        "so101_vlm.policy.x",
        "so101_vlm.policy.y",
        "so101_vlm.policy.y.z",
    }


def test_init内の相対importは自パッケージ直下を指す():
    got = imported_modules("from . import x\n", "so101_vlm.policy", True)
    assert "so101_vlm.policy.x" in got
    assert "so101_vlm.x" not in got


def test_二段上の相対import():
    got = imported_modules("from .. import eval\n", "so101_vlm.policy.base", False)
    assert "so101_vlm.eval" in got
    got = imported_modules("from ..eval import x\n", "so101_vlm.policy.sub.mod", False)
    assert "so101_vlm.policy.eval" in got  # 3 段目のモジュールでは .. は policy を指す


def test_トップより上へ出る相対importはエラー():
    with pytest.raises(ValueError, match="so101_vlm.x"):
        imported_modules("from .. import y\n", "so101_vlm.x", False)


def test_外部importは返さない():
    got = imported_modules("import os\nfrom numpy import array\n", "so101_vlm.policy.base", False)
    assert got == set()


# === 循環と開始モジュール ===
def test_循環importでも終わる(tmp_path):
    files = {
        "so101_vlm/policy/a.py": "import so101_vlm.policy.b\n",
        "so101_vlm/policy/b.py": "import so101_vlm.policy.a\n",
    }
    assert check(tmp_path, files, "so101_vlm.policy.a") is None


def test_循環の中の違反を検出し_開始自身のimportも判定する(tmp_path):
    files = {
        "so101_vlm/policy/a.py": "import so101_vlm.policy.b\nimport so101_vlm.eval.x\n",
        "so101_vlm/policy/b.py": "import so101_vlm.policy.a\n",
    }
    assert check(tmp_path, files, "so101_vlm.policy.b") is not None
    assert check(tmp_path, files, "so101_vlm.policy.a") == [
        "so101_vlm.policy.a",
        "so101_vlm.eval.x",
    ]


# === 非定数の動的 import ===
def test_ガード対象の非定数動的importは検査不能として検出する(tmp_path):
    dyn = 'import importlib\nn = "x"\nimportlib.import_module(f"so101_vlm.{n}")\n'
    src = make_pkg(
        tmp_path,
        {"so101_vlm/policy/base.py": dyn, "so101_vlm/safety/config.py": dyn},
    )
    got = guarded_unresolved(src)
    assert list(got) == ["so101_vlm.policy.base"]  # 推移先(safety)は対象外
    assert got["so101_vlm.policy.base"] == [3]


# === 実リポジトリ ===
SRC_ROOT = Path(__file__).resolve().parents[1] / "src"


def test_実リポジトリでpolicyとperceptionはevalに依存しない():
    # 🔵 FR-106 / FR-401: eval・scene_spec を直接も推移でも import しない
    raw = collect_raw_imports(SRC_ROOT)
    graph = build_graph(SRC_ROOT, raw)
    targets = sorted(m for m in graph if is_guarded(m))
    assert len(targets) >= 13, f"検査対象が少なすぎます: {targets}"
    broken = {m: " -> ".join(p) for m in targets if (p := find_violation(graph, raw, m))}
    assert not broken, f"policy/perception が eval に依存しています: {broken}"
    assert guarded_unresolved(SRC_ROOT) == {}, "静的に追えない動的 import があります"
