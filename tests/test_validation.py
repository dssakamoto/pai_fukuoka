"""中立の検証ヘルパ so101_vlm.validation のテスト (SGT-001)。

【機能概要】: eval/validation.py から移設した require_* の振る舞い・再エクスポート・
eval 非依存を検証する。
"""

import ast
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

import so101_vlm.eval.validation as eval_validation
import so101_vlm.validation as neutral
from so101_vlm.validation import require_exact_keys, require_int, require_min_gap, require_number

PKG_DIR = Path(__file__).resolve().parent.parent / "src" / "so101_vlm"

# eval.validation を import していた非 eval の 9 ファイル 🔵
MIGRATED_FILES = [
    "nodes/node_config.py",
    "nodes/registry_types.py",
    "perception/calibration.py",
    "policy/base.py",
    "policy/trajectory_config.py",
    "record/record_config.py",
    "safety/config.py",
    "safety/_validation.py",
    "vlm/config.py",
]
# registry_types は評価(eval.fixed_pick)を正当に使う層なので eval 非依存の検査から除く 🟡
EVAL_FREE_FILES = [f for f in MIGRATED_FILES if f != "nodes/registry_types.py"]


def _imported_modules(relative: str) -> set[str]:
    """【機能概要】: ファイル内の全 import (TYPE_CHECKING 内も含む) のモジュール名を返す 🔵"""
    tree = ast.parse((PKG_DIR / relative).read_text(encoding="utf-8"))
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.add(node.module)
    return modules


def _is_eval(module: str) -> bool:
    return module == "so101_vlm.eval" or module.startswith("so101_vlm.eval.")


class TestRequireInt:
    def test_整数とnumpy整数はintで返る(self) -> None:
        """正常系: int と np.int64 は int で返る。"""
        assert require_int("n", 5, 0) == 5
        result = require_int("n", np.int64(5), 0)
        assert result == 5 and type(result) is int

    def test_boolはValueError(self) -> None:
        """bool は整数として扱わない。"""
        with pytest.raises(ValueError, match="n は整数で指定してください: n=True"):
            require_int("n", True, 0)

    @pytest.mark.parametrize("value", ["3", 3.0])
    def test_整数でない型はValueError(self, value: object) -> None:
        """文字列と float は拒否する。"""
        with pytest.raises(ValueError, match="n は整数で指定してください"):
            require_int("n", value, 0)

    def test_minimum未満はValueError(self) -> None:
        """minimum 未満はメッセージに名前と値を含めて拒否する。"""
        with pytest.raises(ValueError, match="n は 0 以上で指定してください: n=-1"):
            require_int("n", -1, 0)


class TestRequireNumber:
    @pytest.mark.parametrize("value", [1, 1.5, np.float64(2.5)])
    def test_実数はfloatで返る(self, value: object) -> None:
        """int / float / numpy 実数は float で返る。"""
        result = require_number("x", value)
        assert type(result) is float and result == float(value)

    @pytest.mark.parametrize("value", [True, "1"])
    def test_boolと文字列はValueError(self, value: object) -> None:
        """bool と文字列は数値として扱わない。"""
        with pytest.raises(ValueError, match="x は数値で指定してください"):
            require_number("x", value)

    @pytest.mark.parametrize("value", [float("nan"), float("inf")])
    def test_非有限はValueError(self, value: float) -> None:
        """nan / inf は拒否する。"""
        with pytest.raises(ValueError, match="x は有限の数値で指定してください"):
            require_number("x", value)


class TestRequireMinGap:
    @pytest.mark.parametrize("value", [0, 0.5])
    def test_0以上は通る(self, value: float) -> None:
        """0 と正の値は float で返る。"""
        assert require_min_gap("g", value) == float(value)

    def test_負はValueError(self) -> None:
        """負の間隔は拒否する。"""
        with pytest.raises(ValueError, match="g は 0 以上で指定してください: g=-1"):
            require_min_gap("g", -1)


class TestRequireExactKeys:
    def test_過不足なしはmappingを返す(self) -> None:
        """キーが一致すれば同じ mapping を返す。"""
        value = {"a": 1, "b": 2}
        assert require_exact_keys("cfg", value, frozenset({"a", "b"})) is value

    def test_未知キーはValueError(self) -> None:
        """未知キーは名前順で列挙する。"""
        with pytest.raises(ValueError, match=r"^未知のキー: a, b \(cfg\)$"):
            require_exact_keys("cfg", {"b": 1, "a": 2}, frozenset())

    def test_欠落キーはValueError(self) -> None:
        """欠落キーは必須キーがありませんとして報告する。"""
        with pytest.raises(ValueError, match=r"^必須キーがありません: x \(cfg\)$"):
            require_exact_keys("cfg", {}, frozenset({"x"}))

    def test_mappingでない値はValueError(self) -> None:
        """mapping 以外は拒否する。"""
        with pytest.raises(ValueError, match="cfg は mapping で指定してください: cfg=3"):
            require_exact_keys("cfg", 3, frozenset())


@pytest.mark.parametrize("name", ["require_int", "require_number", "require_min_gap", "require_exact_keys"])
def test_eval側の再エクスポートは同一オブジェクト(name: str) -> None:
    """eval.validation は中立側の関数をそのまま再エクスポートする。"""
    assert getattr(neutral, name) is getattr(eval_validation, name)


def test_validationはevalをimportしない() -> None:
    """別プロセスで import しても sys.modules に so101_vlm.eval* が現れない。"""
    code = (
        "import sys, so101_vlm.validation; "
        "bad = [m for m in sys.modules if m == 'so101_vlm.eval' or m.startswith('so101_vlm.eval.')]; "
        "print(bad); sys.exit(1 if bad else 0)"
    )
    proc = subprocess.run([sys.executable, "-I", "-c", code], capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stdout + proc.stderr


@pytest.mark.parametrize("relative", MIGRATED_FILES)
def test_移行対象はeval_validationをimportしない(relative: str) -> None:
    """9 ファイルとも so101_vlm.eval.validation を import しない。"""
    assert "so101_vlm.eval.validation" not in _imported_modules(relative)


@pytest.mark.parametrize("relative", EVAL_FREE_FILES)
def test_移行対象はevalに依存しない(relative: str) -> None:
    """registry_types を除く 8 ファイルは so101_vlm.eval 配下を import しない。"""
    assert [m for m in _imported_modules(relative) if _is_eval(m)] == []
