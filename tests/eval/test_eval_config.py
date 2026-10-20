"""load_eval_config / EvalConfig / PlacementConfig のテスト (TASK-0006, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0006/layout-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import dataclasses

import pytest
import yaml

from so101_vlm.eval.eval_config import (
    DEFAULT_EVAL_CONFIG_PATH,
    EvalConfig,
    PlacementConfig,
    load_eval_config,
)
from so101_vlm.robot.model_paths import REPO_ROOT

VALID = {
    "n_trials": 20,
    "base_seed": 0,
    "placement": {"n_blocks": 3, "area": 0.20, "min_gap": 0.05, "max_attempts": 10000},
}


def _write(tmp_path, obj):
    """obj が str ならそのまま、それ以外は safe_dump して eval.yaml に書く。"""
    path = tmp_path / "eval.yaml"
    path.write_text(obj if isinstance(obj, str) else yaml.safe_dump(obj), encoding="utf-8")
    return path


def _with(path_keys, value):
    """VALID の深いコピーの path_keys 位置を value にした dict を返す (VALID は不変)。"""
    obj = copy.deepcopy(VALID)
    target = obj
    for key in path_keys[:-1]:
        target = target[key]
    target[path_keys[-1]] = value
    return obj


def _without(path_keys):
    obj = copy.deepcopy(VALID)
    target = obj
    for key in path_keys[:-1]:
        target = target[key]
    del target[path_keys[-1]]
    return obj


# =============================================================== 正常系
def test_load_eval_config_reads_repository_file():
    """TC-C-01 リポジトリの configs/eval.yaml の値 🔵/🟡"""
    cfg = load_eval_config()  # configs/eval.yaml は Green で作成する
    assert isinstance(cfg, EvalConfig)  # 【確認内容】: 型
    assert isinstance(cfg.placement, PlacementConfig)  # 【確認内容】: 入れ子の型
    assert cfg.n_trials == 20
    assert cfg.base_seed == 0
    assert cfg.placement.n_blocks == 3
    assert cfg.placement.area == pytest.approx(0.20)
    assert cfg.placement.min_gap == pytest.approx(0.05)
    assert cfg.placement.max_attempts == 10000


def test_default_path_is_repository_configs_eval_yaml():
    """TC-C-02 既定パスはリポジトリの configs/eval.yaml 🟡"""
    assert DEFAULT_EVAL_CONFIG_PATH == REPO_ROOT / "configs" / "eval.yaml"
    assert DEFAULT_EVAL_CONFIG_PATH.is_file()  # 【確認内容】: 実ファイルが存在
    assert load_eval_config() == load_eval_config(DEFAULT_EVAL_CONFIG_PATH)


def test_config_is_frozen():
    """TC-C-03 設定は frozen 🔵"""
    cfg = load_eval_config(DEFAULT_EVAL_CONFIG_PATH)
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.n_trials = 1
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.placement.area = 0.1


@pytest.mark.parametrize("as_str", [False, True])
def test_load_eval_config_reads_values_from_given_path(tmp_path, as_str):
    """TC-C-04 tmp_path の別の値を正しく読む (Path と str の両方) 🟡/🔴"""
    obj = {
        "n_trials": 5,
        "base_seed": 100,
        "placement": {"n_blocks": 2, "area": 0.1, "min_gap": 0.03, "max_attempts": 50},
    }
    path = _write(tmp_path, obj)
    cfg = load_eval_config(str(path) if as_str else path)
    assert cfg.n_trials == 5
    assert cfg.base_seed == 100
    assert cfg.placement.n_blocks == 2
    assert cfg.placement.area == pytest.approx(0.1)
    assert cfg.placement.min_gap == pytest.approx(0.03)
    assert cfg.placement.max_attempts == 50


def test_load_eval_config_normalizes_int_to_float(tmp_path):
    """TC-C-05 YAML の int を float フィールドに受け入れ、型を正規化 🔴"""
    cfg = load_eval_config(_write(tmp_path, _with(["placement", "min_gap"], 0)))
    assert cfg.placement.min_gap == 0.0
    assert type(cfg.placement.min_gap) is float  # 【確認内容】: float に正規化
    assert type(cfg.n_trials) is int
    assert type(cfg.base_seed) is int
    assert type(cfg.placement.n_blocks) is int
    assert type(cfg.placement.max_attempts) is int


# =============================================================== 異常系
def test_load_eval_config_missing_file_raises_file_not_found(tmp_path):
    """TC-C-E01 ファイルがない → FileNotFoundError 🔴"""
    with pytest.raises(FileNotFoundError):
        load_eval_config(tmp_path / "missing.yaml")


@pytest.mark.parametrize(
    ("obj", "key"),
    [
        ({**copy.deepcopy(VALID), "n_trial": 20}, "n_trial"),
        (_with(["placement", "min_gapp"], 0.05), "min_gapp"),
    ],
)
def test_load_eval_config_rejects_unknown_key(tmp_path, obj, key):
    """TC-C-E02 未知のキー → ValueError (キー名入り) 🔴"""
    with pytest.raises(ValueError, match=key):
        load_eval_config(_write(tmp_path, obj))


@pytest.mark.parametrize(
    "path_keys",
    [
        ["n_trials"],
        ["base_seed"],
        ["placement"],
        ["placement", "n_blocks"],
        ["placement", "area"],
        ["placement", "min_gap"],
        ["placement", "max_attempts"],
    ],
)
def test_load_eval_config_rejects_missing_required_key(tmp_path, path_keys):
    """TC-C-E03 必須キーの欠落 → ValueError (欠けたキー名入り) 🔴"""
    with pytest.raises(ValueError, match=path_keys[-1]):
        load_eval_config(_write(tmp_path, _without(path_keys)))


@pytest.mark.parametrize(
    ("path_keys", "value"),
    [
        (["n_trials"], 0),
        (["n_trials"], "20"),
        (["n_trials"], 1.5),
        (["n_trials"], True),
        (["base_seed"], -1),
        (["base_seed"], True),
        (["placement", "n_blocks"], 0),
        (["placement", "area"], "0.2"),
        (["placement", "area"], 0.3),
        (["placement", "area"], float("nan")),
        (["placement", "min_gap"], -0.1),
        (["placement", "max_attempts"], 0),
    ],
)
def test_load_eval_config_rejects_invalid_value(tmp_path, path_keys, value):
    """TC-C-E04 型・値域の不正 → ValueError (キー名入り) 🔴/🟡"""
    # nan は safe_dump が `.nan` として出力する
    path = _write(tmp_path, _with(path_keys, value))
    with pytest.raises(ValueError, match=path_keys[-1]):
        load_eval_config(path)


@pytest.mark.parametrize(
    "body",
    [
        "- 1\n- 2\n",
        "42\n",
        "",
        "n_trials: 20\nbase_seed: 0\nplacement: [1, 2]\n",
        "n_trials: 20\nbase_seed: 0\nplacement:\n",
    ],
    ids=["list", "scalar", "empty", "placement-list", "placement-null"],
)
def test_load_eval_config_rejects_non_mapping(tmp_path, body):
    """TC-C-E05 mapping でない → ValueError (AttributeError/TypeError を漏らさない) 🔴"""
    with pytest.raises(ValueError):
        load_eval_config(_write(tmp_path, body))


def test_load_eval_config_uses_safe_load(tmp_path):
    """TC-C-E06 危険な YAML タグは解釈しない (safe_load) 🔵"""
    path = _write(tmp_path, "!!python/object/apply:os.getcwd []\n")
    with pytest.raises((yaml.YAMLError, ValueError)):
        load_eval_config(path)


# =============================================================== 境界値
def test_load_eval_config_accepts_lower_bounds(tmp_path):
    """TC-C-B01 下限値ちょうど (area=0.20 は幅 0.19999999999999998 との許容差が要る) 🟡"""
    obj = {
        "n_trials": 1,
        "base_seed": 0,
        "placement": {"n_blocks": 1, "area": 0.20, "min_gap": 0, "max_attempts": 1},
    }
    cfg = load_eval_config(_write(tmp_path, obj))
    assert cfg.n_trials == 1 and cfg.placement.max_attempts == 1
