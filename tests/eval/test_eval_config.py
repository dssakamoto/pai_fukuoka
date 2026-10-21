"""load_eval_config / EvalConfig / PlacementConfig のテスト (TASK-0006, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0006/layout-testcases.md
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import copy
import dataclasses

import pytest
import yaml

# 【TASK-0013】JudgeConfig / FailureConfig はモジュール経由で参照する (未実装の間も、
# 既存テストが ImportError で全滅しないよう、import 時ではなくテスト実行時に名前を引く)
import so101_vlm.eval.eval_config as eval_config_module
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
    # TASK-0013: 成功判定と失敗分類のしきい値 (pick-testcases.md 2.0 節) 🔵
    "judge": {"margin_m": 0.0, "speed_max_mps": 0.01, "settle_s": 1.0},
    "failure": {
        "lift_min_m": 0.03,
        "carry_separation_m": 0.02,
        "other_move_m": 0.005,
        "tilt_max_deg": 20.0,
    },
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
    # --- TASK-0013 追加: 判定と失敗分類の設定 (configs/eval.yaml は Green で更新する) 🔵 ---
    judge_cls, failure_cls = eval_config_module.JudgeConfig, eval_config_module.FailureConfig
    assert isinstance(cfg.judge, judge_cls)  # 【確認内容】: 型
    assert isinstance(cfg.failure, failure_cls)  # 【確認内容】: 型
    assert cfg.judge == judge_cls(0.0, 0.01, 1.0)  # 【確認内容】: リポジトリの値 (FR-07)
    assert cfg.failure == failure_cls(0.03, 0.02, 0.005, 20.0)  # 【確認内容】: 初期しきい値


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
    # --- TASK-0013 追加 🔵 ---
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.judge.margin_m = 0.1
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.failure.tilt_max_deg = 1.0


@pytest.mark.parametrize("as_str", [False, True])
def test_load_eval_config_reads_values_from_given_path(tmp_path, as_str):
    """TC-C-04 tmp_path の別の値を正しく読む (Path と str の両方) 🟡/🔴"""
    obj = {
        "n_trials": 5,
        "base_seed": 100,
        "placement": {"n_blocks": 2, "area": 0.1, "min_gap": 0.03, "max_attempts": 50},
        "judge": {"margin_m": 0.005, "speed_max_mps": 0.02, "settle_s": 2.0},
        "failure": {
            "lift_min_m": 0.04,
            "carry_separation_m": 0.03,
            "other_move_m": 0.01,
            "tilt_max_deg": 30.0,
        },
    }
    path = _write(tmp_path, obj)
    cfg = load_eval_config(str(path) if as_str else path)
    assert cfg.n_trials == 5
    assert cfg.base_seed == 100
    assert cfg.placement.n_blocks == 2
    assert cfg.placement.area == pytest.approx(0.1)
    assert cfg.placement.min_gap == pytest.approx(0.03)
    assert cfg.placement.max_attempts == 50
    # --- TASK-0013 追加: 別の値が正しく読める 🟡 ---
    assert cfg.judge.margin_m == pytest.approx(0.005)
    assert cfg.judge.speed_max_mps == pytest.approx(0.02)
    assert cfg.judge.settle_s == pytest.approx(2.0)
    assert cfg.failure.lift_min_m == pytest.approx(0.04)
    assert cfg.failure.carry_separation_m == pytest.approx(0.03)
    assert cfg.failure.other_move_m == pytest.approx(0.01)
    assert cfg.failure.tilt_max_deg == pytest.approx(30.0)


def test_load_eval_config_normalizes_int_to_float(tmp_path):
    """TC-C-05 YAML の int を float フィールドに受け入れ、型を正規化 🔴"""
    cfg = load_eval_config(_write(tmp_path, _with(["placement", "min_gap"], 0)))
    assert cfg.placement.min_gap == 0.0
    assert type(cfg.placement.min_gap) is float  # 【確認内容】: float に正規化
    assert type(cfg.n_trials) is int
    assert type(cfg.base_seed) is int
    assert type(cfg.placement.n_blocks) is int
    assert type(cfg.placement.max_attempts) is int


def test_load_eval_config_normalizes_judge_failure_int_to_float(tmp_path):
    """TC-C-05 (TASK-0013 追加) judge / failure の int を float に正規化 🔴"""
    obj = _with(["judge", "settle_s"], 1)
    obj["failure"]["tilt_max_deg"] = 20
    cfg = load_eval_config(_write(tmp_path, obj))
    assert type(cfg.judge.settle_s) is float  # 【確認内容】: float に正規化
    assert cfg.judge.settle_s == 1.0
    assert type(cfg.failure.tilt_max_deg) is float
    assert cfg.failure.tilt_max_deg == 20.0


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
        # --- TASK-0013 追加 🔵 ---
        (_with(["judge", "margin"], 0.0), "margin"),
        (_with(["failure", "tilt_deg"], 20), "tilt_deg"),
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
        # --- TASK-0013 追加 🔵 ---
        ["judge"],
        ["failure"],
        ["judge", "margin_m"],
        ["judge", "speed_max_mps"],
        ["judge", "settle_s"],
        ["failure", "lift_min_m"],
        ["failure", "carry_separation_m"],
        ["failure", "other_move_m"],
        ["failure", "tilt_max_deg"],
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
        # --- TASK-0013 追加 🟡 ---
        (["judge", "margin_m"], -0.001),
        (["judge", "margin_m"], 0.04),  # box_inner_half_size 未満であること
        (["judge", "margin_m"], "0"),
        (["judge", "speed_max_mps"], 0),
        (["judge", "speed_max_mps"], True),
        (["judge", "settle_s"], -0.1),
        (["judge", "settle_s"], float("nan")),
        (["failure", "lift_min_m"], 0),
        (["failure", "carry_separation_m"], -0.02),
        (["failure", "other_move_m"], float("inf")),
        (["failure", "tilt_max_deg"], 0),
        (["failure", "tilt_max_deg"], 90),
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
        # --- TASK-0013 追加 🔴: judge / failure が mapping でない (他のキーは正しい) ---
        yaml.safe_dump({**copy.deepcopy(VALID), "judge": [1, 2]}),
        yaml.safe_dump({**copy.deepcopy(VALID), "failure": None}),
    ],
    ids=["list", "scalar", "empty", "placement-list", "placement-null", "judge-list", "failure-null"],
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
        # TASK-0013 追加: judge は下限ちょうど (margin_m=0, settle_s=0) 🟡
        "judge": {"margin_m": 0, "speed_max_mps": 0.01, "settle_s": 0},
        "failure": copy.deepcopy(VALID["failure"]),
    }
    cfg = load_eval_config(_write(tmp_path, obj))
    assert cfg.n_trials == 1 and cfg.placement.max_attempts == 1
    assert cfg.judge.margin_m == 0.0 and cfg.judge.settle_s == 0.0  # 【確認内容】: 下限は受け入れる


def test_load_eval_config_accepts_just_below_upper_bounds(tmp_path):
    """TC-C-B02 (TASK-0013 追加) 上限の直前は受け入れる (0.04 と 90 は E04 で拒否) 🟡"""
    obj = _with(["judge", "margin_m"], 0.0399)
    obj["failure"]["tilt_max_deg"] = 89.9
    cfg = load_eval_config(_write(tmp_path, obj))
    assert cfg.judge.margin_m == pytest.approx(0.0399)  # 【確認内容】: 内寸未満
    assert cfg.failure.tilt_max_deg == pytest.approx(89.9)  # 【確認内容】: (0, 90) の内側
