"""calibration の YAML 入出力 (save_calibration / load_calibration / ドリフトガード) の単体テスト (TASK-0009, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0009/calibration-testcases.md
対象: src/so101_vlm/perception/calibration.py の保存・読み込み・検証。MuJoCo 計算とデータクラスは test_calibration.py。
共有フィクスチャは conftest.py、共有定数・ヘルパは calibration_support.py。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定
"""

import copy
import math
import re
import socket
import struct

import numpy as np
import pytest
import yaml
from so101_vlm.eval.scene_spec import SCENE_SPEC
from so101_vlm.perception.calibration import (
    DEFAULT_CALIBRATION_PATH,
    Calibration,
    calibration_from_mujoco,
    calibrations_close,
    load_calibration,
    save_calibration,
)
from so101_vlm.perception.projection import (
    Camera,
)
from so101_vlm.robot.model_paths import REPO_ROOT

from calibration_support import (
    FY_60_480,
    SIM_T_ROWS,
    H,
    W,
    forwarded,
    golden_calibration,
)

DRIFT_ATOL = 1e-9  # --check と同じ (要件 2.10)
HEADER_SIM = (
    "# 自動生成ファイル (source: sim)。手で編集しないこと。\n"
    "# 再生成: uv run python scripts/generate_calibration.py\n"
    "# 確認:   uv run python scripts/generate_calibration.py --check\n"
)


def _bits(x):
    return struct.pack("<d", x)


def _write(tmp_path, obj, name="c.yaml"):
    p = tmp_path / name
    p.write_text(obj if isinstance(obj, str) else yaml.safe_dump(obj, sort_keys=False), encoding="utf-8")
    return p


# ================================================================ 1.4 save_calibration
def _golden_text():
    rows = "\n".join("- [" + ", ".join(repr(v) for v in r) + "]" for r in SIM_T_ROWS)
    return (
        HEADER_SIM
        + "version: 1\nmetadata:\n  source: sim\n  camera_name: cam_top\n  base_frame: baseframe\n"
        + f"camera:\n  fx: {FY_60_480!r}\n  fy: {FY_60_480!r}\n  cx: 319.5\n  cy: 239.5\n  width: 640\n  height: 480\n"
        + "t_base_cam:\n"
        + rows
        + "\n"
    )


def test_save_calibration_matches_golden_text(tmp_path):
    """Y-01: 書いた内容が要件 2.6 の形式とバイト一致 (ゴールデン)。🔴

    数値は実測値のリテラルから作った Calibration を使う (MuJoCo の版差に依存させない)。
    """
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration(), p)
    assert p.read_text(encoding="utf-8") == _golden_text()  # 【確認内容】: コメント 3 行 + 本文


def test_save_calibration_sim_header(tmp_path):
    """Y-02: 先頭のコメント (sim)。🔴"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration(), p)
    lines = p.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "# 自動生成ファイル (source: sim)。手で編集しないこと。"
    assert any("uv run python scripts/generate_calibration.py" in l and "--check" in l for l in lines[:3])
    assert "#" not in "".join(yaml.safe_load(p.read_text(encoding="utf-8")).keys())  # コメントは値に現れない


def test_save_calibration_real_header(tmp_path):
    """Y-03: 先頭のコメント (real)。🔴"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration("real"), p)
    head = "".join(p.read_text(encoding="utf-8").splitlines(keepends=True)[:3])
    assert "実機の計測値" in head and "scripts/generate_calibration.py で上書きしないこと" in head
    assert "自動生成ファイル" not in head


def test_save_calibration_key_order(tmp_path):
    """Y-04: キーの順序。🔴"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration(), p)
    text = p.read_text(encoding="utf-8")
    assert re.findall(r"^([a-z_]+):", text, re.M) == ["version", "metadata", "camera", "t_base_cam"]
    assert re.findall(r"^  ([a-z_]+):", text, re.M) == [
        "source", "camera_name", "base_frame", "fx", "fy", "cx", "cy", "width", "height",
    ]  # fmt: skip


def test_save_calibration_matrix_rows_are_flow_lines(tmp_path):
    """Y-05: 行列の各行が 1 行の flow 形式。🔴"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration(), p)
    tail = p.read_text(encoding="utf-8").split("t_base_cam:\n")[1].splitlines()
    assert len(tail) == 4
    for line in tail:
        assert re.fullmatch(r"- \[[^\]]+\]", line), line
        assert len(line[3:-1].split(",")) == 4


def test_save_load_round_trip_is_bit_identical_for_real_values(sim_calibration, tmp_path):
    """Y-06: 往復でビット一致 (実際の値)。🔵"""
    p = tmp_path / "c.yaml"
    save_calibration(sim_calibration, p)
    back = load_calibration(p)
    assert calibrations_close(sim_calibration, back, atol=0)
    assert np.array_equal(sim_calibration.T_base_cam.view(np.uint64), back.T_base_cam.view(np.uint64))
    for n in ("fx", "fy", "cx", "cy"):
        assert _bits(getattr(sim_calibration.camera, n)) == _bits(getattr(back.camera, n)), n


def _random_rotation(rng):
    q = rng.normal(size=4)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def test_save_load_round_trip_bit_identical_random_and_special_values(tmp_path):
    """Y-07 / B-06: 乱数と特殊値 (-0.0, 非正規化数, 最大値) でもビット一致。🔵"""
    rng = np.random.default_rng(1)
    specials_fx = [5e-324, 1.7976931348623157e308]
    specials_c = [-0.0, 5e-324, 1.1125369292536007e-308]
    cases = []
    for i in range(20):
        t = rng.uniform(-1e3, 1e3, 3)
        if i == 0:
            t = np.array([-0.0, 5e-324, 1.1125369292536007e-308])
        fx = specials_fx[i] if i < 2 else float(rng.uniform(1, 2000))
        cx = specials_c[i] if i < 3 else float(rng.normal())
        cases.append((_random_rotation(rng), t, fx, cx))
    R_neg0 = np.eye(3)
    R_neg0[0, 1] = R_neg0[1, 2] = -0.0
    cases.append((R_neg0, np.zeros(3), 1.0, -0.0))
    for k, (R, t, fx, cx) in enumerate(cases):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        cam = Camera(fx=fx, fy=fx, cx=cx, cy=-1.5, width=640, height=480)
        cal = Calibration(cam, T, "sim", "cam_top", "baseframe")
        p = tmp_path / f"c{k}.yaml"
        save_calibration(cal, p)
        back = load_calibration(p)
        assert np.array_equal(T.view(np.uint64), back.T_base_cam.view(np.uint64)), k  # 【確認内容】: -0.0 の符号も保つ
        assert _bits(fx) == _bits(back.camera.fx) and _bits(cx) == _bits(back.camera.cx), k
    assert math.copysign(1, load_calibration(tmp_path / f"c{len(cases) - 1}.yaml").camera.cx) == -1


def test_save_calibration_accepts_numpy_typed_fields(tmp_path):
    """Y-08: numpy の型を含む Calibration も保存できる (safe_dump は np 型を拒否する)。🔵"""
    cam = Camera(fx=np.float64(FY_60_480), fy=np.float64(FY_60_480), cx=np.float32(319.5),
                 cy=np.float32(239.5), width=np.int64(640), height=np.int64(480))  # fmt: skip
    cal = Calibration(cam, np.array(SIM_T_ROWS, dtype=np.float64), "sim", "cam_top", "baseframe")
    p = tmp_path / "c.yaml"
    save_calibration(cal, p)
    text = p.read_text(encoding="utf-8")
    assert "!!python" not in text and "numpy" not in text
    back = load_calibration(p)
    assert (back.camera.cx, back.camera.cy, back.camera.width) == (319.5, 239.5, 640)


def test_save_calibration_is_deterministic_without_timestamps(tmp_path):
    """Y-09: 非決定的な内容 (時刻・ホスト名) がない。🔴"""
    a, b, other = tmp_path / "a.yaml", tmp_path / "b.yaml", tmp_path / "o.yaml"
    cal = golden_calibration()
    save_calibration(cal, a)
    save_calibration(golden_calibration("real"), other)
    save_calibration(cal, b)
    assert a.read_bytes() == b.read_bytes()
    text = a.read_text(encoding="utf-8")
    assert not re.search(r"\d{4}-\d{2}-\d{2}", text) and socket.gethostname() not in text


def test_save_calibration_overwrites_existing_file(tmp_path):
    """Y-10: 既存ファイルを上書きする。🟡"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration("real"), p)
    save_calibration(golden_calibration(), p)
    assert p.read_text(encoding="utf-8") == _golden_text()


def test_save_calibration_accepts_str_and_path(tmp_path):
    """Y-11: str と Path の両方を受け付ける。🔵"""
    cal = golden_calibration()
    save_calibration(cal, str(tmp_path / "s.yaml"))
    save_calibration(cal, tmp_path / "p.yaml")
    assert (tmp_path / "s.yaml").read_bytes() == (tmp_path / "p.yaml").read_bytes()


def test_save_calibration_writes_utf8(tmp_path):
    """Y-12: UTF-8 で書く (日本語のコメントが読める)。🟡"""
    p = tmp_path / "c.yaml"
    save_calibration(golden_calibration(), p)
    assert "自動生成ファイル" in p.read_bytes().decode("utf-8")


# ================================================================ 1.5 load_calibration
def test_load_calibration_reads_requirements_yaml(valid_doc, tmp_path):
    """L-01: 要件 2.6 の YAML を読める。🔵"""
    cal = load_calibration(_write(tmp_path, valid_doc))
    assert isinstance(cal, Calibration)
    assert cal.camera == Camera(fx=FY_60_480, fy=FY_60_480, cx=319.5, cy=239.5, width=640, height=480)
    assert np.array_equal(cal.T_base_cam, np.array(SIM_T_ROWS))  # 【確認内容】: ビット一致
    assert cal.source == "sim"


def test_default_calibration_path_points_to_committed_file():
    """L-02: 既定パスは configs/calibration.yaml で、読める。🔵"""
    assert DEFAULT_CALIBRATION_PATH == REPO_ROOT / "configs" / "calibration.yaml"
    assert DEFAULT_CALIBRATION_PATH.is_file()
    load_calibration()  # 例外なし


def test_load_calibration_accepts_str_and_path(valid_doc, tmp_path):
    """L-03: str と Path の両方。🔵"""
    p = _write(tmp_path, valid_doc)
    assert calibrations_close(load_calibration(str(p)), load_calibration(p), atol=0)


def test_load_calibration_reads_real_source(valid_doc, tmp_path):
    """L-04: source: real も読める (TASK-0036 用)。🔵"""
    valid_doc["metadata"]["source"] = "real"
    assert load_calibration(_write(tmp_path, valid_doc)).source == "real"


def test_load_calibration_converts_int_intrinsics_to_float(valid_doc, tmp_path):
    """L-05 / B-08: 整数の内部パラメータは float にする。🟡"""
    valid_doc["camera"].update(fx=415, cx=320)
    cam = load_calibration(_write(tmp_path, valid_doc)).camera
    assert type(cam.fx) is float and cam.fx == 415.0 and type(cam.cx) is float


def test_load_calibration_reads_commented_saved_file(sim_calibration, tmp_path):
    """L-06: コメント付き (save の出力) も読める。🔵"""
    p = tmp_path / "c.yaml"
    save_calibration(sim_calibration, p)
    assert calibrations_close(load_calibration(p), sim_calibration, atol=0)


def test_load_calibration_T_is_read_only(valid_doc, tmp_path):
    """L-07: 読み込んだ T は書き込み不可。🔴"""
    cal = load_calibration(_write(tmp_path, valid_doc))
    with pytest.raises(ValueError):
        cal.T_base_cam[0, 0] = 0.0


# ================================================================ 1.7 ドリフトガード
def test_committed_yaml_matches_scene(committed, scene_model):
    """R-01: コミット済みの YAML がシーンから作った値と一致。🟡"""
    fresh = calibration_from_mujoco(
        scene_model, forwarded(scene_model), SCENE_SPEC.camera_name, SCENE_SPEC.image_width, SCENE_SPEC.image_height
    )
    assert calibrations_close(committed, fresh, atol=DRIFT_ATOL), (
        "configs/calibration.yaml がシーンと食い違っています: uv run python scripts/generate_calibration.py で再生成"
    )


def test_committed_yaml_is_byte_identical_to_save_output(committed, scene_model, tmp_path):
    """R-02: コミット済みのファイルは save の出力とバイト一致 (手編集の検出)。🔴

    MuJoCo の版差で末尾の桁が揺れて落ちる場合は R-01 だけ残す判断を verify-complete に記録する。
    """
    fresh = calibration_from_mujoco(scene_model, forwarded(scene_model), "cam_top", W, H)
    p = tmp_path / "fresh.yaml"
    save_calibration(fresh, p)
    assert DEFAULT_CALIBRATION_PATH.read_bytes() == p.read_bytes()


def test_committed_yaml_matches_task0008_premises(committed):
    """R-03: コミット済みの値が TASK-0008 の前提と一致。🔵"""
    assert committed.camera_name == "cam_top" and committed.base_frame == "baseframe"
    assert (committed.camera.width, committed.camera.height) == (640, 480)
    np.testing.assert_allclose(committed.T_base_cam[:3, 3], [0.45, 0.30, 0.55], atol=1e-12)


# ================================================================ 2.4 save_calibration 異常系
def test_save_calibration_missing_parent_raises_without_creating(tmp_path):
    """Y-E01: 親ディレクトリがなければ FileNotFoundError (自動作成しない)。🔴"""
    with pytest.raises(FileNotFoundError):
        save_calibration(golden_calibration(), tmp_path / "missing" / "c.yaml")
    assert not (tmp_path / "missing").exists()


def test_save_calibration_rejects_non_calibration(tmp_path):
    """Y-E02: Calibration 以外を渡す。🔴"""
    with pytest.raises(ValueError):
        save_calibration({"version": 1}, tmp_path / "c.yaml")  # type: ignore[arg-type]


# ================================================================ 2.5 load_calibration 異常系
def _mutate(doc, path, value):
    target = doc
    for k in path[:-1]:
        target = target[k]
    target[path[-1]] = value
    return doc


def _delete(doc, path):
    target = doc
    for k in path[:-1]:
        target = target[k]
    del target[path[-1]]
    return doc


def test_load_calibration_missing_file_raises(tmp_path):
    """L-E01: ファイルがない。🔵"""
    with pytest.raises(FileNotFoundError):
        load_calibration(tmp_path / "none.yaml")


@pytest.mark.parametrize("text", ["", "[]", "- 1\n", "42", "text"], ids=["empty", "empty-list", "list", "int", "str"])
def test_load_calibration_rejects_non_mapping_top_level(tmp_path, text):
    """L-E02: 空ファイル・最上位が mapping でない。🟡"""
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, text))


@pytest.mark.parametrize("text", ["version: [1\n", "a: b: c\n", "key: 'unterminated\n"], ids=["bracket", "nested-colon", "quote"])
def test_load_calibration_wraps_yaml_syntax_error_in_value_error(tmp_path, text):
    """L-E02b: YAML として読めない文字列は yaml.YAMLError ではなく ValueError (パス入りのメッセージ)。🟡"""
    path = _write(tmp_path, text)
    with pytest.raises(ValueError, match="YAML として読めません") as excinfo:
        load_calibration(path)
    assert str(path) in str(excinfo.value)
    assert isinstance(excinfo.value.__cause__, yaml.YAMLError)


@pytest.mark.parametrize(
    ("path", "key"),
    [(("extra",), "extra"), (("metadata", "note"), "note"), (("camera", "k1"), "k1")],
    ids=["top", "metadata", "camera-distortion"],
)
def test_load_calibration_rejects_unknown_keys(valid_doc, tmp_path, path, key):
    """L-E03 / L-E05 / L-E06: 未知キー (歪み係数 k1 も OPEN-1 により拒否)。🟡"""
    with pytest.raises(ValueError, match=key):
        load_calibration(_write(tmp_path, _mutate(valid_doc, path, 1)))


@pytest.mark.parametrize(
    "path",
    [("version",), ("metadata",), ("camera",), ("t_base_cam",)]
    + [("metadata", k) for k in ("source", "camera_name", "base_frame")]
    + [("camera", k) for k in ("fx", "fy", "cx", "cy", "width", "height")],
    ids=lambda p: ".".join(p),
)
def test_load_calibration_rejects_missing_keys(valid_doc, tmp_path, path):
    """L-E04 / L-E05 / L-E06: 欠落キー (キー名入りのメッセージ)。🟡"""
    with pytest.raises(ValueError, match=path[-1]):
        load_calibration(_write(tmp_path, _delete(valid_doc, path)))


@pytest.mark.parametrize(("key", "value"), [("metadata", "sim"), ("camera", [1, 2])])
def test_load_calibration_rejects_non_mapping_sections(valid_doc, tmp_path, key, value):
    """L-E07: metadata / camera が mapping でない。🟡"""
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, _mutate(valid_doc, (key,), value)))


@pytest.mark.parametrize("bad", [2, 0, "1", 1.0, True, None])
def test_load_calibration_rejects_bad_version(valid_doc, tmp_path, bad):
    """L-E08: version が整数 1 でない。🔴"""
    with pytest.raises(ValueError, match="version"):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("version",), bad)))


@pytest.mark.parametrize("bad", ["test", "SIM", "", None, 1])
def test_load_calibration_rejects_bad_source(valid_doc, tmp_path, bad):
    """L-E09: source が不正。🔴"""
    with pytest.raises(ValueError, match="source"):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("metadata", "source"), bad)))


@pytest.mark.parametrize("key", ["camera_name", "base_frame"])
@pytest.mark.parametrize("bad", ["", None, 1, ["cam_top"]])
def test_load_calibration_rejects_bad_names(valid_doc, tmp_path, key, bad):
    """L-E10: camera_name / base_frame が不正。🟡"""
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("metadata", key), bad)))


def test_yaml_scientific_notation_without_dot_is_a_string():
    """前提 (L-E11 / B-07): PyYAML では `1e-3` は文字列、`1.0e-3` は float として読まれる。🔵"""
    assert yaml.safe_load("fx: 1e-3") == {"fx": "1e-3"}
    assert yaml.safe_load("fx: 1.0e-3") == {"fx": 0.001}


@pytest.mark.parametrize("key", ["fx", "fy", "cx", "cy"])
@pytest.mark.parametrize(
    "bad",
    ["1e-3", "415.0", True, float("inf"), float("-inf"), float("nan"), None, [1]],
    ids=["str-1e-3", "str-quoted", "bool", "inf", "-inf", "nan", "null", "list"],
)
def test_load_calibration_rejects_non_finite_or_non_numeric_intrinsics(valid_doc, tmp_path, key, bad):
    """L-E11: fx / fy / cx / cy が有限の実数でない。🟡"""
    with pytest.raises(ValueError, match=key):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("camera", key), bad)))


@pytest.mark.parametrize("key", ["fx", "fy"])
@pytest.mark.parametrize("bad", [0, 0.0, -1, -415.69])
def test_load_calibration_rejects_non_positive_focal_length(valid_doc, tmp_path, key, bad):
    """L-E12: fx / fy が 0 以下。🔵"""
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("camera", key), bad)))


@pytest.mark.parametrize("key", ["width", "height"])
@pytest.mark.parametrize("bad", [640.0, 0, -480, True, "640", None])
def test_load_calibration_rejects_bad_image_size(valid_doc, tmp_path, key, bad):
    """L-E13 / B-08: width / height が正の整数でない (640.0 は拒否)。🟡"""
    with pytest.raises(ValueError, match=key):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("camera", key), bad)))


def _rows(**_):
    return copy.deepcopy(SIM_T_ROWS)


_BAD_SHAPES = {
    "3-rows": SIM_T_ROWS[:3],
    "5-rows": SIM_T_ROWS + [[0.0, 0.0, 0.0, 1.0]],
    "row-3": [SIM_T_ROWS[0][:3]] + SIM_T_ROWS[1:],
    "row-5": [SIM_T_ROWS[0] + [0.0]] + SIM_T_ROWS[1:],
    "flat-16": [v for r in SIM_T_ROWS for v in r],
    "null": None,
    "mapping": {"a": 1},
    "scalar": 1.0,
}


@pytest.mark.parametrize("name", list(_BAD_SHAPES))
def test_load_calibration_rejects_bad_T_shape(valid_doc, tmp_path, name):
    """L-E14: t_base_cam の形が 4x4 でない。🟡"""
    with pytest.raises(ValueError, match="t_base_cam"):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("t_base_cam",), _BAD_SHAPES[name])))


@pytest.mark.parametrize("bad", ["0.45", "1e-3", True, None, float("nan"), float("inf")],
                         ids=["str", "str-1e-3", "bool", "null", "nan", "inf"])  # fmt: skip
def test_load_calibration_rejects_bad_T_element(valid_doc, tmp_path, bad):
    """L-E15: t_base_cam の要素の型が不正。🟡"""
    doc = _mutate(valid_doc, ("t_base_cam",), _rows())
    doc["t_base_cam"][0][3] = bad
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, doc))


def _non_orthogonal(rows):
    rows[0][1] += 1e-3
    return rows


def _scaled(rows):
    return [[2 * v for v in r[:3]] + [r[3]] for r in rows[:3]] + [rows[3]]


def _mirrored(rows):
    for r in rows[:3]:
        r[2] *= -1
    return rows


@pytest.mark.parametrize("make", [_non_orthogonal, _scaled, _mirrored])
def test_load_calibration_rejects_non_rigid_rotation(valid_doc, tmp_path, make):
    """L-E16: t_base_cam の回転が直交でない・スケール・鏡映。🔵"""
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("t_base_cam",), make(_rows()))))


@pytest.mark.parametrize("bottom", [[0, 0, 0, 2], [0, 0, 1e-6, 1]])
def test_load_calibration_rejects_bad_bottom_row(valid_doc, tmp_path, bottom):
    """L-E17 / B-05: t_base_cam の最下行が不正。🔵"""
    rows = _rows()
    rows[3] = bottom
    with pytest.raises(ValueError):
        load_calibration(_write(tmp_path, _mutate(valid_doc, ("t_base_cam",), rows)))


def test_load_calibration_does_not_modify_file(valid_doc, tmp_path):
    """L-E18: 失敗時もファイルを変えない (読み込みは書き込まない)。🟡"""
    p = _write(tmp_path, _mutate(valid_doc, ("camera", "fx"), "1e-3"))
    before = p.read_bytes()
    with pytest.raises(ValueError):
        load_calibration(p)
    assert p.read_bytes() == before


# ================================================================ 3 境界値 (load)
def test_load_calibration_accepts_small_orthogonality_error(valid_doc, tmp_path):
    """B-04: 回転の直交誤差 1e-8 (< 1e-6) は読める。🔵"""
    rows = _rows()
    rows[0][1] += 1e-8
    cal = load_calibration(_write(tmp_path, _mutate(valid_doc, ("t_base_cam",), rows)))
    assert cal.T_base_cam[0, 1] == rows[0][1]  # 【確認内容】: 正規化せず検証済みの値を保持


def test_load_calibration_accepts_bottom_row_within_tolerance(valid_doc, tmp_path):
    """B-05: 最下行 [0,0,0,1+1e-12] は読める (正規化しない)。🟡"""
    rows = _rows()
    rows[3] = [0.0, 0.0, 0.0, 1 + 1e-12]
    cal = load_calibration(_write(tmp_path, _mutate(valid_doc, ("t_base_cam",), rows)))
    assert cal.T_base_cam[3, 3] == 1 + 1e-12


def test_load_calibration_reads_decimal_scientific_float(valid_doc, tmp_path):
    """B-07: `fx: 1.0e-3` は float として読める。🟡"""
    text = yaml.safe_dump(valid_doc, sort_keys=False)
    text = re.sub(r"^  fx: .*$", "  fx: 1.0e-3", text, flags=re.M)
    assert load_calibration(_write(tmp_path, text)).camera.fx == 0.001
