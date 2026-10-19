import so101_vlm


def test_package_importable_with_version():
    assert isinstance(so101_vlm.__version__, str)
    assert so101_vlm.__version__
