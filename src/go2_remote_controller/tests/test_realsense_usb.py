"""sysfs RealSense detection: presence + USB 2 / USB 3 classification."""

from app.services.realsense_usb import classify_speed, scan_realsense


def _dev(root, name, vendor, product, speed, serial="SN1", version=" 3.20"):
    d = root / name
    d.mkdir()
    (d / "idVendor").write_text(vendor + "\n")
    (d / "product").write_text(product + "\n")
    (d / "speed").write_text(str(speed) + "\n")
    (d / "serial").write_text(serial + "\n")
    (d / "version").write_text(version + "\n")


def test_absent(tmp_path):
    _dev(tmp_path, "1-1", "046d", "USB Receiver", 12)
    r = scan_realsense(str(tmp_path))
    assert r["connected"] is False
    assert r["usb_class"] is None


def test_missing_root():
    r = scan_realsense("/nonexistent/sysfs/root")
    assert r["connected"] is False


def test_usb3(tmp_path):
    _dev(tmp_path, "1-1", "046d", "USB Receiver", 12)
    _dev(tmp_path, "2-3", "8086", "Intel(R) RealSense(TM) Depth Camera 435i", 5000,
         serial="238222076237")
    r = scan_realsense(str(tmp_path))
    assert r["connected"] is True
    assert r["usb_class"] == "USB 3"
    assert r["speed_mbps"] == 5000
    assert r["serial"] == "238222076237"
    assert r["usb_version"] == "3.20"


def test_usb2(tmp_path):
    _dev(tmp_path, "1-4", "8086", "Intel(R) RealSense(TM) Depth Camera 435i", 480,
         version=" 2.10")
    r = scan_realsense(str(tmp_path))
    assert r["connected"] is True
    assert r["usb_class"] == "USB 2"


def test_intel_non_realsense_ignored(tmp_path):
    _dev(tmp_path, "1-1", "8086", "Intel Bluetooth", 12)
    assert scan_realsense(str(tmp_path))["connected"] is False


def test_classify_speed():
    assert classify_speed(None) is None
    assert classify_speed(12) == "USB 1"
    assert classify_speed(480) == "USB 2"
    assert classify_speed(5000) == "USB 3"
    assert classify_speed(10000) == "USB 3"
