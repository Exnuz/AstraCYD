import argparse
import hashlib
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from serial.tools.list_ports import comports

ROOT = Path(__file__).resolve().parent
FLASH_SIZE = 4 * 1024 * 1024
IMAGES = (
    ("0x1000", "bootloader.bin"),
    ("0x8000", "partitions.bin"),
    ("0xe000", "boot_app0.bin"),
    ("0x10000", "firmware.bin"),
)
USB_UARTS = {(0x1A86, 0x7523), (0x1A86, 0x55D4), (0x10C4, 0xEA60)}


def run_esptool(port, *args, capture=False):
    command = [
        sys.executable, "-m", "esptool", "--chip", "esp32",
        "--port", port, "--baud", "115200", *map(str, args),
    ]
    if not capture:
        subprocess.run(command, check=True)
        return ""
    result = subprocess.run(command, capture_output=True, text=True)
    print(result.stdout, end="", flush=True)
    print(result.stderr, end="", file=sys.stderr, flush=True)
    result.check_returncode()
    return result.stdout


def usb_port(requested):
    ports = {port.device.upper(): port for port in comports()}
    if requested:
        selected = ports.get(requested.upper())
        if selected and selected.vid is not None:
            return selected.device
        raise ValueError(f"USB serial port {requested} is unavailable.")
    candidates = [port.device for port in ports.values() if (port.vid, port.pid) in USB_UARTS]
    if len(candidates) != 1:
        raise ValueError("Specify a USB serial port with --port COMx.")
    return candidates[0]


def identify(port):
    info = run_esptool(port, "flash_id", capture=True)
    mac = re.search(r"MAC: ((?:[0-9a-f]{2}:){5}[0-9a-f]{2})", info, re.IGNORECASE)
    if not mac or "Detected flash size: 4MB" not in info:
        raise ValueError("Expected a classic ESP32 with 4 MB flash.")
    return mac.group(1).replace(":", "").lower()


def backup(port, mac, directory):
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    destination = directory / f"AstraCYD-original-{mac}-{stamp}.bin"
    partial = destination.with_suffix(".partial")
    run_esptool(port, "read_flash", "0x0", "ALL", partial)
    if partial.stat().st_size != FLASH_SIZE:
        raise ValueError(f"Incomplete flash image: {partial}")
    digest = hashlib.sha256(partial.read_bytes()).hexdigest()
    partial.rename(destination)
    destination.with_suffix(".bin.sha256").write_text(
        f"{digest}  {destination.name}\n", encoding="ascii"
    )
    print(f"Backup: {destination}\nSHA-256: {digest}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="AstraCYD USB installation and full flash backup")
    parser.add_argument("--port", help="Windows USB serial port, e.g. COM14")
    parser.add_argument("--backup-dir", type=Path, default=ROOT / "backups")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--probe", action="store_true")
    modes.add_argument("--backup-only", action="store_true")
    modes.add_argument("--install", action="store_true",
                       help="Install on a confirmed ILI9341 + XPT2046 CYD")
    args = parser.parse_args()

    port = usb_port(args.port)
    if args.probe:
        identify(port)
        return

    files = []
    if args.install:
        for address, filename in IMAGES:
            path = ROOT / "firmware" / filename
            if not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f"Missing firmware image: {path}")
            files.extend((address, path))

    mac = identify(port)
    backup(port, mac, args.backup_dir.resolve())
    if args.install:
        if identify(port) != mac:
            raise ValueError("USB device changed after backup.")
        run_esptool(port, "write_flash", "--flash_mode", "dio", "--flash_freq", "40m",
                    "--flash_size", "4MB", *files)
        print("Installation complete. esptool verified the written images.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(f"AstraCYD: {error}")
