import datetime
import hashlib
import hmac
import json
import os
import pathlib
import re
import secrets
import struct
import subprocess
import sys
import ctypes
import atexit
import importlib

COMMAND_LINE_TARGETS = []


def get_build_failures():
    return []


try:
    scons_script = importlib.import_module("SCons.Script")
    COMMAND_LINE_TARGETS = list(getattr(scons_script, "COMMAND_LINE_TARGETS", []))
    get_build_failures = getattr(scons_script, "GetBuildFailures", get_build_failures)
except Exception:
    pass

SECRETS_ROOT = pathlib.Path(r"C:\Users\Youp\Documents\GitHub\YoupSpace_Secrets")
SECRET_KEY_PATH = SECRETS_ROOT / "warranty_key.txt"
SERIAL_REGISTRY_PATH = SECRETS_ROOT / "serial_registry.txt"
PENDING_SERIAL_PATH = SECRETS_ROOT / "pending_serial.json"
DEFAULT_OUTPUT_HEADER = pathlib.Path("include") / "build_info.h"
MAGIC_HEADER = 0xA57B09A2
MANUFACTURER_ID = "YOUPSPACE"
PRODUCT_ID = "NANO"
DEFAULT_HARDWARE_MAJOR = 1
DEFAULT_HARDWARE_MINOR = 1

ANSI_RESET = "\033[0m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"
ANSI_CYAN = "\033[36m"
SERIAL_ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"

UPLOAD_STATUS = {
    "requested": False,
    "completed": False,
    "serial_number": None,
    "key_mode": "dummy",
}

SEMANTIC_TAG_PATTERN = re.compile(r"^(?:firmware[-_])?v?(\d+)\.(\d+)\.(\d+)$")


def try_import_platformio_env():
    importer = globals().get("Import")
    if not callable(importer):
        return None

    try:
        importer("env")
        return globals().get("env")
    except Exception:
        return None


def run_git_command(args):
    return subprocess.check_output(args, stderr=subprocess.DEVNULL).decode("ascii").strip()


def parse_semantic_firmware_tag(tag_text):
    if not tag_text:
        return None

    match = SEMANTIC_TAG_PATTERN.fullmatch(tag_text.strip())
    if not match:
        return None

    return {
        "tag": tag_text.strip(),
        "version": [
            int(match.group(1)),
            int(match.group(2)),
            int(match.group(3)),
        ],
    }


def get_required_firmware_tag_info():
    try:
        head_tags = run_git_command(["git", "tag", "--points-at", "HEAD", "--sort=-v:refname"])
    except Exception as error:
        raise RuntimeError("FATAL: Unable to read git tags for the current commit.") from error

    for candidate in head_tags.splitlines():
        parsed = parse_semantic_firmware_tag(candidate)
        if parsed is not None:
            return parsed

    raise RuntimeError(
        "FATAL: The current commit must have a semantic firmware git tag such as 'firmware-v1.1.0' or 'v1.1.0'."
    )


def validate_required_build_values(firmware_tag_info, version, production_date, production_second, hardware_major, hardware_minor, initial_firmware):
    if firmware_tag_info is None or not firmware_tag_info.get("tag"):
        raise RuntimeError("FATAL: Firmware git tag is required.")
    if not version:
        raise RuntimeError("FATAL: Firmware version string is required.")
    if not production_date or len(production_date) != 16:
        raise RuntimeError("FATAL: Production date must be present in HH-MM-DD-MM-YYYY format.")
    if production_second is None or production_second < 0 or production_second > 59:
        raise RuntimeError("FATAL: Production second must be present in range 0..59.")
    if hardware_major is None or hardware_minor is None:
        raise RuntimeError("FATAL: Hardware version is required.")
    if any(part is None for part in initial_firmware) or len(initial_firmware) != 3:
        raise RuntimeError("FATAL: Initial firmware version array is required.")


def get_output_path(cli_args):
    if len(cli_args) > 1:
        return pathlib.Path(cli_args[1])
    return DEFAULT_OUTPUT_HEADER


def get_platformio_option(platformio_env, option_name, fallback=""):
    if platformio_env is None:
        return fallback
    try:
        return platformio_env.GetProjectOption(option_name, fallback)
    except TypeError:
        try:
            return platformio_env.GetProjectOption(option_name)
        except Exception:
            return fallback
    except Exception:
        return fallback


def color_text(color_code, message):
    return f"{color_code}{message}{ANSI_RESET}"


def format_display_serial(serial_number):
    if serial_number is None:
        return "AN-A0A0A0A0A0"

    state = int(serial_number) & 0xFFFFFFFF
    encoded = ["0"] * 10
    has_digit = False
    has_letter = False

    for index in range(10):
        state = (1664525 * state + 1013904223 + index) & 0xFFFFFFFF
        ch = SERIAL_ALPHABET[state % len(SERIAL_ALPHABET)]
        encoded[index] = ch
        if "0" <= ch <= "9":
            has_digit = True
        else:
            has_letter = True

    # Guarantee mixed content to keep serials human-friendly.
    if not has_letter:
        encoded[1] = "A"
    if not has_digit:
        encoded[2] = "7"

    return f"AN-{''.join(encoded)}"


def build_status_suffix(serial_number, key_mode):
    return f" [serial={format_display_serial(serial_number)} key={key_mode}]"


def enable_windows_virtual_terminal():
    if os.name != "nt":
        return

    kernel32 = ctypes.windll.kernel32
    enable_flag = 0x0004
    for std_handle in (-11, -12):
        handle = kernel32.GetStdHandle(std_handle)
        if handle in (0, -1):
            continue

        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)) == 0:
            continue
        kernel32.SetConsoleMode(handle, mode.value | enable_flag)


def parse_uint32(value_text):
    if value_text is None:
        return None
    candidate = str(value_text).strip()
    if not candidate:
        return None

    try:
        value = int(candidate, 0)
    except ValueError as error:
        raise ValueError(f"Invalid serial number '{candidate}'. Expected a uint32 value.") from error

    if value < 0 or value > 0xFFFFFFFF:
        raise ValueError(f"Serial number '{candidate}' is outside the uint32 range.")
    return value


def load_registered_serials():
    if not SECRETS_ROOT.exists():
        raise RuntimeError(f"FATAL: Required secrets folder not found: {SECRETS_ROOT}")

    if not SERIAL_REGISTRY_PATH.exists():
        SERIAL_REGISTRY_PATH.write_text("", encoding="ascii")

    serials = set()
    for line_number, line in enumerate(SERIAL_REGISTRY_PATH.read_text(encoding="ascii").splitlines(), start=1):
        stripped = line.strip()
        if not stripped:
            continue

        try:
            value = int(stripped, 10)
        except ValueError as error:
            raise RuntimeError(f"FATAL: Invalid serial_registry.txt entry on line {line_number}: '{stripped}'") from error

        if value < 0 or value > 0xFFFFFFFF:
            raise RuntimeError(f"FATAL: Serial registry value out of uint32 range on line {line_number}: '{stripped}'")
        serials.add(value)

    return serials


def load_pending_serial_record():
    if not PENDING_SERIAL_PATH.exists():
        return None

    try:
        record = json.loads(PENDING_SERIAL_PATH.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"FATAL: Invalid pending serial file: {PENDING_SERIAL_PATH}") from error

    if not isinstance(record, dict):
        raise RuntimeError(f"FATAL: Invalid pending serial file structure: {PENDING_SERIAL_PATH}")
    return record


def save_pending_serial_record(record):
    PENDING_SERIAL_PATH.write_text(json.dumps(record, indent=2, sort_keys=True), encoding="utf-8")


def build_pending_serial_context(version, hardware_major, hardware_minor, initial_firmware, key_mode):
    return {
        "version": version,
        "hardware_major": hardware_major,
        "hardware_minor": hardware_minor,
        "initial_firmware": list(initial_firmware),
        "key_mode": key_mode,
    }


def assign_unique_serial_number(version, hardware_major, hardware_minor, initial_firmware, key_mode):
    registered_serials = load_registered_serials()
    pending_context = build_pending_serial_context(
        version,
        hardware_major,
        hardware_minor,
        initial_firmware,
        key_mode,
    )
    existing_record = load_pending_serial_record()
    if existing_record is not None:
        existing_serial = parse_uint32(existing_record.get("serial_number"))
        if existing_serial is None:
            raise RuntimeError("FATAL: Pending serial file is missing a valid serial_number.")
        if existing_serial in registered_serials:
            raise RuntimeError(
                f"FATAL: Pending serial number {existing_serial} is already present in serial_registry.txt."
            )

        existing_context = {key: existing_record.get(key) for key in pending_context}
        if existing_context == pending_context:
            print(color_text(ANSI_CYAN, f"INFO: Reusing pending AstroNav serial number: {format_display_serial(existing_serial)}"))
            production_second = int(existing_record.get("production_second", 0))
            return existing_record["production_date"], production_second, existing_serial

    serial_number = secrets.randbits(32)
    if serial_number in registered_serials:
        raise RuntimeError(
            f"FATAL: Generated serial number collision for {serial_number}. Aborting build to prevent duplicates."
        )

    pending_record = dict(pending_context)
    now = datetime.datetime.now()
    pending_record["production_date"] = now.strftime("%H-%M-%d-%m-%Y")
    pending_record["production_second"] = now.second
    pending_record["serial_number"] = serial_number
    save_pending_serial_record(pending_record)

    print(color_text(ANSI_CYAN, f"INFO: Assigned pending AstroNav serial number: {format_display_serial(serial_number)}"))
    return pending_record["production_date"], pending_record["production_second"], serial_number


def commit_uploaded_serial_number():
    registered_serials = load_registered_serials()
    pending_record = load_pending_serial_record()
    if pending_record is None:
        print(color_text(ANSI_YELLOW, "WARNING: No pending serial record found after upload. Nothing was written to serial_registry.txt."))
        return

    serial_number = parse_uint32(pending_record.get("serial_number"))
    if serial_number is None:
        raise RuntimeError("FATAL: Pending serial file is missing a valid serial_number.")
    if serial_number in registered_serials:
        raise RuntimeError(f"FATAL: Uploaded serial number {serial_number} is already present in serial_registry.txt.")

    with SERIAL_REGISTRY_PATH.open("a", encoding="ascii", newline="\n") as registry_file:
        registry_file.write(f"{serial_number}\n")

    PENDING_SERIAL_PATH.unlink(missing_ok=True)
    key_mode = str(pending_record.get("key_mode") or "dummy")
    print(color_text(ANSI_GREEN, f"SUCCESS: Uploaded AstroNav serial number recorded.{build_status_suffix(serial_number, key_mode)}"))


def get_hardware_version(platformio_env):
    major = parse_uint32(os.getenv("ASTRONAV_HW_MAJOR"))
    minor = parse_uint32(os.getenv("ASTRONAV_HW_MINOR"))

    if major is None:
        major = parse_uint32(get_platformio_option(platformio_env, "custom_hardware_major", ""))
    if minor is None:
        minor = parse_uint32(get_platformio_option(platformio_env, "custom_hardware_minor", ""))

    if major is None:
        major = DEFAULT_HARDWARE_MAJOR
    if minor is None:
        minor = DEFAULT_HARDWARE_MINOR

    if major > 0xFF or minor > 0xFF:
        raise ValueError("Hardware major/minor values must fit in uint8_t.")

    return major, minor


def get_secret_key_bytes():
    if SECRET_KEY_PATH.exists():
        print(color_text(ANSI_GREEN, "SUCCESS: YoupSpace secret key found. Generating official warranty signature."))
        key_text = SECRET_KEY_PATH.read_text(encoding="ascii").strip()
        if not re.fullmatch(r"[0-9A-Fa-f]{64}", key_text):
            raise RuntimeError("FATAL: warranty_key.txt must contain exactly 64 hexadecimal characters.")
        return bytes.fromhex(key_text), True

    print(color_text(ANSI_YELLOW, "WARNING: Secret key not found. Generating dummy signature for open source build."))
    return b"OPEN_SOURCE_DUMMY_WARRANTY_KEY", False


def preview_key_mode():
    if not SECRET_KEY_PATH.exists():
        return "dummy"

    key_text = SECRET_KEY_PATH.read_text(encoding="ascii").strip()
    if not re.fullmatch(r"[0-9A-Fa-f]{64}", key_text):
        raise RuntimeError("FATAL: warranty_key.txt must contain exactly 64 hexadecimal characters.")
    return "official"


def build_otp_payload(hardware_major, hardware_minor, production_date, production_second, serial_number, initial_firmware):
    reserved = bytearray(16)
    reserved[0] = production_second & 0xFF

    return struct.pack(
        "<I10s5sBB17sI3BI16s",
        MAGIC_HEADER,
        MANUFACTURER_ID.encode("ascii") + b"\0",
        PRODUCT_ID.encode("ascii") + b"\0",
        hardware_major,
        hardware_minor,
        production_date.encode("ascii") + b"\0",
        serial_number,
        initial_firmware[0],
        initial_firmware[1],
        initial_firmware[2],
        0,
        bytes(reserved),
    )


def calculate_warranty_signature(secret_key, payload_without_signature):
    digest = hmac.new(secret_key, payload_without_signature, hashlib.sha256).digest()
    return int.from_bytes(digest[:4], byteorder="big", signed=False)


def c_string_literal(value):
    return value.replace("\\", "\\\\").replace('"', '\\"')


def generate_header(output_path, version, hardware_major, hardware_minor, production_date, production_second, serial_number, initial_firmware, warranty_signature, official_signature):
    header_text = f'''#pragma once

#define AUTO_VERSION "{c_string_literal(version)}"
#define ASTRONAV_BUILD_PRODUCTION_DATE "{production_date}"
#define ASTRONAV_BUILD_PRODUCTION_SECOND {production_second}
#define ASTRONAV_BUILD_SERIAL_NUMBER UINT32_C({serial_number})
#define ASTRONAV_BUILD_WARRANTY_SIGNATURE UINT32_C(0x{warranty_signature:08X})
#define ASTRONAV_BUILD_HARDWARE_MAJOR {hardware_major}
#define ASTRONAV_BUILD_HARDWARE_MINOR {hardware_minor}
#define ASTRONAV_BUILD_WARRANTY_SIGNATURE_OFFICIAL {1 if official_signature else 0}
#define ASTRONAV_BUILD_INITIAL_FIRMWARE_MAJOR {initial_firmware[0]}
#define ASTRONAV_BUILD_INITIAL_FIRMWARE_MINOR {initial_firmware[1]}
#define ASTRONAV_BUILD_INITIAL_FIRMWARE_PATCH {initial_firmware[2]}
'''
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(header_text, encoding="ascii")


def register_platformio_upload_hook(platformio_env):
    if platformio_env is None:
        return

    UPLOAD_STATUS["requested"] = any(target == "upload" for target in COMMAND_LINE_TARGETS)

    def after_upload(source, target, env):
        del source
        del target
        del env
        UPLOAD_STATUS["completed"] = True
        suffix = build_status_suffix(UPLOAD_STATUS["serial_number"], UPLOAD_STATUS["key_mode"])
        print(color_text(ANSI_GREEN, f"SUCCESS: AstroNav upload completed.{suffix}"))
        commit_uploaded_serial_number()

    platformio_env.AddPostAction("upload", after_upload)


def report_upload_failure_if_needed():
    if not UPLOAD_STATUS["requested"] or UPLOAD_STATUS["completed"]:
        return

    failures = get_build_failures()
    if not failures:
        return

    serial_number = UPLOAD_STATUS["serial_number"]
    key_mode = UPLOAD_STATUS["key_mode"]
    suffix = build_status_suffix(serial_number, key_mode) if serial_number is not None else ""
    print(color_text(ANSI_RED, f"FAILED: AstroNav upload failed.{suffix}"))


def main(cli_args):
    enable_windows_virtual_terminal()
    platformio_env = try_import_platformio_env()
    register_platformio_upload_hook(platformio_env)
    output_path = get_output_path(cli_args)
    firmware_tag_info = get_required_firmware_tag_info()
    version = firmware_tag_info["tag"]
    hardware_major, hardware_minor = get_hardware_version(platformio_env)
    initial_firmware = firmware_tag_info["version"]
    now = datetime.datetime.now()
    production_date = now.strftime("%H-%M-%d-%m-%Y")
    production_second = now.second
    validate_required_build_values(
        firmware_tag_info,
        version,
        production_date,
        production_second,
        hardware_major,
        hardware_minor,
        initial_firmware,
    )
    key_mode = preview_key_mode()
    production_date, production_second, serial_number = assign_unique_serial_number(version, hardware_major, hardware_minor, initial_firmware, key_mode)
    UPLOAD_STATUS["serial_number"] = serial_number
    UPLOAD_STATUS["key_mode"] = key_mode
    secret_key, official_signature = get_secret_key_bytes()

    payload_without_signature = build_otp_payload(
        hardware_major,
        hardware_minor,
        production_date,
        production_second,
        serial_number,
        initial_firmware,
    )
    warranty_signature = calculate_warranty_signature(secret_key, payload_without_signature)
    generate_header(
        output_path,
        version,
        hardware_major,
        hardware_minor,
        production_date,
        production_second,
        serial_number,
        initial_firmware,
        warranty_signature,
        official_signature,
    )

    print(color_text(ANSI_CYAN, f"--- Automated Versioning: Building {version} ---"))
    print(color_text(ANSI_CYAN, f"--- Generated build info header: {output_path} ---"))

    if platformio_env is not None:
        platformio_env.Append(CPPDEFINES=[("AUTO_VERSION", f'\\"{version}\\"')])


if __name__ == "__main__":
    try:
        main(sys.argv)
    except Exception as error:
        print(color_text(ANSI_RED, str(error)))
        raise SystemExit(1) from error
else:
    main([sys.argv[0]])


atexit.register(report_upload_failure_if_needed)
