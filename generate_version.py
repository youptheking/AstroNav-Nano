import datetime
import csv
import hashlib
import hmac
import json
import os
import pathlib
import re
import secrets
import base64
import struct
import subprocess
import sys
import ctypes
import atexit
import importlib
import time

COMMAND_LINE_TARGETS = []


def get_build_failures():
    return []


try:
    scons_script = importlib.import_module("SCons.Script")
    COMMAND_LINE_TARGETS = list(getattr(scons_script, "COMMAND_LINE_TARGETS", []))
    get_build_failures = getattr(scons_script, "GetBuildFailures", get_build_failures)
except Exception:
    pass

SECRETS_ROOT = pathlib.Path.home() / "Documents" / "GitHub" / "YoupSpace_Secrets"
SECRET_KEY_PATH = SECRETS_ROOT / "warranty_key.txt"
SERIAL_REGISTRY_CSV_PATH = SECRETS_ROOT / "serial_registry.csv"
LEGACY_SERIAL_REGISTRY_PATH = SECRETS_ROOT / "serial_registry.txt"
SERIAL_REGISTRY_PATH = SERIAL_REGISTRY_CSV_PATH
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
TRUE_ENV_VALUES = {"1", "true", "yes", "y", "on"}

UPLOAD_STATUS = {
    "requested": False,
    "serial_number": None,
    "key_mode": "dummy",
    "track_serial": False,
    "upload_port": "",
}

SEMANTIC_TAG_PATTERN = re.compile(r"^(?:firmware[-_])?v?(\d+)\.(\d+)\.(\d+)$")
SEMANTIC_DESCRIBE_PATTERN = re.compile(
    r"^(?P<tag>(?:firmware[-_])?v?\d+\.\d+\.\d+)-(?P<ahead>\d+)-g(?P<sha>[0-9a-fA-F]+)$"
)
DISPLAY_SERIAL_PATTERN = re.compile(r"^(?:AN-)?[0-9A-Z]{10}$")
OTP_SERIAL_LINE_PATTERN = re.compile(r"\[OTP\]\s+serial_number=([0-9A-Z-]+)", re.IGNORECASE)


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


def build_semantic_commit_version(version_parts, ahead_count, commit_sha):
    return f"V{version_parts[0]}.{version_parts[1]}.{version_parts[2]}_{ahead_count}_{commit_sha.lower()}"


def parse_semantic_describe_output(describe_text):
    if not describe_text:
        return None

    match = SEMANTIC_DESCRIBE_PATTERN.fullmatch(describe_text.strip())
    if not match:
        return None

    base_info = parse_semantic_firmware_tag(match.group("tag"))
    if base_info is None:
        return None

    ahead_count = int(match.group("ahead"))
    commit_sha = match.group("sha").lower()
    return {
        "tag": build_semantic_commit_version(base_info["version"], ahead_count, commit_sha),
        "version": base_info["version"],
        "base_tag": base_info["tag"],
        "ahead_count": ahead_count,
        "commit_sha": commit_sha,
    }


def get_required_firmware_tag_info():
    try:
        head_short_sha = run_git_command(["git", "rev-parse", "--short=7", "HEAD"]).lower()
    except Exception as error:
        raise RuntimeError("FATAL: Unable to read git commit hash for HEAD.") from error

    try:
        head_tags = run_git_command(["git", "tag", "--points-at", "HEAD", "--sort=-v:refname"])
    except Exception as error:
        raise RuntimeError("FATAL: Unable to read git tags for the current commit.") from error

    for candidate in head_tags.splitlines():
        parsed = parse_semantic_firmware_tag(candidate)
        if parsed is not None:
            parsed["base_tag"] = parsed["tag"]
            parsed["ahead_count"] = 0
            parsed["commit_sha"] = head_short_sha
            parsed["tag"] = build_semantic_commit_version(parsed["version"], 0, head_short_sha)
            return parsed

    if parse_bool_env("ASTRONAV_REQUIRE_HEAD_TAG"):
        raise RuntimeError(
            "FATAL: The current commit must have a semantic firmware git tag such as 'firmware-v1.1.0' or 'v1.1.0'."
        )

    try:
        describe_text = run_git_command(
            [
                "git",
                "describe",
                "--tags",
                "--long",
                "--match",
                "firmware-v[0-9]*.[0-9]*.[0-9]*",
                "--match",
                "v[0-9]*.[0-9]*.[0-9]*",
                "HEAD",
            ]
        )
    except Exception as error:
        raise RuntimeError(
            "FATAL: No semantic tag found on HEAD and unable to derive one from commit history. "
            "Tag the commit (for example 'firmware-v1.1.0') or ensure a prior semantic tag exists."
        ) from error

    described = parse_semantic_describe_output(describe_text)
    if described is None:
        raise RuntimeError(
            "FATAL: No usable semantic firmware tag was found. "
            "Expected a tag like 'firmware-v1.1.0' or 'v1.1.0' on HEAD or in reachable history."
        )

    print(
        color_text(
            ANSI_YELLOW,
            f"INFO: HEAD is {described['ahead_count']} commit(s) ahead of semantic tag {described['base_tag']}. "
            f"Using {described['tag']}.",
        )
    )

    return described


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


def resolve_upload_port(platformio_env):
    def normalize_upload_port_value(value_text):
        value = str(value_text).strip().strip('"').strip("'")
        if not value:
            return ""

        upper_value = value.upper()
        if upper_value.startswith("COM") or value.startswith("/dev/"):
            return value

        try:
            decoded = base64.b64decode(value, validate=True).decode("ascii", errors="ignore").strip()
        except Exception:
            return value

        decoded_upper = decoded.upper()
        if decoded_upper.startswith("COM") or decoded.startswith("/dev/"):
            return decoded
        return value

    try:
        scons_arguments = getattr(scons_script, "ARGUMENTS", {})
    except Exception:
        scons_arguments = {}

    if isinstance(scons_arguments, dict):
        for key in ("UPLOAD_PORT", "upload_port"):
            value = normalize_upload_port_value(scons_arguments.get(key, ""))
            if value:
                return value

    for index, token in enumerate(COMMAND_LINE_TARGETS):
        if token.startswith("--upload-port="):
            value = normalize_upload_port_value(token.split("=", 1)[1])
            if value:
                return value

        if token in ("--upload-port", "-P") and index + 1 < len(COMMAND_LINE_TARGETS):
            value = normalize_upload_port_value(COMMAND_LINE_TARGETS[index + 1])
            if value:
                return value

    env_upload_port = normalize_upload_port_value(os.getenv("UPLOAD_PORT", ""))
    if env_upload_port:
        return env_upload_port

    env_upload_port = normalize_upload_port_value(os.getenv("ASTRONAV_UPLOAD_PORT", ""))
    if env_upload_port:
        return env_upload_port

    option_upload_port = normalize_upload_port_value(get_platformio_option(platformio_env, "upload_port", ""))
    if option_upload_port:
        return option_upload_port

    if platformio_env is not None:
        try:
            resolved = normalize_upload_port_value(platformio_env.subst("$UPLOAD_PORT"))
            if resolved and resolved != "$UPLOAD_PORT":
                return resolved
        except Exception:
            pass

        try:
            resolved = normalize_upload_port_value(platformio_env.get("UPLOAD_PORT", ""))
            if resolved:
                return resolved
        except Exception:
            pass

    return ""


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


def parse_bool_env(var_name):
    value = os.getenv(var_name)
    if value is None:
        return False
    return value.strip().lower() in TRUE_ENV_VALUES


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


def normalize_display_serial_text(value_text):
    if value_text is None:
        return None

    candidate = str(value_text).strip().upper()
    if not candidate:
        return None
    if not DISPLAY_SERIAL_PATTERN.fullmatch(candidate):
        return None
    if candidate.startswith("AN-"):
        return candidate
    return f"AN-{candidate}"


def ensure_serial_registry_csv_exists():
    if not SECRETS_ROOT.exists():
        print(color_text(ANSI_YELLOW, f"WARNING: Serial registry folder not found. Skipping serial registry checks for this build: {SECRETS_ROOT}"))
        return False

    if SERIAL_REGISTRY_CSV_PATH.exists():
        return True

    print(color_text(ANSI_YELLOW, f"WARNING: Serial registry CSV not found. Skipping serial registry checks for this build: {SERIAL_REGISTRY_CSV_PATH}"))
    return False


def load_registered_serials():
    if not ensure_serial_registry_csv_exists():
        return {
            "numeric": set(),
            "display": set(),
        }

    serials = {
        "numeric": set(),
        "display": set(),
    }
    with SERIAL_REGISTRY_PATH.open("r", encoding="ascii", newline="") as registry_file:
        reader = csv.reader(registry_file)
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue

            stripped = str(row[0]).strip()
            if not stripped:
                continue

            if line_number == 1 and stripped.lower() in ("serial", "serial_number", "serialnumber"):
                continue

            parsed_display = normalize_display_serial_text(stripped)
            if parsed_display is not None:
                serials["display"].add(parsed_display)
                continue

            try:
                value = int(stripped, 10)
            except ValueError as error:
                raise RuntimeError(
                    f"FATAL: Invalid {SERIAL_REGISTRY_PATH.name} entry on line {line_number}: '{stripped}'. "
                    "Expected decimal uint32 or AN-XXXXXXXXXX format."
                ) from error

            if value < 0 or value > 0xFFFFFFFF:
                raise RuntimeError(f"FATAL: Serial registry value out of uint32 range on line {line_number}: '{stripped}'")
            serials["numeric"].add(value)

    return serials


def serial_exists_in_registry(serial_number, registered_serials):
    display_serial = format_display_serial(serial_number)
    return (
        serial_number in registered_serials["numeric"]
        or display_serial in registered_serials["display"]
    )


def append_display_serial_if_missing(display_serial):
    normalized = normalize_display_serial_text(display_serial)
    if normalized is None:
        raise RuntimeError(f"FATAL: Invalid display serial format: '{display_serial}'")

    registered_serials = load_registered_serials()
    if normalized in registered_serials["display"]:
        print(color_text(ANSI_YELLOW, f"INFO: Serial already present in {SERIAL_REGISTRY_PATH.name}. Continuing without adding: {normalized}"))
        return False

    with SERIAL_REGISTRY_PATH.open("a", encoding="ascii", newline="") as registry_file:
        writer = csv.writer(registry_file)
        writer.writerow([normalized])

    print(color_text(ANSI_GREEN, f"SUCCESS: Uploaded AstroNav serial number recorded. [serial={normalized}]"))
    return True


def clear_pending_if_matches_display_serial(display_serial):
    pending_record = load_pending_serial_record()
    if pending_record is None:
        return

    pending_serial = parse_uint32(pending_record.get("serial_number"))
    if pending_serial is None:
        return

    if format_display_serial(pending_serial) == display_serial:
        PENDING_SERIAL_PATH.unlink(missing_ok=True)


def read_otp_display_serial_from_device(upload_port):
    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except Exception:
        print(color_text(ANSI_YELLOW, "WARNING: pyserial not available in PlatformIO Python environment. OTP serial readback skipped."))
        return None

    def query_port(port_name):
        try:
            with serial.Serial(port_name, 115200, timeout=0.25, write_timeout=1) as port:
                port.reset_input_buffer()
                port.write(b"\nDUMP_OTP\n")

                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    raw_line = port.readline()
                    if not raw_line:
                        continue

                    text_line = raw_line.decode("utf-8", errors="ignore").strip()
                    if not text_line:
                        continue

                    match = OTP_SERIAL_LINE_PATTERN.search(text_line)
                    if not match:
                        continue

                    parsed = normalize_display_serial_text(match.group(1))
                    if parsed is not None:
                        return parsed
        except Exception:
            return None

        return None

    preferred_port = upload_port.strip()
    if not preferred_port:
        print(color_text(ANSI_YELLOW, "WARNING: Upload port is unknown. Scanning available serial ports for OTP readback."))

    discovery_deadline = time.monotonic() + 12.0
    while time.monotonic() < discovery_deadline:
        candidate_ports = []
        if preferred_port:
            candidate_ports.append(preferred_port)

        try:
            for port_info in list_ports.comports():
                device_name = str(getattr(port_info, "device", "")).strip()
                if device_name and device_name not in candidate_ports:
                    candidate_ports.append(device_name)
        except Exception:
            pass

        for candidate in candidate_ports:
            parsed = query_port(candidate)
            if parsed is not None:
                return parsed

        time.sleep(0.4)

    print(color_text(ANSI_YELLOW, "WARNING: OTP serial line not received from device. Readback skipped."))
    return None


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


def has_reusable_pending_serial(version, hardware_major, hardware_minor, initial_firmware, key_mode):
    pending_context = build_pending_serial_context(
        version,
        hardware_major,
        hardware_minor,
        initial_firmware,
        key_mode,
    )
    existing_record = load_pending_serial_record()
    if existing_record is None:
        return False

    existing_serial = parse_uint32(existing_record.get("serial_number"))
    if existing_serial is None:
        raise RuntimeError("FATAL: Pending serial file is missing a valid serial_number.")

    # Ignore stale pending records that were already committed.
    if serial_exists_in_registry(existing_serial, load_registered_serials()):
        return False

    existing_context = {key: existing_record.get(key) for key in pending_context}
    return existing_context == pending_context


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
        if serial_exists_in_registry(existing_serial, registered_serials):
            PENDING_SERIAL_PATH.unlink(missing_ok=True)
            existing_record = None
        else:
            existing_context = {key: existing_record.get(key) for key in pending_context}
            if existing_context == pending_context:
                print(color_text(ANSI_CYAN, f"INFO: Reusing pending AstroNav serial number: {format_display_serial(existing_serial)}"))
                production_second = int(existing_record.get("production_second", 0))
                return existing_record["production_date"], production_second, existing_serial

    serial_number = secrets.randbits(32)
    if serial_exists_in_registry(serial_number, registered_serials):
        raise RuntimeError(
            f"FATAL: Generated serial number collision for {format_display_serial(serial_number)}. Aborting build to prevent duplicates."
        )

    pending_record = dict(pending_context)
    now = datetime.datetime.now()
    pending_record["production_date"] = now.strftime("%H-%M-%d-%m-%Y")
    pending_record["production_second"] = now.second
    pending_record["serial_number"] = serial_number
    save_pending_serial_record(pending_record)

    print(color_text(ANSI_CYAN, f"INFO: Assigned pending AstroNav serial number: {format_display_serial(serial_number)}"))
    return pending_record["production_date"], pending_record["production_second"], serial_number


def resolve_serial_number_for_build(should_assign_serial, version, hardware_major, hardware_minor, initial_firmware, key_mode):
    if should_assign_serial:
        return assign_unique_serial_number(version, hardware_major, hardware_minor, initial_firmware, key_mode)

    # Build-only runs should never consume manufacturing serial numbers.
    now = datetime.datetime.now()
    print(color_text(ANSI_YELLOW, "INFO: Build-only run detected. Not reserving a manufacturing serial number."))
    return now.strftime("%H-%M-%d-%m-%Y"), now.second, 0


def commit_uploaded_serial_number():
    registered_serials = load_registered_serials()
    pending_record = load_pending_serial_record()
    if pending_record is None:
        print(color_text(ANSI_YELLOW, f"WARNING: No pending serial record found after upload. Nothing was written to {SERIAL_REGISTRY_PATH.name}."))
        return

    serial_number = parse_uint32(pending_record.get("serial_number"))
    if serial_number is None:
        raise RuntimeError("FATAL: Pending serial file is missing a valid serial_number.")
    if serial_exists_in_registry(serial_number, registered_serials):
        PENDING_SERIAL_PATH.unlink(missing_ok=True)
        print(color_text(ANSI_YELLOW, f"INFO: Serial already present in {SERIAL_REGISTRY_PATH.name}. Continuing without adding: {format_display_serial(serial_number)}"))
        return

    display_serial = format_display_serial(serial_number)
    with SERIAL_REGISTRY_PATH.open("a", encoding="ascii", newline="") as registry_file:
        writer = csv.writer(registry_file)
        writer.writerow([display_serial])

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


def generate_header(output_path, version, hardware_major, hardware_minor, production_date, production_second, serial_number, initial_firmware, firmware_ahead_count, firmware_commit_sha, warranty_signature, official_signature):
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
#define ASTRONAV_BUILD_FIRMWARE_AHEAD_COUNT {firmware_ahead_count}
#define ASTRONAV_BUILD_FIRMWARE_COMMIT_SHA "{c_string_literal(str(firmware_commit_sha).lower())}"
'''
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(header_text, encoding="ascii")


def register_platformio_upload_hook(platformio_env):
    if platformio_env is None:
        return

    UPLOAD_STATUS["requested"] = any(target == "upload" for target in COMMAND_LINE_TARGETS)
    UPLOAD_STATUS["upload_port"] = resolve_upload_port(platformio_env)


def finalize_upload_status():
    if not UPLOAD_STATUS["requested"]:
        return

    failures = get_build_failures()
    if failures:
        serial_number = UPLOAD_STATUS["serial_number"]
        key_mode = UPLOAD_STATUS["key_mode"]
        suffix = build_status_suffix(serial_number, key_mode) if serial_number is not None else ""
        print(color_text(ANSI_RED, f"FAILED: AstroNav upload failed.{suffix}"))
        return

    otp_display_serial = read_otp_display_serial_from_device(UPLOAD_STATUS["upload_port"])
    if otp_display_serial is not None:
        print(color_text(ANSI_CYAN, f"INFO: OTP readback serial: {otp_display_serial}"))
        append_display_serial_if_missing(otp_display_serial)
        clear_pending_if_matches_display_serial(otp_display_serial)
        print(color_text(ANSI_GREEN, "SUCCESS: AstroNav upload completed."))
        return

    if UPLOAD_STATUS["track_serial"]:
        suffix = build_status_suffix(UPLOAD_STATUS["serial_number"], UPLOAD_STATUS["key_mode"])
        print(color_text(ANSI_YELLOW, f"WARNING: OTP readback unavailable. Falling back to pending serial commit.{suffix}"))
        commit_uploaded_serial_number()
    else:
        print(color_text(ANSI_GREEN, "SUCCESS: AstroNav upload completed."))
        print(color_text(ANSI_YELLOW, "INFO: Service upload mode and OTP readback unavailable. Serial list unchanged."))


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

    if UPLOAD_STATUS["requested"]:
        is_new_board_armed = parse_bool_env("ASTRONAV_NEW_BOARD")
        has_pending = has_reusable_pending_serial(
            version,
            hardware_major,
            hardware_minor,
            initial_firmware,
            key_mode,
        )

        UPLOAD_STATUS["track_serial"] = is_new_board_armed or has_pending

        if is_new_board_armed:
            print(color_text(ANSI_CYAN, "INFO: New-board arming detected (ASTRONAV_NEW_BOARD=1)."))
        elif has_pending:
            print(color_text(ANSI_CYAN, "INFO: Reusing existing pending serial for upload retry."))
        else:
            print(color_text(ANSI_YELLOW, "INFO: Existing-firmware upload detected. Serial list unchanged. Set ASTRONAV_NEW_BOARD=1 only for first-time OTP provisioning."))

    production_date, production_second, serial_number = resolve_serial_number_for_build(
        UPLOAD_STATUS["track_serial"],
        version,
        hardware_major,
        hardware_minor,
        initial_firmware,
        key_mode,
    )
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
        int(firmware_tag_info.get("ahead_count", 0)),
        str(firmware_tag_info.get("commit_sha", "0000000")),
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


atexit.register(finalize_upload_status)
