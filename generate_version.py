import subprocess
import datetime
Import("env")


def get_firmware_version():
    try:
        tag = subprocess.check_output(["git", "describe", "--tags", "--abbrev=0"], stderr=subprocess.DEVNULL).decode("ascii").strip()
        commit_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode("ascii").strip()
        return f"{tag}-{commit_hash}"
    except Exception:
        pass

    try:
        commit_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode("ascii").strip()
        return commit_hash
    except Exception:
        pass

    branch_prefix = ""
    try:
        branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]).decode("ascii").strip()
        branch_prefix = branch + "-"
    except Exception:
        pass

    timestamp = datetime.datetime.now().strftime("%y.%m.%d-%H%M")
    return f"{branch_prefix}V{timestamp}"


version = get_firmware_version()
print(f"--- Automated Versioning: Building {version} ---")
env.Append(CPPDEFINES=[("AUTO_VERSION", f'\\"{version}\\"')])
