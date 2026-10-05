"""PX4 Windows connector with bounded command cleanup and file-backed output.

Keep the existing MCP tool names and arguments. Long WSL jobs should be
launched independently with input closed and output redirected to files.
"""

import locale
import os
from pathlib import Path
import signal
import subprocess
import tempfile


OUTPUT_LIMIT = 120000
MAX_COMMAND_SECONDS = 90


def _read_tail(handle):
    handle.flush()
    handle.seek(0, os.SEEK_END)
    handle.seek(max(0, handle.tell() - OUTPUT_LIMIT))
    return handle.read().decode(locale.getpreferredencoding(False), errors="replace")


def _terminate_tree(process):
    """Attempt tree cleanup without waiting on inherited output pipes."""
    errors = []
    if os.name == "nt":
        try:
            result = subprocess.run(
                ["taskkill.exe", "/PID", str(process.pid), "/T", "/F"],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
            if result.returncode != 0:
                errors.append("taskkill did not confirm descendant cleanup")
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"tree cleanup failed: {type(exc).__name__}")
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except OSError as exc:
            errors.append(f"tree cleanup failed: {type(exc).__name__}")
    if process.poll() is None:
        try:
            process.kill()
            process.wait(timeout=2)
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"parent cleanup failed: {type(exc).__name__}")
    return errors


def _run_command(arguments, cwd=None, timeout_seconds=30):
    """Return within the command deadline plus bounded cleanup allowance."""
    timeout_seconds = max(1, min(int(timeout_seconds), MAX_COMMAND_SECONDS))
    process = None
    try:
        # Descendants can keep pipe handles open after their parent exits.
        # Regular files let us read partial output without draining those pipes.
        with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
            process = subprocess.Popen(
                arguments,
                cwd=cwd,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                start_new_session=os.name != "nt",
            )
            timed_out = False
            cleanup_errors = []
            try:
                process.wait(timeout=timeout_seconds)
            except subprocess.TimeoutExpired:
                timed_out = True
                cleanup_errors = _terminate_tree(process)
            output = _read_tail(stdout)
            error_output = _read_tail(stderr)
            if timed_out:
                error_output += f"\nCommand timed out after {timeout_seconds} seconds."
                if cleanup_errors:
                    error_output += "\n" + "; ".join(cleanup_errors)
            return {
                "exit_code": -2 if timed_out else process.returncode,
                "stdout": output,
                "stderr": error_output,
            }
    except (OSError, ValueError) as exc:
        if process is not None and process.poll() is None:
            _terminate_tree(process)
        return {"exit_code": -3, "stdout": "", "stderr": str(exc)}


def system_info() -> dict:
    """Return basic information about the local Windows machine and drives."""
    return _run_command([
        "powershell.exe", "-NoLogo", "-NoProfile", "-NonInteractive", "-Command",
        "Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,"
        "OSArchitecture; Get-PSDrive -PSProvider FileSystem | "
        "Select-Object Name,Root,Used,Free",
    ], timeout_seconds=30)


def run_powershell(
    command: str, working_directory: str = "F:\\", timeout_seconds: int = 120
) -> dict:
    """Execute PowerShell; timeout is capped at 90 seconds before bounded cleanup."""
    cwd = Path(working_directory)
    if not cwd.is_dir():
        return {
            "exit_code": -1, "stdout": "",
            "stderr": f"Working directory does not exist: {working_directory}",
        }
    result = _run_command([
        "powershell.exe", "-NoLogo", "-NoProfile", "-NonInteractive",
        "-ExecutionPolicy", "Bypass", "-Command", command,
    ], cwd=str(cwd), timeout_seconds=timeout_seconds)
    result["working_directory"] = str(cwd)
    return result


def read_text_file(path: str, max_chars: int = 200000) -> str:
    """Read a local text file from the Windows computer."""
    return Path(path).read_text(encoding="utf-8", errors="replace")[:max_chars]


def write_text_file(path: str, content: str) -> str:
    """Create or replace a local text file on the Windows computer."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content, encoding="utf-8")
    return f"Wrote {len(content)} characters to {target}"


def list_directory(path: str = "F:\\") -> list[str]:
    """List files and directories at a local Windows path."""
    return [str(item) for item in Path(path).iterdir()]


def main():
    from mcp.server.fastmcp import FastMCP

    mcp = FastMCP("PX4 Local PowerShell")
    for function in (
        system_info, run_powershell, read_text_file, write_text_file, list_directory
    ):
        mcp.tool()(function)
    mcp.run()


if __name__ == "__main__":
    main()
