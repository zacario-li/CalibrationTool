# AGENTS.md

## Cursor Cloud specific instructions

### Product overview
CalibrationTool is a standalone **wxPython desktop GUI** for camera calibration (mono, stereo, hand-eye) and stereo disparity/depth estimation. Entry point: `python3 main.py`. No backend services, databases, or containers needed.

### Running the application
- Requires a display server. On Cloud VMs, use the VNC display (`DISPLAY=:1`) so the GUI is visible in the Desktop pane.
- Launch: `DISPLAY=:1 python3 main.py`
- GTK theme warnings (`Gtk-WARNING`, `Gtk-CRITICAL`) at startup are cosmetic and do not affect functionality.

### System dependencies (already installed in snapshot)
- GTK3 dev libraries (`libgtk-3-dev`, `libwebkit2gtk-4.1-dev`, etc.) are required for wxPython.
- `python3-dev` headers are required for wxPython compilation (though prebuilt wheels are preferred).
- wxPython prebuilt wheels: install from `https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-24.04` to avoid long compile times.

### Installing dependencies
```
pip3 install -f https://extras.wxpython.org/wxPython4/extras/linux/gtk3/ubuntu-24.04 -r requirements.txt
```

### Linting
No project-specific linter config exists. Use `flake8` with relaxed line length:
```
python3 -m flake8 --max-line-length=150 main.py utils/ ui/
```

### Testing
- `test.py` contains manual integration tests for storage and calibration utilities. These require specific calibration image/param files not included in the repo, so only the `test()` function (SQLite storage test) runs in CI-like environments.
- Run the storage test: `python3 -c "from test import test; test()"`
- No formal test framework (pytest, unittest) is configured.

### Key gotchas
- wxPython has **no prebuilt PyPI wheel** for Linux; always use the extras URL above or builds take 10+ minutes.
- The codebase uses `import *` from `utils.ophelper` extensively, which causes flake8 F403/F405 warnings — this is existing code style, not a bug.
- Python `SyntaxWarning: invalid escape sequence` messages at startup are from existing docstrings and do not affect runtime.
