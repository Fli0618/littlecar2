#!/usr/bin/env python3
"""Launch the PID tuner GUI without command-line connection arguments."""

from pid_tuner.gui.app import main


if __name__ == "__main__":
    raise SystemExit(main())
