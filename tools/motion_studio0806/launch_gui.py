#!/usr/bin/env python3
"""Launch the chassis motion studio 0806 GUI."""

import sys
import os

# Ensure package importability when launched directly
sys.path.insert(0, os.path.dirname(__file__))

from motion_studio0806.app import main


if __name__ == "__main__":
    main()
