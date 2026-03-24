#!/usr/bin/env python3
"""Compatibility entrypoint for mission node.

The old monolithic detector implementation has been replaced by a modular
MissionManager architecture.
"""

from ridgeback_vision_detection.mission_manager import main


if __name__ == "__main__":
    main()
