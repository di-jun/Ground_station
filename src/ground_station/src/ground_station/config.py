"""Configuration loading with conservative defaults."""

import os

import yaml


def load_config(path: str) -> dict:
    with open(os.path.expanduser(path), "r") as stream:
        config = yaml.safe_load(stream) or {}
    config.setdefault("grid", {})
    config.setdefault("ros", {})
    config["ros"].setdefault("topics", {})
    config["grid"].setdefault("rows", 9)
    config["grid"].setdefault("columns", 7)
    return config