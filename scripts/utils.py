#!/usr/bin/env python3

import yaml
import typing as tp

def read_config(path: str) -> tp.List:
        with open(path) as f:
            config = yaml.safe_load(f)
        return config