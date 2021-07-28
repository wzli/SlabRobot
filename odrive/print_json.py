#!/usr/bin/env python3
import json
import sys

with open(sys.argv[1], "r") as json_file:
    data = json.load(json_file)
    print(json.dumps(data, indent=2))
