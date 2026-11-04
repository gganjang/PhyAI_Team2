#!/bin/bash
# Print sim time and block positions from one /sim/scene message.
timeout 5 ros2 topic echo --once /sim/scene --field data | python3 -c "
import json,sys
d=json.loads(sys.stdin.read().split(chr(10)+\"---\")[0])
print(\"t\",round(d[\"simulation_time\"],2))
for b in d[\"blocks\"]: print(b[\"name\"],[round(v,4) for v in b[\"position\"]])"
