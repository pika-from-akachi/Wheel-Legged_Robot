import os
import re

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

with open("esp32_app/data/index.html", "r", encoding="utf-8") as f:
    content = f.read()

match = re.search(r"<script>([\s\S]*)</script>", content)
script = match.group(1) if match else ""

opens = script.count("{")
closes = script.count("}")

print(f"Script found:  {bool(script)}")
print(f"Open braces:   {opens}")
print(f"Close braces:  {closes}")
print(f"Balanced:      {opens == closes}")

checks = [
    "轮足机器人遥控台",
    "WASD 遥控",
    "connectWebSocket",
    "sendDrive",
    "setKey",
    "keydown",
    "keyup",
    "enableSwitch",
    'type: "drive"',
    "max_rpm",
]

for check in checks:
    haystack = content if check.startswith("轮") or check.startswith("WASD") else script
    print(f"  {check}: {'OK' if check in haystack else 'MISSING'}")
