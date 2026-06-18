"""Usage: python set_host_ip.py <new_ip>"""

import re
import sys

if len(sys.argv) != 2:
    print("Usage: python set_host_ip.py <new_ip>")
    sys.exit(1)

new_ip = sys.argv[1]

files = {
    "src/beavr/teleop/configs/constants/network.py": r'(HOST_ADDRESS\s*=\s*")[^"]+(")',
    "configs/environment/dev.yaml":                  r'(host_address:\s*")[^"]+(")',
}

for path, pattern in files.items():
    text = open(path).read()
    updated, n = re.subn(pattern, rf"\g<1>{new_ip}\g<2>", text)
    if n:
        open(path, "w").write(updated)
        print(f"  {path}  →  {new_ip}")
    else:
        print(f"  {path}  — no match found, skipped")
