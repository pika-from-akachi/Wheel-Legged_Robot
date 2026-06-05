import re, os

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

with open('esp32_app/data/index.html', 'r', encoding='utf-8') as f:
    content = f.read()

m = re.search(r'function tuningApp\(\)\s*\{', content)
start = m.start()
script = content[start:]

opens = script.count('{')
closes = script.count('}')

print(f'Open braces:  {opens}')
print(f'Close braces: {closes}')
print(f'Balanced:     {opens == closes}')

checks = [
    'loadRealModel',
    'buildPlaceholderModel',
    'GLTFLoader',
    'startAnimationLoop',
    'connectWebSocket',
]
for c in checks:
    found = c in script
    print(f'  {c}: {"OK" if found else "MISSING"}')
