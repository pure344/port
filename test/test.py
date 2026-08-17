import urllib.request
import json
import os

url = 'http://127.0.0.1:8080'

# 1. 读取同目录下的 test.json
current_dir = os.path.dirname(__file__)
json_file = os.path.join(current_dir, 'test.json')

with open(json_file, 'r', encoding='utf-8') as f:
    payload = json.load(f)

# 2. 构造标准 HTTP POST 请求
req = urllib.request.Request(
    url,
    data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
    headers={'Content-Type': 'application/json'},
    method='POST'
)

# 3. 发送请求并格式化打印响应结果
print(f"正在向 {url} 发送 POST 测试请求...")
with urllib.request.urlopen(req) as resp:
    res_text = resp.read().decode('utf-8')
    res_json = json.loads(res_text)
    print(f"[成功] 收到 HTTP {resp.status} 响应！")
    print(f"得分 overallScore: {res_json.get('metrics', {}).get('overallScore')}")