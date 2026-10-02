import json
import urllib.request

payload = {
    "model": "qwen3-vl:8b-instruct",
    "messages": [
        {"role": "user", "content": "Hello!"}
    ],
    "stream": False,
}

request = urllib.request.Request(
    "http://127.0.0.1:11434/api/chat",
    data=json.dumps(payload).encode("utf-8"),
    headers={"Content-Type": "application/json"},
    method="POST",
)

with urllib.request.urlopen(request, timeout=300) as response:
    result = json.load(response)

print(result["message"]["content"])