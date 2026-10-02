"""Ollama HTTP client"""

import base64
import json
import urllib.error
import urllib.request

from problem_json_to_pddl import PROJECT_ROOT

DEFAULT_MODEL = "qwen3-vl:8b-instruct"
DEFAULT_ENDPOINT = "http://127.0.0.1:11434/api/chat"


def chat_with_image(prompt, image_path, schema, *, model=DEFAULT_MODEL,
                    endpoint=DEFAULT_ENDPOINT, timeout=600):
    payload = {
        "model": model,
        "messages": [{
            "role": "user",
            "content": prompt,
            "images": [base64.b64encode(image_path.read_bytes()).decode("ascii")],
        }],
        "format": schema,
        "stream": False,
        "options": {"temperature": 0, "num_ctx": 32768},
    }


    # write prompt variable to a file for debugging
    with open(PROJECT_ROOT / "output" / "prompt.txt", "w", encoding="utf-8") as f:
        f.write(prompt)

    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(
            f"Ollama error {exc.code}: {exc.read().decode('utf-8', errors='replace')}"
        ) from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Cannot reach Ollama: {exc.reason}") from exc
    except (TimeoutError, ValueError) as exc:
        raise RuntimeError(f"Invalid or timed-out Ollama response: {exc}") from exc

    if not isinstance(result, dict):
        raise RuntimeError("Ollama response must be a JSON object.")
    if result.get("error"):
        raise RuntimeError(f"Ollama error: {result['error']}")
    message = result.get("message")
    if not isinstance(message, dict) or not isinstance(message.get("content"), str):
        raise RuntimeError("Ollama response is missing message.content text.")

    return result
