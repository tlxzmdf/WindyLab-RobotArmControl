#!/usr/bin/env bash
# Convert Markdown to PDF via Pandoc (HTML) + Chromium headless print.
# Handles Chinese text, code blocks, tables, and Mermaid diagrams.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${PROJECT_ROOT}/docs/pdf"
CACHE_DIR="${OUT_DIR}/.cache"
mkdir -p "$OUT_DIR" "$CACHE_DIR"

CHROMIUM=""
for c in chromium chromium-browser google-chrome /snap/bin/chromium; do
  if command -v "$c" >/dev/null 2>&1; then
    CHROMIUM="$c"
    break
  fi
done
if [[ -z "$CHROMIUM" ]]; then
  echo "Error: Chromium/Chrome not found" >&2
  exit 1
fi

CJK_FONT="Noto Sans CJK SC"
if ! fc-list :lang=zh family 2>/dev/null | grep -q "Noto Sans CJK SC"; then
  CJK_FONT="Droid Sans Fallback"
fi

MONO_FONT="Noto Sans Mono CJK SC"
if ! fc-list :lang=zh family 2>/dev/null | grep -q "Noto Sans Mono CJK SC"; then
  MONO_FONT="$CJK_FONT"
fi

convert_md() {
  local src="$1"
  local pdf="$2"
  local title="$3"
  local html
  html="$(mktemp "${CACHE_DIR}/md2pdf-XXXXXX.html")"

  pandoc "$src" \
    -f markdown \
    -t html5 \
    --standalone \
    --metadata title="$title" \
    -V lang=zh-CN \
    -o "$html"

  # Inject print CSS, CJK fonts, and Mermaid rendering before </head>
  python3 - "$html" "$CJK_FONT" "$MONO_FONT" <<'PY'
import sys
from pathlib import Path

html_path, cjk_font, mono_font = sys.argv[1:4]
text = Path(html_path).read_text(encoding="utf-8")

inject = f"""
<style>
  @page {{
    size: A4;
    margin: 20mm 18mm 22mm 18mm;
  }}
  html {{
    font-size: 11pt;
  }}
  body {{
    font-family: "{cjk_font}", "Droid Sans Fallback", sans-serif;
    line-height: 1.55;
    color: #1a1a1a;
    max-width: 100%;
    word-wrap: break-word;
    overflow-wrap: anywhere;
  }}
  h1, h2, h3, h4 {{
    font-family: "{cjk_font}", "Droid Sans Fallback", sans-serif;
    page-break-after: avoid;
  }}
  header#title-block-header {{
    display: none;
  }}
  h1 {{ font-size: 1.65rem; border-bottom: 1px solid #ddd; padding-bottom: 0.3em; }}
  h2 {{ font-size: 1.25rem; margin-top: 1.4em; }}
  code, pre, pre code {{
    font-family: "{mono_font}", "Droid Sans Fallback", monospace;
    font-size: 0.88em;
  }}
  pre {{
    background: #f6f8fa;
    border: 1px solid #e1e4e8;
    border-radius: 4px;
    padding: 0.75em 1em;
    white-space: pre-wrap;
    word-break: break-word;
    page-break-inside: avoid;
  }}
  :not(pre) > code {{
    background: #f0f0f0;
    padding: 0.1em 0.35em;
    border-radius: 3px;
  }}
  table {{
    border-collapse: collapse;
    width: 100%;
    margin: 1em 0;
    font-size: 0.95em;
    page-break-inside: avoid;
  }}
  th, td {{
    border: 1px solid #ccc;
    padding: 0.45em 0.65em;
    text-align: left;
    vertical-align: top;
  }}
  th {{
    background: #f0f0f0;
  }}
  blockquote {{
    border-left: 4px solid #ccc;
    margin-left: 0;
    padding-left: 1em;
    color: #444;
  }}
  a {{
    color: #0366d6;
    text-decoration: none;
  }}
  hr {{
    border: none;
    border-top: 1px solid #ddd;
    margin: 1.5em 0;
  }}
  .mermaid {{
    text-align: center;
    margin: 1.2em 0;
    page-break-inside: avoid;
  }}
  img {{
    max-width: 100%;
    height: auto;
  }}
</style>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js"></script>
<script>
  mermaid.initialize({{ startOnLoad: false, theme: "default", securityLevel: "loose" }});
</script>
"""

if "</head>" in text:
    text = text.replace("</head>", inject + "</head>", 1)
else:
    text = inject + text

# Pandoc emits <pre class="mermaid"><code>…</code></pre> — convert to renderable div
import html
import re

def mermaid_repl(m):
    body = html.unescape(m.group(1).strip())
    return f'<div class="mermaid">\n{body}\n</div>'

for pattern in (
    r'<pre class="mermaid"><code>(.*?)</code></pre>',
    r'<pre><code class="language-mermaid">(.*?)</code></pre>',
):
    text = re.sub(pattern, mermaid_repl, text, flags=re.DOTALL)

wait_script = """
<script>
window.addEventListener('load', async () => {
  const blocks = document.querySelectorAll('.mermaid');
  if (blocks.length && window.mermaid) {
    await mermaid.run({ nodes: blocks });
  }
  document.body.setAttribute('data-ready', '1');
});
</script>
"""
text = text.replace("</body>", wait_script + "</body>", 1)

Path(html_path).write_text(text, encoding="utf-8")
PY

  # Chromium print (wait for mermaid if present)
  local wait_ms=8000
  if grep -q 'class="mermaid"' "$html"; then
    wait_ms=25000
  fi

  "$CHROMIUM" \
    --headless=new \
    --disable-gpu \
    --no-sandbox \
    --disable-dev-shm-usage \
    --no-pdf-header-footer \
    --virtual-time-budget="$wait_ms" \
    --run-all-compositor-stages-before-draw \
    --print-to-pdf="$pdf" \
    "file://${html}" 2>/dev/null

  rm -f "$html"

  if [[ ! -f "$pdf" ]]; then
    echo "Error: failed to create $pdf" >&2
    exit 1
  fi
  echo "Created: $pdf ($(du -h "$pdf" | cut -f1))"
}

convert_md "${PROJECT_ROOT}/README.md" "${OUT_DIR}/README.pdf" "机械臂末端位姿稳定 — README"

echo "Done. PDFs in ${OUT_DIR}/"
echo "For project doc PDF, run: ./scripts/build_doc_pdf.sh"
