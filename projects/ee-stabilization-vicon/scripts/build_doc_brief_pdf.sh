#!/usr/bin/env bash
# 编译 docs/main_brief.tex → docs/pdf/项目说明-简化版.pdf
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DOCS_DIR="${PROJECT_ROOT}/docs"
OUT_DIR="${DOCS_DIR}/pdf"
mkdir -p "$OUT_DIR"

if ! command -v xelatex >/dev/null 2>&1; then
  echo "Error: xelatex not found. Install: sudo apt install texlive-xetex texlive-lang-chinese" >&2
  exit 1
fi

build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

cp "${DOCS_DIR}/main_brief.tex" "${DOCS_DIR}/1.png" "$build_dir/"
cd "$build_dir"

xelatex -interaction=nonstopmode main_brief.tex >/dev/null
xelatex -interaction=nonstopmode main_brief.tex >/dev/null

mv main_brief.pdf "${OUT_DIR}/项目说明-简化版.pdf"
echo "Created: ${OUT_DIR}/项目说明-简化版.pdf ($(du -h "${OUT_DIR}/项目说明-简化版.pdf" | cut -f1))"
