#!/usr/bin/env python3
"""Convert Markdown files to PDF via pandoc (HTML) + WeasyPrint."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

from weasyprint import HTML

ROOT = Path(__file__).resolve().parents[2]
PDF_DIR = Path(__file__).resolve().parent
CSS_PATH = PDF_DIR / "md2pdf.css"
ASSETS_DIR = PDF_DIR / "assets"


def render_mermaid_block(code: str, stem: str) -> str:
    """Render a mermaid block to a local PNG via graphviz (fallback for blocked remote APIs)."""
    dot_src = ASSETS_DIR / f"{stem}.dot"
    png_path = ASSETS_DIR / f"{stem}.png"
    if not dot_src.exists():
        return (
            f"```text\n{code}\n```\n\n"
            f"*（流程图：见 Markdown 源文件 mermaid 代码块）*\n"
        )

    subprocess.run(
        ["dot", "-Tpng", "-Gdpi=150", str(dot_src), "-o", str(png_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    rel = png_path.relative_to(PDF_DIR).as_posix()
    return (
        f'<div class="mermaid-diagram">'
        f'<img src="{rel}" alt="控制周期数据流图" />'
        f"</div>\n\n"
        f"<!-- mermaid source:\n{code}\n-->\n"
    )


def preprocess_mermaid(md_text: str) -> str:
    """Replace mermaid fenced blocks with locally rendered diagram images."""

    counter = 0

    def repl(match: re.Match[str]) -> str:
        nonlocal counter
        counter += 1
        code = match.group(1).strip()
        stem = "control-flow" if counter == 1 else f"diagram-{counter}"
        return render_mermaid_block(code, stem)

    pattern = re.compile(r"```mermaid\n(.*?)```", re.DOTALL)
    return pattern.sub(repl, md_text)


def md_to_html(md_path: Path, html_path: Path) -> None:
    md_text = md_path.read_text(encoding="utf-8")
    md_text = preprocess_mermaid(md_text)
    tmp_md = html_path.with_suffix(".tmp.md")
    tmp_md.write_text(md_text, encoding="utf-8")
    try:
        subprocess.run(
            [
                "pandoc",
                str(tmp_md),
                "-f",
                "markdown",
                "-t",
                "html5",
                "--standalone",
                "--metadata",
                "title=",
                "-o",
                str(html_path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    finally:
        tmp_md.unlink(missing_ok=True)


def inject_styles(html_path: Path) -> str:
    html = html_path.read_text(encoding="utf-8")
    css_link = f'<link rel="stylesheet" href="{CSS_PATH.as_uri()}">'
    if "</head>" in html:
        return html.replace("</head>", f"  {css_link}\n</head>")
    return f"<!DOCTYPE html><html><head>{css_link}</head><body>{html}</body></html>"


def html_to_pdf(html_content: str, pdf_path: Path) -> None:
    HTML(string=html_content, base_url=str(PDF_DIR)).write_pdf(str(pdf_path))


def convert(md_rel: str, pdf_name: str | None = None) -> Path:
    md_path = ROOT / md_rel
    if not md_path.exists():
        raise FileNotFoundError(md_path)
    pdf_path = PDF_DIR / (pdf_name or f"{md_path.stem}.pdf")
    html_path = PDF_DIR / f"{pdf_path.stem}.html"

    print(f"Converting {md_rel} -> {pdf_path.name}")
    md_to_html(md_path, html_path)
    styled_html = inject_styles(html_path)
    html_path.write_text(styled_html, encoding="utf-8")
    html_to_pdf(styled_html, pdf_path)
    print(f"  OK: {pdf_path} ({pdf_path.stat().st_size // 1024} KB)")
    return pdf_path


def main() -> int:
    targets = [
        ("README.md", "README.pdf"),
        ("docs/项目说明.md", "项目说明.pdf"),
    ]
    errors: list[str] = []
    for md_rel, pdf_name in targets:
        try:
            convert(md_rel, pdf_name)
        except Exception as exc:  # noqa: BLE001 - report all conversion failures
            errors.append(f"{md_rel}: {exc}")
            print(f"  FAIL: {exc}", file=sys.stderr)

    if errors:
        print("\nErrors:", file=sys.stderr)
        for err in errors:
            print(f"  - {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
