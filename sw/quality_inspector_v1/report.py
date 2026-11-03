"""Self-contained report packaging for measured or imported sessions."""

from __future__ import annotations

import csv
import hashlib
import html
import json
import re
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _json_default(value: Any):
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return str(value)


def _safe_name(path: Path, used: set[str]) -> str:
    name = re.sub(r"[^A-Za-z0-9._-]+", "_", path.name).strip("._") or "artifact"
    base, suffix = Path(name).stem, Path(name).suffix
    candidate, index = name, 2
    while candidate.lower() in used:
        candidate = f"{base}_{index}{suffix}"
        index += 1
    used.add(candidate.lower())
    return candidate


def _path_key(path: Path) -> str:
    try:
        return str(path.resolve()).casefold()
    except OSError:
        return str(path.absolute()).casefold()


def export_report(session: dict, destination: Path) -> Path:
    """Write HTML, JSON, CSV, a provenance manifest, copied files and ZIP.

    ``destination`` must not exist. The returned path is the new report folder.
    Missing artifact references are recorded as missing in the manifest.
    """
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    rows = session.get("results", {}) or {}
    artifacts = list(session.get("artifacts", []) or [])
    for row in rows.values():
        artifacts.extend(row.get("evidence", []) or [])

    artifact_dir = destination / "artifacts"
    artifact_dir.mkdir()
    manifest = []
    links: dict[str, str] = {}
    used: set[str] = set()
    seen: dict[str, dict] = {}
    for raw_path in artifacts:
        source = Path(str(raw_path))
        key = _path_key(source)
        if key in seen:
            links[str(source)] = seen[key]["copiedPath"]
            continue
        record = {"sourcePath": str(source), "exists": source.is_file()}
        if source.is_file():
            name = _safe_name(source, used)
            target = artifact_dir / name
            source_digest = hashlib.sha256(source.read_bytes()).hexdigest()
            shutil.copy2(source, target)
            copied_digest = hashlib.sha256(target.read_bytes()).hexdigest()
            rel = f"artifacts/{name}"
            record.update({"copiedPath": rel, "sha256Source": source_digest,
                           "sha256Copied": copied_digest, "sha256": source_digest,
                           "sizeBytes": target.stat().st_size})
            links[str(source)] = rel
            seen[key] = record
        manifest.append(record)
    session_copy = dict(session)
    copied_rows = {}
    for key, row in rows.items():
        row_copy = dict(row)
        row_copy["evidence"] = [links.get(str(path), f"MISSING: {Path(str(path)).name}")
                                for path in (row.get("evidence", []) or [])]
        copied_rows[key] = row_copy
    session_copy["results"] = copied_rows
    session_copy["artifacts"] = [links.get(str(item), str(item)) for item in (session.get("artifacts", []) or [])]
    session_copy["reportCreatedUtc"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    session_copy["sourceIntegrity"] = manifest
    (destination / "session.json").write_text(json.dumps(session_copy, ensure_ascii=False, indent=2, default=_json_default), encoding="utf-8")
    (destination / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    numeric_metrics = {key: row.get("metrics", {}) or {} for key, row in rows.items()}
    (destination / "numeric_metrics.json").write_text(
        json.dumps(numeric_metrics, ensure_ascii=False, indent=2, default=_json_default), encoding="utf-8")

    with (destination / "results.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=["key", "title", "status", "value", "detail", "source", "limits"])
        writer.writeheader()
        for key, row in rows.items():
            writer.writerow({"key": key, "title": row.get("title", key), "status": row.get("status", "not_measured"),
                             "value": row.get("value", ""), "detail": row.get("detail", ""),
                             "source": row.get("source", session.get("source", "unknown")),
                             "limits": "; ".join(map(str, row.get("limits", []) or []))})

    esc = lambda value: html.escape(str(value if value is not None else ""), quote=True)
    source = str(session.get("source", "unknown"))
    historical = "historical" in source.lower() or any(str(r.get("source", "")).lower() == "historical" for r in rows.values())
    banner = "历史记录：此报告来自既有测量，不代表当前实时状态。" if historical else "测量结果仅记录观测值，不代表官方竞赛通过。"
    html_rows = []
    for key, row in rows.items():
        evidence = []
        for raw in row.get("evidence", []) or []:
            rel = links.get(str(raw))
            if rel:
                label = esc(Path(rel).name)
                if Path(rel).suffix.lower() in (".png", ".jpg", ".jpeg", ".webp"):
                    evidence.append(f'<a href="{esc(rel)}">{label}</a><br><img class="evidence" src="{esc(rel)}" alt="{label}">')
                else:
                    evidence.append(f'<a href="{esc(rel)}">{label}</a>')
        limits = "<br>".join(esc(limit) for limit in (row.get("limits", []) or []))
        metrics_text = json.dumps(row.get("metrics", {}) or {}, ensure_ascii=False, indent=2, default=_json_default)
        metrics = f"<details><summary>数值指标</summary><pre>{esc(metrics_text)}</pre></details>"
        state_labels={'not_measured':'未测','running':'测量中','measured':'已测','condition_required':'条件不足','cancelled':'已取消','error':'错误'}
        source_labels={'live':'本次采集','historical':'历史记录','imported':'外部导入'}
        html_rows.append("<tr>" + "".join(f"<td>{esc(value)}</td>" for value in
            (row.get("title", key), state_labels.get(row.get('status'),row.get('status','未测')), row.get("value", ""), row.get("detail", ""), source_labels.get(row.get('source',source),row.get('source',source)))) +
            f"<td>{limits}</td><td>{metrics}</td><td>{'<br>'.join(evidence)}</td></tr>")
    notes = "".join(f"<li>{esc(note)}</li>" for note in (session.get("notes", []) or []))
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>画质测量报告</title><style>body{{font:15px Segoe UI,Arial,sans-serif;max-width:1200px;margin:32px auto;padding:0 18px;color:#20383a}}h1{{color:#126f6b}}.banner{{background:#e7f4f1;padding:12px;border-left:4px solid #16877d}}table{{border-collapse:collapse;width:100%;margin:18px 0}}td,th{{border:1px solid #d7e2e0;padding:8px;text-align:left;vertical-align:top}}th{{background:#f1f6f5}}.evidence{{display:block;max-width:420px;max-height:280px;object-fit:contain}}</style>
<h1>画质测量报告</h1><p class="banner">{esc(banner)}</p><p>会话：{esc(session.get('id',''))}<br>测量时间：{esc(session.get('createdUtc',''))}<br>来源：{esc(source)}</p>
<table><thead><tr><th>项目</th><th>状态</th><th>结果</th><th>说明</th><th>来源</th><th>限制/条件</th><th>数值指标</th><th>证据</th></tr></thead><tbody>{''.join(html_rows)}</tbody></table>
<h2>备注</h2><ul>{notes}</ul><p>原始文件及 SHA-256 清单见同目录 manifest.json；完整材料见 report.zip。</p></html>'''
    (destination / "index.html").write_text(page, encoding="utf-8")
    zip_path = destination / "report.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in destination.rglob("*"):
            if path.is_file() and path != zip_path:
                archive.write(path, path.relative_to(destination).as_posix())
    return destination


def _select_pdf_images(session: dict) -> list[tuple[str, Path]]:
    """Return the first available SFR curve and straightness image, at most two."""
    candidates = list(session.get("artifacts", []) or [])
    for row in (session.get("results", {}) or {}).values():
        candidates.extend(row.get("evidence", []) or [])
    selected = []
    seen = set()
    for raw in candidates:
        path = Path(str(raw))
        try:
            key = str(path.resolve()).casefold()
        except OSError:
            key = str(path.absolute()).casefold()
        if key in seen or not path.is_file() or path.suffix.lower() not in ('.png','.jpg','.jpeg','.webp'):
            continue
        seen.add(key)
        name = path.name.casefold()
        if "sfr" in name and not any(label == "SFR 曲线" for label, _ in selected):
            selected.append(("SFR 曲线", path))
        elif "straight" in name and not any(label == "直线度" for label, _ in selected):
            selected.append(("直线度", path))
        if len(selected) == 2:
            break
    return selected


def _pdf_summary_html(session: dict, image_links: list[tuple[str, str]]) -> str:
    """Build a compact printable summary; full metrics/evidence stay in HTML/JSON."""
    esc = lambda value: html.escape(str(value if value is not None else ""), quote=True)
    rows = []
    for key, row in (session.get("results", {}) or {}).items():
        title = row.get("title", key)
        limits = "<br>".join(esc(item) for item in (row.get("limits", []) or []))
        states={'not_measured':'未测','running':'测量中','measured':'已测','condition_required':'条件不足','cancelled':'已取消','error':'错误'}
        sources={'historical':'历史记录','imported':'外部导入','live':'本次采集'}
        origin=sources.get(row.get('source',session.get('source')),row.get('source','未知来源'))
        detail=str(row.get('detail',''))+' / '+str(origin)
        if row.get('measuredUtc'):detail+=' / 原始测量：'+str(row['measuredUtc'])
        fields = (title, states.get(row.get('status'),row.get('status','未测')), row.get("value", ""), detail)
        rows.append("<tr>" + "".join(f"<td>{esc(field)}</td>" for field in fields) + f"<td>{limits}</td></tr>")
    images = "".join(f'<p><img src="{esc(path)}"></p><p>{esc(label)}</p>'
                      for label, path in image_links)
    historical=session.get('source')=='historical' or any(row.get('source')=='historical' for row in (session.get('results') or {}).values())
    provenance='包含历史测量；不代表当前实时状态。' if historical else '按记录的采集时间与来源解释，不代表官方通过。'
    return f'''<!doctype html><html lang="zh-CN" data-source="{esc(session.get('source','unknown'))}"><meta charset="utf-8"><style>
body{{font:10pt "Microsoft YaHei",Arial,sans-serif;color:#20383a;margin:18mm}}h1{{color:#126f6b;font-size:18pt}}
table{{border-collapse:collapse;width:100%;font-size:9pt}}td,th{{border:1px solid #9aabaa;padding:6px;vertical-align:top;text-align:left}}
th{{background:#edf4f3}}img{{width:540px;height:auto}}
</style><h1>画质测量摘要</h1>
<p>汇总会话创建时间：{esc(session.get('createdUtc', session.get('reportCreatedUtc', '')))}<br>{esc(provenance)}</p>
<table><thead><tr><th>项目</th><th>状态</th><th>结果</th><th>说明</th><th>限制/条件</th></tr></thead><tbody>{''.join(rows)}</tbody></table>
{('<h2>代表性图像</h2>' + images) if images else ''}</html>'''


def export_pdf(session: dict, destination: Path) -> Path:
    """Write a compact A4 summary PDF with at most one image of each requested type."""
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(destination)
    try:
        from PySide6.QtGui import QFont, QFontDatabase, QImage, QPageSize, QPdfWriter, QTextDocument
    except ImportError as exc:
        raise ImportError("PySide6 Qt PDF support is unavailable") from exc
    import os
    import shutil
    import tempfile
    from PySide6.QtCore import QUrl, QMarginsF
    destination.parent.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix=f".{destination.stem}-pdf-", dir=destination.parent))
    try:
        artifacts_dir = folder / "artifacts"
        artifacts_dir.mkdir()
        used_names = set()
        image_links = []
        for label, source_path in _select_pdf_images(session):
            name = _safe_name(source_path, used_names)
            shutil.copy2(source_path, artifacts_dir / name)
            image_links.append((label, QUrl.fromLocalFile(str((artifacts_dir / name).resolve())).toString()))
        font_dir = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts"
        for font_file in ("arial.ttf", "msyh.ttc"):
            font_path = font_dir / font_file
            if font_path.is_file():
                QFontDatabase.addApplicationFont(str(font_path))
        document = QTextDocument()
        base_url = QUrl.fromLocalFile(str(folder.resolve()) + os.sep)
        document.setBaseUrl(base_url)
        document.setDefaultFont(QFont("Arial", 9))
        for _, image_path in image_links:
            image_url = QUrl(image_path)
            document.addResource(QTextDocument.ResourceType.ImageResource, image_url,
                                 QImage(image_url.toLocalFile()))
        document.setHtml(_pdf_summary_html(session, image_links))
        pdf_writer = QPdfWriter(str(destination))
        pdf_writer.setPageSize(QPageSize(QPageSize.PageSizeId.A4))
        pdf_writer.setPageMargins(QMarginsF(12, 12, 12, 12))
        document.print_(pdf_writer)
    finally:
        resolved_folder = folder.resolve()
        resolved_parent = destination.parent.resolve()
        expected_prefix = f".{destination.stem}-pdf-"
        if resolved_folder.parent == resolved_parent and resolved_folder.name.startswith(expected_prefix):
            shutil.rmtree(resolved_folder)
    return destination
