"""Pure, compact presentation summaries for inspector result rows."""
from __future__ import annotations

from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
from typing import Any


TITLES = {
    "geometry": "校正精度", "sharpness": "清晰度", "performance": "帧率与掉帧",
    "edges": "边缘与视野", "gamma": "Gamma", "cnn": "CNN",
    "resources": "资源与时序", "latency": "硬件延迟", "comparison": "校正前后对比",
}
STATUS_LABELS = {
    "not_measured": "未测", "running": "测量中", "measured": "已测",
    "condition_required": "条件不足", "error": "未完成", "cancelled": "已停止",
}
METHODS = {
    "geometry": ("检测三组图像中的共同棋盘格线。",
                 "比较拟合直线的点到线垂直偏差，单位为输出像素。"),
    "sharpness": ("提取黑白斜边的亮度变化，计算SFR曲线及其降到50%时的频率（MTF50）。",
                  "分别测量校正前、校正后与重复基线的同一条斜边。"),
    "performance": ("定时读取FPGA相机帧、源帧和结果帧的累计计数。",
                    "用新增帧数除以测量秒数计算FPS，并记录新增丢帧。"),
    "edges": ("检查全光栅每个输出像素对应的四邻点取样坐标是否越界。",
              "统计无效坐标数量，不检测所有真实场景中的边缘断裂。"),
    "gamma": ("读取FPGA输出画面的整帧亮度统计和已应用档位。",
              "比较连续帧号，确认统计更新，再展示亮度和档位。"),
    "cnn": ("读取FPGA回传的模型加载、相机参数、启用和预测计数。",
            "本项检查模型运行状态，不计算预测准确率。"),
    "resources": ("读取用户导入的 PDS 资源与时序报告。",
                  "只汇总文件中实际存在的资源和时序字段。"),
    "latency": ("优先显示 R16 记录的 T0 到首次显示时间戳差。",
                "按角度汇总样本数与 P95；外部 CSV 保留其仪器方法说明。"),
    "comparison": ("显示同一采集路径的校正前、校正后和重复基线图像。",
                   "图窗可能缩放画面；可以打开原图查看像素。"),
}
INTERPRETATIONS = {
    "geometry": "这是棋盘格线的垂直直线度偏差，不是相机重投影误差；RMS 越低表示线越直。",
    "sharpness": "MTF50 单位为 cycles/output-pixel；数值越高通常表示边缘越锐，但不代表几何校正更准。",
    "performance": "FPS 来自 A5 相机帧、源帧和结果帧计数差分，不是 HDMI 解码帧率。",
    "edges": "只说明本次配置的映射范围，不证明所有角度的实景都没有边缘断裂。",
    "gamma": "帧统计和档位是输出观测，不能证明画面增强效果或准确度。",
    "cnn": "模型、K 和预测计数只描述运行状态，不能说明预测精度。",
    "resources": "导入的资源报告不证明当前 bitstream 身份，也不代替完整时序与 CDC 审查。",
    "latency": "时间戳描述本次记录的硬件处理路径；外部导入值以其仪器和同步方法为准。",
    "comparison": "对照来自同源 HDMI 采集图像，会受相机曝光、颜色处理和 HDMI 采集压缩影响，不等于传感器 RAW。",
}

BEIJING = timezone(timedelta(hours=8), "北京时间")


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: Any, digits: int = 2, suffix: str = "") -> str | None:
    number = _finite(value)
    if number is None:
        return None
    return f"{number:.{digits}f}{suffix}"


def _cell(label: str, value: Any) -> dict[str, str]:
    return {"label": str(label), "value": str(value)}


def _angle_label(raw: Any) -> str:
    try:
        value = float(raw)
    except (TypeError, ValueError, OverflowError):
        return str(raw)
    degrees = value / 65536.0
    return f"{degrees:g}°"


def _last_observation(metrics: dict, key: str) -> dict:
    values = metrics.get(key)
    return values[-1] if isinstance(values, list) and values and isinstance(values[-1], dict) else {}


def _cells(key: str, metrics: dict) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    if key == "geometry":
        stages = metrics.get("stages") or {}
        before = [stages.get(k, {}).get("meanLineRmsMedianPx") for k in ("baseline1", "baseline2")]
        before = [_finite(v) for v in before if _finite(v) is not None]
        calibrated = _finite(stages.get("calibrated", {}).get("meanLineRmsMedianPx"))
        if before:
            out.append(_cell("校正前直线度 RMS", f"{sum(before) / len(before):.3f} px"))
        if calibrated is not None:
            out.append(_cell("校正后直线度 RMS", f"{calibrated:.3f} px"))
        maximum = max((_finite(v.get("maximumDeviationPx")) for v in stages.values()
                       if isinstance(v, dict) and _finite(v.get("maximumDeviationPx")) is not None),
                      default=None)
        if maximum is not None:
            out.append(_cell("三组最大点到线偏差", f"{maximum:.3f} px"))
        drift = _fmt(metrics.get("repeatMaximumCornerDriftPx"), 3, " px")
        if drift:
            out.append(_cell("重复基线角点漂移", drift))
        if metrics.get("commonCorners") is not None:
            out.append(_cell("共同角点", metrics["commonCorners"]))
    elif key == "sharpness":
        rois = metrics.get("rois") or []
        for direction in ("vertical", "horizontal"):
            matches = [r for r in rois if str(r.get("name", "")).startswith("center_")
                       and r.get("orientation") == direction]
            stage = (matches[0].get("calibrated") or {}) if matches else {}
            median = _finite((stage.get("mtf50_cpp") or {}).get("median"))
            if median is not None:
                out.append(_cell(f"中心{('垂直' if direction == 'vertical' else '水平')} MTF50",
                                 f"{median:.3f} cycles/output-pixel"))
        if metrics.get("accepted") is not None and metrics.get("total") is not None:
            out.append(_cell("有效斜边样本", f"{metrics['accepted']}/{metrics['total']}"))
        stable = sum(1 for item in rois if item.get("stable") is True)
        if rois:
            out.append(_cell("基线稳定位置", f"{stable}/{len(rois)}"))
    elif key == "performance":
        for metric, label in (("camera_fps", "相机输入"), ("source_commit_fps", "源帧提交"),
                              ("result_commit_fps", "结果帧提交")):
            value = _fmt(metrics.get(metric), 2, " FPS")
            if value:
                out.append(_cell(label, value))
        elapsed = _fmt(metrics.get("elapsed_seconds"), 1, " 秒")
        if elapsed:
            out.append(_cell("观测时长", elapsed))
        if metrics.get("sourceFrameDropCount") is not None:
            out.append(_cell("源丢帧", metrics["sourceFrameDropCount"]))
        elif metrics.get("source_drop_fps") is not None:
            out.append(_cell("源丢帧速率", _fmt(metrics["source_drop_fps"], 2, " 帧/秒")))
    elif key == "edges":
        audit = metrics.get("configurationRangeAudit") or {}
        invalid, total = audit.get("invalidFetch4Count"), audit.get("count")
        if invalid is not None and total is not None:
            out.append(_cell("无效映射坐标", f"{invalid}/{total}"))
        for field, label in (("fetch4Count", "四点取样数"), ("minimumMarginPx", "最小边界余量")):
            if audit.get(field) is not None:
                out.append(_cell(label, audit[field]))
    elif key == "gamma":
        obs = _last_observation(metrics, "observations")
        bank = obs.get("applied_bank")
        out.append(_cell("实际档位", f"{bank}" if bank is not None else "未知"))
        mean = _fmt(obs.get("mean_luminance"), 1)
        if mean:
            out.append(_cell("帧平均亮度", mean))
        if obs.get("valid_count") is not None:
            out.append(_cell("有效像素", f"{obs.get('valid_count'):,}"))
        if obs.get("frame_id") is not None:
            out.append(_cell("最新帧号", obs["frame_id"]))
    elif key == "cnn":
        obs = metrics.get("observation") or {}
        for field, label in (("loaded", "模型加载"), ("kReady", "K 参数"), ("enabled", "预测启用")):
            value = obs.get(field)
            out.append(_cell(label, "未知" if value is None else "是" if value is True else "否"))
        if obs.get("predictionCount") is not None:
            out.append(_cell("预测次数", obs["predictionCount"]))
        if obs.get("predictionCycles") is not None:
            out.append(_cell("预测周期", obs["predictionCycles"]))
    elif key == "resources":
        resources = metrics.get("resources") or {}
        for name in ("LUT", "FF", "APM", "DRM_grid"):
            item = resources.get(name)
            if isinstance(item, dict) and item.get("used") is not None:
                percent = _fmt(item.get("percent"), 1, "%")
                value = f"{item['used']:,}/{item.get('available', '?')}"
                if percent:
                    value += f" · {percent}"
                out.append(_cell(name, value))
        parsed = sum(f.get("status") == "parsed" for f in metrics.get("files", []))
        total = len(metrics.get("files", []))
        if total:
            out.append(_cell("已解析报告", f"{parsed}/{total}"))
        for item in metrics.get("timingSummaries", [])[:2]:
            slack = _fmt(item.get("worstSlackNs"), 3, " ns")
            if slack:
                out.append(_cell(str(item.get("class", "最差时序")), slack))
        summaries = metrics.get("timingSummaries") or []
        violations = [item for item in summaries
                      if (_finite(item.get("worstSlackNs")) is not None and _finite(item.get("worstSlackNs")) < 0)
                      or (_finite(item.get("failingEndpoints")) is not None and _finite(item.get("failingEndpoints")) > 0)]
        if violations:
            count = sum(int(_finite(item.get("failingEndpoints")) or 0) for item in violations)
            out.insert(0, _cell("时序状态", f"存在违例 · {count} 个失败端点" if count else "存在负 WNS"))
    elif key == "latency":
        per_angle = metrics.get("per_angle") or {}
        if per_angle:
            for angle, item in sorted(per_angle.items(), key=lambda p: _finite(p[0]) or 0):
                stats = (item.get("latency_ms") or {}).get("t2_minus_t0") or {}
                avg = _fmt(stats.get("avg"), 3)
                maximum = _fmt(stats.get("max"), 3)
                p95 = _fmt(stats.get("p95"), 3)
                count = item.get("sample_count", stats.get("count"))
                values = []
                if avg is not None:
                    values.append(f"均值 {avg} ms")
                if maximum is not None:
                    values.append(f"最大 {maximum} ms")
                if p95 is not None:
                    values.append(f"P95 {p95} ms")
                if values:
                    out.append(_cell(_angle_label(angle), "\n".join(values) + (f" · {count} 帧" if count is not None else "")))
        elif metrics.get("mean_ms") is not None:
            out.append(_cell("平均延迟", _fmt(metrics.get("mean_ms"), 3, " ms")))
            if metrics.get("minimum_ms") is not None:
                out.append(_cell("最小 / 最大", f"{_fmt(metrics.get('minimum_ms'), 3)} / {_fmt(metrics.get('maximum_ms'), 3)} ms"))
            if metrics.get("samples") is not None:
                out.append(_cell("样本数", metrics["samples"]))
            method = metrics.get("method")
            if method:
                out.append(_cell("测量方法", "; ".join(method) if isinstance(method, list) else method))
    return out[:6]


def _timestamp(row: dict) -> str:
    raw = row.get("measuredUtc") or row.get("updatedUtc") or row.get("endedUtc") or row.get("startedUtc")
    if not raw:
        return ""
    try:
        value = datetime.fromisoformat(str(raw).replace("Z", "+00:00"))
        if value.tzinfo is None:
            value = value.replace(tzinfo=timezone.utc)
        return value.astimezone(BEIJING).strftime("%Y-%m-%d %H:%M:%S 北京时间")
    except (TypeError, ValueError, OverflowError):
        return ""


def _source_label(row: dict) -> str:
    source = row.get("source")
    if source == "historical":
        return "历史记录"
    if source == "live":
        return "本次测量"
    if source in ("imported", "external"):
        return "导入数据"
    return "未标记来源"


def _unique_notes(values) -> list[str]:
    result = []
    seen = set()
    for value in values:
        text = str(value).strip()
        if text and text not in seen:
            seen.add(text)
            result.append(text)
    return result


def build_presentation(key: str, row: dict | None, *, results: dict | None = None) -> dict:
    """Map persisted result data to compact display fields without changing it."""
    row = row or {}
    results = results or {}
    key = key if key in TITLES else "geometry"
    status = str(row.get("status", "not_measured"))
    metrics = row.get("metrics") if isinstance(row.get("metrics"), dict) else {}
    measured = status == "measured"
    value = row.get("value")
    if status == "not_measured":
        summary = "未测"
    elif status == "running":
        summary = "测量中"
    elif status == "cancelled":
        summary = "已停止"
    elif status in ("condition_required", "error"):
        summary = "未取得"
    else:
        summary = str(value).strip() if value not in (None, "") else STATUS_LABELS.get(status, "未取得")

    conclusion = str(row.get("detail") or "").strip()
    if not measured:
        reason = conclusion or (str(value).strip() if value not in (None, "") else "")
        if status == "condition_required":
            conclusion = f"条件不足：{reason}" if reason else "尚未取得有效测量数据。"
        elif status == "error":
            conclusion = f"测量未完成：{reason}" if reason else "测量未完成，未取得有效结果。"
        elif status == "cancelled":
            conclusion = f"已停止：{reason}" if reason else "测量已停止。"
        elif status == "running":
            conclusion = "正在采集或分析数据。"
        else:
            conclusion = reason or "尚未开始测量。"
        if status in ("condition_required", "error") and metrics:
            conclusion += " 部分数据未满足有效性条件，仅作诊断参考。"
        if key == "resources" and not metrics.get("timingSummaries"):
            conclusion += " 时序未评估。"
    else:
        if key == "sharpness":
            rois = metrics.get("rois") or []
            stable = sum(item.get("stable") is True for item in rois)
            if rois:
                conclusion = f"{stable}/{len(rois)} 个位置满足稳定性条件，可查看 SFR 曲线作对比。"
            else:
                conclusion = "已取得 SFR 数值，可查看曲线比较不同阶段。"
        elif key == "geometry":
            conclusion = "已取得棋盘格线直线度数据，可比较校正前后偏差。"
        elif key == "performance":
            conclusion = "已取得 A5 计数和计时数据，可查看输入与提交速率。"
        elif key == "edges":
            conclusion = "已取得当前配置的映射范围统计。"
        elif key == "gamma":
            conclusion = "已取得帧亮度统计和实际档位观测。"
        elif key == "cnn":
            conclusion = "已取得板端模型与运行状态观测。"
        elif key == "resources":
            summaries = metrics.get("timingSummaries") or []
            violations = [item for item in summaries
                          if (_finite(item.get("worstSlackNs")) is not None and _finite(item.get("worstSlackNs")) < 0)
                          or (_finite(item.get("failingEndpoints")) is not None and _finite(item.get("failingEndpoints")) > 0)]
            if violations:
                endpoints = sum(int(_finite(item.get("failingEndpoints")) or 0) for item in violations)
                conclusion = f"导入报告存在时序违例：{endpoints} 个失败端点；仅反映报告覆盖范围。" if endpoints else "导入报告存在负 WNS；仅反映报告覆盖范围。"
            elif summaries:
                conclusion = f"所导入报告的 {len(summaries)} 类时序摘要中未见负 WNS 或失败端点；仅反映报告覆盖范围。"
            else:
                conclusion = "时序未评估：导入报告未提供可解析的时序摘要。"
        elif key == "latency":
            per_angle = metrics.get("per_angle") or {}
            if per_angle:
                conclusion = f"已取得 {len(per_angle)} 个角度的延迟样本，可比较均值、最大值和 P95。"
            else:
                conclusion = "已取得延迟样本，可按所用仪器方法查看统计。"
        elif key == "comparison":
            conclusion = "同源图像对照已准备，可切换查看各阶段图像。"

    source = _source_label(row)
    timestamp = _timestamp(row)
    evidence = list(dict.fromkeys(str(p) for p in row.get("evidence", []) if p))
    notes = list(row.get("limits", []) or [])
    if status == "measured":
        notes.append("“已测”仅表示取得了结果，不代表官方合格或认证通过。")
    if key == "resources":
        notes.append("外部 PDS 报告不证明当前 bitstream 身份，也不代替完整 CDC 认证。")
    elif key == "comparison":
        notes.append("窗口可能缩放画面；相机曝光、颜色处理和 HDMI 采集压缩会影响对照，不含显示器观察。")
    notes = _unique_notes(notes)

    if key == "comparison":
        all_rows = list(results.values())
        historical = any(item.get("source") == "historical" for item in all_rows)
        source = "历史记录" if historical else "本次测量"
        evidence = list(dict.fromkeys(str(path) for item in all_rows
                                      for path in item.get("evidence", [])
                                      if str(path).lower().endswith((".png", ".jpg", ".jpeg"))))
        notes = _unique_notes([note for item in all_rows for note in item.get("limits", [])] + notes)
        sharp = results.get("sharpness", {})
        geom = results.get("geometry", {})
        row_metrics = {
            "sharpness": sharp.get("metrics", {}),
            "geometry": geom.get("metrics", {}),
        }
        comparison_cells = []
        for title, result in (("直线度", row_metrics["geometry"]), ("清晰度", row_metrics["sharpness"])):
            if result:
                comparison_cells.append(_cell(title, results.get("geometry" if title == "直线度" else "sharpness", {}).get("value", "未取得")))
        cells = comparison_cells[:4]
        summary = "同源HDMI图像对照" if all_rows else "未取得对照图"
        conclusion = "查看校正前、校正后与重复基线；它们不是传感器 RAW 图像。"
        title = TITLES["comparison"]
        details = {"metrics": row_metrics, "evidence": evidence}
        preferred = "before_after_comparison.png"
    else:
        cells = _cells(key, metrics)
        title = str(row.get("title") or TITLES[key])
        details = {"metrics": metrics, "evidence": evidence,
                   "record_detail": row.get("detail"), "record_value": row.get("value"),
                   "record_status": status}
        preferred = "sfr_curves.png" if key == "sharpness" else "before_after_comparison.png" if key == "comparison" else ""

    if not measured:
        cells = []

    if preferred:
        evidence.sort(key=lambda path: (0 if preferred.lower() in Path(path).name.lower() else 1, path))

    method = list(METHODS[key])
    interpretation = INTERPRETATIONS[key]
    if key == "latency" and metrics.get("per_angle"):
        method = ["FPGA用同一时钟记录输入帧开始、处理完成和对应帧首次输出的时间。",
                  "按帧ID配对后计算时间差，分角度统计；UART只传数据，不参与计时。"]
        interpretation = "P95：95%的有效样本不超过该延迟。范围：FPGA输入至HDMI首像素，不含曝光与显示器响应。"

    return {
        "key": key, "title": title, "status": status,
        "status_label": STATUS_LABELS.get(status, status), "summary": summary,
        "conclusion": conclusion, "method": method,
        "interpretation": interpretation, "cells": cells[:6],
        "notes": notes, "source_label": source, "timestamp": timestamp,
        "images": evidence, "preferred_image": preferred, "details": details,
    }
