"""Windows measurement GUI. Startup never connects or writes to hardware."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))

import argparse
import json
import queue
import threading
import time
import uuid
import zipfile
from datetime import datetime

from PySide6.QtCore import QThread,QTimer,Signal
from PySide6.QtGui import QImage,QDesktopServices,QFont,QFontDatabase,QPixmap
from PySide6.QtCore import QUrl
from PySide6.QtWidgets import (
    QApplication,QFileDialog,QDialog,QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,
    QComboBox,QPushButton,QPlainTextEdit,QToolButton,QWidget,QScrollArea,QFrame,
)
from PySide6.QtCore import Qt
from sw.quality_inspector_v1.view import InspectorWindow
from sw.quality_inspector_v1.engine import InspectorEngine
from sw.quality_inspector_v1.live import LiveAdapter
from sw.quality_inspector_v1.report import export_report,export_pdf
from sw.quality_inspector_v1.result_presentation import build_presentation

HERE=Path(__file__).resolve().parent
HISTORY=ROOT/'data/quality/sfr_20261101/run_pair_02/session.json'


class _Fold(QWidget):
    def __init__(self,title,text,parent=None):
        super().__init__(parent)
        layout=QVBoxLayout(self);layout.setContentsMargins(0,0,0,0);layout.setSpacing(3)
        self.button=QToolButton();self.button.setText(title);self.button.setCheckable(True)
        self.button.setChecked(False);self.button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.button.setArrowType(Qt.ArrowType.RightArrow);self.button.setStyleSheet('font-weight:600;text-align:left')
        self.body=QPlainTextEdit();self.body.setReadOnly(True);self.body.setPlainText(text)
        self.body.setMaximumHeight(130 if title=='测量说明' else 210);self.body.setVisible(False)
        self.button.toggled.connect(self._toggle)
        layout.addWidget(self.button);layout.addWidget(self.body)

    def _toggle(self,checked):
        self.body.setVisible(checked)
        self.button.setArrowType(Qt.ArrowType.DownArrow if checked else Qt.ArrowType.RightArrow)


class ResultDetailsDialog(QDialog):
    def __init__(self,parent,key,row,results=None):
        super().__init__(parent)
        self.presentation=build_presentation(key,row,results=results)
        data=self.presentation
        self.setWindowTitle(data['title']+' · 测量结果');self.resize(1000,740)
        outer=QVBoxLayout(self)
        scroll=QScrollArea();scroll.setWidgetResizable(True);outer.addWidget(scroll,1)
        body=QWidget();scroll.setWidget(body);layout=QVBoxLayout(body);layout.setSpacing(10)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        top=QHBoxLayout()
        source=QLabel(data['source_label']+(' · '+data['timestamp'] if data['timestamp'] else ''))
        source.setStyleSheet('color:#596579');top.addWidget(source);top.addStretch(1)
        status=QLabel(data['status_label']);status.setStyleSheet('font-weight:600;color:#146f68')
        top.addWidget(status);layout.addLayout(top)

        summary=QLabel(data['summary']);summary.setWordWrap(True)
        summary.setStyleSheet('font-size:19px;font-weight:700;padding:4px 0;color:#18324a')
        layout.addWidget(summary)

        if data['cells']:
            grid=QGridLayout();grid.setHorizontalSpacing(10);grid.setVerticalSpacing(8)
            for index,cell in enumerate(data['cells'][:6]):
                box=QFrame();box.setFrameShape(QFrame.Shape.StyledPanel)
                box.setStyleSheet('QFrame{background:#f5f8fa;border:1px solid #dbe4ea;border-radius:5px}')
                item=QVBoxLayout(box);item.setContentsMargins(10,7,10,7)
                label=QLabel(cell['label']);label.setStyleSheet('color:#596579;font-size:10pt;border:0;background:transparent')
                value=QLabel(cell['value']);value.setWordWrap(True);value.setStyleSheet('font-size:13pt;font-weight:600;color:#18324a;border:0;background:transparent')
                item.addWidget(label);item.addWidget(value)
                columns=2 if key=='latency' else 3
                grid.addWidget(box,index//columns,index%columns)
            layout.addLayout(grid)

        conclusion=QLabel(data['conclusion']);conclusion.setWordWrap(True)
        conclusion.setStyleSheet('font-size:11pt;font-weight:600;padding:3px 0')
        layout.addWidget(conclusion)
        method_title=QLabel('怎么测');method_title.setStyleSheet('font-weight:600')
        layout.addWidget(method_title)
        method=QLabel('\n'.join(data['method']));method.setWordWrap(True)
        method.setStyleSheet('color:#33485c;padding-left:4px')
        layout.addWidget(method)
        interpretation=QLabel(data['interpretation']);interpretation.setWordWrap(True)
        interpretation.setStyleSheet('color:#596579;padding-left:4px')
        layout.addWidget(interpretation)

        images=[Path(p) for p in data['images'] if Path(p).suffix.lower() in ('.png','.jpg','.jpeg') and Path(p).is_file()]
        self.image_combo=None;self.image_label=None
        if images:
            preferred=data.get('preferred_image','')
            images.sort(key=lambda p:(0 if preferred and preferred.lower() in p.name.lower() else 1,p.name))
            self.image_paths=images
            row_layout=QHBoxLayout();row_layout.addWidget(QLabel('图像 / 曲线'))
            self.image_combo=QComboBox();self.image_combo.addItems([p.name for p in images]);row_layout.addWidget(self.image_combo,1)
            layout.addLayout(row_layout)
            self.image_label=QLabel();self.image_label.setMinimumHeight(245)
            self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.image_label.setStyleSheet('background:#edf1f4;border:1px solid #dbe4ea')
            layout.addWidget(self.image_label)
            self.image_combo.currentIndexChanged.connect(self._show_image);self._show_image(0)
            open_button=QPushButton('打开所选原图 / 曲线')
            open_button.clicked.connect(lambda:QDesktopServices.openUrl(
                QUrl.fromLocalFile(str(self.image_paths[self.image_combo.currentIndex()]))))
            layout.addWidget(open_button,alignment=Qt.AlignmentFlag.AlignLeft)

        self.notes_fold=_Fold('测量说明','\n'.join(data['notes']) if data['notes'] else data['interpretation'])
        layout.addWidget(self.notes_fold)
        details=data['details']
        raw=json.dumps(details,ensure_ascii=False,indent=2)
        self.raw_fold=_Fold('原始数据与证据',raw)
        layout.addWidget(self.raw_fold)
        close=QPushButton('关闭');close.clicked.connect(self.accept);outer.addWidget(close,alignment=Qt.AlignmentFlag.AlignRight)

    def _show_image(self,index):
        if self.image_label is None or not self.image_paths:return
        path=self.image_paths[index];image=QImage(str(path))
        if image.isNull():
            import cv2
            array=cv2.imread(str(path))
            if array is not None:
                image=QImage(array.data,array.shape[1],array.shape[0],array.strides[0],
                             QImage.Format.Format_BGR888).copy()
        if not image.isNull():
            self.image_label.setPixmap(QPixmap.fromImage(image).scaled(
                930,330,Qt.AspectRatioMode.KeepAspectRatio,Qt.TransformationMode.SmoothTransformation))


class Worker(QThread):
    event=Signal(str,object)
    def __init__(self,workdir):
        super().__init__();self.workdir=Path(workdir);self.commands=queue.Queue();self.stop_requested=threading.Event()
        self.adapter=LiveAdapter();self.engine=None;self.previous=None

    def submit(self,command,*args):
        if command!='close':self.stop_requested.clear()
        self.commands.put((command,args))

    def run(self):
        self.engine=InspectorEngine(self.workdir,self.adapter,notify=self.event.emit)
        self.event.emit('session',self.engine.session)
        while True:
            try:command,args=self.commands.get(timeout=1.5)
            except queue.Empty:
                if self.engine.connected:
                    try:
                        sample=self.adapter.sample(include_caps=False);now=sample['monotonic'];a5=sample['a5']
                        message='串口诊断有效；点击单项按钮或一键测量。'
                        if self.previous is not None:
                            elapsed=now-self.previous['monotonic'];delta=(a5['resultFrameCommitCount']-self.previous['a5']['resultFrameCommitCount'])&0xffffffff
                            if 0<elapsed<5 and delta/elapsed<=240:message=f"实时结果提交 {delta/elapsed:.1f} FPS · 当前累计源丢帧 {a5.get('sourceFrameDropCount','未知')}"
                        self.previous=sample
                        self.event.emit('overview',{'message':message})
                    except Exception as exc:self.event.emit('overview',{'message':'诊断暂不可用；可断开后重连。'+str(exc)})
                continue
            if command=='close':
                self.engine.disconnect();break
            try:
                if command=='connect':self.engine.connect(*args)
                elif command=='disconnect':self.engine.disconnect();self.previous=None
                elif command=='measure':
                    if len(args)>2:self.engine.measure(args[0],args[1],self.stop_requested,args[2])
                    else:self.engine.measure(args[0],args[1],self.stop_requested)
                elif command=='history':
                    self.engine.import_history(*args)
                    raw=json.loads(Path(args[0]).read_text(encoding='utf-8'))
                    if 'stages' in raw:
                        image_path=raw['stages']['calibrated']['images'][len(raw['stages']['calibrated']['images'])//2]
                        self.event.emit('history_image',image_path)
                    else:
                        evidence=[p for r in self.engine.session['results'].values() for p in r.get('evidence',[]) if str(p).lower().endswith('.png')]
                        if evidence:self.event.emit('history_image',evidence[0])
                    self.event.emit('overview',{'message':'历史记录已加载；不代表当前实时状态。'})
                elif command=='resources':self.engine.import_resources(*args)
                elif command=='latency':self.engine.import_latency(*args)
                elif command=='export':
                    out=export_report(self.engine.session,Path(args[0]))
                    try:
                        export_pdf(self.engine.session,out/'report.pdf')
                        with zipfile.ZipFile(out/'report.zip','a',zipfile.ZIP_DEFLATED) as z:z.write(out/'report.pdf','report.pdf')
                    except Exception as exc:self.event.emit('log','PDF未生成；HTML、CSV、JSON与ZIP仍可用：'+str(exc))
                    self.event.emit('exported',str(out))
                self.event.emit('session',self.engine.session)
            except Exception as exc:self.event.emit('error',str(exc))
            finally:self.event.emit('idle',None)


class Window(InspectorWindow):
    def __init__(self,workdir):
        super().__init__();self._closing=False;self._can_close=False;self._frame_id=None
        self.worker=Worker(workdir);self.worker.event.connect(self.on_event)
        self.worker.finished.connect(self.safe_closed)
        self.connect_requested.connect(lambda port,device:self.command('connect',port.strip(),device.strip()))
        self.disconnect_requested.connect(lambda:self.command('disconnect'))
        self.run_requested.connect(self.measure)
        self.stop_requested.connect(self.stop)
        self.open_requested.connect(self.open_history)
        self.resources_requested.connect(self.open_resources)
        self.latency_requested.connect(self.open_latency)
        self.latency_help_requested.connect(self.show_latency_guide)
        self.export_requested.connect(self.export)
        self.details_requested.connect(self.show_details)
        self.worker.start();self.preview_timer=QTimer(self);self.preview_timer.timeout.connect(self.paint_preview);self.preview_timer.start(33)

    def command(self,name,*args):
        if self._busy:return
        self.set_busy(True,{'connect':'正在连接；不会修改板卡参数','history':'正在核对原始证据并计算曲线','export':'正在打包测量报告'}.get(name,'正在处理，请等待'))
        self.worker.submit(name,*args)

    def measure(self,keys):
        if not keys:self.set_overview({'message':'请至少勾选一项，或点击右侧单项测量。'});return
        self.command('measure',list(keys),self.duration_seconds(),int(self.latency_samples_combo.currentText()))

    def stop(self):
        self.worker.stop_requested.set();self.set_overview({'message':'正在停止；如已切换配置，先完成安全恢复，请勿拔线。'})

    def on_event(self,event,value):
        if event=='item':self.set_item(*value)
        elif event=='connected':self.set_connected(*value)
        elif event=='overview':self.set_overview(value)
        elif event=='log':self.append_log(str(value));self.set_overview({'message':value})
        elif event=='session':
            self.set_session(value)
            if any(v.get('source')=='historical' for v in value.get('results',{}).values()):
                self.connection_label.setText(('已连接 · ' if self._connected else '未连接 · ')+'含历史测量；实时与历史结果分开标记')
        elif event=='error':self.append_log('操作未完成：'+str(value));self.set_overview({'message':'操作未完成：'+str(value)})
        elif event=='idle':
            if not self._closing:self.set_busy(False)
        elif event=='exported':
            self.set_overview({'message':'报告已保存：'+value});self.append_log('报告目录：'+value)
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(value)/'index.html')))
        elif event=='history_image':
            self.preview_title.setText('历史采集画面（非当前实时）')
            self.preview_label.setText('历史采集图');image=QImage(str(value))
            if image.isNull():
                import cv2
                array=cv2.imread(str(value))
                if array is not None:image=QImage(array.data,array.shape[1],array.shape[0],array.strides[0],QImage.Format.Format_BGR888).copy()
            if not image.isNull():self.set_preview(image)

    def paint_preview(self):
        preview=self.worker.adapter.preview
        if preview is None:return
        latest=preview.latest_with_identity()
        if latest is None:
            state=preview.status()
            if state['state'] in ('stale','error'):
                self.preview_title.setText('HDMI画面未更新；最后图像仅供参考')
                error=state.get('error')
                if error and error!=getattr(self,'_preview_error',None) and not self._busy:
                    self._preview_error=error;self.append_log('预览不可用：'+error)
                    self.set_overview({'message':'未取得HDMI画面，请关闭其他相机预览或检查采集卡；串口测量可独立进行。'})
            return
        generation,sequence,stamp,array=latest
        if (generation,sequence)==self._frame_id:return
        self._frame_id=(generation,sequence)
        self.preview_title.setText('实时画面 · HDMI预览（处理帧率见右侧测量）')
        image=QImage(array.data,array.shape[1],array.shape[0],array.strides[0],QImage.Format.Format_BGR888).copy()
        self.set_preview(image)

    def open_history(self):
        path,_=QFileDialog.getOpenFileName(self,'打开测量记录（历史数据）',str(HISTORY),'测量记录 (*.json)')
        if path:self.command('history',path)

    def open_resources(self):
        base=ROOT/'wanchengceshi/algorithm_v04_1080p_upgrade/algorithm1080_nn_r1/build_revision15_gamma/prj_tasks/pnr_1'
        paths,_=QFileDialog.getOpenFileNames(self,'导入同一版本PDS资源/时序报告',str(base),'PDS报告 (*.rur *.rtr)')
        if paths:self.command('resources',[Path(p) for p in paths])

    def open_latency(self):
        path,_=QFileDialog.getOpenFileName(self,'导入真实硬件延迟CSV：latency_ms,method',str(HERE),'CSV (*.csv)')
        if path:self.command('latency',Path(path))

    def export(self):
        folder=QFileDialog.getExistingDirectory(self,'选择报告保存位置',str(ROOT/'data/quality'))
        if folder:self.command('export',str(Path(folder)/('quality_report_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:4])))

    def show_latency_guide(self):
        dialog=QDialog(self);dialog.setWindowTitle('端到端延迟：测量方法与边界');dialog.resize(900,700)
        layout=QVBoxLayout(dialog);text=QPlainTextEdit();text.setReadOnly(True)
        text.setPlainText((Path(__file__).parent/'LATENCY_GUIDE.md').read_text(encoding='utf-8'));layout.addWidget(text)
        close=QPushButton('关闭');close.clicked.connect(dialog.accept);layout.addWidget(close);dialog.exec()

    def show_details(self,key):
        rows=self._session.get('results') or {}
        row=rows.get(key,{})
        dialog=ResultDetailsDialog(self,key,row,results=rows)
        dialog.exec()

    def closeEvent(self,event):
        if self._can_close:event.accept();return
        event.ignore()
        if not self._closing:
            self._closing=True;self.stop();self.set_busy(True,'正在安全退出并释放自己打开的采集进程。')
            self.worker.submit('close')

    def safe_closed(self):
        self._can_close=True;self.preview_timer.stop();self.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workdir',type=Path,default=ROOT/'data/quality/inspector_sessions')
    parser.add_argument('--history',type=Path,help='Explicitly open an existing capture; no hardware')
    parser.add_argument('--screenshot',type=Path)
    parser.add_argument('--exit-after',type=float,default=0,help='UI QA only; cooperative close')
    args=parser.parse_args()
    app=QApplication(sys.argv[:1]);app.setStyle('Fusion');app.setApplicationName('FPGA画质检测')
    for name in ('msyh.ttc','msyhbd.ttc','arial.ttf'):
        font=Path('C:/Windows/Fonts')/name
        if font.is_file():QFontDatabase.addApplicationFont(str(font))
    app.setFont(QFont('Microsoft YaHei',10))
    window=Window(args.workdir);window.show()
    if args.history:QTimer.singleShot(100,lambda:window.command('history',str(args.history.resolve())))
    if args.screenshot:
        def shot():
            args.screenshot.parent.mkdir(parents=True,exist_ok=True);window.grab().save(str(args.screenshot))
        QTimer.singleShot(max(1000,int(args.exit_after*1000)-1000) if args.exit_after else 3000,shot)
    if args.exit_after:QTimer.singleShot(int(args.exit_after*1000),window.close)
    return app.exec()


if __name__=='__main__':raise SystemExit(main())
