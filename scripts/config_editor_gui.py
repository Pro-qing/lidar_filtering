#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import re
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTabWidget, QLabel, QPushButton, QFormLayout, 
                             QSpinBox, QDoubleSpinBox, QCheckBox, QScrollArea, 
                             QMessageBox, QTextEdit, QLineEdit, QGroupBox, QGridLayout)
from PyQt5.QtGui import QPainter, QColor, QPen, QPolygonF, QFont
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QObject

# 使用 ruamel.yaml 保留原文件的所有注释和缩进格式
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO
from ruamel.yaml.comments import CommentedMap  # 强制生成单行字典格式

# ================= ROS 相关导入 =================
HAS_ROS = False
try:
    import rospy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2
    HAS_ROS = True
except ImportError:
    print("未检测到 rospy 或 sensor_msgs，点云显示功能将被禁用。")

# =====================================================================
# 参数显示映射表
# =====================================================================

ALLOWED_YAML_PARAMS = {
    "crop_radius": "最大过滤范围 (米)",
    "crop_radius_x": "X轴前后误差补偿 (米)",
    "height_min": "最小高度过滤阈值 (米)",
    "voxel_filter": "体素滤波下采样精度 (米)",
    "filter_floor": "是否开启地面过滤",
    "rect": "车辆本体轮廓", 
    "charge_enble": "是否开启充电桩过滤",
    "charge_length": "充电桩长度 (米)"
}

ALLOWED_SAFE_YAML_PARAMS = {
    "max_longitudinal_scale": "前后方向最大缩放比例",
    "min_longitudinal_scale": "前后方向最小缩放比例",
    "max_lateral_scale": "左右方向最大缩放比例",
    "min_lateral_scale": "左右方向最小缩放比例",
    "longitudinal_sensitivity": "纵向灵敏度 (前后方向)",
    "lateral_sensitivity": "横向灵敏度 (左右方向)",
    "reference_speed": "参考速度 (m/s)",
    "exigencyrect_1": "常规尺寸_急停框 (前进)",
    "reverse_exigencyrect_1": "常规尺寸_急停框 (后退)",
    "exigencyrect_2": "窄道尺寸_急停框 (前进)",
    "reverse_exigencyrect_2": "窄道尺寸_急停框 (后退)",
    "slowrect_1": "常规尺寸_减速框 (前进)",
    "reverse_slowrect_1": "常规尺寸_减速框 (后退)",
    "slowrect_2": "窄道尺寸_减速框 (前进)",
    "reverse_slowrect_2": "窄道尺寸_减速框 (后退)"
}

ALLOWED_CFG_PARAMS = {
    "maxSpeed": "最大运行速度限制 (m/s)",
    "leftLimit_min_angle": "左侧雷达限制最小角度 (度)",
    "leftLimit_max_angle": "左侧雷达限制最大角度 (度)",
    "crop_radius": "动态最大过滤范围 (米)",
    "height_max": "点云最高裁剪高度 (米)",
    "voxel_filter": "动态体素滤波精度 (米)",
    "charge_enble": "动态充电桩过滤开关",
    "consistency_enable": "是否开启双雷达一致性校验"
}

def safe_float(val, default=0.0):
    if val is None: return default
    try: return float(val)
    except (ValueError, TypeError): return default

def create_flow_point(x, y):
    pt = CommentedMap()
    pt['x'] = x
    pt['y'] = y
    pt.fa.set_flow_style()
    return pt

# =====================================================================
# UI 样式库 (QSS)
# =====================================================================
GLOBAL_STYLE = """
QWidget {
    font-family: "Segoe UI", "Microsoft YaHei", Arial, sans-serif;
    font-size: 14px;
    color: #333333;
    background-color: #F5F7FA;
}
QTabWidget::pane {
    border: 1px solid #E0E0E0;
    background-color: #FFFFFF;
    border-radius: 8px;
    margin-top: -1px;
}
QTabBar::tab {
    background-color: #E0E0E0;
    color: #666666;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 10px 20px;
    margin-right: 2px;
    font-weight: bold;
}
QTabBar::tab:selected {
    background-color: #FFFFFF;
    color: #1976D2;
    border: 1px solid #E0E0E0;
    border-bottom: 2px solid #1976D2;
}
QGroupBox {
    background-color: #FFFFFF;
    border: 1px solid #D6D6D6;
    border-radius: 6px;
    margin-top: 18px;
    padding-top: 15px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 15px;
    padding: 0 5px;
    color: #1976D2;
    font-weight: bold;
    font-size: 15px;
}
QPushButton {
    border-radius: 5px;
    padding: 8px 15px;
    font-weight: bold;
    border: none;
}
QPushButton:hover { opacity: 0.8; }
QPushButton:pressed { background-color: rgba(0, 0, 0, 0.1); }
QDoubleSpinBox, QSpinBox, QLineEdit {
    border: 1px solid #BDBDBD;
    border-radius: 4px;
    padding: 5px;
    background-color: #FAFAFA;
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {
    border: 1px solid #1976D2;
    background-color: #FFFFFF;
}
QScrollArea { border: none; background-color: transparent; }
QScrollArea > QWidget > QWidget { background-color: transparent; }
"""

# =====================================================================
# ROS 点云监听
# =====================================================================
class ROSListener(QObject):
    cloud_updated = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        if HAS_ROS:
            self.sub = rospy.Subscriber("/points_filter", PointCloud2, self.pc_callback, queue_size=1)

    def pc_callback(self, msg):
        points = []
        try:
            gen = pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
            for i, p in enumerate(gen):
                if i % 3 == 0:
                    x, y = p[0], p[1]
                    if -15.0 < x < 15.0 and -15.0 < y < 15.0:
                        points.append((x, y))
            self.cloud_updated.emit(points)
        except Exception:
            pass

# =====================================================================
# 自定义画板
# =====================================================================
class VehiclePreviewWidget(QWidget):
    def __init__(self, color_r=0, color_g=255, color_b=255):
        super().__init__()
        self.points = []       
        self.ref_points = []   
        self.cloud_points = []  
        self.setMinimumSize(320, 320)
        self.fill_color = QColor(color_r, color_g, color_b, 60)
        self.line_color = QColor(color_r, color_g, color_b)

    def set_points(self, points):
        self.points = points
        self.update()

    def set_reference_points(self, points):
        self.ref_points = points
        self.update()
        
    def set_cloud_points(self, points):
        self.cloud_points = points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#121212"))

            w, h = self.width(), self.height()
            cx, cy = w / 2, h / 2

            max_range = 1.0 
            all_pts = []
            if isinstance(self.points, list): all_pts += self.points
            if isinstance(self.ref_points, list): all_pts += self.ref_points
            
            for p in all_pts:
                if isinstance(p, dict):
                    max_range = max(max_range, abs(safe_float(p.get('x'))), abs(safe_float(p.get('y'))))
            scale = (min(w, h) / 2.0) / (max_range * 1.25) if max_range > 0 else 1.0

            if self.cloud_points:
                painter.setPen(QPen(QColor(255, 255, 255, 140), 2))
                cloud_poly = QPolygonF()
                for cx_pt, cy_pt in self.cloud_points:
                    qx = cx - (cy_pt * scale)
                    qy = cy - (cx_pt * scale)
                    cloud_poly.append(QPointF(qx, qy))
                painter.drawPoints(cloud_poly)

            painter.setPen(QPen(QColor(255, 80, 80, 150), 1, Qt.DashLine))
            painter.drawLine(int(cx), 0, int(cx), int(h)) 
            painter.setPen(QPen(QColor(80, 255, 80, 150), 1, Qt.DashLine))
            painter.drawLine(0, int(cy), int(w), int(cy)) 

            painter.setPen(QPen(QColor(200, 200, 200)))
            painter.setFont(QFont("Microsoft YaHei", 8))
            painter.drawText(int(cx) + 5, 15, "X(前)")
            painter.drawText(5, int(cy) - 5, "Y(左)")

            painter.setBrush(QColor(255, 255, 255, 50))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(cx, cy), 10, 10)
            painter.setBrush(QColor(255, 255, 255))
            painter.drawEllipse(QPointF(cx, cy), 4, 4)
            
            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.setFont(QFont("Arial", 9, QFont.Bold))
            painter.drawText(int(cx) + 12, int(cy) - 12, "velodyne")

            def build_poly(pt_list):
                poly = QPolygonF()
                valid = []
                for i, p in enumerate(pt_list):
                    if isinstance(p, dict) and 'x' in p and 'y' in p:
                        rx, ry = safe_float(p['x']), safe_float(p['y'])
                        qx = cx - (ry * scale)
                        qy = cy - (rx * scale)
                        pt = QPointF(qx, qy)
                        poly.append(pt)
                        valid.append((pt, i))
                return poly, valid

            if self.ref_points:
                ref_poly, _ = build_poly(self.ref_points)
                painter.setPen(QPen(QColor(0, 255, 255, 150), 2))
                painter.setBrush(QColor(0, 255, 255, 30))
                painter.drawPolygon(ref_poly)

            if self.points and isinstance(self.points, list):
                main_poly, valid_pts = build_poly(self.points)
                painter.setPen(QPen(self.line_color, 2))
                painter.setBrush(self.fill_color)
                painter.drawPolygon(main_poly)

                painter.setFont(QFont("Arial", 9, QFont.Bold))
                for pt, idx in valid_pts:
                    painter.setBrush(QColor(255, 255, 0))
                    painter.setPen(Qt.NoPen)
                    painter.drawEllipse(pt, 5, 5)
                    painter.setPen(QPen(QColor(255, 255, 255)))
                    painter.drawText(int(pt.x()) + 8, int(pt.y()) - 8, f"P{idx+1}")
        except Exception:
            pass

# =====================================================================
# 统一动态多边形编辑器组件 (用于车体 & 安全框)
# =====================================================================
class PolygonEditor(QWidget):
    def __init__(self, init_points, raw_key, yaml_data_source, hidden_text_edit, preview_widget):
        super().__init__()
        self.points = []
        for p in init_points:
            self.points.append({'x': safe_float(p.get('x')), 'y': safe_float(p.get('y'))})
            
        self.raw_key = raw_key
        self.yaml_data_source = yaml_data_source
        self.hidden_text_edit = hidden_text_edit
        self.preview = preview_widget
        
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        adj_group = QGroupBox("快捷整体扩缩 (正数向外扩大，负数向内缩小)")
        adj_layout = QGridLayout(adj_group)
        
        self.sb_adj_f = QDoubleSpinBox(); self.sb_adj_f.setRange(-10, 10); self.sb_adj_f.setSingleStep(0.05)
        self.sb_adj_b = QDoubleSpinBox(); self.sb_adj_b.setRange(-10, 10); self.sb_adj_b.setSingleStep(0.05)
        self.sb_adj_l = QDoubleSpinBox(); self.sb_adj_l.setRange(-10, 10); self.sb_adj_l.setSingleStep(0.05)
        self.sb_adj_r = QDoubleSpinBox(); self.sb_adj_r.setRange(-10, 10); self.sb_adj_r.setSingleStep(0.05)
        
        adj_layout.addWidget(QLabel("向前:"), 0, 0); adj_layout.addWidget(self.sb_adj_f, 0, 1)
        adj_layout.addWidget(QLabel("向后:"), 0, 2); adj_layout.addWidget(self.sb_adj_b, 0, 3)
        adj_layout.addWidget(QLabel("向左:"), 1, 0); adj_layout.addWidget(self.sb_adj_l, 1, 1)
        adj_layout.addWidget(QLabel("向右:"), 1, 2); adj_layout.addWidget(self.sb_adj_r, 1, 3)
        
        btn_apply = QPushButton("一键应用")
        btn_apply.setStyleSheet("background-color: #673AB7; color: white;")
        btn_apply.clicked.connect(self.apply_bulk_adjustment)
        adj_layout.addWidget(btn_apply, 0, 4, 2, 1)
        self.layout.addWidget(adj_group)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.points_container = QWidget()
        self.points_layout = QGridLayout(self.points_container)
        self.points_layout.setContentsMargins(5, 5, 5, 5)
        self.points_layout.setHorizontalSpacing(10)
        self.scroll.setWidget(self.points_container)
        self.layout.addWidget(self.scroll)
        
        self.add_btn = QPushButton("+ 添加点位")
        self.add_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.add_btn.clicked.connect(self.add_point)
        self.layout.addWidget(self.add_btn)
        
        self.spin_pairs = []
        self.render_points()

    def apply_bulk_adjustment(self):
        df, db, dl, dr = self.sb_adj_f.value(), self.sb_adj_b.value(), self.sb_adj_l.value(), self.sb_adj_r.value()
        for sx, sy in self.spin_pairs:
            x, y = sx.value(), sy.value()
            if x > 0.01: x += df
            elif x < -0.01: x -= db 
            if y > 0.01: y += dl
            elif y < -0.01: y -= dr
            sx.setValue(x); sy.setValue(y)
        self.sb_adj_f.setValue(0); self.sb_adj_b.setValue(0); self.sb_adj_l.setValue(0); self.sb_adj_r.setValue(0)

    def render_points(self):
        for i in reversed(range(self.points_layout.count())): 
            w = self.points_layout.itemAt(i).widget()
            if w: w.setParent(None)
        self.spin_pairs.clear()
        
        for i, pt in enumerate(self.points):
            lbl_p = QLabel(f"P{i+1}:")
            lbl_p.setStyleSheet("color: #D32F2F; font-weight: bold; font-size: 15px;")
            
            sx = QDoubleSpinBox(); sx.setRange(-50.0, 50.0); sx.setDecimals(2); sx.setSingleStep(0.05)
            sx.setValue(pt['x'])
            sy = QDoubleSpinBox(); sy.setRange(-50.0, 50.0); sy.setDecimals(2); sy.setSingleStep(0.05)
            sy.setValue(pt['y'])
            
            del_btn = QPushButton("X")
            del_btn.setFixedSize(30, 30)
            del_btn.setStyleSheet("background-color: #E53935; color: white; font-weight: bold; border-radius: 15px;")
            del_btn.clicked.connect(lambda chk, idx=i: self.remove_point(idx))
            
            sx.valueChanged.connect(self.on_value_changed)
            sy.valueChanged.connect(self.on_value_changed)
            
            self.points_layout.addWidget(lbl_p, i, 0)
            self.points_layout.addWidget(QLabel("X:"), i, 1)
            self.points_layout.addWidget(sx, i, 2)
            self.points_layout.addWidget(QLabel("Y:"), i, 3)
            self.points_layout.addWidget(sy, i, 4)
            self.points_layout.addWidget(del_btn, i, 5)
            
            self.spin_pairs.append((sx, sy))
            
        self.update_output()

    def add_point(self):
        new_pt = self.points[-1].copy() if self.points else {'x': 0.0, 'y': 0.0}
        self.points.append(new_pt)
        self.render_points()

    def remove_point(self, idx):
        if len(self.points) > 1:
            self.points.pop(idx)
            self.render_points()
        else:
            QMessageBox.warning(self, "警告", "至少需要保留一个点！")

    def on_value_changed(self):
        for i, (sx, sy) in enumerate(self.spin_pairs):
            self.points[i]['x'] = round(sx.value(), 2)
            self.points[i]['y'] = round(sy.value(), 2)
        self.update_output()
        
    def update_output(self):
        self.preview.set_points(self.points)
        flow_pts = []
        for p in self.points:
            flow_pts.append(create_flow_point(p['x'], p['y']))
            
        self.yaml_data_source[self.raw_key] = flow_pts
        b = StringIO(); YAML().dump(flow_pts, b)
        self.hidden_text_edit.setPlainText(b.getvalue())

# =====================================================================
# 主界面
# =====================================================================
class ConfigEditorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.yaml_path = '/home/getq/work/workspace/config/perception/vehicle_size.yaml'
        self.safe_yaml_path = '/home/getq/work/workspace/config/perception/safe_obstacle.yaml'
        self.cfg_path = '/home/getq/work/autoware.ai/install/lidar_filtering/share/lidar_filtering/cfg/LidarFiltering.cfg'
        
        self.yaml_parser = YAML(); self.yaml_parser.preserve_quotes = True
        self.yaml_data = None; self.safe_yaml_data = None
        self.previews_basic = []; self.previews_safe = []

        if HAS_ROS:
            try:
                rospy.init_node('config_editor_node', anonymous=True, disable_signals=True)
                self.ros_listener = ROSListener()
                self.ros_listener.cloud_updated.connect(self.dispatch_cloud)
            except Exception as e:
                print("ROS 节点启动失败:", e)

        self.init_ui()

    def dispatch_cloud(self, points):
        for p in self.previews_basic + self.previews_safe:
            try: p.set_cloud_points(points)
            except: pass

    def init_ui(self):
        self.setWindowTitle("Lidar Filtering 可视化调参工具")
        self.resize(1000, 950)
        self.setStyleSheet(GLOBAL_STYLE)
        
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        self.tab_yaml = QWidget(); self.setup_yaml_tab(); self.tabs.addTab(self.tab_yaml, "常规车辆参数")
        self.tab_safe = QWidget(); self.setup_safe_tab(); self.tabs.addTab(self.tab_safe, "安全检测框设置")
        self.tab_cfg  = QWidget(); self.setup_cfg_tab();  self.tabs.addTab(self.tab_cfg,  "高级动态配置")
        
        layout.addWidget(self.tabs)

    def save_generic_yaml(self, widgets_dict, data_source, path):
        for k, w in widgets_dict.items():
            if isinstance(w, QCheckBox): data_source[k] = w.isChecked()
            elif isinstance(w, QDoubleSpinBox): data_source[k] = w.value()
            elif isinstance(w, QTextEdit):
                try: data_source[k] = YAML().load(w.toPlainText())
                except: return False
        with open(path, 'w', encoding='utf-8') as f: self.yaml_parser.dump(data_source, f)
        return True

    def setup_yaml_tab(self):
        l = QVBoxLayout(self.tab_yaml)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_yaml)
        br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("💾 保存修改"); bs.clicked.connect(self.save_yaml)
        bs.setStyleSheet("background-color: #4CAF50; color: white;")
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)
        self.scroll_y = QScrollArea(); self.scroll_y.setWidgetResizable(True)
        self.yaml_form_widget = QWidget(); self.yaml_form_layout = QFormLayout(self.yaml_form_widget)
        self.scroll_y.setWidget(self.yaml_form_widget); l.addWidget(self.scroll_y)
        self.load_yaml()

    def load_yaml(self):
        if not os.path.exists(self.yaml_path): return
        with open(self.yaml_path, 'r', encoding='utf-8') as f: self.yaml_data = self.yaml_parser.load(f)
        for i in reversed(range(self.yaml_form_layout.count())):
            w = self.yaml_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
        self.previews_basic = []; self.yaml_widgets = {}
        
        for k, lbl in ALLOWED_YAML_PARAMS.items():
            if k not in self.yaml_data: continue
            val = self.yaml_data[k]
            
            if k == "rect":
                # ============== 车辆本体，统一替换为点位编辑器 ==============
                box = QGroupBox(lbl)
                bl = QHBoxLayout(box)
                te = QTextEdit(); te.hide(); self.yaml_widgets[k] = te
                
                pv = VehiclePreviewWidget(0, 255, 255)
                pv._tag = 'yaml'
                self.previews_basic.append(pv)
                
                ed = PolygonEditor(val, k, self.yaml_data, te, pv)
                bl.addWidget(ed, stretch=1)
                bl.addWidget(pv, stretch=1)
                self.yaml_form_layout.addRow(box)
            else:
                label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
                if isinstance(val, bool): w = QCheckBox(); w.setChecked(val)
                else: w = QDoubleSpinBox(); w.setRange(-1000, 1000); w.setDecimals(2); w.setValue(val)
                self.yaml_widgets[k] = w; self.yaml_form_layout.addRow(label, w)

    def save_yaml(self):
        if self.save_generic_yaml(self.yaml_widgets, self.yaml_data, self.yaml_path):
            QMessageBox.information(self, "提示", "常规车辆参数填写成功,请重启系统。")

    def setup_safe_tab(self):
        l = QVBoxLayout(self.tab_safe)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_safe_yaml)
        br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("💾 保存安全框修改"); bs.clicked.connect(self.save_safe_yaml); bs.setStyleSheet("background-color: #FF9800; color: white;")
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)
        self.scroll_s = QScrollArea(); self.scroll_s.setWidgetResizable(True)
        self.safe_form_widget = QWidget(); self.safe_form_layout = QFormLayout(self.safe_form_widget)
        self.scroll_s.setWidget(self.safe_form_widget); l.addWidget(self.scroll_s)
        self.load_safe_yaml()

    def load_safe_yaml(self):
        if not os.path.exists(self.safe_yaml_path): return
        with open(self.safe_yaml_path, 'r', encoding='utf-8') as f: self.safe_yaml_data = self.yaml_parser.load(f)
        for i in reversed(range(self.safe_form_layout.count())):
            w = self.safe_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
        self.previews_safe = []; self.safe_widgets = {}
        
        # 提取车辆本体作为参考
        ref = self.yaml_data.get('rect', []) if self.yaml_data else []
        
        for k, lbl in ALLOWED_SAFE_YAML_PARAMS.items():
            if k not in self.safe_yaml_data: continue
            val = self.safe_yaml_data[k]
            if isinstance(val, list):
                box = QGroupBox(lbl)
                bl = QHBoxLayout(box); t = QTextEdit(); t.hide(); self.safe_widgets[k] = t
                pv = VehiclePreviewWidget(255, 150, 0); pv.set_reference_points(ref); pv._tag = 'safe'
                self.previews_safe.append(pv)
                ed = PolygonEditor(val, k, self.safe_yaml_data, t, pv)
                bl.addWidget(ed, stretch=1); bl.addWidget(pv, stretch=1)
                self.safe_form_layout.addRow(box)
            else:
                label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
                w = QDoubleSpinBox(); w.setRange(-100, 100); w.setDecimals(2); w.setValue(val)
                self.safe_widgets[k] = w; self.safe_form_layout.addRow(label, w)

    def save_safe_yaml(self):
        if self.save_generic_yaml(self.safe_widgets, self.safe_yaml_data, self.safe_yaml_path):
            QMessageBox.information(self, "提示", "安全检测框填写成功,请重启系统。")

    def setup_cfg_tab(self):
        l = QVBoxLayout(self.tab_cfg)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_cfg)
        br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("⚡ 保存动态修改"); bs.clicked.connect(self.save_cfg); bs.setStyleSheet("background-color: #f44336; color: white;")
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)
        self.scroll_c = QScrollArea(); self.scroll_c.setWidgetResizable(True)
        self.cfg_form_widget = QWidget(); self.cfg_form_layout = QFormLayout(self.cfg_form_widget)
        self.scroll_c.setWidget(self.cfg_form_widget); l.addWidget(self.scroll_c)
        self.load_cfg()

    def load_cfg(self):
        if not os.path.exists(self.cfg_path): return
        with open(self.cfg_path, 'r', encoding='utf-8') as f: self.cfg_lines = f.readlines()
        for i in reversed(range(self.cfg_form_layout.count())):
            w = self.cfg_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
        self.cfg_widgets = {}; self.cfg_meta = {}
        for idx, line in enumerate(self.cfg_lines):
            if "gen.add" not in line or line.strip().startswith("#"): continue
            s = line.find('(')+1; e = line.rfind(')')
            p = [x.strip() for x in re.split(r',\s*(?=(?:[^"]*"[^"]*")*[^"]*$)', line[s:e])]
            if len(p) >= 5:
                name = p[0].strip('"\'')
                self.cfg_meta[name] = {'idx':idx, 'type':p[1], 'parts':p, 'prefix':line[:s]}
        for k, lbl in ALLOWED_CFG_PARAMS.items():
            if k not in self.cfg_meta: continue
            m = self.cfg_meta[k]; pt, dv = m['type'], m['parts'][4]
            label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
            if pt == 'bool_t': w = QCheckBox(); w.setChecked(dv == "True")
            else:
                w = QDoubleSpinBox(); w.setDecimals(2); w.setValue(float(dv))
                w.setRange(float(m['parts'][5]) if len(m['parts'])>5 else -100, float(m['parts'][6]) if len(m['parts'])>6 else 100)
            self.cfg_widgets[k] = w; self.cfg_form_layout.addRow(label, w)

    def save_cfg(self):
        for k, w in self.cfg_widgets.items():
            m = self.cfg_meta[k]; p = m['parts'].copy()
            p[4] = "True" if (isinstance(w, QCheckBox) and w.isChecked()) else ("False" if isinstance(w, QCheckBox) else str(round(w.value(), 2)))
            self.cfg_lines[m['idx']] = f"{m['prefix']}{', '.join(p)})\n"
        with open(self.cfg_path, 'w', encoding='utf-8') as f: f.writelines(self.cfg_lines)
        QMessageBox.information(self, "提示", "动态配置修改成功,请重启系统。")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ConfigEditorGUI()
    gui.show()
    
    if HAS_ROS:
        app.aboutToQuit.connect(rospy.signal_shutdown)
        
    sys.exit(app.exec_())