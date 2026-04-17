#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import re
import math
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QTabWidget, QLabel, QPushButton, QFormLayout, 
                             QSpinBox, QDoubleSpinBox, QCheckBox, QScrollArea, 
                             QMessageBox, QTextEdit, QLineEdit, QGroupBox, QGridLayout)
from PyQt5.QtGui import QPainter, QColor, QPen, QPolygonF, QFont
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QObject

# 使用 ruamel.yaml 保留原文件的所有注释和缩进格式
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO
from ruamel.yaml.comments import CommentedMap

# ================= ROS 相关导入 =================
HAS_ROS = False
try:
    import rospy
    import std_msgs.msg
    from sensor_msgs.msg import PointCloud2, LaserScan
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
    "charge_length": "充电桩长度 (米)",
    "charge_wide": "充电桩宽度 (米)",
    "charge_high": "充电桩高度 (米)",
    "charge_error": "与车头方向的误差距离"
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
    "top_min_angle": "补盲雷达限制最小角度 (度)",
    "top_max_angle": "补盲雷达限制最大角度 (度)",
    "left_filter_enable": "是否开启左侧雷达过滤",
    "left_min_angle": "左侧雷达限制最小角度 (度)",
    "left_max_angle": "左侧雷达限制最大角度 (度)",
    "left_max_dis": "左侧雷达最大距离",
    "right_filter_enable": "是否开启右侧雷达过滤",
    "right_min_angle": "右侧雷达限制最小角度 (度)",
    "right_max_angle": "右侧雷达限制最大角度 (度)",
    "right_max_dis": "右侧雷达最大距离",
    "consistency_enable": "是否开启双雷达一致性校验"
}

# 标定参数分组配置 (映射到话题和发布节点)
CALIB_GROUPS = {
    "main":   {"name": "主雷达 (预留)", "raw_topic": "", "calib_topic": ""},
    "top":    {"name": "补盲雷达", "raw_topic": "/points_mid", "calib_topic": "/points_mid_calibration"},
    "left":   {"name": "左前单线", "raw_topic": "/scan_left", "calib_topic": "/points_left_calibration"},
    "right":  {"name": "右前单线", "raw_topic": "/scan_right", "calib_topic": "/points_right_calibration"},
    "bleft":  {"name": "左后单线", "raw_topic": "/scan_bleft", "calib_topic": "/points_bleft_calibration"},
    "bright": {"name": "右后单线", "raw_topic": "/scan_bright", "calib_topic": "/points_bright_calibration"}
}

def safe_float(val, default=0.0):
    if val is None: return default
    try: return float(val)
    except (ValueError, TypeError): return default

def create_flow_point(x, y):
    pt = CommentedMap()
    pt['x'] = x; pt['y'] = y
    pt.fa.set_flow_style()
    return pt

# =====================================================================
# 3D 坐标变换算法 (用于 Python 端实时预览和发布)
# =====================================================================
def apply_tf_3d(points_3d, tx, ty, tz, yaw, pitch, roll):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    
    # 标准欧拉角 ZYX 旋转矩阵
    R11, R12, R13 = cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr
    R21, R22, R23 = sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr
    R31, R32, R33 = -sp, cp*sr, cp*cr
    
    res = []
    for x, y, z in points_3d:
        nx = R11*x + R12*y + R13*z + tx
        ny = R21*x + R22*y + R23*z + ty
        nz = R31*x + R32*y + R33*z + tz
        res.append((nx, ny, nz))
    return res

# =====================================================================
# UI 样式库
# =====================================================================
GLOBAL_STYLE = """
QWidget { font-family: "Segoe UI", Arial, sans-serif; font-size: 14px; color: #333; background-color: #F5F7FA; }
QTabWidget::pane { border: 1px solid #E0E0E0; background-color: #FFFFFF; border-radius: 8px; margin-top: -1px; }
QTabBar::tab { background-color: #E0E0E0; color: #666; border-top-left-radius: 6px; border-top-right-radius: 6px; padding: 10px 20px; margin-right: 2px; font-weight: bold; }
QTabBar::tab:selected { background-color: #FFFFFF; color: #1976D2; border: 1px solid #E0E0E0; border-bottom: 2px solid #1976D2; }
QGroupBox { background-color: #FFFFFF; border: 1px solid #D6D6D6; border-radius: 6px; margin-top: 18px; padding-top: 15px; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 15px; padding: 0 5px; color: #1976D2; font-weight: bold; font-size: 15px; }
QPushButton { border-radius: 5px; padding: 8px 15px; font-weight: bold; border: none; }
QPushButton:hover { opacity: 0.8; }
QPushButton:pressed { background-color: rgba(0,0,0,0.1); }
QDoubleSpinBox, QSpinBox, QLineEdit { border: 1px solid #BDBDBD; border-radius: 4px; padding: 5px; background-color: #FAFAFA; }
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus { border: 1px solid #1976D2; background-color: #FFFFFF; }
QScrollArea { border: none; background-color: transparent; }
QScrollArea > QWidget > QWidget { background-color: transparent; }
"""

# =====================================================================
# ROS 监听器与发布器
# =====================================================================
class ROSListener(QObject):
    ref_cloud_updated = pyqtSignal(list)
    sensor_data_updated = pyqtSignal(str, list) # 返回 (key, points_2d)

    def __init__(self):
        super().__init__()
        self.current_tfs = {k: (0,0,0,0,0,0) for k in CALIB_GROUPS.keys()}
        self.pubs = {}
        if not HAS_ROS: return

        # 1. 订阅基础参考系雷达 (/points_16)
        rospy.Subscriber("/points_16", PointCloud2, self.ref_callback, queue_size=1)

        # 2. 为其他雷达创建订阅者和发布者
        for key, config in CALIB_GROUPS.items():
            if not config['raw_topic']: continue

            # 注册 Calibration 发布者
            self.pubs[key] = rospy.Publisher(config['calib_topic'], PointCloud2, queue_size=1)

            # 注册 Raw 订阅者
            if "scan" in config['raw_topic']:
                rospy.Subscriber(config['raw_topic'], LaserScan, lambda msg, k=key: self.scan_callback(k, msg), queue_size=1)
            else:
                rospy.Subscriber(config['raw_topic'], PointCloud2, lambda msg, k=key: self.pc2_callback(k, msg), queue_size=1)

    def update_tf(self, key, tf_tuple):
        """ 界面滑块变化时更新内部的 TF 矩阵参数 """
        self.current_tfs[key] = tf_tuple

    def publish_transformed_cloud(self, key, points_3d, stamp):
        """ 将计算后的 3D 点云发布给 ROS (RViz等可见) """
        if key not in self.pubs: return
        header = std_msgs.msg.Header()
        header.stamp = stamp
        header.frame_id = "velodyne"
        cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
        self.pubs[key].publish(cloud_msg)

    def ref_callback(self, msg):
        pts = []
        try:
            for i, p in enumerate(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)):
                # 降采样并过滤远距离点，提升 UI 流畅度
                if i % 4 == 0 and -20 < p[0] < 20 and -20 < p[1] < 20: 
                    pts.append((p[0], p[1]))
            self.ref_cloud_updated.emit(pts)
        except: pass

    def pc2_callback(self, key, msg):
        pts_3d = []
        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                # 提取一定范围内的真实 3D 点
                if -15.0 < p[0] < 15.0 and -15.0 < p[1] < 15.0:
                    pts_3d.append((p[0], p[1], p[2]))
            
            # 1. 应用 TF 变换
            transformed_3d = apply_tf_3d(pts_3d, *self.current_tfs[key])
            
            # 2. 发布给 ROS 网络 (Calibration 话题)
            self.publish_transformed_cloud(key, transformed_3d, msg.header.stamp)

            # 3. 抽样并降维传递给 UI 预览窗
            ui_pts = [(p[0], p[1]) for i, p in enumerate(transformed_3d) if i % 3 == 0]
            self.sensor_data_updated.emit(key, ui_pts)
        except: pass

    def scan_callback(self, key, msg):
        pts_3d = []
        try:
            angle = msg.angle_min
            for r in msg.ranges:
                if msg.range_min < r < msg.range_max and r < 15.0:
                    # 单线雷达转换极坐标，z 默认为 0
                    pts_3d.append((r * math.cos(angle), r * math.sin(angle), 0.0))
                angle += msg.angle_increment

            # 1. 应用 TF 变换
            transformed_3d = apply_tf_3d(pts_3d, *self.current_tfs[key])
            
            # 2. 发布
            self.publish_transformed_cloud(key, transformed_3d, msg.header.stamp)

            # 3. 传递 UI
            ui_pts = [(p[0], p[1]) for p in transformed_3d]
            self.sensor_data_updated.emit(key, ui_pts)
        except: pass

# =====================================================================
# 画板: 车辆轮廓与安全框
# =====================================================================
class VehiclePreviewWidget(QWidget):
    def __init__(self, color_r=0, color_g=255, color_b=255):
        super().__init__()
        self.points, self.ref_points, self.cloud_points = [], [], []
        self.setMinimumSize(320, 320)
        self.fill_color = QColor(color_r, color_g, color_b, 60)
        self.line_color = QColor(color_r, color_g, color_b)

    def set_points(self, points): self.points = points; self.update()
    def set_reference_points(self, points): self.ref_points = points; self.update()
    def set_cloud_points(self, points): self.cloud_points = points; self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#121212"))
            w, h = self.width(), self.height(); cx, cy = w / 2, h / 2

            max_range = 1.0 
            all_pts = (self.points if isinstance(self.points, list) else []) + (self.ref_points if isinstance(self.ref_points, list) else [])
            for p in all_pts:
                if isinstance(p, dict):
                    max_range = max(max_range, abs(safe_float(p.get('x'))), abs(safe_float(p.get('y'))))
            scale = (min(w, h) / 2.0) / (max_range * 1.25) if max_range > 0 else 1.0

            if self.cloud_points:
                painter.setPen(QPen(QColor(255, 255, 255, 140), 2))
                cloud_poly = QPolygonF()
                for cx_pt, cy_pt in self.cloud_points:
                    cloud_poly.append(QPointF(cx - cy_pt * scale, cy - cx_pt * scale))
                painter.drawPoints(cloud_poly)

            painter.setPen(QPen(QColor(255, 80, 80, 150), 1, Qt.DashLine))
            painter.drawLine(int(cx), 0, int(cx), int(h)) 
            painter.setPen(QPen(QColor(80, 255, 80, 150), 1, Qt.DashLine))
            painter.drawLine(0, int(cy), int(w), int(cy)) 

            painter.setPen(QPen(QColor(200, 200, 200))); painter.setFont(QFont("Arial", 8))
            painter.drawText(int(cx) + 5, 15, "X(前)"); painter.drawText(5, int(cy) - 5, "Y(左)")

            def build_poly(pt_list):
                poly = QPolygonF(); valid = []
                for i, p in enumerate(pt_list):
                    if isinstance(p, dict) and 'x' in p and 'y' in p:
                        rx, ry = safe_float(p['x']), safe_float(p['y'])
                        pt = QPointF(cx - ry * scale, cy - rx * scale)
                        poly.append(pt); valid.append((pt, i))
                return poly, valid

            if self.ref_points:
                ref_poly, _ = build_poly(self.ref_points)
                painter.setPen(QPen(QColor(0, 255, 255, 150), 2))
                painter.setBrush(QColor(0, 255, 255, 30)); painter.drawPolygon(ref_poly)

            if self.points and isinstance(self.points, list):
                main_poly, valid_pts = build_poly(self.points)
                painter.setPen(QPen(self.line_color, 2))
                painter.setBrush(self.fill_color); painter.drawPolygon(main_poly)
                painter.setFont(QFont("Arial", 9, QFont.Bold))
                for pt, idx in valid_pts:
                    painter.setBrush(QColor(255, 255, 0)); painter.setPen(Qt.NoPen)
                    painter.drawEllipse(pt, 5, 5)
                    painter.setPen(QPen(QColor(255, 255, 255))); painter.drawText(int(pt.x()) + 8, int(pt.y()) - 8, f"P{idx+1}")
        except: pass

# =====================================================================
# 专属画板: 雷达标定实时预览
# =====================================================================
class CalibrationPreviewWidget(QWidget):
    def __init__(self, title):
        super().__init__()
        self.title = title
        self.ref_cloud = []     
        self.sensor_cloud = []  
        self.setMinimumSize(350, 350)

    def set_ref_cloud(self, pts_2d):
        self.ref_cloud = pts_2d; self.update()

    def set_sensor_cloud(self, pts_2d):
        # 此时接收到的已经是应用过 TF 变换的 2D 坐标
        self.sensor_cloud = pts_2d; self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#121212"))
            w, h = self.width(), self.height(); cx, cy = w / 2, h / 2

            scale = min(w, h) / 30.0 # 标定视野更大一点

            painter.setPen(QPen(QColor(255, 80, 80, 100), 1)); painter.drawLine(int(cx), 0, int(cx), int(h))
            painter.setPen(QPen(QColor(80, 255, 80, 100), 1)); painter.drawLine(0, int(cy), int(w), int(cy))

            # 绘制主雷达底图 (白色半透明)
            if self.ref_cloud:
                painter.setPen(QPen(QColor(255, 255, 255, 80), 2))
                poly = QPolygonF()
                for px, py in self.ref_cloud: poly.append(QPointF(cx - py * scale, cy - px * scale))
                painter.drawPoints(poly)

            # 绘制正在校准的传感器点云 (亮绿色)
            if self.sensor_cloud:
                painter.setPen(QPen(QColor(0, 255, 0, 200), 3))
                t_poly = QPolygonF()
                for px, py in self.sensor_cloud: t_poly.append(QPointF(cx - py * scale, cy - px * scale))
                painter.drawPoints(t_poly)

            painter.setPen(QPen(QColor(255, 255, 0))); painter.setFont(QFont("Arial", 10, QFont.Bold))
            painter.drawText(10, 20, self.title)
        except: pass

# =====================================================================
# 统一动态多边形编辑器
# =====================================================================
class PolygonEditor(QWidget):
    def __init__(self, init_points, raw_key, yaml_data_source, hidden_text_edit, preview_widget):
        super().__init__()
        self.points = [{'x': safe_float(p.get('x')), 'y': safe_float(p.get('y'))} for p in init_points]
        self.raw_key = raw_key; self.yaml_data_source = yaml_data_source
        self.hidden_text_edit = hidden_text_edit; self.preview = preview_widget
        self.layout = QVBoxLayout(self); self.layout.setContentsMargins(0,0,0,0)
        
        g = QGroupBox("整体外扩/缩小"); gl = QGridLayout(g)
        self.sf, self.sb, self.sl, self.sr = [QDoubleSpinBox() for _ in range(4)]
        for s in (self.sf, self.sb, self.sl, self.sr): s.setRange(-10,10); s.setSingleStep(0.05)
        gl.addWidget(QLabel("前:"),0,0); gl.addWidget(self.sf,0,1); gl.addWidget(QLabel("后:"),0,2); gl.addWidget(self.sb,0,3)
        gl.addWidget(QLabel("左:"),1,0); gl.addWidget(self.sl,1,1); gl.addWidget(QLabel("右:"),1,2); gl.addWidget(self.sr,1,3)
        btn = QPushButton("应用"); btn.setStyleSheet("background-color: #673AB7; color: white;"); btn.clicked.connect(self.apply_bulk)
        gl.addWidget(btn,0,4,2,1); self.layout.addWidget(g)

        self.scroll = QScrollArea(); self.scroll.setWidgetResizable(True)
        self.p_cont = QWidget(); self.p_lay = QGridLayout(self.p_cont)
        self.scroll.setWidget(self.p_cont); self.layout.addWidget(self.scroll)
        
        self.add_btn = QPushButton("+ 添加点位"); self.add_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.add_btn.clicked.connect(self.add_point); self.layout.addWidget(self.add_btn)
        
        self.spin_pairs = []; self.render_points()

    def apply_bulk(self):
        df, db, dl, dr = self.sf.value(), self.sb.value(), self.sl.value(), self.sr.value()
        for sx, sy in self.spin_pairs:
            x, y = sx.value(), sy.value()
            if x>0.01: x+=df 
            elif x<-0.01: x-=db 
            if y>0.01: y+=dl 
            elif y<-0.01: y-=dr
            sx.setValue(x); sy.setValue(y)
        self.sf.setValue(0); self.sb.setValue(0); self.sl.setValue(0); self.sr.setValue(0)

    def render_points(self):
        for i in reversed(range(self.p_lay.count())): 
            w = self.p_lay.itemAt(i).widget(); 
            if w: w.setParent(None)
        self.spin_pairs.clear()
        
        for i, pt in enumerate(self.points):
            lbl = QLabel(f"P{i+1}:"); lbl.setStyleSheet("color: #D32F2F; font-weight: bold;")
            sx = QDoubleSpinBox(); sx.setRange(-50.0, 50.0); sx.setDecimals(2); sx.setValue(pt['x']); sx.setSingleStep(0.05)
            sy = QDoubleSpinBox(); sy.setRange(-50.0, 50.0); sy.setDecimals(2); sy.setValue(pt['y']); sy.setSingleStep(0.05)
            db = QPushButton("X"); db.setFixedSize(30,30)
            db.setStyleSheet("background-color:#E53935;color:white;border-radius:15px;"); db.clicked.connect(lambda _, idx=i: self.rm(idx))
            sx.valueChanged.connect(self.up); sy.valueChanged.connect(self.up)
            
            self.p_lay.addWidget(lbl, i, 0); self.p_lay.addWidget(QLabel("X:"), i, 1); self.p_lay.addWidget(sx, i, 2)
            self.p_lay.addWidget(QLabel("Y:"), i, 3); self.p_lay.addWidget(sy, i, 4); self.p_lay.addWidget(db, i, 5)
            self.spin_pairs.append((sx, sy))
        self.out()

    def add_point(self): self.points.append(self.points[-1].copy() if self.points else {'x':0.0,'y':0.0}); self.render_points()
    def rm(self, idx): 
        if len(self.points)>1: self.points.pop(idx); self.render_points()
    def up(self):
        for i, (sx, sy) in enumerate(self.spin_pairs): self.points[i]['x'], self.points[i]['y'] = round(sx.value(),2), round(sy.value(),2)
        self.out()
    def out(self):
        self.preview.set_points(self.points)
        pts = [create_flow_point(p['x'], p['y']) for p in self.points]
        self.yaml_data_source[self.raw_key] = pts
        b = StringIO(); YAML().dump(pts, b); self.hidden_text_edit.setPlainText(b.getvalue())

# =====================================================================
# 主界面
# =====================================================================
class ConfigEditorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.yaml_path = '/home/getq/work/workspace/config/perception/vehicle_size.yaml'
        self.safe_yaml_path = '/home/getq/work/workspace/config/perception/safe_obstacle.yaml'
        self.calib_yaml_path = '/home/getq/work/workspace/param/lidar_calibration.yaml'
        self.cfg_path = '/home/getq/work/autoware.ai/install/lidar_filtering/share/lidar_filtering/cfg/LidarFiltering.cfg'
        
        self.yaml_parser = YAML(); self.yaml_parser.preserve_quotes = True
        self.yaml_data = None; self.safe_yaml_data = None; self.calib_yaml_data = None
        self.cfg_lines = []; self.cfg_meta = {}

        self.previews_basic = []; self.previews_safe = []
        self.calib_previews = {} 

        if HAS_ROS:
            try:
                rospy.init_node('config_editor_node', anonymous=True, disable_signals=True)
                self.ros_listener = ROSListener()
                self.ros_listener.ref_cloud_updated.connect(self.dispatch_ref_cloud)
                self.ros_listener.sensor_data_updated.connect(self.dispatch_sensor_cloud)
            except Exception as e: print("ROS 节点启动失败:", e)

        self.init_ui()

    def dispatch_ref_cloud(self, points):
        for p in self.previews_basic + self.previews_safe:
            try: p.set_cloud_points(points)
            except: pass
        for p in self.calib_previews.values():
            try: p.set_ref_cloud(points)
            except: pass

    def dispatch_sensor_cloud(self, key, points_2d):
        if key in self.calib_previews:
            try: self.calib_previews[key].set_sensor_cloud(points_2d)
            except: pass

    def init_ui(self):
        self.setWindowTitle("Lidar Filtering 可视化调参工具")
        self.resize(1200, 950)
        self.setStyleSheet(GLOBAL_STYLE)
        
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        self.tab_yaml  = QWidget(); self.setup_yaml_tab();  self.tabs.addTab(self.tab_yaml, "常规车辆参数")
        self.tab_safe  = QWidget(); self.setup_safe_tab();  self.tabs.addTab(self.tab_safe, "安全检测框设置")
        self.tab_cfg   = QWidget(); self.setup_cfg_tab();   self.tabs.addTab(self.tab_cfg,  "高级动态配置")
        self.tab_calib = QWidget(); self.setup_calib_tab(); self.tabs.addTab(self.tab_calib, "雷达标定配置")
        
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

    # ========================== 1. 常规参数 (YAML) ==========================
    def setup_yaml_tab(self):
        l = QVBoxLayout(self.tab_yaml)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_yaml); br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("💾 保存修改"); bs.clicked.connect(self.save_yaml); bs.setStyleSheet("background-color: #4CAF50; color: white;")
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
                box = QGroupBox(lbl); bl = QHBoxLayout(box)
                t = QTextEdit(); t.hide(); self.yaml_widgets[k] = t
                pv = VehiclePreviewWidget(0, 255, 255); self.previews_basic.append(pv)
                ed = PolygonEditor(val, k, self.yaml_data, t, pv)
                bl.addWidget(ed, stretch=1); bl.addWidget(pv, stretch=1)
                self.yaml_form_layout.addRow(box)
            else:
                label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
                if isinstance(val, bool): w = QCheckBox(); w.setChecked(val)
                else: w = QDoubleSpinBox(); w.setRange(-1000, 1000); w.setDecimals(2); w.setValue(val)
                self.yaml_widgets[k] = w; self.yaml_form_layout.addRow(label, w)

    def save_yaml(self):
        if self.save_generic_yaml(self.yaml_widgets, self.yaml_data, self.yaml_path):
            QMessageBox.information(self, "提示", "常规参数保存成功！")

    # ========================== 2. 安全框 (Safe) ==========================
    def setup_safe_tab(self):
        l = QVBoxLayout(self.tab_safe)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_safe_yaml); br.setStyleSheet("background-color: #757575; color: white;")
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
        ref = self.yaml_data.get('rect', []) if self.yaml_data else []
        for k, lbl in ALLOWED_SAFE_YAML_PARAMS.items():
            if k not in self.safe_yaml_data: continue
            val = self.safe_yaml_data[k]
            if isinstance(val, list):
                box = QGroupBox(lbl)
                bl = QHBoxLayout(box); t = QTextEdit(); t.hide(); self.safe_widgets[k] = t
                pv = VehiclePreviewWidget(255, 150, 0); pv.set_reference_points(ref); self.previews_safe.append(pv)
                ed = PolygonEditor(val, k, self.safe_yaml_data, t, pv); bl.addWidget(ed, stretch=1); bl.addWidget(pv, stretch=1)
                self.safe_form_layout.addRow(box)
            else:
                label = QLabel(lbl); label.setStyleSheet("font-weight: bold; margin-top: 5px;")
                w = QDoubleSpinBox(); w.setRange(-100, 100); w.setDecimals(2); w.setValue(val)
                self.safe_widgets[k] = w; self.safe_form_layout.addRow(label, w)

    def save_safe_yaml(self):
        if self.save_generic_yaml(self.safe_widgets, self.safe_yaml_data, self.safe_yaml_path):
            QMessageBox.information(self, "提示", "安全检测框保存成功！")

    # ========================== 3. 雷达标定 (Calibration) ==========================
    def setup_calib_tab(self):
        l = QVBoxLayout(self.tab_calib)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取标定"); br.clicked.connect(self.load_calib_yaml); br.setStyleSheet("background-color: #757575; color: white;")
        bs = QPushButton("🎯 保存雷达标定"); bs.clicked.connect(self.save_calib_yaml); bs.setStyleSheet("background-color: #E91E63; color: white;")
        h.addWidget(br); h.addWidget(bs); l.addLayout(h)
        self.scroll_calib = QScrollArea(); self.scroll_calib.setWidgetResizable(True)
        self.calib_form_widget = QWidget(); self.calib_form_layout = QVBoxLayout(self.calib_form_widget)
        self.scroll_calib.setWidget(self.calib_form_widget); l.addWidget(self.scroll_calib)
        self.load_calib_yaml()

    def load_calib_yaml(self):
        if not os.path.exists(self.calib_yaml_path): return
        with open(self.calib_yaml_path, 'r', encoding='utf-8') as f: 
            self.calib_yaml_data = self.yaml_parser.load(f)
        for i in reversed(range(self.calib_form_layout.count())):
            w = self.calib_form_layout.itemAt(i).widget()
            if w: w.setParent(None)
            
        self.calib_widgets = {}; self.calib_previews.clear()
        if 'tf_calibration' not in self.calib_yaml_data: return
        tf_data = self.calib_yaml_data['tf_calibration']

        for prefix, config in CALIB_GROUPS.items():
            group = QGroupBox(f"{config['name']} -> 发布话题: {config['calib_topic']}")
            hl = QHBoxLayout(group)
            
            gl = QGridLayout()
            params = ['x', 'y', 'z', 'yaw', 'pitch', 'roll']
            spins = {}
            for i, p in enumerate(params):
                key = f"{prefix}_{p}"; val = tf_data.get(key, 0.0)
                sb = QDoubleSpinBox()
                sb.setRange(-3.14159*2 if p in ['yaw','pitch','roll'] else -10.0, 3.14159*2 if p in ['yaw','pitch','roll'] else 10.0)
                sb.setDecimals(4); sb.setSingleStep(0.01); sb.setValue(val)
                self.calib_widgets[key] = sb; spins[p] = sb
                gl.addWidget(QLabel(f"{p.upper()}:"), i//2, (i%2)*2)
                gl.addWidget(sb, i//2, (i%2)*2 + 1)
            
            w_left = QWidget(); w_left.setLayout(gl); hl.addWidget(w_left, stretch=1)

            if prefix != "main":
                pv = CalibrationPreviewWidget(f"实时校准: {prefix}")
                self.calib_previews[prefix] = pv
                hl.addWidget(pv, stretch=1)
                
                # 连接滑块，实时通知 ROSListener 改变 TF
                def make_update_func(k, p_spins):
                    def update_tf():
                        tx, ty, tz = p_spins['x'].value(), p_spins['y'].value(), p_spins['z'].value()
                        yw, pt, rl = p_spins['yaw'].value(), p_spins['pitch'].value(), p_spins['roll'].value()
                        if HAS_ROS: self.ros_listener.update_tf(k, (tx, ty, tz, yw, pt, rl))
                    return update_tf
                
                up_func = make_update_func(prefix, spins)
                for s in spins.values(): s.valueChanged.connect(up_func)
                up_func() # 初始化

            self.calib_form_layout.addWidget(group)

    def save_calib_yaml(self):
        if not self.calib_yaml_data or 'tf_calibration' not in self.calib_yaml_data: return
        tf_data = self.calib_yaml_data['tf_calibration']
        for k, w in self.calib_widgets.items(): tf_data[k] = w.value()
        try:
            with open(self.calib_yaml_path, 'w', encoding='utf-8') as f: self.yaml_parser.dump(self.calib_yaml_data, f)
            QMessageBox.information(self, "提示", "雷达标定参数保存成功！")
        except Exception as e: QMessageBox.critical(self, "错误", f"保存失败: {e}")

    # ========================== 4. CFG ==========================
    def setup_cfg_tab(self):
        l = QVBoxLayout(self.tab_cfg)
        h = QHBoxLayout(); br = QPushButton("🔄 重新读取"); br.clicked.connect(self.load_cfg); br.setStyleSheet("background-color: #757575; color: white;")
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