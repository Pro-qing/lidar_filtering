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
from PyQt5.QtCore import Qt, QPointF

# 使用 ruamel.yaml 保留原文件的所有注释和缩进格式
from ruamel.yaml import YAML
from ruamel.yaml.compat import StringIO

# =====================================================================
# 参数显示映射表（字典）
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
    try:
        return float(val)
    except (ValueError, TypeError):
        return default

# =====================================================================
# 自定义画板
# =====================================================================
class VehiclePreviewWidget(QWidget):
    def __init__(self, color_r=0, color_g=255, color_b=255):
        super().__init__()
        self.points = []       
        self.ref_points = []   
        self.setMinimumSize(300, 300)
        self.fill_color = QColor(color_r, color_g, color_b, 50)
        self.line_color = QColor(color_r, color_g, color_b)

    def set_points(self, points):
        self.points = points
        self.update()

    def set_reference_points(self, points):
        self.ref_points = points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#1e1e1e"))

            w, h = self.width(), self.height()
            cx, cy = w / 2, h / 2

            # 坐标轴
            painter.setPen(QPen(QColor(255, 50, 50), 1, Qt.DashLine))
            painter.drawLine(int(cx), 0, int(cx), int(h)) 
            painter.setPen(QPen(QColor(50, 255, 50), 1, Qt.DashLine))
            painter.drawLine(0, int(cy), int(w), int(cy)) 

            painter.setPen(QPen(QColor(200, 200, 200)))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(int(cx) + 5, 15, "X(前)")
            painter.drawText(5, int(cy) - 5, "Y(左)")

            painter.setPen(QPen(QColor(255, 255, 255)))
            painter.setFont(QFont("Arial", 9, QFont.Bold))
            painter.drawText(int(cx) + 6, int(cy) - 6, "velodyne")
            painter.setBrush(QColor(255, 255, 255))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(cx, cy), 3.5, 3.5)

            if self.ref_points:
                painter.setPen(QPen(QColor(0, 255, 255)))
                painter.drawText(10, 25, "■ 青色: 车辆本体")
                painter.setPen(QPen(self.line_color))
                painter.drawText(10, 40, "■ 橙色: 正在编辑的框")

            max_range = 1.0 
            for p in (self.points + self.ref_points):
                if isinstance(p, dict):
                    max_range = max(max_range, abs(safe_float(p.get('x'))), abs(safe_float(p.get('y'))))
            
            scale = (min(w, h) / 2.0) / (max_range * 1.15) if max_range > 0 else 1.0

            def build_poly(pt_list):
                poly = QPolygonF()
                valid = []
                for p in pt_list:
                    if isinstance(p, dict) and 'x' in p and 'y' in p:
                        rx, ry = safe_float(p['x']), safe_float(p['y'])
                        qx = cx - (ry * scale)
                        qy = cy - (rx * scale)
                        pt = QPointF(qx, qy)
                        poly.append(pt)
                        valid.append(pt)
                return poly, valid

            if self.ref_points:
                ref_poly, _ = build_poly(self.ref_points)
                painter.setPen(QPen(QColor(0, 255, 255), 2))
                painter.setBrush(QColor(0, 255, 255, 40))
                painter.drawPolygon(ref_poly)

            if self.points and isinstance(self.points, list):
                main_poly, valid_pts = build_poly(self.points)
                painter.setPen(QPen(self.line_color, 2))
                painter.setBrush(self.fill_color)
                painter.drawPolygon(main_poly)

                painter.setBrush(QColor(255, 255, 0))
                painter.setPen(Qt.NoPen)
                for pt in valid_pts:
                    painter.drawEllipse(pt, 3, 3)
        except Exception:
            pass

# =====================================================================
# 动态多边形编辑器组件 (用于安全框的动态增删)
# =====================================================================
class SafePolygonEditor(QWidget):
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
        
        # 为了应对点数过多，增加一个内部的滚动区域
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("QScrollArea { border: none; }")
        
        self.points_container = QWidget()
        self.points_layout = QGridLayout(self.points_container)
        self.points_layout.setContentsMargins(0, 0, 0, 0)
        self.scroll_area.setWidget(self.points_container)
        
        self.layout.addWidget(self.scroll_area)
        
        btn_layout = QHBoxLayout()
        self.add_btn = QPushButton("+ 添加点位")
        self.add_btn.setStyleSheet("color: white; background-color: #2196F3; font-weight: bold; border-radius: 4px; padding: 5px;")
        self.add_btn.clicked.connect(self.add_point)
        btn_layout.addWidget(self.add_btn)
        btn_layout.addStretch()
        self.layout.addLayout(btn_layout)
        
        self.spin_pairs = []
        self.render_points()

    def render_points(self):
        for i in reversed(range(self.points_layout.count())): 
            widget = self.points_layout.itemAt(i).widget()
            if widget is not None:
                widget.setParent(None)
                
        self.spin_pairs.clear()
        
        for i, pt in enumerate(self.points):
            lbl_p = QLabel(f"点 {i+1}:")
            lbl_p.setStyleSheet("color: #555; font-weight: bold;")
            
            sb_x = QDoubleSpinBox()
            sb_x.setRange(-50.0, 50.0)
            sb_x.setDecimals(2)
            sb_x.setSingleStep(0.05)
            sb_x.setValue(pt['x'])
            
            sb_y = QDoubleSpinBox()
            sb_y.setRange(-50.0, 50.0)
            sb_y.setDecimals(2)
            sb_y.setSingleStep(0.05)
            sb_y.setValue(pt['y'])
            
            del_btn = QPushButton("删除")
            del_btn.setStyleSheet("color: white; background-color: #f44336; border-radius: 4px; padding: 2px 6px;")
            del_btn.clicked.connect(lambda checked, idx=i: self.remove_point(idx))
            
            sb_x.valueChanged.connect(self.on_value_changed)
            sb_y.valueChanged.connect(self.on_value_changed)
            
            self.points_layout.addWidget(lbl_p, i, 0)
            self.points_layout.addWidget(QLabel("X:"), i, 1)
            self.points_layout.addWidget(sb_x, i, 2)
            self.points_layout.addWidget(QLabel("Y:"), i, 3)
            self.points_layout.addWidget(sb_y, i, 4)
            self.points_layout.addWidget(del_btn, i, 5)
            
            self.spin_pairs.append((sb_x, sb_y))
            
        self.update_output()

    def add_point(self):
        if self.points:
            last_pt = self.points[-1]
            self.points.append({'x': last_pt['x'], 'y': last_pt['y']})
        else:
            self.points.append({'x': 0.0, 'y': 0.0})
        self.render_points()

    def remove_point(self, idx):
        if 0 <= idx < len(self.points):
            self.points.pop(idx)
            self.render_points()

    def on_value_changed(self):
        for i, (sx, sy) in enumerate(self.spin_pairs):
            self.points[i]['x'] = round(sx.value(), 2)
            self.points[i]['y'] = round(sy.value(), 2)
        self.update_output()
        
    def update_output(self):
        self.preview.set_points(self.points)
        self.yaml_data_source[self.raw_key] = self.points
        b = StringIO()
        YAML().dump(self.points, b)
        self.hidden_text_edit.blockSignals(True)
        self.hidden_text_edit.setPlainText(b.getvalue())
        self.hidden_text_edit.blockSignals(False)


# =====================================================================
# 主界面
# =====================================================================
class ConfigEditorGUI(QWidget):
    def __init__(self):
        super().__init__()
        
        self.yaml_path = '/home/getq/work/workspace/config/perception/vehicle_size.yaml'
        self.safe_yaml_path = '/home/getq/work/workspace/config/perception/safe_obstacle.yaml'
        self.cfg_path = '/home/getq/work/autoware.ai/install/lidar_filtering/share/lidar_filtering/cfg/LidarFiltering.cfg'
        
        self.yaml_parser = YAML()
        self.yaml_parser.preserve_quotes = True
        
        self.yaml_data = None
        self.safe_yaml_data = None
        self.cfg_lines = []
        self.cfg_params_meta = {}

        self.yaml_widgets = {}
        self.safe_widgets = {}
        self.cfg_widgets = {}

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Lidar Filtering 参数配置工具")
        self.resize(850, 950)
        main_layout = QVBoxLayout(self)
        tabs = QTabWidget()
        
        self.tab_yaml = QWidget()
        self.setup_yaml_tab()
        tabs.addTab(self.tab_yaml, "常规车辆参数")

        self.tab_safe = QWidget()
        self.setup_safe_tab()
        tabs.addTab(self.tab_safe, "安全检测框设置")

        self.tab_cfg = QWidget()
        self.setup_cfg_tab()
        tabs.addTab(self.tab_cfg, "高级动态配置")

        main_layout.addWidget(tabs)

    def build_yaml_ui(self, layout, widgets_dict, allowed_dict, yaml_data_source, color_tuple=(0, 255, 255), ref_polygon=None):
        for raw_key, custom_label in allowed_dict.items():
            if raw_key not in yaml_data_source: continue

            value = yaml_data_source[raw_key]

            if isinstance(value, list) and len(value) > 0 and isinstance(value[0], dict):
                group_box = QGroupBox(f"{custom_label}")
                group_box.setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid silver; border-radius: 6px; margin-top: 10px; }")
                box_layout = QVBoxLayout(group_box)
                h_layout = QHBoxLayout()

                if raw_key == "rect":
                    # ================= 车辆本体 rect (物理参数生成模式) =================
                    form_widget = QWidget()
                    form_layout = QFormLayout(form_widget)

                    def make_spin(gui_key, default_val):
                        sb = QDoubleSpinBox()
                        sb.setRange(-20.0, 30.0)
                        sb.setDecimals(2)
                        sb.setSingleStep(0.05)
                        sb.setValue(yaml_data_source.get(gui_key, default_val))
                        return sb

                    sb_vf = make_spin('_gui_velo_front', 0.50)
                    sb_tl = make_spin('_gui_total_len', 2.16)
                    sb_hl = make_spin('_gui_head_len', 0.50)
                    sb_hw = make_spin('_gui_head_w', 0.92)
                    sb_bw = make_spin('_gui_body_w', 1.50)
                    sb_margin = make_spin('_gui_margin', 0.05)

                    form_layout.addRow("Velodyne距车头最前距(m):", sb_vf)
                    form_layout.addRow("车辆总长度(m):", sb_tl)
                    form_layout.addRow("车头区长度(m):", sb_hl)
                    form_layout.addRow("车头区宽度(m):", sb_hw)
                    form_layout.addRow("车身主体宽度(m):", sb_bw)
                    form_layout.addRow("额外安全外扩余量(m):", sb_margin)
                    
                    h_layout.addWidget(form_widget, stretch=1)

                    text_editor = QTextEdit()
                    text_editor.hide()
                    widgets_dict[raw_key] = text_editor

                    preview = VehiclePreviewWidget(color_tuple[0], color_tuple[1], color_tuple[2])
                    h_layout.addWidget(preview, stretch=1)

                    def update_rect():
                        vf, tl, hl, hw, bw, m = sb_vf.value(), sb_tl.value(), sb_hl.value(), sb_hw.value(), sb_bw.value(), sb_margin.value()
                        yaml_data_source['_gui_velo_front'] = vf
                        yaml_data_source['_gui_total_len'] = tl
                        yaml_data_source['_gui_head_len'] = hl
                        yaml_data_source['_gui_head_w'] = hw
                        yaml_data_source['_gui_body_w'] = bw
                        yaml_data_source['_gui_margin'] = m
                        
                        f_x, b_x = vf + m, vf - tl - m
                        hx_min = vf - hl
                        hw_half, bw_half = hw/2.0 + m, bw/2.0 + m
                        pts = [
                            {'x': round(f_x, 2), 'y': round(-hw_half, 2)},
                            {'x': round(hx_min, 2), 'y': round(-hw_half, 2)},
                            {'x': round(hx_min, 2), 'y': round(-bw_half, 2)},
                            {'x': round(b_x, 2), 'y': round(-bw_half, 2)},
                            {'x': round(b_x, 2), 'y': round(bw_half, 2)},
                            {'x': round(hx_min, 2), 'y': round(bw_half, 2)},
                            {'x': round(hx_min, 2), 'y': round(hw_half, 2)},
                            {'x': round(f_x, 2), 'y': round(hw_half, 2)},
                        ]
                        yaml_data_source[raw_key] = pts
                        preview.set_points(pts)

                        b = StringIO()
                        YAML().dump(pts, b)
                        text_editor.blockSignals(True)
                        text_editor.setPlainText(b.getvalue())
                        text_editor.blockSignals(False)

                    for sb in [sb_vf, sb_tl, sb_hl, sb_hw, sb_bw, sb_margin]:
                        sb.valueChanged.connect(update_rect)
                    update_rect()

                else:
                    # ================= 安全框 (按点输入模式，支持动态增删，实时联动) =================
                    text_editor = QTextEdit()
                    text_editor.hide()
                    widgets_dict[raw_key] = text_editor

                    preview = VehiclePreviewWidget(color_tuple[0], color_tuple[1], color_tuple[2])
                    if ref_polygon:
                        preview.set_reference_points(ref_polygon)

                    # 调用专门为安全框编写的编辑器模块
                    editor_widget = SafePolygonEditor(value, raw_key, yaml_data_source, text_editor, preview)
                    
                    h_layout.addWidget(editor_widget, stretch=1)
                    h_layout.addWidget(preview, stretch=1)

                box_layout.addLayout(h_layout)
                layout.addRow(group_box)
                continue

            # 常规基础类型控件
            lbl = QLabel(custom_label + " :")
            lbl.setWordWrap(True)
            lbl.setStyleSheet("font-weight: bold; margin-top: 5px;")

            if isinstance(value, bool):
                w = QCheckBox()
                w.setChecked(value)
                widgets_dict[raw_key] = w
                layout.addRow(lbl, w)
            elif isinstance(value, int) and not isinstance(value, bool):
                w = QSpinBox()
                w.setRange(-10000, 10000)
                w.setValue(value)
                widgets_dict[raw_key] = w
                layout.addRow(lbl, w)
            elif isinstance(value, float):
                w = QDoubleSpinBox()
                w.setRange(-10000.0, 10000.0)
                w.setDecimals(2)
                w.setSingleStep(0.1)
                w.setValue(value)
                widgets_dict[raw_key] = w
                layout.addRow(lbl, w)
            elif isinstance(value, str):
                w = QLineEdit()
                w.setText(value)
                widgets_dict[raw_key] = w
                layout.addRow(lbl, w)

    def save_generic_yaml(self, widgets_dict, data_source, path, allowed_dict):
        for raw_key, w in widgets_dict.items():
            if isinstance(w, QCheckBox):
                data_source[raw_key] = w.isChecked()
            elif isinstance(w, QSpinBox) or isinstance(w, QDoubleSpinBox):
                data_source[raw_key] = w.value()
            elif isinstance(w, QLineEdit):
                data_source[raw_key] = w.text()
            elif isinstance(w, QTextEdit):
                text = w.toPlainText()
                tmp_yaml = YAML()
                try:
                    parsed_val = tmp_yaml.load(text)
                    data_source[raw_key] = parsed_val
                except Exception as e:
                    QMessageBox.warning(self, "解析错误", f"键 '{allowed_dict.get(raw_key, raw_key)}' 格式错误:\n{e}")
                    return False
        try:
            with open(path, 'w', encoding='utf-8') as f:
                self.yaml_parser.dump(data_source, f)
            return True
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败:\n{e}")
            return False

    # ========================== 1. 基本车辆 YAML ==========================
    def setup_yaml_tab(self):
        layout = QVBoxLayout(self.tab_yaml)
        top_layout = QHBoxLayout()
        btn_load = QPushButton("重新读取配置")
        btn_load.clicked.connect(self.load_yaml)
        btn_save = QPushButton("保存基本修改")
        btn_save.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        btn_save.clicked.connect(self.save_yaml)
        top_layout.addWidget(btn_load)
        top_layout.addWidget(btn_save)
        layout.addLayout(top_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.yaml_form_layout = QFormLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        self.load_yaml()

    def load_yaml(self):
        if not os.path.exists(self.yaml_path): return
        with open(self.yaml_path, 'r', encoding='utf-8') as f:
            self.yaml_data = self.yaml_parser.load(f)

        for i in reversed(range(self.yaml_form_layout.count())): 
            self.yaml_form_layout.itemAt(i).widget().setParent(None)
        self.yaml_widgets.clear()
        
        self.build_yaml_ui(self.yaml_form_layout, self.yaml_widgets, ALLOWED_YAML_PARAMS, self.yaml_data, color_tuple=(0, 255, 255))

    def save_yaml(self):
        if self.yaml_data is None: return
        if self.save_generic_yaml(self.yaml_widgets, self.yaml_data, self.yaml_path, ALLOWED_YAML_PARAMS):
            QMessageBox.information(self, "提示", "填写成功,请重启系统。")

    # ========================== 2. 车辆安全框 YAML ==========================
    def setup_safe_tab(self):
        layout = QVBoxLayout(self.tab_safe)
        top_layout = QHBoxLayout()
        btn_load = QPushButton("重新读取配置")
        btn_load.clicked.connect(self.load_safe_yaml)
        btn_save = QPushButton("保存安全框修改")
        btn_save.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 5px;")
        btn_save.clicked.connect(self.save_safe_yaml)
        top_layout.addWidget(btn_load)
        top_layout.addWidget(btn_save)
        layout.addLayout(top_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.safe_form_layout = QFormLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        self.load_safe_yaml()

    def load_safe_yaml(self):
        if not os.path.exists(self.safe_yaml_path): return
        with open(self.safe_yaml_path, 'r', encoding='utf-8') as f:
            self.safe_yaml_data = self.yaml_parser.load(f)

        for i in reversed(range(self.safe_form_layout.count())): 
            self.safe_form_layout.itemAt(i).widget().setParent(None)
        self.safe_widgets.clear()

        ref_rect = self.yaml_data.get('rect', []) if self.yaml_data else []
        self.build_yaml_ui(self.safe_form_layout, self.safe_widgets, ALLOWED_SAFE_YAML_PARAMS, self.safe_yaml_data, color_tuple=(255, 150, 0), ref_polygon=ref_rect)

    def save_safe_yaml(self):
        if self.safe_yaml_data is None: return
        if self.save_generic_yaml(self.safe_widgets, self.safe_yaml_data, self.safe_yaml_path, ALLOWED_SAFE_YAML_PARAMS):
            QMessageBox.information(self, "提示", "填写成功,请重启系统。")

    # ========================== 3. CFG ==========================
    def setup_cfg_tab(self):
        layout = QVBoxLayout(self.tab_cfg)
        top_layout = QHBoxLayout()
        btn_load = QPushButton("重新读取配置")
        btn_load.clicked.connect(self.load_cfg)
        btn_save = QPushButton("保存动态修改")
        btn_save.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 5px;")
        btn_save.clicked.connect(self.save_cfg)
        top_layout.addWidget(btn_load)
        top_layout.addWidget(btn_save)
        layout.addLayout(top_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.cfg_form_layout = QFormLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        self.load_cfg()

    def load_cfg(self):
        if not os.path.exists(self.cfg_path): return
        with open(self.cfg_path, 'r', encoding='utf-8') as f:
            self.cfg_lines = f.readlines()

        self.cfg_params_meta.clear()
        for i in reversed(range(self.cfg_form_layout.count())): 
            self.cfg_form_layout.itemAt(i).widget().setParent(None)
        self.cfg_widgets.clear()

        for idx, line in enumerate(self.cfg_lines):
            line_strip = line.strip()
            if line_strip.startswith("#") or "gen.add" not in line_strip:
                continue

            start_idx = line.find('(') + 1
            end_idx = line.rfind(')')
            if start_idx <= 0 or end_idx <= 0: continue
            
            inner = line[start_idx:end_idx]
            parts = re.split(r',\s*(?=(?:[^"]*"[^"]*")*[^"]*$)', inner)
            parts = [p.strip() for p in parts]

            if len(parts) >= 5:
                p_name = parts[0].strip('"\'')
                self.cfg_params_meta[p_name] = {
                    'line_idx': idx, 
                    'type': parts[1], 
                    'parts': parts, 
                    'prefix': line[:start_idx],
                    'default': parts[4],
                    'min': parts[5] if len(parts) > 5 else None,
                    'max': parts[6] if len(parts) > 6 else None
                }

        for raw_name, custom_label in ALLOWED_CFG_PARAMS.items():
            if raw_name not in self.cfg_params_meta: continue

            meta = self.cfg_params_meta[raw_name]
            p_type, p_default, p_min, p_max = meta['type'], meta['default'], meta['min'], meta['max']

            w = None
            if p_type == 'bool_t':
                w = QCheckBox()
                w.setChecked(p_default == "True")
            elif p_type == 'int_t':
                w = QSpinBox()
                if p_min: w.setMinimum(int(float(p_min)))
                if p_max: w.setMaximum(int(float(p_max)))
                w.setValue(int(float(p_default)))
            elif p_type == 'double_t':
                w = QDoubleSpinBox()
                w.setDecimals(2)
                w.setSingleStep(0.1)
                if p_min: w.setMinimum(float(p_min))
                if p_max: w.setMaximum(float(p_max))
                w.setValue(float(p_default))

            if w is not None:
                self.cfg_widgets[raw_name] = w
                lbl = QLabel(custom_label + " :")
                lbl.setWordWrap(True)
                lbl.setStyleSheet("font-weight: bold; margin-top: 5px;")
                self.cfg_form_layout.addRow(lbl, w)

    def save_cfg(self):
        if not self.cfg_lines: return

        for raw_name, w in self.cfg_widgets.items():
            meta = self.cfg_params_meta[raw_name]
            idx, parts = meta['line_idx'], meta['parts'].copy()

            if meta['type'] == 'bool_t': new_val_str = "True" if w.isChecked() else "False"
            else: new_val_str = str(w.value())

            parts[4] = new_val_str
            new_inner = ", ".join(parts)
            new_line = f"{meta['prefix']}{new_inner})\n"
            self.cfg_lines[idx] = new_line

        try:
            with open(self.cfg_path, 'w', encoding='utf-8') as f:
                f.writelines(self.cfg_lines)
            QMessageBox.information(self, "提示", "填写成功,请重启系统。")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存 CFG 失败:\n{e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = ConfigEditorGUI()
    gui.show()
    sys.exit(app.exec_())