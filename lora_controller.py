"""
水下机器人 - LoRa无线遥控上位机软件 (双路独立动力调控版)
Underwater Robot - LoRa Remote Controller (Dual Independent Motor Control)
--------------------------------------------------
核心特性:
1. 左右电机独立调速 (Independent Dual-Motor Control):
   - 支持分别独立设定左电机 (Motor A) 与右电机 (Motor B) 输出动力 (0~1000)
   - 支持“🔗 左右电机同步联动 (Link)”开关，一键在同步模式与独立差速模式间自由切换
   - 实时计算并展示差速配平状态与偏置差值 (Δ = L - R)
   - 提供快捷按键：[Q]/[E] 整体调速，[1]/[2] 调左电机，[3]/[4] 调右电机
2. 单电机独立测试模式 (Individual Motor Testing):
   - 提供【⚡ 按住测试左电机】与【⚡ 按住测试右电机】点动按钮，便于单独验证单侧推进器转动
3. 全局事件过滤器 (Global EventFilter):
   - 彻底解决因子控件焦点抢占导致的键盘 WSAD 失灵问题
4. 串口防冲突与异常诊断保护:
   - 友好提示 XCOM 等串口独占冲突，防止程序卡死或异常刷屏
--------------------------------------------------
"""

import sys
import time
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QSlider, QTextEdit,
    QGroupBox, QGridLayout, QFrame, QMessageBox, QProgressBar,
    QCheckBox
)

# 协议常量定义 (与 STM32 端 lora.h 完全对应)
FRAME_HEAD1 = 0x55
FRAME_HEAD2 = 0xAA
FRAME_TAIL1 = 0x0D
FRAME_TAIL2 = 0x0A

CMD_STOP        = 0x00  # 停止
CMD_DIRECT_PWM  = 0x01  # 双电机直接差速
CMD_FORWARD     = 0x02  # 前进
CMD_BACKWARD    = 0x03  # 后退
CMD_TURN_LEFT   = 0x04  # 左转
CMD_TURN_RIGHT  = 0x05  # 右转
CMD_BRAKE       = 0x06  # 刹车


def build_lora_packet(cmd: int, left_speed: int, right_speed: int) -> bytes:
    """构建 10 字节抗干扰控制数据包"""
    left_speed = max(-1000, min(1000, int(left_speed)))
    right_speed = max(-1000, min(1000, int(right_speed)))

    left_u16 = left_speed & 0xFFFF
    right_u16 = right_speed & 0xFFFF

    cmd_b = cmd & 0xFF
    lh = (left_u16 >> 8) & 0xFF
    ll = left_u16 & 0xFF
    rh = (right_u16 >> 8) & 0xFF
    rl = right_u16 & 0xFF

    checksum = (cmd_b + lh + ll + rh + rl) & 0xFF
    return bytes([FRAME_HEAD1, FRAME_HEAD2, cmd_b, lh, ll, rh, rl, checksum, FRAME_TAIL1, FRAME_TAIL2])


class LoRaControllerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("水下机器人 LoRa 遥控上位机 v2.0 (双路独立动力调控版)")
        self.resize(960, 780)
        self.setMinimumSize(880, 700)

        # 核心通信与控制参数
        self.serial_port = None
        self.left_speed = 600         # 左电机 (Motor A) 设定速度 (0 ~ 1000)
        self.right_speed = 600        # 右电机 (Motor B) 设定速度 (0 ~ 1000)
        self.speed_step = 50          # 调速步进值
        self.active_keys = set()      # 当前正被按下的有效控制键
        self.current_motion = "STOP"  # 当前运动状态
        self.was_moving = False
        self.last_warn_time = 0

        # 发送循环定时器 (按键按下期间以 80ms 周期持续发送心跳帧, 保证下位机平稳输出 PWM 且空口不堵塞)
        self.send_timer = QTimer(self)
        self.send_timer.setInterval(80)  # 12.5 Hz (适应 LoRa 模块空中无线传输时长)
        self.send_timer.timeout.connect(self.on_send_timer_tick)

        # 串口接收监控定时器
        self.recv_timer = QTimer(self)
        self.recv_timer.setInterval(30)
        self.recv_timer.timeout.connect(self.read_serial_data)

        # 初始化 UI
        self.init_ui()
        self.apply_dark_theme()
        self.refresh_ports()

        # 安装全局事件过滤器，杜绝因焦点问题导致的按键无法响应！
        QApplication.instance().installEventFilter(self)

    @property
    def current_speed(self):
        """兼容属性: 左右电机均值"""
        return (self.left_speed + self.right_speed) // 2

    def init_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(16, 16, 16, 16)

        # 1. 顶部标题栏与输入法提示
        title_box = QVBoxLayout()
        title_label = QLabel("🌊 水下微型机器人 (ROV) - LoRa 无线遥控驾驶台")
        title_label.setFont(QFont("Microsoft YaHei", 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_box.addWidget(title_label)

        sub_label = QLabel("💡 提示：键盘 WSAD 航行请切至【英文输入法】；支持左右电机独立设定与差速配平！")
        sub_label.setFont(QFont("Microsoft YaHei", 9))
        sub_label.setStyleSheet("color: #FFD54F;")
        sub_label.setAlignment(Qt.AlignCenter)
        title_box.addWidget(sub_label)
        main_layout.addLayout(title_box)

        # 2. 串口配置栏
        port_group = QGroupBox("串口通信连接")
        port_layout = QHBoxLayout(port_group)

        port_layout.addWidget(QLabel("串口号:"))
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(180)
        self.combo_ports.setFocusPolicy(Qt.NoFocus)
        port_layout.addWidget(self.combo_ports)

        btn_refresh = QPushButton("🔄 刷新串口")
        btn_refresh.setFocusPolicy(Qt.NoFocus)
        btn_refresh.clicked.connect(self.refresh_ports)
        port_layout.addWidget(btn_refresh)

        port_layout.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["9600", "115200", "19200", "38400", "57600"])
        self.combo_baud.setCurrentText("9600")
        self.combo_baud.setFocusPolicy(Qt.NoFocus)
        port_layout.addWidget(self.combo_baud)

        self.btn_connect = QPushButton("🔌 打开串口")
        self.btn_connect.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        self.btn_connect.setFocusPolicy(Qt.NoFocus)
        self.btn_connect.clicked.connect(self.toggle_serial_connection)
        port_layout.addWidget(self.btn_connect)

        self.lbl_status = QLabel("● 未连接")
        self.lbl_status.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        self.lbl_status.setStyleSheet("color: #FF5252;")
        port_layout.addWidget(self.lbl_status)
        port_layout.addStretch()

        main_layout.addWidget(port_group)

        # 3. 中间核心控制区: 左右分栏
        middle_layout = QHBoxLayout()

        # 3.1 左侧: 双路推进器动力调控与独立设定 (核心改进)
        speed_group = QGroupBox("双路推进器动力调控与独立设定")
        speed_layout = QVBoxLayout(speed_group)
        speed_layout.setSpacing(10)

        # 顶部模式栏: 联动开关、左右对齐、恢复默认
        top_ctrl_layout = QHBoxLayout()
        self.chk_link = QCheckBox("🔗 左右电机同步联动 (Link)")
        self.chk_link.setChecked(True)
        self.chk_link.setFocusPolicy(Qt.NoFocus)
        self.chk_link.toggled.connect(self.on_link_toggled)
        top_ctrl_layout.addWidget(self.chk_link)

        top_ctrl_layout.addStretch()

        self.btn_align = QPushButton("⚖️ 左右对齐 (L=R)")
        self.btn_align.setToolTip("将右电机设定值快速对齐为左电机当前值")
        self.btn_align.setFocusPolicy(Qt.NoFocus)
        self.btn_align.clicked.connect(self.align_speeds)
        top_ctrl_layout.addWidget(self.btn_align)

        self.btn_reset = QPushButton("🔄 重置默认 (600)")
        self.btn_reset.setToolTip("将左右电机设定值恢复至标准默认 600 (60%)")
        self.btn_reset.setFocusPolicy(Qt.NoFocus)
        self.btn_reset.clicked.connect(self.reset_default_speeds)
        top_ctrl_layout.addWidget(self.btn_reset)

        speed_layout.addLayout(top_ctrl_layout)

        # 双电机独立控制卡片 (并排两列展示)
        dual_motor_layout = QHBoxLayout()
        dual_motor_layout.setSpacing(12)

        # ----------------- 左电机卡片 (Motor A) -----------------
        self.card_left = QFrame()
        self.card_left.setObjectName("cardLeft")
        self.card_left.setStyleSheet("""
            QFrame#cardLeft {
                background-color: #0F172A;
                border: 1px solid #00B0FF;
                border-radius: 8px;
                padding: 6px;
            }
        """)
        layout_l = QVBoxLayout(self.card_left)
        layout_l.setSpacing(8)

        lbl_title_l = QLabel("🔵 左推进器 (Motor A / 左路)")
        lbl_title_l.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        lbl_title_l.setStyleSheet("color: #00E5FF;")
        layout_l.addWidget(lbl_title_l)

        # 速度大字显示
        info_l = QHBoxLayout()
        self.lbl_left_val = QLabel(f"{self.left_speed}")
        self.lbl_left_val.setFont(QFont("Impact", 30))
        self.lbl_left_val.setStyleSheet("color: #00E5FF;")

        self.lbl_left_pct = QLabel(f"({self.left_speed // 10}%)")
        self.lbl_left_pct.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        self.lbl_left_pct.setStyleSheet("color: #80D8FF;")

        info_l.addWidget(self.lbl_left_val)
        info_l.addWidget(self.lbl_left_pct)
        info_l.addStretch()
        layout_l.addLayout(info_l)

        # 进度条
        self.bar_left = QProgressBar()
        self.bar_left.setRange(0, 1000)
        self.bar_left.setValue(self.left_speed)
        self.bar_left.setTextVisible(False)
        self.bar_left.setFixedHeight(10)
        self.bar_left.setStyleSheet("""
            QProgressBar { background-color: #1E293B; border-radius: 5px; }
            QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0091EA, stop:1 #00E5FF); border-radius: 5px; }
        """)
        layout_l.addWidget(self.bar_left)

        # 滑块
        self.slider_left = QSlider(Qt.Horizontal)
        self.slider_left.setRange(0, 1000)
        self.slider_left.setSingleStep(10)
        self.slider_left.setPageStep(50)
        self.slider_left.setValue(self.left_speed)
        self.slider_left.setFocusPolicy(Qt.NoFocus)
        self.slider_left.valueChanged.connect(self.on_slider_left_changed)
        layout_l.addWidget(self.slider_left)

        # 微调按钮
        btn_box_l = QHBoxLayout()
        self.btn_dec_left = QPushButton("➖ -50 [1]")
        self.btn_dec_left.setFocusPolicy(Qt.NoFocus)
        self.btn_dec_left.clicked.connect(self.dec_left)
        btn_box_l.addWidget(self.btn_dec_left)

        self.btn_inc_left = QPushButton("➕ +50 [2]")
        self.btn_inc_left.setFocusPolicy(Qt.NoFocus)
        self.btn_inc_left.clicked.connect(self.inc_left)
        btn_box_l.addWidget(self.btn_inc_left)
        layout_l.addLayout(btn_box_l)

        # 单独测试按钮 (点动测试)
        self.btn_test_left = QPushButton("⚡ 按住单独测试左电机")
        self.btn_test_left.setFocusPolicy(Qt.NoFocus)
        self.btn_test_left.setStyleSheet("background-color: #0E3A52; color: #80D8FF; font-weight: bold; border: 1px solid #00B0FF; padding: 6px;")
        self.btn_test_left.pressed.connect(self.on_test_left_press)
        self.btn_test_left.released.connect(self.on_test_left_release)
        layout_l.addWidget(self.btn_test_left)

        dual_motor_layout.addWidget(self.card_left)

        # ----------------- 右电机卡片 (Motor B) -----------------
        self.card_right = QFrame()
        self.card_right.setObjectName("cardRight")
        self.card_right.setStyleSheet("""
            QFrame#cardRight {
                background-color: #0F172A;
                border: 1px solid #00E676;
                border-radius: 8px;
                padding: 6px;
            }
        """)
        layout_r = QVBoxLayout(self.card_right)
        layout_r.setSpacing(8)

        lbl_title_r = QLabel("🟢 右推进器 (Motor B / 右路)")
        lbl_title_r.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        lbl_title_r.setStyleSheet("color: #00E676;")
        layout_r.addWidget(lbl_title_r)

        # 速度大字显示
        info_r = QHBoxLayout()
        self.lbl_right_val = QLabel(f"{self.right_speed}")
        self.lbl_right_val.setFont(QFont("Impact", 30))
        self.lbl_right_val.setStyleSheet("color: #00E676;")

        self.lbl_right_pct = QLabel(f"({self.right_speed // 10}%)")
        self.lbl_right_pct.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        self.lbl_right_pct.setStyleSheet("color: #A7F3D0;")

        info_r.addWidget(self.lbl_right_val)
        info_r.addWidget(self.lbl_right_pct)
        info_r.addStretch()
        layout_r.addLayout(info_r)

        # 进度条
        self.bar_right = QProgressBar()
        self.bar_right.setRange(0, 1000)
        self.bar_right.setValue(self.right_speed)
        self.bar_right.setTextVisible(False)
        self.bar_right.setFixedHeight(10)
        self.bar_right.setStyleSheet("""
            QProgressBar { background-color: #1E293B; border-radius: 5px; }
            QProgressBar::chunk { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00B348, stop:1 #00E676); border-radius: 5px; }
        """)
        layout_r.addWidget(self.bar_right)

        # 滑块
        self.slider_right = QSlider(Qt.Horizontal)
        self.slider_right.setRange(0, 1000)
        self.slider_right.setSingleStep(10)
        self.slider_right.setPageStep(50)
        self.slider_right.setValue(self.right_speed)
        self.slider_right.setFocusPolicy(Qt.NoFocus)
        self.slider_right.valueChanged.connect(self.on_slider_right_changed)
        layout_r.addWidget(self.slider_right)

        # 微调按钮
        btn_box_r = QHBoxLayout()
        self.btn_dec_right = QPushButton("➖ -50 [3]")
        self.btn_dec_right.setFocusPolicy(Qt.NoFocus)
        self.btn_dec_right.clicked.connect(self.dec_right)
        btn_box_r.addWidget(self.btn_dec_right)

        self.btn_inc_right = QPushButton("➕ +50 [4]")
        self.btn_inc_right.setFocusPolicy(Qt.NoFocus)
        self.btn_inc_right.clicked.connect(self.inc_right)
        btn_box_r.addWidget(self.btn_inc_right)
        layout_r.addLayout(btn_box_r)

        # 单独测试按钮 (点动测试)
        self.btn_test_right = QPushButton("⚡ 按住单独测试右电机")
        self.btn_test_right.setFocusPolicy(Qt.NoFocus)
        self.btn_test_right.setStyleSheet("background-color: #0B3D2E; color: #A7F3D0; font-weight: bold; border: 1px solid #00E676; padding: 6px;")
        self.btn_test_right.pressed.connect(self.on_test_right_press)
        self.btn_test_right.released.connect(self.on_test_right_release)
        layout_r.addWidget(self.btn_test_right)

        dual_motor_layout.addWidget(self.card_right)

        speed_layout.addLayout(dual_motor_layout)

        # 差速配平状态栏与航行姿态
        status_box = QVBoxLayout()
        status_box.setSpacing(4)

        self.lbl_trim_diff = QLabel("⚖️ 左右平衡 (L:600 / R:600, 差值 0)")
        self.lbl_trim_diff.setFont(QFont("Microsoft YaHei", 9))
        self.lbl_trim_diff.setStyleSheet("color: #80D8FF;")
        status_box.addWidget(self.lbl_trim_diff)

        motion_layout = QHBoxLayout()
        motion_layout.addWidget(QLabel("航行状态:"))
        self.lbl_motion = QLabel("🛑 静止停机 (IDLE)")
        self.lbl_motion.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        self.lbl_motion.setStyleSheet("color: #FFB300;")
        motion_layout.addWidget(self.lbl_motion)
        motion_layout.addStretch()
        status_box.addLayout(motion_layout)

        speed_layout.addLayout(status_box)

        speed_layout.addStretch()
        middle_layout.addWidget(speed_group, stretch=6)

        # 3.2 右侧: 虚拟键盘与操作指南
        nav_group = QGroupBox("按键操控面板 (支持键盘按压与鼠标按住)")
        nav_layout = QVBoxLayout(nav_group)

        # 虚拟 WASD 面板
        grid = QGridLayout()
        grid.setSpacing(10)

        self.btn_w = QPushButton("W\n▲ 前进")
        self.btn_a = QPushButton("A\n◀ 左转")
        self.btn_s = QPushButton("S\n▼ 后退")
        self.btn_d = QPushButton("D\n▶ 右转")
        self.btn_space = QPushButton("SPACE 空格\n紧急刹车")

        for btn in [self.btn_w, self.btn_a, self.btn_s, self.btn_d]:
            btn.setFont(QFont("Microsoft YaHei", 11, QFont.Bold))
            btn.setFixedSize(95, 60)
            btn.setFocusPolicy(Qt.NoFocus)

        self.btn_space.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        self.btn_space.setFixedHeight(45)
        self.btn_space.setFocusPolicy(Qt.NoFocus)

        # 绑定鼠标按下/松开事件，支持用鼠标直接按住操纵！
        self.btn_w.pressed.connect(lambda: self.on_virtual_key_press('W'))
        self.btn_w.released.connect(lambda: self.on_virtual_key_release('W'))
        self.btn_s.pressed.connect(lambda: self.on_virtual_key_press('S'))
        self.btn_s.released.connect(lambda: self.on_virtual_key_release('S'))
        self.btn_a.pressed.connect(lambda: self.on_virtual_key_press('A'))
        self.btn_a.released.connect(lambda: self.on_virtual_key_release('A'))
        self.btn_d.pressed.connect(lambda: self.on_virtual_key_press('D'))
        self.btn_d.released.connect(lambda: self.on_virtual_key_release('D'))
        self.btn_space.clicked.connect(self.on_virtual_brake)

        grid.addWidget(self.btn_w, 0, 1)
        grid.addWidget(self.btn_a, 1, 0)
        grid.addWidget(self.btn_s, 1, 1)
        grid.addWidget(self.btn_d, 1, 2)
        grid.addWidget(self.btn_space, 2, 0, 1, 3)

        nav_layout.addLayout(grid)

        # 操作规则提示卡片
        tip_frame = QFrame()
        tip_frame.setFrameShape(QFrame.StyledPanel)
        tip_layout = QVBoxLayout(tip_frame)
        tip_layout.setContentsMargins(8, 8, 8, 8)
        tip_layout.addWidget(QLabel("📌 <b>核心操纵与调速指南:</b>"))
        tip_layout.addWidget(QLabel("• <b>WSAD 航行</b>：按住持续发包航行，松手立即发送 STOP 停转"))
        tip_layout.addWidget(QLabel("• <b>[Q] / [E]</b>：双轮整体降低 / 提高设定功率（步长 50）"))
        tip_layout.addWidget(QLabel("• <b>[1] / [2]</b>：独立降低 / 增加<b>左电机</b>动力"))
        tip_layout.addWidget(QLabel("• <b>[3] / [4]</b>：独立降低 / 增加<b>右电机</b>动力"))
        tip_layout.addWidget(QLabel("• <b>取消联动</b>：取消勾选【同步联动】，左右电机可随意差速配平"))
        tip_layout.addWidget(QLabel("• <b>单电机点动</b>：按住【⚡ 单独测试】按钮可单独调试单侧电机"))
        tip_layout.addWidget(QLabel("• <b>[空格键]</b>：触发两路 H 桥短路紧急制动"))
        nav_layout.addWidget(tip_frame)

        middle_layout.addWidget(nav_group, stretch=5)
        main_layout.addLayout(middle_layout)

        # 4. 底部: 实时数据交互日志
        log_group = QGroupBox("LoRa 数据通信监视")
        log_layout = QVBoxLayout(log_group)

        log_ctrl_layout = QHBoxLayout()
        log_ctrl_layout.addWidget(QLabel("TX/RX 串口数据流:"))
        log_ctrl_layout.addStretch()

        btn_clear_log = QPushButton("清空日志")
        btn_clear_log.setFocusPolicy(Qt.NoFocus)
        btn_clear_log.clicked.connect(self.clear_log)
        log_ctrl_layout.addWidget(btn_clear_log)
        log_layout.addLayout(log_ctrl_layout)

        self.log_text = QTextEdit()
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setReadOnly(True)
        self.log_text.setFixedHeight(120)
        self.log_text.setFocusPolicy(Qt.NoFocus)
        log_layout.addWidget(self.log_text)

        main_layout.addWidget(log_group)

    def apply_dark_theme(self):
        """深海科技感界面暗黑配色"""
        dark_style = """
        QWidget {
            background-color: #121820;
            color: #E0E6ED;
            font-family: 'Segoe UI', 'Microsoft YaHei';
        }
        QGroupBox {
            border: 1px solid #2B3A4A;
            border-radius: 8px;
            margin-top: 14px;
            padding-top: 14px;
            font-weight: bold;
            color: #4FC3F7;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 0 6px;
            left: 12px;
        }
        QCheckBox {
            color: #E0E6ED;
            font-size: 11px;
            font-weight: bold;
            spacing: 6px;
        }
        QCheckBox::indicator {
            width: 15px;
            height: 15px;
            border-radius: 3px;
            border: 1px solid #38BDF8;
            background-color: #1E293B;
        }
        QCheckBox::indicator:checked {
            background-color: #00E5FF;
            border: 1px solid #FFFFFF;
        }
        QPushButton {
            background-color: #1E293B;
            border: 1px solid #334155;
            border-radius: 6px;
            padding: 6px 12px;
            color: #F8FAFC;
        }
        QPushButton:hover {
            background-color: #334155;
            border-color: #38BDF8;
        }
        QPushButton:pressed {
            background-color: #0284C7;
            border-color: #0284C7;
        }
        QComboBox {
            background-color: #1E293B;
            border: 1px solid #334155;
            border-radius: 4px;
            padding: 4px 8px;
            color: #F8FAFC;
        }
        QSlider::groove:horizontal {
            height: 6px;
            background: #1E293B;
            border-radius: 3px;
        }
        QSlider::sub-page:horizontal {
            background: #00E5FF;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #FFFFFF;
            border: 2px solid #00E5FF;
            width: 16px;
            margin-top: -5px;
            margin-bottom: -5px;
            border-radius: 8px;
        }
        QTextEdit {
            background-color: #0A0F14;
            border: 1px solid #1E293B;
            border-radius: 6px;
            color: #A0AEC0;
        }
        """
        self.setStyleSheet(dark_style)

    # -------------------------------------------------------------
    # 左右电机独立调速逻辑
    # -------------------------------------------------------------
    def set_left_speed(self, val: int, sync: bool = True):
        """设定左电机速度 (0 ~ 1000)"""
        val = max(0, min(1000, int(val)))
        self.left_speed = val
        self.lbl_left_val.setText(str(val))
        self.lbl_left_pct.setText(f"({val // 10}%)" if val > 0 else "(停转)")
        self.bar_left.setValue(val)
        self.slider_left.blockSignals(True)
        self.slider_left.setValue(val)
        self.slider_left.blockSignals(False)

        if sync and self.chk_link.isChecked():
            self.set_right_speed(val, sync=False)

        self.update_trim_info()

    def set_right_speed(self, val: int, sync: bool = True):
        """设定右电机速度 (0 ~ 1000)"""
        val = max(0, min(1000, int(val)))
        self.right_speed = val
        self.lbl_right_val.setText(str(val))
        self.lbl_right_pct.setText(f"({val // 10}%)" if val > 0 else "(停转)")
        self.bar_right.setValue(val)
        self.slider_right.blockSignals(True)
        self.slider_right.setValue(val)
        self.slider_right.blockSignals(False)

        if sync and self.chk_link.isChecked():
            self.set_left_speed(val, sync=False)

        self.update_trim_info()

    def on_slider_left_changed(self, val: int):
        self.set_left_speed(val, sync=True)

    def on_slider_right_changed(self, val: int):
        self.set_right_speed(val, sync=True)

    def inc_left(self):
        """左电机加速"""
        self.set_left_speed(self.left_speed + self.speed_step, sync=self.chk_link.isChecked())

    def dec_left(self):
        """左电机减速"""
        self.set_left_speed(self.left_speed - self.speed_step, sync=self.chk_link.isChecked())

    def inc_right(self):
        """右电机加速"""
        self.set_right_speed(self.right_speed + self.speed_step, sync=self.chk_link.isChecked())

    def dec_right(self):
        """右电机减速"""
        self.set_right_speed(self.right_speed - self.speed_step, sync=self.chk_link.isChecked())

    def increase_speed(self):
        """双轮整体加速 [E] / [↑]"""
        if self.chk_link.isChecked():
            new_v = min(1000, max(self.left_speed, self.right_speed) + self.speed_step)
            self.set_left_speed(new_v, sync=False)
            self.set_right_speed(new_v, sync=False)
        else:
            self.set_left_speed(self.left_speed + self.speed_step, sync=False)
            self.set_right_speed(self.right_speed + self.speed_step, sync=False)

    def decrease_speed(self):
        """双轮整体减速 [Q] / [↓]"""
        if self.chk_link.isChecked():
            new_v = max(0, min(self.left_speed, self.right_speed) - self.speed_step)
            self.set_left_speed(new_v, sync=False)
            self.set_right_speed(new_v, sync=False)
        else:
            self.set_left_speed(self.left_speed - self.speed_step, sync=False)
            self.set_right_speed(self.right_speed - self.speed_step, sync=False)

    def align_speeds(self):
        """一键左右对齐"""
        target = self.left_speed
        self.chk_link.setChecked(True)
        self.set_right_speed(target, sync=False)
        self.append_log(f"[系统] 左右推进器速度已对齐为 {target}")

    def reset_default_speeds(self):
        """恢复默认速度 600"""
        self.chk_link.setChecked(True)
        self.set_left_speed(600, sync=False)
        self.set_right_speed(600, sync=False)
        self.append_log("[系统] 左右推进器速度已恢复默认值 600 (60%)")

    def on_link_toggled(self, checked: bool):
        if checked:
            # 开启联动时，将右电机自动对齐到左电机
            self.set_right_speed(self.left_speed, sync=False)
            self.append_log("[系统] 已开启【左右同步联动】模式")
        else:
            self.append_log("[系统] 已解除联动，进入【双路独立调速/差速配平】模式")
        self.update_trim_info()

    def update_trim_info(self):
        """更新差速配平与偏差显示"""
        diff = self.left_speed - self.right_speed
        if diff == 0:
            self.lbl_trim_diff.setText(f"⚖️ 左右平衡 (L:{self.left_speed} / R:{self.right_speed}, 差值 0)")
            self.lbl_trim_diff.setStyleSheet("color: #80D8FF; font-size: 11px;")
        elif diff > 0:
            self.lbl_trim_diff.setText(f"◀ 左强右弱 (L:{self.left_speed} / R:{self.right_speed}, 差值 +{diff}) - 航向将微右偏修正")
            self.lbl_trim_diff.setStyleSheet("color: #FFD54F; font-size: 11px;")
        else:
            self.lbl_trim_diff.setText(f"▶ 右强左弱 (L:{self.left_speed} / R:{self.right_speed}, 差值 {diff}) - 航向将微左偏修正")
            self.lbl_trim_diff.setStyleSheet("color: #FFD54F; font-size: 11px;")

    # -------------------------------------------------------------
    # 单电机独立点动测试逻辑
    # -------------------------------------------------------------
    def on_test_left_press(self):
        """鼠标按住: 仅转动左电机"""
        self.active_keys.add('TEST_L')
        self.btn_test_left.setStyleSheet("background-color: #00E5FF; color: #000; font-weight: bold; border: 1px solid #FFF; padding: 6px;")
        self.update_motion_state()

    def on_test_left_release(self):
        """鼠标松开: 停止左电机"""
        self.active_keys.discard('TEST_L')
        self.btn_test_left.setStyleSheet("background-color: #0E3A52; color: #80D8FF; font-weight: bold; border: 1px solid #00B0FF; padding: 6px;")
        self.update_motion_state()

    def on_test_right_press(self):
        """鼠标按住: 仅转动右电机"""
        self.active_keys.add('TEST_R')
        self.btn_test_right.setStyleSheet("background-color: #00E676; color: #000; font-weight: bold; border: 1px solid #FFF; padding: 6px;")
        self.update_motion_state()

    def on_test_right_release(self):
        """鼠标松开: 停止右电机"""
        self.active_keys.discard('TEST_R')
        self.btn_test_right.setStyleSheet("background-color: #0B3D2E; color: #A7F3D0; font-weight: bold; border: 1px solid #00E676; padding: 6px;")
        self.update_motion_state()

    # -------------------------------------------------------------
    # 全局事件过滤器 (杜绝子控件焦点抢占导致键盘按键失灵)
    # -------------------------------------------------------------
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress:
            if not event.isAutoRepeat():
                if self.handle_key_down(event):
                    return True
        elif event.type() == QEvent.KeyRelease:
            if not event.isAutoRepeat():
                if self.handle_key_up(event):
                    return True
        return super().eventFilter(obj, event)

    def handle_key_down(self, event):
        """统一按键按下处理"""
        key = event.key()
        text = event.text().upper()

        # 双轮整体加速/减速键
        if key in (Qt.Key_Q, Qt.Key_Down, Qt.Key_Minus) or text == 'Q':
            self.decrease_speed()
            return True
        elif key in (Qt.Key_E, Qt.Key_Up, Qt.Key_Plus, Qt.Key_Equal) or text == 'E':
            self.increase_speed()
            return True

        # 左电机独立调速 [1] 减速, [2] 加速
        if key == Qt.Key_1 or text == '1':
            self.dec_left()
            return True
        elif key == Qt.Key_2 or text == '2':
            self.inc_left()
            return True

        # 右电机独立调速 [3] 减速, [4] 加速
        if key == Qt.Key_3 or text == '3':
            self.dec_right()
            return True
        elif key == Qt.Key_4 or text == '4':
            self.inc_right()
            return True

        # 运动控制键
        if key == Qt.Key_W or text == 'W':
            self.on_virtual_key_press('W')
            return True
        elif key == Qt.Key_S or text == 'S':
            self.on_virtual_key_press('S')
            return True
        elif key == Qt.Key_A or text == 'A':
            self.on_virtual_key_press('A')
            return True
        elif key == Qt.Key_D or text == 'D':
            self.on_virtual_key_press('D')
            return True
        elif key == Qt.Key_Space:
            self.on_virtual_brake()
            return True

        return False

    def handle_key_up(self, event):
        """统一按键释放处理"""
        key = event.key()
        text = event.text().upper()

        if key == Qt.Key_W or text == 'W':
            self.on_virtual_key_release('W')
            return True
        elif key == Qt.Key_S or text == 'S':
            self.on_virtual_key_release('S')
            return True
        elif key == Qt.Key_A or text == 'A':
            self.on_virtual_key_release('A')
            return True
        elif key == Qt.Key_D or text == 'D':
            self.on_virtual_key_release('D')
            return True
        elif key == Qt.Key_Space:
            self.highlight_button(self.btn_space, False)
            return True

        return False

    def on_virtual_key_press(self, key_char: str):
        """虚拟按键/键盘通用的按下动作"""
        self.active_keys.add(key_char)
        if key_char == 'W':
            self.highlight_button(self.btn_w, True)
        elif key_char == 'S':
            self.highlight_button(self.btn_s, True)
        elif key_char == 'A':
            self.highlight_button(self.btn_a, True)
        elif key_char == 'D':
            self.highlight_button(self.btn_d, True)

        self.update_motion_state()

    def on_virtual_key_release(self, key_char: str):
        """虚拟按键/键盘通用的松开动作"""
        self.active_keys.discard(key_char)
        if key_char == 'W':
            self.highlight_button(self.btn_w, False)
        elif key_char == 'S':
            self.highlight_button(self.btn_s, False)
        elif key_char == 'A':
            self.highlight_button(self.btn_a, False)
        elif key_char == 'D':
            self.highlight_button(self.btn_d, False)

        self.update_motion_state()

    def on_virtual_brake(self):
        """触发刹车"""
        self.active_keys.clear()
        self.highlight_button(self.btn_space, True)
        self.send_packet(CMD_BRAKE, 0, 0)
        self.update_motion_label("BRAKE")
        QTimer.singleShot(400, lambda: self.highlight_button(self.btn_space, False))

    # -------------------------------------------------------------
    # 航行状态计算与周期发送循环
    # -------------------------------------------------------------
    def update_motion_state(self):
        """根据当前 active_keys 计算当前航行动作并更新发包"""
        if not self.active_keys:
            # 没有按键按下 -> 立即停止发送并向下位机发送 STOP
            if self.was_moving:
                self.send_stop_packet()
                self.was_moving = False
            self.send_timer.stop()
            self.current_motion = "STOP"
            self.update_motion_label("STOP")
        else:
            self.was_moving = True
            if 'TEST_L' in self.active_keys:
                self.current_motion = "TEST_L"
            elif 'TEST_R' in self.active_keys:
                self.current_motion = "TEST_R"
            elif 'W' in self.active_keys and 'S' not in self.active_keys:
                if 'A' in self.active_keys:
                    self.current_motion = "LEFT_FWD"
                elif 'D' in self.active_keys:
                    self.current_motion = "RIGHT_FWD"
                else:
                    self.current_motion = "FORWARD"
            elif 'S' in self.active_keys and 'W' not in self.active_keys:
                self.current_motion = "BACKWARD"
            elif 'A' in self.active_keys:
                self.current_motion = "TURN_LEFT"
            elif 'D' in self.active_keys:
                self.current_motion = "TURN_RIGHT"
            else:
                self.current_motion = "STOP"

            self.update_motion_label(self.current_motion)

            # 立即触发一次发送，并在存在持续运动时保持 80ms 周期发送
            self.on_send_timer_tick()
            if not self.send_timer.isActive() and self.current_motion != "STOP":
                self.send_timer.start()

    def on_send_timer_tick(self):
        """按键处于按下状态期间，以 12.5Hz 频率持续向下位机发送控制帧"""
        if not self.active_keys or self.current_motion == "STOP":
            return

        lspd = self.left_speed
        rspd = self.right_speed

        if self.current_motion == "TEST_L":
            # 仅向左电机输出设定 PWM，右电机为 0
            self.send_packet(CMD_DIRECT_PWM, lspd, 0)
        elif self.current_motion == "TEST_R":
            # 仅向右电机输出设定 PWM，左电机为 0
            self.send_packet(CMD_DIRECT_PWM, 0, rspd)
        elif self.current_motion == "FORWARD":
            # 前进：分别输出左右电机的独立设定值 (完全支持差速直线修正)
            self.send_packet(CMD_DIRECT_PWM, lspd, rspd)
        elif self.current_motion == "BACKWARD":
            # 后退：分别反向输出左右电机设定值
            self.send_packet(CMD_DIRECT_PWM, -lspd, -rspd)
        elif self.current_motion == "TURN_LEFT":
            # 左转：左轮反转，右轮正转
            self.send_packet(CMD_DIRECT_PWM, -lspd, rspd)
        elif self.current_motion == "TURN_RIGHT":
            # 右转：左轮正转，右轮反转
            self.send_packet(CMD_DIRECT_PWM, lspd, -rspd)
        elif self.current_motion == "LEFT_FWD":
            # 前进偏左：左轮按 40% 功率降速
            self.send_packet(CMD_DIRECT_PWM, int(lspd * 0.4), rspd)
        elif self.current_motion == "RIGHT_FWD":
            # 前进偏右：右轮按 40% 功率降速
            self.send_packet(CMD_DIRECT_PWM, lspd, int(rspd * 0.4))

    def update_motion_label(self, motion: str):
        """更新界面上的航行状态提示"""
        styles = {
            "STOP": ("🛑 静止停机 (IDLE)", "#FFB300"),
            "FORWARD": ("▲ 前进航行 (FORWARD)", "#00E676"),
            "BACKWARD": ("▼ 后退倒车 (BACKWARD)", "#FF9100"),
            "TURN_LEFT": ("◀ 原地/差速左转 (LEFT)", "#00E5FF"),
            "TURN_RIGHT": ("▶ 原地/差速右转 (RIGHT)", "#00E5FF"),
            "LEFT_FWD": ("▲◀ 前进左偏航", "#76FF03"),
            "RIGHT_FWD": ("▲▶ 前进右偏航", "#76FF03"),
            "BRAKE": ("⚠️ 紧急制动 (BRAKE)", "#FF1744"),
            "TEST_L": ("🔵 单独测试左电机 (MOTOR A)", "#00E5FF"),
            "TEST_R": ("🟢 单独测试右电机 (MOTOR B)", "#00E676"),
        }
        text, color = styles.get(motion, ("🛑 静止停机", "#FFB300"))
        self.lbl_motion.setText(text)
        self.lbl_motion.setStyleSheet(f"color: {color}; font-weight: bold;")

    def highlight_button(self, btn: QPushButton, active: bool):
        """按键按下/松开时的虚拟界面按钮高亮动效"""
        if active:
            btn.setStyleSheet("""
                background-color: #00E5FF;
                color: #000000;
                border: 2px solid #FFFFFF;
                font-weight: bold;
            """)
        else:
            btn.setStyleSheet("")

    # -------------------------------------------------------------
    # 串口连接与通信底层
    # -------------------------------------------------------------
    def refresh_ports(self):
        """扫描本地串口"""
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(f"{p.device} ({p.description})", p.device)
        if not ports:
            self.combo_ports.addItem("未检测到串口", "")

    def toggle_serial_connection(self):
        """打开 / 关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            # 执行断开
            self.send_stop_packet()
            self.send_timer.stop()
            self.recv_timer.stop()
            self.serial_port.close()
            self.serial_port = None
            self.btn_connect.setText("🔌 打开串口")
            self.lbl_status.setText("● 未连接")
            self.lbl_status.setStyleSheet("color: #FF5252;")
            self.append_log("[系统] 串口已断开")
        else:
            # 执行连接
            port = self.combo_ports.currentData()
            if not port:
                QMessageBox.warning(self, "警告", "请选择有效的 COM 端口！")
                return

            baud = int(self.combo_baud.currentText())
            try:
                self.serial_port = serial.Serial(port, baud, timeout=0.1)
                self.btn_connect.setText("🔌 关闭串口")
                self.lbl_status.setText("● 已连接 " + port)
                self.lbl_status.setStyleSheet("color: #00E676;")
                self.append_log(f"[系统] 成功打开串口 {port}, 波特率 {baud}")
                self.recv_timer.start()
            except Exception as e:
                err_msg = str(e)
                if "拒绝访问" in err_msg or "PermissionError" in err_msg or "Access is denied" in err_msg:
                    tip = f"无法打开串口 {port}：访问被拒绝！\n\n【原因】：当前端口正被其他软件（如 XCOM、SSCOM 串口调试助手）独占使用。\n【解决办法】：请在其他串口软件中点击“关闭串口”或退出该软件，然后再试。"
                else:
                    tip = f"无法打开串口 {port}：\n{err_msg}"
                QMessageBox.critical(self, "串口打开失败", tip)
                self.append_log(f"[错误] 打开串口失败: {err_msg}")

    def send_packet(self, cmd: int, left_speed: int, right_speed: int):
        """通过串口发送 10 字节协议包"""
        pkt = build_lora_packet(cmd, left_speed, right_speed)
        hex_str = " ".join([f"{b:02X}" for b in pkt])

        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(pkt)
                self.append_log(f"[TX] {hex_str} (CMD:{cmd}, L:{left_speed}, R:{right_speed})")
            except Exception as e:
                self.handle_serial_error(e)
        else:
            now = time.time()
            if now - self.last_warn_time > 1.5:
                self.append_log(f"[模拟控制] {hex_str} (L:{left_speed}, R:{right_speed} - 串口未连接)")
                self.last_warn_time = now

    def handle_serial_error(self, err: Exception):
        """串口出现严重异常（如被抢占、拔出、拒绝访问）时的自动保护处理"""
        err_str = str(err)
        self.send_timer.stop()
        self.recv_timer.stop()
        self.active_keys.clear()
        self.update_motion_label("STOP")

        if "拒绝访问" in err_str or "PermissionError" in err_str or "Access is denied" in err_str:
            diag_msg = "【串口被占用】Windows拒绝访问该COM口！请关闭其他占用该端口的串口助手后重试！"
        else:
            diag_msg = f"【串口通信中断】{err_str}"

        self.append_log(f"[错误] {diag_msg}")

        try:
            if self.serial_port:
                self.serial_port.close()
        except Exception:
            pass
        self.serial_port = None

        self.btn_connect.setText("🔌 打开串口")
        self.lbl_status.setText("● 端口异常断开")
        self.lbl_status.setStyleSheet("color: #FF5252;")
        QMessageBox.warning(self, "串口连接异常", f"{diag_msg}\n\n已自动断开连接，请排查后重新打开串口。")

    def send_stop_packet(self):
        """发送停止包 (连续发送3次增强抗干扰)"""
        for _ in range(3):
            self.send_packet(CMD_STOP, 0, 0)
            time.sleep(0.005)

    def read_serial_data(self):
        """读取串口回传信息"""
        if self.serial_port and self.serial_port.is_open:
            try:
                n = self.serial_port.in_waiting
                if n > 0:
                    data = self.serial_port.read(n)
                    hex_str = " ".join([f"{b:02X}" for b in data])
                    self.append_log(f"[RX] {hex_str}")
            except Exception:
                pass

    def append_log(self, text: str):
        """向监视窗口添加日志"""
        t = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{t}] {text}")
        self.log_text.moveCursor(self.log_text.textCursor().End)

    def clear_log(self):
        self.log_text.clear()

    def closeEvent(self, event):
        """窗口关闭时安全停转下位机并释放串口"""
        if self.serial_port and self.serial_port.is_open:
            self.send_stop_packet()
            self.serial_port.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = LoRaControllerWindow()
    window.show()
    sys.exit(app.exec_())
