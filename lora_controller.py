"""
水下机器人 - LoRa无线遥控上位机软件 (增强版)
Underwater Robot - LoRa Remote Controller (Enhanced Focus & Input)
--------------------------------------------------
改进特性:
1. 全局事件过滤器 (Global EventFilter): 彻底解决因焦点在输入框/滑块/按钮上导致的按键失灵问题
2. 支持键盘大小写及输入法文本双重匹配
3. 支持鼠标在虚拟按键上【按住启动/松开停止】
4. 串口未连接时友好提示，消除"按键无反馈"困惑
5. 实时按键高亮动效与水下状态监视
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
    QGroupBox, QGridLayout, QFrame, QMessageBox, QProgressBar
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
        self.setWindowTitle("水下机器人 LoRa 遥控上位机 v1.1")
        self.resize(900, 750)
        self.setMinimumSize(820, 680)

        # 核心通信与控制参数
        self.serial_port = None
        self.current_speed = 600      # 默认初始速度 600 (范围 100 ~ 1000)
        self.speed_step = 50          # 调速步进值
        self.active_keys = set()      # 当前正被按下的有效移动键
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

        sub_label = QLabel("💡 提示：使用键盘 WSAD 航行时，请将输入法切换为【英文】模式（按键盘 Shift 键切换）")
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
        self.combo_ports.setFocusPolicy(Qt.NoFocus)  # 防止抢占按键焦点
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

        # 3.1 左侧: 速度仪表与调速控制
        speed_group = QGroupBox("速度设定与动力监控")
        speed_layout = QVBoxLayout(speed_group)
        speed_layout.setSpacing(14)

        # 速度大字显示
        speed_info_layout = QHBoxLayout()
        self.lbl_speed_val = QLabel(f"{self.current_speed}")
        self.lbl_speed_val.setFont(QFont("Impact", 36))
        self.lbl_speed_val.setStyleSheet("color: #00E5FF;")

        self.lbl_speed_pct = QLabel(f"({self.current_speed // 10}%)")
        self.lbl_speed_pct.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        self.lbl_speed_pct.setStyleSheet("color: #80D8FF;")

        speed_info_layout.addWidget(QLabel("当前设定速度:"))
        speed_info_layout.addWidget(self.lbl_speed_val)
        speed_info_layout.addWidget(self.lbl_speed_pct)
        speed_info_layout.addStretch()
        speed_layout.addLayout(speed_info_layout)

        # 速度进度条
        self.speed_bar = QProgressBar()
        self.speed_bar.setRange(100, 1000)
        self.speed_bar.setValue(self.current_speed)
        self.speed_bar.setTextVisible(False)
        self.speed_bar.setFixedHeight(14)
        speed_layout.addWidget(self.speed_bar)

        # 速度滑块调节
        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(100, 1000)
        self.slider_speed.setSingleStep(50)
        self.slider_speed.setValue(self.current_speed)
        self.slider_speed.setFocusPolicy(Qt.NoFocus)  # 避免滑块劫持方向键
        self.slider_speed.valueChanged.connect(self.on_slider_speed_changed)
        speed_layout.addWidget(self.slider_speed)

        # 调速按钮与微调
        speed_btn_layout = QHBoxLayout()
        self.btn_dec_speed = QPushButton("➖ 减速 [Q / ↓]")
        self.btn_dec_speed.setFocusPolicy(Qt.NoFocus)
        self.btn_dec_speed.clicked.connect(self.decrease_speed)
        speed_btn_layout.addWidget(self.btn_dec_speed)

        self.btn_inc_speed = QPushButton("➕ 加速 [E / ↑]")
        self.btn_inc_speed.setFocusPolicy(Qt.NoFocus)
        self.btn_inc_speed.clicked.connect(self.increase_speed)
        speed_btn_layout.addWidget(self.btn_inc_speed)

        speed_layout.addLayout(speed_btn_layout)

        # 当前动作状态
        motion_layout = QHBoxLayout()
        motion_layout.addWidget(QLabel("航行状态:"))
        self.lbl_motion = QLabel("🛑 静止停机 (IDLE)")
        self.lbl_motion.setFont(QFont("Microsoft YaHei", 13, QFont.Bold))
        self.lbl_motion.setStyleSheet("color: #FFB300;")
        motion_layout.addWidget(self.lbl_motion)
        motion_layout.addStretch()
        speed_layout.addLayout(motion_layout)

        speed_layout.addStretch()
        middle_layout.addWidget(speed_group, stretch=5)

        # 3.2 右侧: 虚拟键盘与操作指南
        nav_group = QGroupBox("按键操控面板 (支持键盘按压与鼠标按住)")
        nav_layout = QVBoxLayout(nav_group)

        # 虚拟 WASD 面板 (既支持键盘按下发光，也支持鼠标按住操纵)
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
        tip_layout.addWidget(QLabel("📌 <b>核心操纵规则:</b>"))
        tip_layout.addWidget(QLabel("• <b>按住按键</b>：持续发送航行指令，水下推进器输出 PWM 运转"))
        tip_layout.addWidget(QLabel("• <b>松开按键</b>：立即发送 STOP 停机指令，推进器立刻停转"))
        tip_layout.addWidget(QLabel("• <b>[Q] / [E]</b>：实时降低 / 增加航行功率（步长 50）"))
        tip_layout.addWidget(QLabel("• <b>[空格]</b>：触发紧急制动"))
        nav_layout.addWidget(tip_frame)

        middle_layout.addWidget(nav_group, stretch=6)
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
        QPushButton {
            background-color: #1E293B;
            border: 1px solid #334155;
            border-radius: 6px;
            padding: 6px 14px;
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
        QProgressBar {
            background-color: #1E293B;
            border-radius: 7px;
        }
        QProgressBar::chunk {
            background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00B0FF, stop:1 #00E5FF);
            border-radius: 7px;
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
    # 全局事件过滤器 (彻底解决由于子控件焦点导致按键丢失的现象)
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

        # 调速键
        if key in (Qt.Key_Q, Qt.Key_Down, Qt.Key_Minus) or text == 'Q':
            self.decrease_speed()
            return True
        elif key in (Qt.Key_E, Qt.Key_Up, Qt.Key_Plus, Qt.Key_Equal) or text == 'E':
            self.increase_speed()
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
    # 串口连接与通信逻辑
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
            # 串口未连接时在日志区给予定期提示，避免用户误以为按键无响应
            now = time.time()
            if now - self.last_warn_time > 1.5:
                self.append_log(f"[模拟控制] {hex_str} (提示: 串口未连接, 请先点击'打开串口')")
                self.last_warn_time = now

    def handle_serial_error(self, err: Exception):
        """串口出现严重异常（如被抢占、拔出、拒绝访问）时的自动保护处理"""
        err_str = str(err)
        # 立即停止周期发送，避免疯狂刷屏
        self.send_timer.stop()
        self.recv_timer.stop()
        self.active_keys.clear()
        self.update_motion_label("STOP")

        if "拒绝访问" in err_str or "PermissionError" in err_str or "Access is denied" in err_str:
            diag_msg = "【串口被占用】Windows拒绝访问该COM口！通常是因为其他软件（如 XCOM、SSCOM、VOFA+ 等串口助手）正打开着该端口。请关闭其他串口软件后重试！"
        else:
            diag_msg = f"【串口通信中断】{err_str}"

        self.append_log(f"[错误] {diag_msg}")

        # 尝试关闭损坏的串口
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
        """发送停止包 (连续发送3次增强抗干扰丢包)"""
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
            except Exception as e:
                pass

    def append_log(self, text: str):
        """向监视窗口添加日志"""
        t = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{t}] {text}")
        self.log_text.moveCursor(self.log_text.textCursor().End)

    def clear_log(self):
        self.log_text.clear()

    # -------------------------------------------------------------
    # 速度控制与调节
    # -------------------------------------------------------------
    def set_speed(self, val: int):
        val = max(100, min(1000, val))
        self.current_speed = val
        self.lbl_speed_val.setText(str(val))
        self.lbl_speed_pct.setText(f"({val // 10}%)")
        self.speed_bar.setValue(val)
        self.slider_speed.blockSignals(True)
        self.slider_speed.setValue(val)
        self.slider_speed.blockSignals(False)

    def increase_speed(self):
        """加速"""
        self.set_speed(self.current_speed + self.speed_step)

    def decrease_speed(self):
        """减速"""
        self.set_speed(self.current_speed - self.speed_step)

    def on_slider_speed_changed(self, val):
        self.set_speed(val)

    # -------------------------------------------------------------
    # 航行状态计算与周期发送循环
    # -------------------------------------------------------------
    def update_motion_state(self):
        """根据当前 active_keys 集合计算当前航行动作"""
        if not self.active_keys:
            # 没有按键按下 -> 立即停止发送并向下位机发送 STOP
            if self.was_moving:
                self.send_stop_packet()
                self.was_moving = False
            self.send_timer.stop()
            self.current_motion = "STOP"
            self.update_motion_label("STOP")
        else:
            # 存在有效按键按下 -> 启动定时发送
            self.was_moving = True
            if 'W' in self.active_keys and 'S' not in self.active_keys:
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

            # 立即触发一次发送，并保持 50ms 周期定时发送
            self.on_send_timer_tick()
            if not self.send_timer.isActive():
                self.send_timer.start()

    def on_send_timer_tick(self):
        """按键处于按下状态期间，以 20Hz 频率持续向下位机发送控制帧"""
        if not self.active_keys or self.current_motion == "STOP":
            return

        spd = self.current_speed

        if self.current_motion == "FORWARD":
            self.send_packet(CMD_FORWARD, spd, spd)
        elif self.current_motion == "BACKWARD":
            self.send_packet(CMD_BACKWARD, spd, spd)
        elif self.current_motion == "TURN_LEFT":
            self.send_packet(CMD_TURN_LEFT, spd, spd)
        elif self.current_motion == "TURN_RIGHT":
            self.send_packet(CMD_TURN_RIGHT, spd, spd)
        elif self.current_motion == "LEFT_FWD":
            # 前进带左偏 (差速)
            self.send_packet(CMD_DIRECT_PWM, int(spd * 0.4), spd)
        elif self.current_motion == "RIGHT_FWD":
            # 前进带右偏 (差速)
            self.send_packet(CMD_DIRECT_PWM, spd, int(spd * 0.4))

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
