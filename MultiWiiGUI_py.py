import sys
import serial
from PyQt5.QtWidgets import QGridLayout, QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QSlider, QWidget, QComboBox, QSlider, QHBoxLayout, QTabWidget, QCheckBox
from PyQt5.QtCore import Qt, QTimer

# Todo: add new msp: pid description with bound and scale

output = None
reader = None
portname_file = "SerialPort.txt"  # Name of file for Autoconnect
GUI_BaudRate = 115200  # Default
SerialPort = None
g_serial = None

# ControlP5 and GUI elements would need to be replaced with a Python GUI library

# Constants
CHECKBOXITEMS = 0
PIDITEMS = 10
tab_height = 20  # Extra height needed for Tabs
Center_limits = [1200, 1800]  # Endpoints of ServoCenterSliders

# Window dimensions
windows_x, windows_y = 1000, 550 + tab_height
x_graph, y_graph = 10, 325 + tab_height
x_obj, y_obj = 520, 293 + tab_height
x_compass, y_compass = 920, 341 + tab_height
x_level_obj, y_level_obj = 920, 80 + tab_height
x_param, y_param = 120, 5 + tab_height
x_rc, y_rc = 690, 10 + tab_height
x_mot, y_mot = 690, 155 + tab_height
x_button, y_button = 845, 231 + tab_height
x_box, y_box = 415, 10 + tab_height
x_gps, y_gps = 853, 438 + tab_height
x_serv, y_serv = 350, 20 + tab_height

# Variables
byte_rc_rate = byte_rc_expo = byte_roll_pitch_rate = byte_yaw_rate = 0
byte_dyn_thr_pid = byte_throttle_expo = byte_throttle_mid = byte_select_setting = 0
cycle_time = i2c_error = 0
version = version_mismatch = horizon_instr_size = 0
gps_distance_to_home = gps_direction_to_home = 0
gps_num_sat = gps_fix = gps_update = gps_altitude = gps_speed = 0
gps_latitude = gps_longitude = 0
init_com = graph_on = p_meter_sum = int_power_trigger = byte_vbat = amperage = rssi = 0

multi_capability = 0  # Bitflags stating what capabilities are/are not present in the compiled code
byte_mp = [0] * 8  # Motor Pins. Varies by multiType and Arduino model (pro Mini, Mega, etc)
m_conf = [0] * 10  # Min/Maxthro etc
byte_p = [0] * PIDITEMS
byte_i = [0] * PIDITEMS
byte_d = [0] * PIDITEMS
activation = []
servo_mid = [0] * 8  # Plane, ppm/pwm conv, heli
servo_rate = [0] * 8
servo_direction = [0] * 8
servo_min = [0] * 8
servo_max = [0] * 8
wing_dir = [0] * 8  # Flying wing
wing_pos = [0] * 8
in_vals = [0] * 8

# Multicopter types
TRI, QUADP, QUADX, BI, GIMBAL = 1, 2, 3, 4, 5
Y6, HEX6, FLYING_WING, Y4, HEX6X = 6, 7, 8, 9, 10
OCTOX8, OCTOFLATX, OCTOFLATP, AIRPLANE = 11, 12, 13, 14
HELI_120_CCPM, HELI_90_DEG, VTAIL4, HEX6H = 15, 16, 17, 18
PPM_TO_SERVO, DUALCOPTER, SINGLECOPTER = 19, 20, 21

# Float variables
gx = gy = gz = ax = ay = az = magx = magy = magz = alt = head = angx = angy = 0.0
debug1 = debug2 = debug3 = debug4 = 0.0
angy_level_control = ang_calc = 0.0
p_version = 0.0

mot = [0.0] * 8
servo = [0.0] * 8
rc_chan = [0.0] * 16

# Constants for array indexing
ROLL, PITCH, YAW, ALT, VEL, LEVEL, MAG = 0, 1, 2, 3, 4, 5, 6

# Boolean flags
ax_graph = ay_graph = az_graph = gx_graph = gy_graph = gz_graph = alt_graph = head_graph = True
magx_graph = magy_graph = magz_graph = True
debug1_graph = debug2_graph = debug3_graph = debug4_graph = False
hide_draw = False
graphics_inited = False
gimbal_config = False
flapperons = False
flaps = False
init_servos = True

toggle_servo = toggle_write_servo = toggle_wing = toggle_write_wing = toggle_live = False
toggle_write_servo_live = toggle_write_wing_live = toggle_save_heli = toggle_wait_heli = False
toggle_gimbal = False
graph_enabled = False
mag_ = False
gimbal = True # false # if connect?
servo_stretch = False
cam_trigger = False
export_servo = False
toggle_trigger = False
servos_active = False

# RC channel constants
RC_THRO, RC_ROLL, RC_PITCH, RC_YAW = 3, 0, 1, 2
RC_AUX1, RC_AUX2, RC_AUX3, RC_AUX4 = 4, 5, 6, 7

# Colors (you'll need to adapt these for your Python GUI library)
yellow = (200, 200, 20)
green = (30, 120, 30)
red = (120, 30, 30)
blue = (50, 50, 100)
grey = (30, 30, 30)
black = (0, 0, 0)
orange = (200, 128, 0)

# Fonts (you'll need to adapt these for your Python GUI library)
font8 = font9 = font12 = font15 = None


class GimbalGraphics(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gimbal_config = False
        self.servo_stretch = False
        self.cam_trigger = False
        self.x_serv = 0
        self.y_serv = 0
        self.gimbal_sliders = []
        self.button_gimbal = None
        self.btn_trigger = None
        self.tab_widget = None

    def create_gimbal_graphics(self):
        if gimbal:  # if connect?
            self.gimbal_config = True
            s_min = 1020
            s_max = 2000
            if self.servo_stretch:
                s_min = 500
                s_max = 2500
            
            step = self.y_serv - 10

            layout = QVBoxLayout()

            # Tilt controls
            tilt_layout = QHBoxLayout()
            self.gimbal_sliders.append(self.create_slider("Tilt_Min", s_min, 1500, self.x_serv+10, step+80))
            self.gimbal_sliders.append(self.create_slider("Tilt_Max", 1500, s_max, self.x_serv+150, step+80))
            self.gimbal_sliders.append(self.create_slider("Channel", 1200, 1700, self.x_serv+100, step+60))
            self.gimbal_sliders.append(self.create_slider("Tilt_Prop", -125, 125, self.x_serv+100, step+100))
            for slider in self.gimbal_sliders[-4:]:
                tilt_layout.addWidget(slider)
            layout.addLayout(tilt_layout)

            step += 90

            # Roll controls
            roll_layout = QHBoxLayout()
            self.gimbal_sliders.append(self.create_slider("Roll_Min", s_min, 1500, self.x_serv+10, step+80))
            self.gimbal_sliders.append(self.create_slider("Roll_Max", 1500, s_max, self.x_serv+150, step+80))
            self.gimbal_sliders.append(self.create_slider("Channel", 1200, 1700, self.x_serv+100, step+60))
            self.gimbal_sliders.append(self.create_slider("Roll_Prop", -125, 125, self.x_serv+100, step+100))
            for slider in self.gimbal_sliders[-4:]:
                roll_layout.addWidget(slider)
            layout.addLayout(roll_layout)

            # Trigger controls
            trigger_layout = QHBoxLayout()
            self.gimbal_sliders.append(self.create_slider("Trig_LO", 500, 2000, self.x_serv+10, step+80))
            self.gimbal_sliders.append(self.create_slider("Trig_HI", 1000, s_max, self.x_serv+150, step+80))
            self.gimbal_sliders.append(self.create_slider("Trigger", 0, 30000, self.x_serv+100, step+60))
            trig_rev_slider = self.create_slider("Trig_Rev", 0, 1, self.x_serv+100, step+100)
            trig_rev_slider.setTickPosition(QSlider.TicksBelow)
            trig_rev_slider.setTickInterval(1)
            self.gimbal_sliders.append(trig_rev_slider)
            for slider in self.gimbal_sliders[-4:]:
                trigger_layout.addWidget(slider)
            layout.addLayout(trigger_layout)

            self.button_gimbal = QPushButton("Gimbal")
            self.button_gimbal.show()
            layout.addWidget(self.button_gimbal)

            if self.cam_trigger:
                self.btn_trigger = QPushButton("Trigger")
                self.btn_trigger.show()
                layout.addWidget(self.btn_trigger)

            self.setLayout(layout)

            # Assuming you have a QTabWidget as the main container
            if self.tab_widget:
                self.tab_widget.addTab(self, "ServoSettings")
                self.tab_widget.setCurrentWidget(self)

    def create_slider(self, name, min_val, max_val, x, y):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setGeometry(x, y, 60, 10)
        slider.setObjectName(name)
        return slider

    def set_tab_widget(self, tab_widget):
        self.tab_widget = tab_widget

        
class ServoGraphics(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.multi_type = None  # Set this based on your application logic
        self.servo_stretch = False
        self.x_serv = 0
        self.y_serv = 0
        self.create_servo_graphics()

    def create_servo_graphics(self):
        main_layout = QVBoxLayout()

        # Create Bbox
        bbox_layout = QGridLayout()
        self.bbox = [QCheckBox(str(i)) for i in range(8)]
        for i, cb in enumerate(self.bbox):
            cb.setStyleSheet("QCheckBox { color: white; }")
            bbox_layout.addWidget(cb, i, 0)
        main_layout.addLayout(bbox_layout)

        # Create Common ServoSliders
        slider_layout = QGridLayout()
        s_min = 500 if self.servo_stretch else 1020
        s_max = 2500 if self.servo_stretch else 2000
        center_limits = [1200, 1800]  # Assuming these values, adjust as needed

        self.servo_slider_c = []
        self.servo_slider_min = []
        self.servo_slider_max = []

        for i in range(8):
            slider_layout.addWidget(QLabel(f"Servo {i}"), i, 0)
            
            self.servo_slider_c.append(QSlider(Qt.Horizontal))
            self.servo_slider_c[i].setRange(center_limits[0], center_limits[1])
            slider_layout.addWidget(self.servo_slider_c[i], i, 1)

            self.servo_slider_min.append(QSlider(Qt.Horizontal))
            self.servo_slider_min[i].setRange(s_min, 1500)
            slider_layout.addWidget(self.servo_slider_min[i], i, 2)

            self.servo_slider_max.append(QSlider(Qt.Horizontal))
            self.servo_slider_max[i].setRange(1500, s_max)
            slider_layout.addWidget(self.servo_slider_max[i], i, 3)

        main_layout.addLayout(slider_layout)

        # ServoGraphics For AirPlane SC & DC
        if self.multi_type in [19, 14]:  # Assuming PPM_TO_SERVO = 19, AIRPLANE = 14
            main_layout.addWidget(QLabel("Servo Rates in %"))
            main_layout.addWidget(QLabel("Offset for servos"))
            main_layout.addWidget(QLabel("Channel for Flaps"))
            main_layout.addWidget(QLabel("Norm/Rev"))

        # ServoGraphics For FlyingWng & TRI & BI || multiType == DUALCOPTER
        if self.multi_type in [8, 1, 4, 20, 21]:  # FLYING_WING, TRI, BI, DUALCOPTER, SINGLECOPTER
            self.create_specific_layout(main_layout)

        # ServoGraphics For Heli 120 && 90
        if self.multi_type in [15, 16]:  # HELI_120_CCPM, HELI_90_DEG
            self.create_heli_layout(main_layout)

        # Common Graphics for servos
        self.create_common_servo_graphics(main_layout)

        self.setLayout(main_layout)

    def create_specific_layout(self, main_layout):
        wbox_layout = QGridLayout()
        label_layout = QVBoxLayout()

        if self.multi_type == 8:  # FLYING_WING
            self.create_wing_checkboxes(wbox_layout, ["L Roll", "R Roll", "L NICK", "R NICK"], 2)
            label_layout.addWidget(QLabel("Left Wing"))
            label_layout.addWidget(QLabel("Right Wing"))
        elif self.multi_type == 1:  # TRI
            self.create_wing_checkboxes(wbox_layout, ["YAW"], 1)
            label_layout.addWidget(QLabel("Yaw Servo"))
        elif self.multi_type == 4:  # BI
            self.create_wing_checkboxes(wbox_layout, ["L Yaw", "R Yaw", "L NICK", "R NICK"], 2)
            label_layout.addWidget(QLabel("Left Servo"))
            label_layout.addWidget(QLabel("Right Servo"))
        elif self.multi_type == 20:  # DUALCOPTER
            self.create_wing_checkboxes(wbox_layout, ["Pitch", "Roll"], 2)
            label_layout.addWidget(QLabel("Roll"))
            label_layout.addWidget(QLabel("Nick"))
        elif self.multi_type == 21:  # SINGLECOPTER
            self.create_wing_checkboxes(wbox_layout, ["Right", "R yaw", "Left", "L yaw", "Front", "F yaw", "Rear", "yaw"], 2)
            label_layout.addWidget(QLabel("MIN"))
            label_layout.addWidget(QLabel("MAX"))
            label_layout.addWidget(QLabel("Offset servos"))

        main_layout.addLayout(wbox_layout)
        main_layout.addLayout(label_layout)
        main_layout.addWidget(QLabel("Change Gyro/Acc Direction"))
        main_layout.addWidget(QLabel("Change Dir in TX To Match"))

    def create_heli_layout(self, main_layout):
        main_layout.addWidget(QLabel("MIN"))
        main_layout.addWidget(QLabel("MAX"))
        main_layout.addWidget(QLabel("Offset servos"))
        main_layout.addWidget(QLabel("Not Used"))
        main_layout.addWidget(QLabel("Servos"))

        mix_layout = QGridLayout()
        mix_labels = ["NICK", "LEFT", "RIGHT", "COLL", "NICK", "ROLL"]
        for i, label in enumerate(mix_labels):
            mix_layout.addWidget(QLabel(label), i % 3, i // 3)

        main_layout.addLayout(mix_layout)

    def create_wing_checkboxes(self, layout, items, items_per_row):
        for i, item in enumerate(items):
            checkbox = QCheckBox(item)
            checkbox.setStyleSheet("QCheckBox { color: white; }")
            layout.addWidget(checkbox, i // items_per_row, i % items_per_row)

    def create_common_servo_graphics(self, main_layout):
        common_layout = QGridLayout()
        self.bt_servo = []
        self.checkbox_rev = []
        self.rate_slider = []

        for i in range(8):
            self.bt_servo.append(QPushButton(f"Servo {i}"))
            common_layout.addWidget(self.bt_servo[i], i, 0)

            self.checkbox_rev.append(QCheckBox())
            common_layout.addWidget(self.checkbox_rev[i], i, 1)

            self.rate_slider.append(QSlider(Qt.Horizontal))
            self.rate_slider[i].setRange(0, 125)
            common_layout.addWidget(self.rate_slider[i], i, 2)

        self.bt_aux = []
        for i in range(5):
            self.bt_aux.append(QPushButton(f"AUX {i+1}" if i < 4 else "Disable"))
            common_layout.addWidget(self.bt_aux[i], i, 3)

        main_layout.addLayout(common_layout)

def create_checkboxes(self, names):
        # Destroy old buttons
        for i in range(self.CHECKBOXITEMS):
            self.buttons[i].setParent(None)
            self.checkboxes[i].setParent(None)

        # Create new list entries and buttons
        self.checkboxes = []
        self.buttons = []
        self.activation = [0] * len(names)

        for i, name in enumerate(names):
            if name == "CAMTRIG":
                self.camTrigger = True
            if name in ["CAMSTAB", "CAMTRIG"]:
                self.gimbal = True

            button = QPushButton(name)
            button.setStyleSheet("background-color: red;")
            button.setGeometry(self.xBox - 30, self.yBox + 20 + 13 * i, 68, 12)

            checkbox_layout = QHBoxLayout()
            for j in range(1, 13):
                cb = QCheckBox()
                cb.setStyleSheet("QCheckBox::indicator:checked { background-color: white; }")
                cb.setStyleSheet("QCheckBox::indicator:unchecked { background-color: gray; }")
                checkbox_layout.addWidget(cb)

            checkbox_widget = QWidget()
            checkbox_widget.setLayout(checkbox_layout)
            checkbox_widget.setGeometry(self.xBox + 40, self.yBox + 20 + 13 * i, 120, 12)

            self.buttons.append(button)
            self.checkboxes.append(checkbox_widget)

            row_layout = QHBoxLayout()
            row_layout.addWidget(button)
            row_layout.addWidget(checkbox_widget)
            self.layout.addLayout(row_layout)

        self.CHECKBOXITEMS = len(names)

def shortify_port_name(port_name, maxlen):
    short_name = port_name
    if short_name.startswith("/dev/cu."):
        short_name = ""  # only collect the corresponding tty. devices
    return short_name

def hide_label(widget):
    widget.setText("")
    return widget

# MSP Header
MSP_HEADER = "$M<"

class MultiWiiConfGUI(QMainWindow):
    
    def __init__(self):
        super().__init__()

        # Initialize serial connection (adjust COM port and baud rate as necessary)
        self.serial_port = None
        self.baud_rate = 115200

        # Set up the main window
        self.setWindowTitle("MultiWiiConf - PyQt5")
        self.setGeometry(100, 100, 600, 400)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.main_layout = QVBoxLayout(self.central_widget)

        self.create_main_controls()
        self.create_tab_widget()

    def create_main_controls(self):
        self.telemetry_label = QLabel("Telemetry Data: ", self)
        self.main_layout.addWidget(self.telemetry_label)

        self.port_combo = QComboBox(self)
        self.port_combo.addItems(self.get_serial_ports())
        self.main_layout.addWidget(self.port_combo)

        self.connect_button = QPushButton("Connect", self)
        self.connect_button.clicked.connect(self.connect_to_serial)
        self.main_layout.addWidget(self.connect_button)

        self.button_send_msp_status = QPushButton("Send MSP Status", self)
        self.button_send_msp_status.clicked.connect(self.send_msp_status)
        self.main_layout.addWidget(self.button_send_msp_status)

        self.button_send_pid_values = QPushButton("Send PID Values", self)
        self.button_send_pid_values.clicked.connect(self.send_pid_values)
        self.main_layout.addWidget(self.button_send_pid_values)

    def create_tab_widget(self):
        self.main_window = QWidget()
        self.tab_widget = QTabWidget(self.main_window)
        self.main_layout.addWidget(self.tab_widget)

        # Add GimbalGraphics as a tab
        self.gimbal_graphics = GimbalGraphics()
        self.gimbal_graphics.set_tab_widget(self.tab_widget)
        self.gimbal_graphics.create_gimbal_graphics()
        self.tab_widget.addTab(self.gimbal_graphics, "Gimbal Settings")

        # Add ServoGraphics as a tab
        self.servo_graphics = ServoGraphics()
        self.tab_widget.addTab(self.servo_graphics, "Servo Settings")
        
        

    def get_serial_ports(self):
        """List available serial ports."""
        # ports = [f"/dev/ttys00{i}" for i in range(0,10)]   # Adjust this based on your OS and available ports.
        ports = [f"COM{i}" for i in range(0,10)] # adjust according OS
        return ports

    def connect_to_serial(self):
        """Connect to the selected serial port."""
        port_name = self.port_combo.currentText()
        
        try:
            if not port_name:
                raise ValueError("No COM port selected")
            
            if not self.serial_port or not self.serial_port.is_open:
                fixed_port_name = "/dev/ttys047" # fixed
                self.serial_port = serial.Serial(fixed_port_name, baudrate=self.baud_rate, timeout=1)
                print(f"Connected to {fixed_port_name}")
                self.telemetry_label.setText(f"Connected to {fixed_port_name}")
            else:
                print(f"Already connected to {port_name}")
        
        except Exception as e:
            print(f"Error connecting to {port_name}: {e}")
            self.telemetry_label.setText(f"Error: {e}")

    def send_request_msp(self, msp_data):
        """Send prepared MSP data over the serial connection."""
        """Send the prepared MSP data over the serial connection."""
        try:
            self.serial_port.write(bytearray(msp_data))  # Send data over serial port
            self.serial_port.flush()  # Flush the output buffer to ensure all data is sent
        except Exception as e:
            print(f"Error sending to receiver")
            self.telemetry_label.setText(f"Error: {e}")
    
    def request_msp(self, msp):
        """Prepare an MSP message without a payload."""
        return request_msp_with_payload(msp, None)

    def send_msp_status(self):
        """Send an example MSP status command."""
        msp_status = 101   # Example: MSP_STATUS command ID is typically around this number.
        msp_data = self.request_msp(msp_status)
        self.send_request_msp(msp_data)

    def send_pid_values(self):
        """Send an example PID values command with a payload."""
        msp_set_pid = 202   # Example: Setting PID values.
        
        pid_payload = [10, 20, 30]   # Example PID values for P/I/D.
        
        msp_pid_data = request_msp_with_payload(msp_set_pid, pid_payload)
        
        self.send_request_msp(msp_pid_data)


# Helper functions for preparing MSP messages

def request_msp_with_payload(msp, payload):
    """Prepare an MSP message with an optional payload."""
    
    if msp < 0:
      return None
    
    bf = bytearray()
    
    # Add header "$M<"
    bf.extend(MSP_HEADER.encode('utf-8'))
    
    pl_size = len(payload) if payload else 0
    
    bf.append(pl_size)   # Payload size
    
    bf.append(msp & 0xFF)   # Command identifier
    
    checksum = pl_size ^ (msp & 0xFF)   # Calculate checksum
    
    if payload:
      for p in payload:
          bf.append(p & 0xFF)
          checksum ^= p & 0xFF
    
    bf.append(checksum)   # Add final checksum
    
    return bf # + b'\n' # Do the real controller need a endline? # add a end line here bc the reciever uses readline!


if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    
    window = MultiWiiConfGUI()
    
    window.show()
    
    sys.exit(app.exec_())