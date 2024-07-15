import bluetooth
import struct
from machine import Pin
import machine
import time

led = Pin(15, Pin.OUT)
# L298N Pins
IN1 = machine.Pin(2, machine.Pin.OUT)
IN2 = machine.Pin(3, machine.Pin.OUT)
ENA = machine.Pin(4, machine.Pin.OUT)

# モーター2のピン定義
IN3 = machine.Pin(5, machine.Pin.OUT)
IN4 = machine.Pin(6, machine.Pin.OUT)
ENA2 = machine.Pin(7, machine.Pin.OUT)

MOTOR_SPEED = 52428 # マックスの80%くらいの速度、これに固定

# PWM設定
ENA_PWM = machine.PWM(ENA)
ENA_PWM.freq(1000)  # 周波数1kHz

ENA2_PWM = machine.PWM(ENA2)
ENA2_PWM.freq(1000)  # 周波数1kHz

def set_motor_speed_right(speed):
    if 0 <= speed <= 1023:
        ENA_PWM.duty_u16(speed * 64)  # 0-1023 を 0-65535 にスケールアップ
    else:
        print("Speed must be between 0 and 1023")

def set_motor_speed_left(speed):
    if 0 <= speed <= 1023:
        ENA2_PWM.duty_u16(speed * 64)  # 0-1023 を 0-65535 にスケールアップ
    else:
        print("Speed must be between 0 and 1023")

def move_motor(direction, speed=818):
    if direction == 'forward':
        IN1.high()
        IN2.low()
        set_motor_speed_right(speed)
        print("forward")
    elif direction == 'backward':
        IN1.low()
        IN2.high()
        set_motor_speed_right(speed)
        print("backward")
    elif direction == 'forward2':
        IN3.high()
        IN4.low()
        set_motor_speed_left(speed)
        print("backward")
    elif direction == 'backward2':
        IN3.low()
        IN4.high()
        set_motor_speed_left(speed)
        print("backward")
    else:
        print("Invalid direction. Use 'forward' or 'backward'.")
        return
    
    
    


def ledtest(received_value):
#    lighting = received_value % 2
#    led.value(lighting)

    if received_value & 0b0001:
        move_motor('forward')
    elif received_value & 0b0010:
        move_motor('backward')
    
    if received_value & 0b0100:
        move_motor('forward2')
    elif received_value & 0b1000:
        move_motor('backward2')

    time.sleep(0.2)
    set_motor_speed_right(0)
    set_motor_speed_left(0)
    #set_motor_speed(32768)  # Set speed to 50%
    #set_motor_speed(MOTOR_SPEED)  # Set speed to 50%
    # Run motor for 5 seconds
    #time.sleep(0.2)
    # Stop motor
    #set_motor_speed(0)

class BLECentral:
    def __init__(self, name):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self.ble_irq)
        self.conn_handle = None
        self.name = name
        self._start_scanning()

    def _start_scanning(self):
        self.ble.gap_scan(2000, 30000, 30000)
        print("Scanning...")

    def ble_irq(self, event, data):
        if event == 5:  # Scan result
            addr_type, addr, adv_type, rssi, adv_data = data
            if self.name.encode() in bytes(adv_data):
                print("Found device, connecting...")
                self.ble.gap_connect(addr_type, addr)
        elif event == 1:  # Central connected
            self.conn_handle, _, _ = data
            print("Connected")
            self.ble.gattc_discover_services(self.conn_handle)
        elif event == 2:  # Central disconnected
            self.conn_handle = None
            self._start_scanning()
            print("Disconnected")
        elif event == 9:  # Service discovered
            conn_handle, start_handle, end_handle, uuid = data
            if uuid == bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"):
                self.ble.gattc_discover_characteristics(self.conn_handle, start_handle, end_handle)
        elif event == 11:  # Characteristic discovered
            conn_handle, def_handle, value_handle, properties, uuid = data
            if uuid == bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"):
                self.rx_handle = value_handle
        elif event == 18:  # Notification
            conn_handle, value_handle, notify_data = data
            received_value = struct.unpack('i', notify_data)[0]  # バイトデータを整数に変換
            print(f"Received: {received_value}")
            ledtest(received_value)

    def start(self):
        while True:
            pass

ble_central = BLECentral("Pico_Sender")
ble_central.start()





