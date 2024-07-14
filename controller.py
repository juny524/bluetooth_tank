import bluetooth
import struct
import machine
import _thread
import time

button_right_front = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)
button_right_rear = machine.Pin(27, machine.Pin.IN, machine.Pin.PULL_UP)
button_left_front = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
button_left_rear = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)

queue = []
lock = _thread.allocate_lock()

def button_polling():
    global queue
    sendval = 0b0000
    if not button_right_front.value():
        sendval = sendval | 0b0001
    if not button_right_rear.value():
        sendval = sendval | 0b0010
    if not button_left_front.value():
        sendval = sendval | 0b0100
    if not button_left_rear.value():
        sendval = sendval | 0b1000
    if sendval != 0: # いずれかのボタンが押されているとき
        with lock:
            queue.append(sendval)
        print(f"Button pressed, added {sendval} to queue")

class BLEPeripheral:
    def __init__(self, name):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self.ble_irq)
        self.conn_handle = None
        self.name = name
        self._start_advertising()

    def _start_advertising(self):
        adv_data = bytearray(b'\x02\x01\x06\x0b\x09' + self.name.encode())
        self.ble.gap_advertise(100, adv_data)
        print("Advertising...")

    def ble_irq(self, event, data):
        if event == 1:  # _IRQ_CENTRAL_CONNECT
            self.conn_handle, _, _ = data
            print("Connected")
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            self.conn_handle = None
            self._start_advertising()
            print("Disconnected")

    def send(self, data):
        if self.conn_handle is not None:
            try:
                self.ble.gatts_notify(self.conn_handle, self.tx_handle, data)
                print(f"Sent: {data}")
            except Exception as e:
                print(f"Failed to send data: {e}")

    def start(self):
        UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        UART_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
        UART_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

        self.tx = (UART_TX_UUID, bluetooth.FLAG_NOTIFY,)
        self.rx = (UART_RX_UUID, bluetooth.FLAG_WRITE,)

        self.service = (UART_SERVICE_UUID, (self.tx, self.rx,))
        ((self.tx_handle, self.rx_handle),) = self.ble.gatts_register_services((self.service,))

        while True:
            button_polling()  # ボタンの状態をポーリング
            with lock:
                if queue:
                    item = queue.pop(0)
                    data = struct.pack('i', item)
                    self.send(data)
                    print(f"Received {item} from queue")
            time.sleep(0.2)

ble_peripheral = BLEPeripheral("Pico_Sender")
ble_peripheral.start()
