from car_setting.car_setting import TTY_RFID
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
import time
class RfidPublisher(Node):

    def __init__(self,name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(String, 'rfid_data', 1)
        self.rfidr_ser = serial.Serial(port=TTY_RFID, baudrate=9600, timeout=0.5)
        if self.rfidr_ser.isOpen():  # 判断端口是否被打开+
            self.get_logger().info('rfid is connect')
            self.send_auto_read_card()
            self.find_rfid_from_ser()
        else:
            self.get_logger().info('rfid is  not connect')

    def CheckSum(self, b, cnt):
        crc = 0
        for i in range(cnt):
            crc ^= b[i]
        return crc

    # 发送自动读块指令
    def send_auto_read_card(self):
        # 指令
        msg = f"7F0D002E020c010000000123125448"
        # 转换为b''
        data = bytes.fromhex(msg)
        # 发送数据
        self.rfidr_ser.write(data)

    def find_rfid_from_ser(self):
        while True:
            try:
                data = self.rfidr_ser.read(2)
                if data != b'':
                    # 收到数据16进制打印
                    if data[0] == 127:
                        data_len = data[1]
                        msg = self.rfidr_ser.read(data_len)
                        self.handle_data(msg)
                time.sleep(1/20)
            except Exception as e:
                self.get_logger().info("except err {}".format(e))

    # 处理收到的数据
    def handle_data(self, data):
        msg = String()
        command = data[1]
        if command == 0x91:
            rfid_type = data[2] + data[3]
            # 卡 if 4位
            rfid_id = data[4:9]
            # 数据 16位
            rfid_num = data[9:25]
            # 校验位 1 位
            check = data[25]
            num = rfid_num[0:1]
            #self.get_logger().info(num)
            #print(type(num[0]),num[0])
            msg.data = str(num[0])
            self.get_logger().info(str(msg.data))
            self.publisher_.publish(msg)
            #print(f"command:{command} rfid_type:{rfid_type} rfid_id:{rfid_id.hex()} rfid_num:{rfid_num.hex()} check:{check}")
        elif command == 0x92:
            #状态位 1位
            state = data[2]
            #卡类型
            rfid_type = data[3]+data[4]
            #卡 if 4位
            rfid_id = data[5:10]
            #校验位 1位
            check = data[9]
            if state == 0:
                self.rfid_id_edit.setText(rfid_id.hex())
                kuai_data = self.num
                # 只要16进制后两位
                kuai_data_hex = hex(kuai_data)[2:].upper()





def main(args=None):
    rclpy.init(args=args)
    rfid_publisher = RfidPublisher("rfid_publisher")
    rclpy.spin(rfid_publisher)
    rfid_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
