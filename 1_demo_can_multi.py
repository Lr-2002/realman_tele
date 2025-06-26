import serial
import time
import threading

# 寄存器字典
regdict = {
    'ID': 1000,
    'baudrate': 1001,
    'clearErr': 1004,
    'forceClb': 1009,
    'angleSet': 1486,
    'forceSet': 1498,
    'speedSet': 1522,
    'angleAct': 1546,
    'forceAct': 1582,
    'errCode': 1606,
    'statusCode': 1612,
    'temp': 1618,
    'actionSeq': 2320,
    'actionRun': 2322
}

class Device:
    def __init__(self, port, id_value):
        self.port = port
        self.id_value = id_value
        self.ser = self.init_serial()

    def init_serial(self):
        """初始化串口连接"""
        try:
            ser = serial.Serial(self.port, 115200, timeout=1)
            print(f"串口 {self.port} 已打开")
            return ser
        except Exception as e:
            print(f"打开串口失败: {e}")
            return None

    def convert_address_to_binary_write(self, address_dec):
        """将十进制地址转换为二进制字符串，用于写操作，前缀第六位为1。"""
        address_bin = bin(address_dec)[2:]
        id_bin = bin(int(self.id_value, 2))[2:].zfill(14)
        formatted_output = f"0000010{address_bin}{id_bin}"  # 第六位为1
        return formatted_output

    def convert_address_to_binary_read(self, address_dec):
        """将十进制地址转换为二进制字符串，用于读操作，前缀第六位为0。"""
        address_bin = bin(address_dec)[2:]
        id_bin = bin(int(self.id_value, 2))[2:].zfill(14)
        formatted_output = f"0000000{address_bin}{id_bin}"  # 第六位为0
        return formatted_output

    def binary_to_hex(self, binary_string):
        """将二进制字符串转换为十六进制"""
        decimal_value = int(binary_string, 2)
        return hex(decimal_value)[2:].upper()

    def convert_number_to_bytes(self, number):
        """将一个整数转换为字节数组"""
        hex_string = hex(number)[2:].upper()
        if len(hex_string) % 2 != 0:
            hex_string = '0' + hex_string
        byte_array = [int(hex_string[i:i+2], 16) for i in range(0, len(hex_string), 2)][::-1]
        return byte_array

    def send_command(self, reg_name, values):
        """发送控制命令"""
        if reg_name in ['angleSet', 'forceSet', 'speedSet']:
            val_reg = [val & 0xFFFF for val in values]
            self.write_register(regdict[reg_name], val_reg)
        else:
            print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'，val为长度为6的list，值为0~1000，允许使用-1作为占位符')

    def write_register(self, address, values):
        """写入寄存器并发送数据"""
        while True:
            chunk = values[:4]  # 取前4个数据
            values = values[4:]

            # 使用写操作的地址转换函数
            ext_id_bin = self.convert_address_to_binary_write(address)
            ext_id_hex = self.binary_to_hex(ext_id_bin)
            print(f"计算的扩展标识符: {ext_id_hex}")

            ext_id_number = int(ext_id_hex, 16)
            ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

            print("提取的扩展标识符字节:")
            for byte in ext_id_bytes:
                print(f"字节: {byte:02X}")

            send_buffer = bytearray()
            send_buffer += bytes([0xAA, 0xAA])
            send_buffer.extend(ext_id_bytes)

            # 添加数据
            for value in chunk:
                send_buffer.append(value & 0xFF)
                send_buffer.append(value >> 8)

            # 计算当前数据长度并添加额外的填充
            data_length = len(chunk) * 2
            if data_length < 8:
                padding_length = 8 - data_length
                send_buffer.extend([0xFF] * padding_length)

            # 添加数据长度和额外的固定字节
            send_buffer.append(len(chunk) * 2 + (padding_length if data_length < 8 else 0))  # 数据长度
            send_buffer.append(0x00)
            send_buffer.append(0x01)
            send_buffer.append(0x00)

            # 计算校验和
            check_sum = sum(send_buffer[2:]) & 0xFF
            send_buffer.append(check_sum)
            send_buffer += bytes([0x55, 0x55])

            self.ser.write(send_buffer)
            print(f"设备 {self.id_value} 发送指令:", send_buffer.hex())
            self.ser.reset_input_buffer()  # 清除输入缓冲区
            break  # 仅执行一次

        # 发送后半段
        new_address = address + 8

        ext_id_bin = self.convert_address_to_binary_write(new_address)
        ext_id_hex = self.binary_to_hex(ext_id_bin)

        ext_id_number = int(ext_id_hex, 16)
        ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

        for byte in ext_id_bytes:
            print(f"字节: {byte:02X}")

        # 继续构建发送缓冲区，使用后半段的扩展标识符字节
        send_buffer = bytearray()
        send_buffer += bytes([0xAA, 0xAA])
        send_buffer.extend(ext_id_bytes)

        # 发送剩余数据
        for value in values:
            send_buffer.append(value & 0xFF)
            send_buffer.append(value >> 8)

        # 计算当前数据长度并添加额外的填充
        data_length = len(values) * 2
        if data_length < 8:
            padding_length = 8 - data_length
            send_buffer.extend([0xFF] * padding_length)

        # 添加数据长度和额外的固定字节
        send_buffer.append(0x04)  # 数据长度
        send_buffer.append(0x00)
        send_buffer.append(0x01)
        send_buffer.append(0x00)

        # 计算校验和
        check_sum = sum(send_buffer[2:]) & 0xFF
        send_buffer.append(check_sum)
        send_buffer += bytes([0x55, 0x55])

        self.ser.write(send_buffer)
        print(f"设备 {self.id_value} 发送指令 (后半段):", send_buffer.hex())

    def read_register(self, address):
        """读取寄存器并发送数据"""
        ext_id_bin = self.convert_address_to_binary_read(address)
        ext_id_hex = self.binary_to_hex(ext_id_bin)
        ext_id_number = int(ext_id_hex, 16)
        ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

        send_buffer = bytearray()
        send_buffer += bytes([0xAA, 0xAA])
        send_buffer.extend(ext_id_bytes)

        # 读取命令
        send_buffer.append(0x08)  # 固定字节
        send_buffer.extend([0x00] * 7)  # 填充固定字节
        send_buffer.append(0x01)  # 固定字节
        send_buffer.append(0x00)  # 固定字节
        send_buffer.append(0x01)  # 固定字节    
        send_buffer.append(0x00)  # 固定字节

        # 计算校验和
        check_sum = sum(send_buffer[2:]) & 0xFF
        send_buffer.append(check_sum)
        send_buffer += bytes([0x55, 0x55])

        self.ser.write(send_buffer)
        print(f"设备 {self.id_value} 发送读取指令:", send_buffer.hex())
        self.ser.reset_input_buffer()  # 清除输入缓冲区
        
        # 等待设备响应
        time.sleep(0.1)  # 等待设备响应
        response1 = self.ser.read(23)  # 根据协议读取响应字节数

        # 根据地址处理响应数据
        if address == regdict['temp']:
            # 处理温度数据
            if len(response1) >= 1:
                frame1_data = response1[6:12]  # 从第 7 位到第 12 位
                values1 = [val for val in frame1_data if val <= 60000]
                print(f"设备 {self.id_value} 读取数据（温度）:", tuple(values1))
        else:
            if len(response1) >= 1:
                frame1_data = response1[6:14]  # 从第 7 位到第 14 位
                values1 = []
                for i in range(0, len(frame1_data), 2):
                    low_byte = frame1_data[i]
                    high_byte = frame1_data[i + 1]
                    value = (high_byte << 8) | low_byte
                    if value > 60000:
                        value = 0
                    values1.append(value)

                # 发送后半段
                new_address = address + 8
                ext_id_bin = self.convert_address_to_binary_read(new_address)
                ext_id_hex = self.binary_to_hex(ext_id_bin)
                ext_id_number = int(ext_id_hex, 16)
                ext_id_bytes = self.convert_number_to_bytes(ext_id_number)

                send_buffer = bytearray()
                send_buffer += bytes([0xAA, 0xAA])
                send_buffer.extend(ext_id_bytes)

                send_buffer.append(0x04)  # 固定字节
                send_buffer.extend([0x00] * 7)  # 填充固定字节
                send_buffer.append(0x01)  # 固定字节
                send_buffer.append(0x00)  # 固定字节
                send_buffer.append(0x01)  # 固定字节    
                send_buffer.append(0x00)  # 固定字节

                # 计算校验和
                check_sum = sum(send_buffer[2:]) & 0xFF
                send_buffer.append(check_sum)
                send_buffer += bytes([0x55, 0x55])

                self.ser.write(send_buffer)
                self.ser.reset_input_buffer()

                # 等待设备响应
                time.sleep(0.1)
                response2 = self.ser.read(23)
                print(f"设备 {self.id_value} 读取的原始内容 (后半段):", response2.hex())

                if len(response2) >= 1:
                    frame2_data = response2[6:10]  # 从第 7 位到第 10 位
                    values2 = []
                    for i in range(0, len(frame2_data), 2):
                        low_byte = frame2_data[i]
                        high_byte = frame2_data[i + 1]
                        value = (high_byte << 8) | low_byte
                        if value > 60000:
                            value = 0
                        values2.append(value)

                combined_values = values1[:4] + values2[:2]  # 前四个第一帧的值 + 后两个第二帧的值
                print(f"设备 {self.id_value} 读取数据:", tuple(combined_values))


def write6(devices, reg_name, val, id_value):
    """写入6个参数的函数"""
    device = devices[id_value]
    device.send_command(reg_name, val)

def read_register(devices, address, id_value):
    """读取寄存器的函数"""
    device = devices[id_value]
    device.read_register(address)

if __name__ == "__main__":
    device_ports = ['/dev/ttyUSB5']  # 设备串口
    device_ids = ['01']  # 二进制设备 ID

    # 创建设备字典
    devices = {}
    for port, id_value in zip(device_ports, device_ids):
        devices[id_value] = Device(port, id_value)
    print(devices)
    print('设置灵巧手运动角度参数0，-1为不设置该运动角度！')  
    # 写入
    write6(devices, 'speedSet', [1000, 1000, 1000, 1000, 1000, 1000], '01')
    time.sleep(1)
    write6(devices, 'angleSet', [400, 400, 400, 400, 400, 0], '01')  # ID 设置为 '01'
    time.sleep(3)
    write6(devices, 'speedSet', [200, 200, 200, 200, 200, 200], '01')
    time.sleep(1)
    # 读取
    read_register(devices, regdict['angleSet'], '01')
    time.sleep(1)
    #read_register(devices, regdict['angleSet'], '10')

    # 关闭所有设备
    for device in devices.values():
        device.ser.close()

