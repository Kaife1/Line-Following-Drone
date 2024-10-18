THRESHOLD = (0, 88, -128, -15, -128, 57) # Grayscale threshold for dark things
import sensor, image, time
from pyb import UART,LED
from machine import I2C
import sensor, image, time
from machine import UART

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(False)
sensor.set_hmirror(False)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # 160x120
sensor.skip_frames(time = 2000)
clock = time.clock()

# UART initialization
uart1 = UART(1, 9600, timeout_char=200)
uart3 = UART(3, 115200)
i2c = I2C(4, freq=100000)  # 设置I2C为100kHz

# Command definitions
Takeoff = 0x33          # b'\xfe'  11
Landing = 0xCC          # b'\x80'  22
Hovering = 0x77         # b'\x98'  12
LineTracking = 0x88     # b'\xe0'  13
EmerStop = 0x55         # b'\x9e'  14
CommCheck = b'\x00'


TARGET_TIME = 50

is_flying = False
comm_test = True  # Default value of comm_test
last_comm_time = time.ticks_ms()  # To track the time of the last communication
failure_print_counter = 0  # Counter for failed communication messages
failure_print_limit = 3  # Limit for how many times the message should print
is_tracking = False
too_close = False
commd = None
no_valid = None
takeoff_command_sent = False
command_timeout = 30  # 30 seconds for no valid command

def get_current_time():
    t = time.localtime()
    return "{:02}:{:02}:{:02}".format(t[3], t[4], t[5])  # HH:MM:SS format

# Function to send UART responses
def send_uart1_response(message):
    uart1.write(message + "\n")


# Function to handle UART commands
def Remote_Control():
    global is_flying, is_tracking, commd,comm_test, last_comm_time, failure_print_counter # To modify the global state
    command_timer = 0  # Initialize the timer for no valid command

    if uart1.any():
        a = uart1.read(2)  # Read exactly 2 bytes

        # Clear remaining data from the buffer
        while uart1.any():
            uart1.read()  # Discard any remaining data in the buffer
        last_comm_time = time.ticks_ms()


        # Process the command
        if a == b'\xfe' and not is_flying:  # Takeoff command
            commd = Takeoff
            is_flying = True
            is_tracking = False
            send_uart1_response("Takeoff")

            print("Takeoff\n")

        elif a == b'\x80':  # Landing command
            commd = Landing
            is_flying = False
            is_tracking = False
            send_uart1_response("Landing")
            print('Landing')

        elif a == b'\x98' and is_flying:  # Hovering command (only if flying)
            commd = Hovering
            is_tracking = False
            send_uart1_response("Hovering")
            print('Hovering')

        elif a == b'\xe0' and is_flying:  # Line Tracking (only if flying)
            commd = LineTracking
            is_flying = True
            is_tracking = True
            send_uart1_response("Line Tracking")
            print('Line Tracking')

        elif a == b'\x9e':  # Emergency Stop command
            commd = EmerStop
            is_flying = False
            is_tracking = False
            send_uart1_response("Emergency Stop")
            print('Emergency Stop')

        elif a == CommCheck:  # Comm test command
                   comm_test = True  # Set comm_test to True if '00' command is received
                   send_uart1_response(f"Comm Check Received at {get_current_time()} - comm_test: {comm_test}")
                   failure_print_counter = 0  # Reset failure counter on successful comm test
                   print(f"Comm Check Received at {get_current_time()} - comm_test: {comm_test}")


        else:
            send_uart1_response("No valid command received")
            print('No valid command received')

# Main loop to keep checking UART
def remote_main():
    global comm_test, last_comm_time, failure_print_counter
    Remote_Control()
    comm_test = True  # Initially assume comm_test is True
    comm_check_interval = 30000  # 30 seconds interval (in milliseconds)

    # Check if 30 seconds have passed since the last communication
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_comm_time) > comm_check_interval:
        # If no communication was received in the last 30 seconds, set comm_test to False
        comm_test = False
        if failure_print_counter < failure_print_limit:
            send_uart_response(f"Comm Test Failed: No command received in the last 30 seconds at {get_current_time()} - comm_test: {comm_test}")
            print(f"Comm Test Failed: No command received in the last 30 seconds at {get_current_time()} - comm_test: {comm_test}")
            failure_print_counter += 1  # Increment counter each time the message is printed
    time.sleep(0.0005)  # Add a small delay to avoid busy looping




# Start the program


# obstacel dectation
# VL53L0X I2C 地址
VL53L0X_I2C_ADDR = 0x29  # 默认7位地址

# 写入 VL53L0X 寄存器的函数
def write_register(register, value):
    try:
        # 将数据写入I2C的寄存器
        i2c.writeto_mem(VL53L0X_I2C_ADDR, register, value.to_bytes(1, 'big'))
    except OSError as e:
        print("I2C Error (Write):", e)

# 从 VL53L0X 读取寄存器的函数
def read_register(register, length):
    try:
        # 从I2C读取寄存器数据
        data = i2c.readfrom_mem(VL53L0X_I2C_ADDR, register, length)
        return data
    except OSError as e:
        print("I2C Error (Read):", e)
        return None

# 初始化 VL53L0X 传感器
def vl53l0x_init():
    # 向 VL53L0X 发送初始化命令
    write_register(0x00, 0x01)  # 假设为启动命令
    print("VL53L0X Initialized")

# 读取距离并判断大小的函数
def check_distance():
    global too_close #check close state
    try:
        # 启动测量
        write_register(0x00, 0x01)
        time.sleep(0.0005)  # 等待测量完成

        # 从寄存器 0x1E 读取 2 字节距离数据
        data = read_register(0x1E, 2)

        if data:
            # 将高低字节合并为 16 位整数
            distance = (data[0] << 8) | data[1]
            distance = distance-60
            print("Distance: {} mm".format(distance))

            # 判断距离,1是大于500.可行走，0是小于500，停止
            if distance > 500 or distance < 0:
                print("Output: 1 (safe)")
                too_close = False
                return 1, distance
            else:
                print("Output: 0 too_close")
                too_close = True
                return 0, distance
        else:
            return None, None
    except OSError as e:
        print("I2C Error:", e)
        return None, None


# 主函数，初始化并读取距离
def obs_main():
    vl53l0x_init()


    result, distance = check_distance()
    if result is not None:
           # 只有在 result 有效时才执行
        time.sleep(0.0005)  # 每隔1秒读取一次距离数据
    else:
           # 处理返回 None 的情况（等待一段时间后重试）
        print("Failed to read distance. Retrying...")
        time.sleep(0.0005)

# 调用主函数


#line-track
def line():
    move_x = 0
    move_angle = 0

    img = sensor.snapshot().binary([THRESHOLD])
    detected_line = img.get_regression([(100, 100)], robust=True)

    if detected_line:
        line_center = (detected_line.x1() + detected_line.x2()) // 2
        img.draw_line(detected_line.line(), color=127)

        if abs(line_center - 80) >= 5:
            if abs(line_center - 80) <= 20:
                if line_center > 80:
                    move_x = 5
                    img.draw_string(80, 50, "R", color=200)
                else:
                    move_x = 6
                    img.draw_string(80, 50, "L", color=200)
            else:
                if line_center > 80:
                    move_x = 7
                    img.draw_string(80, 50, "RR", color=200)
                else:
                    move_x = 8
                    img.draw_string(80, 50, "LL", color=200)
        else:
            move_x = 0
    else:
        move_angle = 9  # e-stop

    if detected_line:
        img.draw_line(detected_line.line(), color=127)
        theta = detected_line.theta()
        if 0 <= theta < 6 or 174 <= theta < 180:
            move_angle = 0
            print(theta)
        elif 6 <= theta < 45:
            move_angle = 1
            print(theta)
            img.draw_string(80, 60, "r", color=200)
        elif 45 <= theta < 84:
            move_angle = 2
            print(theta)
            img.draw_string(80, 60, "rr", color=200)
        elif 96 <= theta < 138:
            move_angle = 3
            print(theta)
            img.draw_string(80, 60, "ll", color=200)
        elif 138 <= theta <= 174:
            move_angle = 4
            print(theta)
            img.draw_string(80, 60, "l", color=200)
        else:
            move_angle = 9  # e-stop
    else:
        move_angle = 9  # e-stop

    return move_x, move_angle

# Function to handle UART outputs based on commands
def handle_uart_output():
    global takeoff_command_sent

    if commd == Takeoff and not takeoff_command_sent:
        uart3.write(bytearray([0x33, 0x33]))
        takeoff_command_sent = True
    elif commd == Landing:
        uart3.write(bytearray([0xcc, 0xcc]))
        takeoff_command_sent = False
    elif commd == Hovering:
        uart3.write(bytearray([0x77]))
    elif commd == EmerStop:
        uart3.write(bytearray([0x55, 0x55]))
        takeoff_command_sent = False
    elif commd == LineTracking and is_tracking:
        if too_close:
            uart3.write(bytearray([0x77]))  # too close signal
        else:
            move_x, move_angle = line()
            handle_line_tracking_output(move_x, move_angle)
    else:
        uart3.write(bytearray([0x44]))
        print("waiting no command sent")


# Handle the UART output for line tracking based on move_x and move_angle
def handle_line_tracking_output(move_x, move_angle):
    # move_x handling
    if move_x == 0:
        uart3.write(bytearray([0x07]))
        print("straightx\n")
    elif move_x == 5:
        uart3.write(bytearray([0x19]))
        print("right\n")
    elif move_x == 6:
        uart3.write(bytearray([0x18]))
        print("left\n")
    elif move_x == 7:
        uart3.write(bytearray([0x1A]))
        print("right right\n")
    elif move_x == 8:
        uart3.write(bytearray([0x17]))
        print("left left\n")
    elif move_x == 9:
        uart3.write(bytearray([0x77]))  # Emergency stop
        print("hover")

    # move_angle handling
    if move_angle == 0:
        uart3.write(bytearray([0x07]))
        print("straighta\n")
    elif move_angle == 1:
        uart3.write(bytearray([0x29]))
        print("Ro right\n")
    elif move_angle == 2:
        uart3.write(bytearray([0x2A]))
        print("Ro right right\n")
    elif move_angle == 3:
        uart3.write(bytearray([0x28]))
        print("Ro left\n")
    elif move_angle == 4:
        uart3.write(bytearray([0x27]))
        print("Ro left left\n")
    elif move_angle == 9:
        uart3.write(bytearray([0x77]))  # Emergency stop
        print("Hover\n")

# Main loop to manage command priorities
while True:
    start_time = time.ticks_ms() #gain the time

    remote_main()  # Priority 1: Handle remote commands
    handle_uart_output()  # Output commands based on the current state

    if commd == LineTracking and is_tracking:  # Priority 2: Obstacle detection and line tracking
        obs_main()  # Run obstacle detection
        if not too_close:  # If it's safe to proceed, run line tracking
            move_x, move_angle = line()  # Run line tracking and get move_x and move_angle
            handle_line_tracking_output(move_x, move_angle)  # Send UART output based on move_x and move_angle

    end_time = time.ticks_ms()  # gain the end loop time
    elapsed_time = time.ticks_diff(end_time, start_time)  #total running time
    sleep_time = max(0, TARGET_TIME - elapsed_time)

    if sleep_time > 0:
        time.sleep(sleep_time / 1000.0)  # 转换为秒

        time.sleep((TARGET_TIME - elapsed_time) / 1000.0)  # Add a small delay
        print("code running time: {} ms".format(elapsed_time))
        print("sleep time: {} ms".format(sleep_time))

