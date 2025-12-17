import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Adafruit_PCA9685
from math import cos, sin, pi

class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
        self.pwm.set_pwm_freq(60)
        self.channels = [0, 4, 8]
        self.STOP = 375
        #self.theta = [0, 2*pi/3, -2*pi/3]
        self.theta = [pi/2, -pi/6, -5*pi/6]

	# サブスクライバ: 値を保存するだけ
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)

        # タイマー: 10Hz (0.1秒) ごとに計算・出力
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.current_vx = 0
        self.current_vy = 0
        self.current_omega = 0

        # 通信途絶監視用の変数（現在時刻で初期化）
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = 0.5 # 0.5秒以上途切れたら停止

    def cmd_vel_callback(self, msg: Twist):
        self.current_vx = msg.linear.x
        self.current_vy = msg.linear.y
        self.current_omega = msg.angular.z

        # メッセージ受信時刻を更新
        self.last_msg_time = self.get_clock().now()  
 
    def timer_callback(self):
        # 現在時刻と最後に受信した時刻の差を計算
        elapsed_time = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9

        # タイムアウト判定
        if elapsed_time > self.timeout_duration:
            vx, vy, omega = 0.0, 0.0, 0.0
        else:
            vx = self.current_vx
            vy = self.current_vy
            omega = self.current_omega

        motor_sign = [1, 1, 1]  # 各モータの回転方向の符号
        pwm_values = []
        for th, sign in zip(self.theta, motor_sign):
            #speed = vx * cos(th) + vy * sin(th) + omega
            speed = vx * cos(th) + vy * sin(th) + omega*-1
            pwm = int(self.STOP + sign * speed * 200)
            pwm = max(150, min(600, pwm))
            if pwm == self.STOP:
               pwm = 0
            pwm_values.append(pwm)

        for ch, pwm_val in zip(self.channels, pwm_values):
            self.pwm.set_pwm(ch, 0, pwm_val)

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

