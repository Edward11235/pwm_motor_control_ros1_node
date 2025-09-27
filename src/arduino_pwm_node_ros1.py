#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from asv_motor_control_ros1_node.msg import Force
import serial
import time
import math

# Configuration
PORT = '/dev/ttyACM0'  # Update for Windows (e.g., 'COM3')
BAUD = 115200
PWM_MIN = 1100
PWM_MAX = 1900
PWM_STOP = 1500


class ArduinoPWMNodeROS1(object):
    def __init__(self):
        self.node_name = 'arduino_pwm_controller'
        rospy.init_node(self.node_name, anonymous=False)

        # Serial setup
        self.ser = None
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            rospy.loginfo("Connected to Arduino on %s at %d baud", PORT, BAUD)
        except serial.SerialException as e:
            rospy.logerr("Failed to connect to Arduino: %s", str(e))
            self.ser = None

        self.sub = rospy.Subscriber( '/mpc_force', Force, self.force_callback, queue_size=10)

        # Ensure safe shutdown
        rospy.on_shutdown(self.on_shutdown)

    def force_callback(self, msg):
        """
        Callback for Force message containing float64[4] data
        Converts force values to PWM values and sends to Arduino
        """
        if self.ser is None:
            rospy.logwarn("No serial connection - cannot send PWM")
            return

        # Expecting 4 force values: [left, right, front, back]
        if len(msg.data) != 4:
            rospy.logwarn("Expected 4 force values, got %d", len(msg.data))
            return

        # Convert force values to PWM values
        # You'll need to define the mapping from force to PWM
        pwm_values = self.force_to_pwm(msg.data)
        
        # Clamp values to valid range
        clamped_values = []
        for pwm in pwm_values:
            if pwm < PWM_MIN or pwm > PWM_MAX:
                rospy.logwarn(
                    "PWM value %d out of range. Clamping to %d-%d",
                    pwm, PWM_MIN, PWM_MAX
                )
                clamped_values.append(max(PWM_MIN, min(pwm, PWM_MAX)))
            else:
                clamped_values.append(pwm)

        left_pwm, right_pwm, front_pwm, back_pwm = clamped_values
        self.send_pwm(left_pwm, right_pwm, front_pwm, back_pwm)

    def force_to_pwm(self, force_kgf) -> float:
        A1L = -1.371921163e-04
        B1L = 4.652741433e-01
        C1L = -3.87238288e+02
        A2H = 1.84721848e-04
        B2H = -4.889302056e-01
        C2H = 3.153301465e+02
        KGF_TO_N = 9.80665
        NEAR_ZERO_N = 0.01
        LO, HI = 1100.0, 1900.0
        MID = 1500.0

        def inv_quad(a: float, b: float, c: float, F: float, lo: float, hi: float) -> float:
            A = a
            B = b
            C = c - F
            if abs(A) < 1e-12:
                if abs(B) < 1e-12:
                    raise ValueError
                p = -C / B
                if lo - 1e-6 <= p <= hi + 1e-6:
                    return float(p)
                raise ValueError
            disc = B * B - 4.0 * A * C
            if disc < 0:
                raise ValueError
            sd = math.sqrt(disc)
            p1 = (-B + sd) / (2.0 * A)
            p2 = (-B - sd) / (2.0 * A)
            cands = [p for p in (p1, p2) if lo - 1e-6 <= p <= hi + 1e-6]
            if cands:
                mid = 0.5 * (lo + hi)
                return float(min(cands, key=lambda p: abs(p - mid)))
            cand = min((p1, p2), key=lambda p: min(abs(p - lo), abs(p - hi)))
            if lo - 5.0 <= cand <= hi + 5.0:
                return float(cand)
            raise ValueError

        F_n = float(force_kgf) * KGF_TO_N
        if -NEAR_ZERO_N <= F_n <= NEAR_ZERO_N:
            return MID
        try:
            pwm = inv_quad(A1L, B1L, C1L, F_n, 1100.0, 1500.0)
        except Exception:
            pwm = inv_quad(A2H, B2H, C2H, F_n, 1500.0, 1900.0)
        if pwm < LO:
            return LO
        if pwm > HI:
            return HI
        return pwm


    def send_pwm(self, left, right, front, back):
        msg = f"<[{left}][{right}][{front}][{back}]>"
        try:
            self.ser.write(msg.encode())
            reply = self.ser.readline().decode().strip()
            rospy.loginfo("Sent: %s | Received: %s", msg, reply)
        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s", str(e))

    def on_shutdown(self):
        if self.ser and self.ser.is_open:
            # Send stop command before closing
            try:
                self.send_pwm(PWM_STOP, PWM_STOP, PWM_STOP, PWM_STOP)
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            rospy.loginfo("Serial connection closed")


def main():
    node = ArduinoPWMNodeROS1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()


