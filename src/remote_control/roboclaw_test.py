import roboclaw_driver as roboclaw
import time
import numpy as np
import diagnostic_msgs
import diagnostic_updater

class Struct:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

class Twist:
    def __init__(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        self.linear = Struct(x=linear_x, y=linear_y, z=linear_z)
        self.angular = Struct(x=angular_x, y=angular_y, z=angular_z)

    def __repr__(self):
        return (
            f"Twist(linear: x={self.linear.x}, y={self.linear.y}, z={self.linear.z}, "
            f"angular: x={self.angular.x}, y={self.angular.y}, z={self.angular.z})"
        )


class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = time.time()

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = time.time()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * np.cos(self.cur_theta)
            self.cur_y += dist * np.sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (np.sin(d_theta + self.cur_theta) - np.sin(self.cur_theta))
            self.cur_y -= r * (np.cos(d_theta + self.cur_theta) - np.cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def print_state(self, enc_l, enc_r):
        vx, vth = self.update(enc_l, enc_r)
        current_time = time.time()

        print("Beginning of Odometry message")
        print(f"Current Time: {current_time}")

        print(f"position.x = {self.cur_x}")
        print(f"position.y = {self.cur_y}")
        print(f"position.z = {0.0}")

        print(f"twist.linear.x = {vx}")
        print(f"twist.linear.y = {0}")
        print(f"twist.angular.z = {vth}")


class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}
        
        self.address = 128
        dev_name = "/dev/ttyACM0"
        baud_rate = 115200

        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            print("Could not connect to Roboclaw")
            raise(e)
        
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            print("Problem getting roboclaw version")
            print(e)
            pass

        if not version[0]:
            print("Could not get version from roboclaw")
        else:
            print(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)
        roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = 2.0
        self.TICKS_PER_METER = 4342.2
        self.BASE_WIDTH = 0.315

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = time.time()


        print("dev %s", dev_name)
        print("baud %d", baud_rate)
        print("address %d", self.address)
        print("max_speed %f", self.MAX_SPEED)
        print("ticks_per_meter %f", self.TICKS_PER_METER)
        print("base_width %f", self.BASE_WIDTH)


    def run(self):
        print("Starting motor drive")
        r_time = 10
        while True:

            if (time.time() - self.last_set_speed_time).to_sec() > 1:
                print("Did not get command for 1 second, stopping")
                try:
                    roboclaw.ForwardM1(self.address, 0)
                    roboclaw.ForwardM2(self.address, 0)
                except OSError as e:
                    print("Could not stop")
                    print(e)
                    raise

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status1, enc1, crc1 = None, None, None
            status2, enc2, crc2 = None, None, None

            try:
                enc1, crc1 = roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                print("ReadEncM1 OSError: %d", e.errno)
                print(e)

            try:
                enc2, crc2 = roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                print("ReadEncM2 OSError: %d", e.errno)
                print(e)
                raise
            except KeyboardInterrupt:
                print("Stopping...")
                raise

            if ('enc1' in vars()) and ('enc2' in vars()):
                print(f" Encoders {enc1} {enc2}")
                self.encodm.print_state(enc1, enc2)
                self.updater.update()

            time.sleep(r_time)

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = time.time()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vr = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vl = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        print("vr_ticks:%d vl_ticks: %d", vr_ticks, vl_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks is 0 and vl_ticks is 0:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            else:
                roboclaw.SpeedM1M2(self.address, vr_ticks, vl_ticks)
        except OSError as e:
            print("SpeedM1M2 OSError: %d", e.errno)
            print(e)
            raise

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            print("Diagnostics OSError: %d", e.errno)
            print(e)
            return

        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", float(roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", float(roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            print("Diagnostics OSError: %d", e.errno)
            print(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        print("Shutting down")
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            print("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                print("Could not shutdown motors!!!!")
                print(e)


import keyboard  # Install with `pip install keyboard`

def keyboard_control(node):
    """
    Allows motor testing using keyboard input.
    - 'w' to move forward
    - 's' to move backward
    - 'a' to turn left
    - 'd' to turn right
    - 'space' to stop
    - 'q' to quit
    """
    print("Keyboard control activated. Use keys to control motors:")
    print("'w': Forward, 's': Backward, 'a': Turn left, 'd': Turn right, 'space': Stop, 'q': Quit")

    try:
        while True:
            if keyboard.is_pressed('w'):
                node.cmd_vel_callback(Twist(linear_x=node.MAX_SPEED, angular_z=0))
            elif keyboard.is_pressed('s'):
                node.cmd_vel_callback(Twist(linear_x=-node.MAX_SPEED, angular_z=0))
            elif keyboard.is_pressed('a'):
                node.cmd_vel_callback(Twist(linear_x=0, angular_z=node.MAX_SPEED / 2))
            elif keyboard.is_pressed('d'):
                node.cmd_vel_callback(Twist(linear_x=0, angular_z=-node.MAX_SPEED / 2))
            elif keyboard.is_pressed('space'):
                node.cmd_vel_callback(Twist(linear_x=0, angular_z=0))
            elif keyboard.is_pressed('q'):
                print("Exiting keyboard control...")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Keyboard control interrupted.")
    finally:
        node.shutdown()


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except Exception as e:
        pass
    print("Exiting")