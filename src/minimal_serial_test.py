import rclpy
from rclpy.node import Node
import serial
import time
import threading

SERIAL_PORT = '/dev/ttyACM0' # *** VERIFY THIS PORT ***
BAUD_RATE = 115200
TIMEOUT_SEC = 1.0

class MinimalSerialTester(Node):
    def __init__(self):
        super().__init__('minimal_serial_tester')
        self.get_logger().info('Minimal Serial Tester Node Started')
        self.serial_conn = None
        self.connect_serial()

        # Send a test command after a short delay
        self.timer = self.create_timer(2.0, self.send_test_command) # Wait 2s then send

        # Optional: Thread to read responses
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def connect_serial(self):
        try:
            self.get_logger().info(f'Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...')
            self.serial_conn = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=TIMEOUT_SEC
            )
            # Allow Arduino time to reset after connection
            time.sleep(2.0)
            self.get_logger().info('Serial connection established.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {SERIAL_PORT}: {e}')
            self.serial_conn = None
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred during serial connection: {e}')
            self.serial_conn = None

    def send_test_command(self):
        if self.serial_conn and self.serial_conn.is_open:
            # --- CHOOSE A COMMAND TO SEND ---
            # command = "H\n"  # Test Homing
            command = "P 0 500\n" # Test moving joint 0 to 500 steps
            # command = "E\n" # Test Emergency Stop
            # --------------------------------

            try:
                self.get_logger().info(f'Sending command: {command.strip()}')
                self.serial_conn.write(command.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
            except Exception as e:
                 self.get_logger().error(f'An unexpected error occurred during serial write: {e}')
        else:
            self.get_logger().warn('Serial port not connected or open. Cannot send command.')

        # Stop the timer after sending once
        self.timer.cancel()
        self.get_logger().info('Test command sent. Timer stopped.')


    def read_serial_loop(self):
        while rclpy.ok() and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f'Received from Arduino: {line}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break # Exit loop on error
            except UnicodeDecodeError:
                self.get_logger().warn('Received non-UTF8 character, skipping line.')
            except Exception as e:
                 self.get_logger().error(f'An unexpected error occurred during serial read: {e}')
                 break # Exit loop on error
            time.sleep(0.01) # Small delay to prevent busy-waiting

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSerialTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()