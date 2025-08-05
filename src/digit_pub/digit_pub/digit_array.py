
from digit_interface import DigitHandler, Digit
import cv2
from pprint import pprint

class DigitArray:
    def __init__(self,show_log=True):
        self.digits_info = DigitHandler.list_digits()
        if show_log:
            pprint(self.digits_info)
        self.digits = []
        self._connect_all()

    def _connect_all(self):
        len_before = len(self.digits_info)
        print(f"Found {len_before} DIGIT devices.")
        for info in self.digits_info:
            try:
                pprint(f"Connecting to {info['serial']} with device name {info['dev_name']}...")
                digit = Digit(info['serial'], dev_name=info['dev_name'])
                digit.connect()
                self.digits.append(digit)
                print(f"Connected to {info['serial']}")
            except Exception as e:
                print(f"Failed to connect to {info['serial']}: {e}")
                # self.digits.pop()  # Remove the last element if connection fails
                # self.digits_info.remove(info)  # Remove the info from the list
                # print(f"Error connecting to {info['serial']}: {e}")


    def show_all_views(self):
        try:
            while True:
                for digit in self.digits:
                    frame = digit.get_frame()
                    cv2.imshow(f"Digit {digit.serial}", frame)
                # ESC to exit
                if cv2.waitKey(1) == 27 :
                    print("Exiting...")
                    break
        except KeyboardInterrupt:
            print("Exiting due to keyboard interrupt...")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.disconnect_all()

    def disconnect_all(self):
        for digit in self.digits:
            digit.disconnect()
            print(f"Disconnected from {digit.serial}")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    digit_array = DigitArray()
    digit_array.show_all_views()
