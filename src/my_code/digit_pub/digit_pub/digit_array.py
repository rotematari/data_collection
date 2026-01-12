from digit_interface import DigitHandler, Digit
import cv2
from pprint import pprint
import os
import numpy as np


class DigitArray:
    def __init__(self,
                resolution="QVGA",
                fps=60,
                intensity=15,
                show_log=False,
                ros_logger=None):
        self.digits_info = DigitHandler.list_digits()
        self.show_log = show_log
        self.ros_logger = ros_logger
        self.resolution = resolution
        self.fps = fps
        self.intensity = intensity
        if show_log:
            pprint(self.digits_info)
            if ros_logger:
                ros_logger.info("Digit information: " + str(self.digits_info))
        self.digits = []
        self.ref_frames = {}  # Store reference frames for each digit
        self._connect_all()

    def _capture_ref_frame(self, digit, num_frames=10):
        """Capture reference frame for a single digit after connection"""
        if self.show_log:
            print(f"Capturing reference frame for digit {digit.serial}...")
        
        frames = []
        
        # Capture frames
        for i in range(num_frames):
            try:
                frame = digit.get_frame()
                if frame.ndim == 2:
                    # DIGITs usually send BayerGB. If colors are swapped (blue face), try BG2BGR
                    frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
                elif frame.ndim == 3 and frame.shape[2] == 1:
                    # Handle case where it is (H, W, 1)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
            except Exception as e:
                print(f"Error capturing frame from {digit.serial}: {e}")
                
            if frame is None:
                print(f"Received empty frame from {digit.serial}")
                continue
            frames.append(frame.astype(np.float32))
            
            if self.show_log and (i + 1) % 10 == 0:
                print(f"  Captured {i + 1}/{num_frames} frames for {digit.dev_name}")
        
        # Calculate mean frame
        mean_frame = np.mean(frames, axis=0).astype(np.uint8)
        # mean_frame = np.median(frames, axis=0).astype(np.uint8)
        
        
        # Store in memory
        self.ref_frames[digit.serial] = mean_frame
        
        if self.show_log:
            print(f"Reference frame captured for {digit.serial}")
        
        return mean_frame
    def capture_all_reference_frames(self, num_frames=30):
        """Capture reference frames for all connected digits"""
        
        for digit in self.digits:
            self._capture_ref_frame(digit, num_frames=num_frames)
        if self.show_log:
            print("All reference frames captured.")
    def _connect_one(self, info):
        try:
            if self.show_log:
                pprint(
                    f"Connecting to {info['serial']} with device name {info['dev_name']}..."
                )
            digit = Digit(info["serial"], name=info["dev_name"])
            digit.connect()
            # try to get a frame to verify connection
            try:
                _ = digit.get_frame()
            except Exception as e:
                if self.show_log:
                    print(f"Failed to get frame from {info['serial']}: {e}")
                return
            self.digits.append(digit)
            if self.show_log:
                print(f"Connected to {info['serial']}")
            
            # Capture reference frame immediately after connection
            self._capture_ref_frame(digit)
            
        except Exception as e:
            if self.show_log:
                print(f"Failed to connect to {info['serial']}: {e}")
    
    def _connect_all(self):
        len_before = len(self.digits_info)
        serials = set([d['serial'] for d in self.digits_info])
        if self.show_log:
            print(f"Found {len_before} DIGIT devices.")
            print(f"serials : {serials}")
            
        # for info in self.digits_info:
        for serial in serials:
            try:
                if self.show_log:
                    pprint(
                        f"Connecting to {serial} ..."
                    )
                digit = Digit(
                    serial=serial, name=f"DIGIT_{serial}"
                )
                digit.connect()
                if self.show_log:
                    print(f"Connected to {serial}, verifying by grabbing a frame...")
                # try to get a frame to verify connection
                try:
                    _ = digit.get_frame()
                except Exception as e:
                    print(f"Failed to get frame from {serial}: {e}")
                    continue
                print(f"Connected to {serial} successfully.\n\n")
                
                # Set to QVGA at 30 fps
                digit.set_resolution(digit.STREAMS[self.resolution])
                digit.set_fps(digit.STREAMS[self.resolution]["fps"][f"{self.fps}fps"])
                digit.set_intensity(self.intensity)
                self.digits.append(digit)
                # Capture reference frame immediately after connection
                self._capture_ref_frame(digit)
                # connected.add(info["serial"])
                # if self.show_log:
                #     print(f"Connected to {info['serial']}")
            except Exception as e:
                print(f"Failed to connect to {serial}: {e}")
                # if self.show_log:
                    # print(f"connect_all : Failed to connect to {info['dev_name']}: {e}")

        print("Successfully connected to")
        pprint(self.digits)

    def save_ref_frames(self, save_dir="ref_frames"):
        """Save all captured reference frames to disk"""
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        for serial, ref_frame in self.ref_frames.items():
            filename = f"ref_frame_{serial}.png"
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, ref_frame)
            if self.show_log:
                print(f"Reference frame saved to {filepath}")
    def load_ref_frames(self, load_dir="assets/ref_frames"):
        """Load reference frames from disk if they exist"""
        if not os.path.exists(load_dir):
            if self.show_log:
                print(f"Reference frame directory {load_dir} does not exist.")
            return
        
        for info in self.digits_info:
            serial = info["serial"]
            filename = f"ref_frame_{serial}.png"
            filepath = os.path.join(load_dir, filename)
            if os.path.exists(filepath):
                ref_frame = cv2.imread(filepath)
                self.ref_frames[serial] = ref_frame
                if self.show_log:
                    print(f"Loaded reference frame for {serial} from {filepath}")
            else:
                if self.show_log:
                    print(f"No reference frame found for {serial} at {filepath}")
    def get_reference_frame(self, digit_serial):
        """Get the reference frame for a specific digit"""
        return self.ref_frames.get(digit_serial, None)
    
    def show_all_views(self,diff_with_ref=False):
        try:
            while True:
                for digit in self.digits:
                    frame = digit.get_frame()
                    # --- FIX START: Handle Raw Bayer Data ---
                    # If frame is single channel (grayscale) or raw, convert it
                    # if frame.ndim == 2:
                    #     # DIGITs usually send BayerGB. If colors are swapped (blue face), try BG2BGR
                    #     frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
                    # elif frame.ndim == 3 and frame.shape[2] == 1:
                    #     # Handle case where it is (H, W, 1)
                    #     frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
                    # --- FIX END ---
                    if diff_with_ref:
                        ref_frame = self.get_reference_frame(digit.serial)
                        if ref_frame is not None:
                            frame = cv2.absdiff(frame, ref_frame)
                    cv2.imshow(f"Digit {digit.serial}", frame)
                # ESC to exit
                if cv2.waitKey(1) == 27:
                    print("Exiting...")
                    break
        except KeyboardInterrupt:
            print("Exiting due to keyboard interrupt...")
        except Exception as e:
            print(f"An error occurred: {e}")
        # finally:
        #     self.disconnect_all()

    def disconnect_all(self):
        for digit in self.digits:
            digit.disconnect()
            print(f"Disconnected from {digit.serial}")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        digit_array = DigitArray(show_log=True ,resolution="QVGA", fps=30, intensity=15)
        print("\n\nStarting to show all DIGIT views. Press ESC to exit.\n\n")
        digit_array.show_all_views(diff_with_ref=False  )
    except Exception as e:
        print(f"Error: {e}")
    finally:
        digit_array.disconnect_all()
        print("Disconnected from all DIGIT devices.")
