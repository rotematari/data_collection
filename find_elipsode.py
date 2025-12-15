import cv2
import numpy as np
import matplotlib.pyplot as plt

def fit_ellipsoid_to_image(image_path):
    # 1. Load the image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not load image at {image_path}")
        return

    # Create a copy for drawing
    output_img = img.copy()

    # 2. Preprocessing
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian Blur to reduce noise and smoothen edges
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 3. Thresholding
    # Use Otsu's method to automatically find the best threshold value
    # This separates the foreground (object) from the background
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 4. Find Contours
    # Retrieve external contours (the outer boundary of the object)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No contours found in the image.")
        return

    # Assume the largest contour is the object we want
    largest_contour = max(contours, key=cv2.contourArea)

    # 5. Fit Ellipse
    # The function returns a rotated rectangle: ((center_x, center_y), (width, height), angle)
    if len(largest_contour) >= 5: # fitEllipse requires at least 5 points
        ellipse = cv2.fitEllipse(largest_contour)
        
        # Unpack parameters
        (xc, yc), (major_axis, minor_axis), angle = ellipse
        
        print(f"Ellipse Found:")
        print(f"  Center (x, y from top-left): ({xc:.2f}, {yc:.2f})")
        print(f"  Major Axis Length: {major_axis:.2f}")
        print(f"  Minor Axis Length: {minor_axis:.2f}")
        print(f"  Rotation Angle: {angle:.2f} degrees")

        # 6. Visualization
        # Draw the ellipse in green (0, 255, 0) with a thickness of 2
        cv2.ellipse(output_img, ellipse, (0, 255, 0), 2)
        
        # Draw the center point in red
        cv2.circle(output_img, (int(xc), int(yc)), 5, (0, 0, 255), -1)

        # Convert BGR to RGB for Matplotlib display
        output_rgb = cv2.cvtColor(output_img, cv2.COLOR_BGR2RGB)
        
        plt.figure(figsize=(10, 10))
        plt.imshow(output_rgb)
        plt.title(f"Detected Ellipse\nCenter: ({xc:.1f}, {yc:.1f})")
        plt.axis('off')
        plt.show()
        
    else:
        print("Contour is too small to fit an ellipse (needs >= 5 points).")

if __name__ == "__main__":
    # Ensure 'digit2.png' is in the same directory or provide full path
    fit_ellipsoid_to_image("digit2.png")