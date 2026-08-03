import cv2
import numpy as np
import time

def initialize_tracker():
    # HSV color range for a specific marker color (e.g., Retro-reflective Green)
    # Tuning this tightly allows us to bypass complex background processing
    lower_color = np.array([35, 100, 100])
    upper_color = np.array([85, 255, 255])
    return lower_color, upper_color

def get_line_intersection(line1, line2):
    """
    Calculates the intersection of two lines.
    Lines are defined by tuples of points: ((x1, y1), (x2, y2))
    Returns the (x, y) intersection as sub-pixel floats.
    """
    # Extract points
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    # Calculate determinant
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None # Lines are parallel

    # Calculate intersection using determinants
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

    return (px, py)

def process_frame_for_center(frame, lower_color, upper_color):
    """
    Processes a 320x240 frame to find the intersection of a cross marker.
    Even if the center is occluded, the visible arm segments will be mathematically 
    extended to find the true center point.
    """
    # 1. Convert to HSV and threshold to isolate the marker
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 2. Morphological closing to fill small gaps in the detection
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 3. Edge detection
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)

    # 4. Probabilistic Hough Transform to find line segments
    # Tuned for 320x240: small minLineLength ensures we catch partial segments
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=30, 
                            minLineLength=20, maxLineGap=10)

    if lines is None:
        return None, mask

    # We will separate lines into two groups based on their slope (angle)
    # Assuming a cross roughly aligned with the camera (e.g. vertical/horizontal or diagonal)
    group_1 = []
    group_2 = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        # Calculate angle of the line segment
        angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
        
        # Normalize angle to 0-180 degrees
        if angle < 0:
            angle += 180
            
        # Group based on orientation (assuming an 'X' shape cross ~45 and ~135 degrees)
        # Adjust these thresholds if your cross is a '+' shape (0 and 90 degrees)
        if 20 < angle < 70:
            group_1.append((x1, y1, x2, y2))
        elif 110 < angle < 160:
            group_2.append((x1, y1, x2, y2))

    if not group_1 or not group_2:
        return None, mask

    # Fit a single mathematical line through all segments in Group 1
    points1 = [(x, y) for x1, y1, x2, y2 in group_1 for x, y in ((x1, y1), (x2, y2))]
    [vx1, vy1, x1, y1] = cv2.fitLine(np.array(points1), cv2.DIST_L2, 0, 0.01, 0.01)
    line1_pts = ((x1 - vx1*1000, y1 - vy1*1000), (x1 + vx1*1000, y1 + vy1*1000))

    # Fit a single mathematical line through all segments in Group 2
    points2 = [(x, y) for x1, y1, x2, y2 in group_2 for x, y in ((x1, y1), (x2, y2))]
    [vx2, vy2, x2, y2] = cv2.fitLine(np.array(points2), cv2.DIST_L2, 0, 0.01, 0.01)
    line2_pts = ((x2 - vx2*1000, y2 - vy2*1000), (x2 + vx2*1000, y2 + vy2*1000))

    # 5. Calculate the precise sub-pixel intersection of the two fitted lines
    center = get_line_intersection(line1_pts, line2_pts)
    
    return center, mask

def main():
    # Set up simulated camera input (replace with PiCamera code)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    # Lock exposure here if using picamera/v4l2

    lower_color, upper_color = initialize_tracker()
    
    prev_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Calculate exact center point
        center, mask = process_frame_for_center(frame, lower_color, upper_color)

        # Track Loop timing (Aiming for < 33ms for 30Hz)
        curr_time = time.time()
        hz = 1.0 / (curr_time - prev_time)
        prev_time = curr_time

        if center:
            cx, cy = center
            # Convert float coordinates to int for drawing only
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Center: {cx:.2f}, {cy:.2f}", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.putText(frame, f"{hz:.1f} Hz", (240, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()