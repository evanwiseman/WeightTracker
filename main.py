import cv2
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture('video.mp4')

# Variables to store the user-defined initial circle
initial_circle = None
drawing = False  # Flag to indicate if we are currently drawing the circle
path_points = []

# Mouse callback to draw a circle
def draw_circle(event, x, y, flags, param):
    global initial_circle, drawing
    if event == cv2.EVENT_LBUTTONDOWN:
        # Start drawing the circle
        initial_circle = (x, y, 0)
        drawing = True
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        # Update radius as the user drags the mouse
        radius = int(np.sqrt((x - initial_circle[0])**2 + (y - initial_circle[1])**2))
        initial_circle = (initial_circle[0], initial_circle[1], radius)
    elif event == cv2.EVENT_LBUTTONUP:
        # Finalize the circle
        radius = int(np.sqrt((x - initial_circle[0])**2 + (y - initial_circle[1])**2))
        initial_circle = (initial_circle[0], initial_circle[1], radius)
        drawing = False

# Setup the mouse callback
cv2.namedWindow("Select Object to Track")
cv2.setMouseCallback("Select Object to Track", draw_circle)

# Initial frame capture to allow user to draw the circle
ret, frame = cap.read()
if not ret:
    print("Failed to capture video.")
    cap.release()
    cv2.destroyAllWindows()
    exit(0)

# Paused loop for user selection
while True:
    # Show frame to allow user to draw the circle
    temp_frame = frame.copy()
    if initial_circle is not None:
        # Draw the user-defined circle
        cv2.circle(temp_frame, (initial_circle[0], initial_circle[1]), initial_circle[2], (0, 255, 0), 2)

    cv2.imshow("Select Object to Track", temp_frame)

    # Wait for 'c' key to continue after circle is drawn
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') and initial_circle is not None:
        break  # Exit loop and start tracking
    elif key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        exit(0)  # Exit the program if 'q' is pressed

cv2.destroyWindow("Select Object to Track")

# Main tracking loop
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)

    # Use HoughCircles to find circles in the current frame
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=100,
        param2=30,
        minRadius=initial_circle[2] - 10,
        maxRadius=initial_circle[2] + 10
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        # Track the circle closest to the initial selection
        target_circle = min(circles, key=lambda c: np.sqrt((c[0] - initial_circle[0])**2 + (c[1] - initial_circle[1])**2))
        
        # Draw and update the path for the tracked circle
        center = (target_circle[0], target_circle[1])
        path_points.append(center)
        cv2.circle(frame, center, target_circle[2], (0, 255, 0), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # Draw the path of the barbell
    for i in range(1, len(path_points)):
        if path_points[i - 1] is None or path_points[i] is None:
            continue
        cv2.line(frame, path_points[i - 1], path_points[i], (255, 0, 0), 2)
    
    
    cv2.imshow("Barbell Path Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
