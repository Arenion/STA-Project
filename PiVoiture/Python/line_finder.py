import numpy as np
import time
from picamera2 import Picamera2
from libcamera import Transform
import socket

SOCKET_PATH = "/tmp/cam.sock"
debug = False

if debug:
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg

medium_lenght = 210
# Define color selection criteria
red_threshold = 100
green_threshold = 100
blue_threshold = 100

rgb_threshold = [red_threshold, green_threshold, blue_threshold]

height = 300

medium_lenght = 228

# Find the right and left lines at a certain height
def find_line(image, direction, height=300, start=640//2, rgb_threshold=rgb_threshold, end=None, step=2):
    if end == None:
        end = 0 if direction == -1 else xsize
    step *= direction
    for i in range(start, end, step):
        if not (image[height, i, :3] > rgb_threshold).all():
            for y in range(i-step+1, i):
                if not (image[height, y, :3] > rgb_threshold).all():
                    return y
            return i

    return None

def complexe_find_line(image, height=300, rgb_threshold=rgb_threshold):
    # compute lenghts between all "dark" spots, and find the best "dark" spot
    lenghts = []
    coordinates = []
    i = 0
    while i < xsize:
        while i < xsize and not (image[height, i, :3] > rgb_threshold).any():
            i += 1
        coordinates.append(i)
        while i < xsize and (image[height, i, :3] > rgb_threshold).any():
            i += 1
        coordinates[-1] = (coordinates[-1], i)
        lenghts.append(coordinates[-1][1]-coordinates[-1][0])
    best = xsize + 1
    besti = -1
    for i in range(len(lenghts)):
        if abs(lenghts[i] - medium_lenght) < best:
            best = abs(lenghts[i] - medium_lenght)
            besti = i

    if best < 100:
        return coordinates[besti]
    return None, None

def get_angle_error(image, start=640//2, height=height, rgb_threshold=rgb_threshold):
    left = find_line(image, -1, height, start, rgb_threshold)
    right = find_line(image, 1, height, start, rgb_threshold)
    if left is None or right is None or right - left < 150:
        print("Initial try failed, performing advanced line finding")
        left, right = complexe_find_line(image, height, rgb_threshold)
    if left is None:
        print("ERROR")
        return None, None, None
    lenght = right - left
    #middle = (left+right)/2
    #angle = np.arctan((middle-xsize//2)/height)
    ratio = ((640/2)-left)/lenght
    return ratio, left, right

# --- CAMERA SETUP (Picamera2) ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)}, transform=Transform(hflip=True, vflip=True))
picam2.configure(config)
picam2.start()

time.sleep(2)  # Warm-up

frame_count = 0

try:
    # --- Socket startup ---
    client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    #print("Connecting socket")
    #client.connect(SOCKET_PATH)

    print("Starting loop")
    while True:
        image = picam2.capture_array()
        ysize, xsize, _ = image.shape
        # Compute angle error
        ratio, left, right = get_angle_error(image)

        if not ratio is None:
            client.sendto(f"{ratio:.3f} LENT caca pipi".encode(), SOCKET_PATH)
        if debug:
            color_select = image.copy()
            color_thresholds = (image[:,:,0] > rgb_threshold[0]) | (image[:,:,1] > rgb_threshold[1]) | (image[:,:,2] > rgb_threshold[2])
            color_select[color_thresholds] = [0, 0, 0, 255]

        print(f"{left=}, {right=}, {ratio=};")
        if not debug:
            continue
        plt.imshow(color_select)
        if not left is None:
            plt.plot([left, right], [300, 300], 'r--')
            plt.plot((left+right)/2,height, 'rx')
            plt.plot([(left+right)/2, xsize/2], [300, ysize], 'g--')
            plt.title(f"Lines found, {ratio=}")
        else:
            plt.title("Lines not found")

        frame_count += 1
        filename = f"frame_{frame_count:04d}.png"
        plt.savefig(filename, bbox_inches='tight', pad_inches=0)
        plt.close()
except KeyboardInterrupt:
    print("\nCapture stopped by user.")
finally:
    picam2.stop()
