import cv2
import json
from functools import partial

try:
    with open("colors.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}

print("Saved color values: ", saved_colors)
color = input("What color to threshold: ")

if color in saved_colors:
    filters = saved_colors[color]
else:
    filters = {
        "min": [0, 0, 0],
        "max": [179, 255, 255]
    }

def save():
    saved_colors[color] = filters

    with open("colors.json", "w") as f:
        f.write(json.dumps(saved_colors))

def update_range(edge, channel, value):
    filters[edge][channel] = value


cv2.namedWindow("mask")

cv2.createTrackbar("h_min", "mask", filters["min"][0], 179, partial(update_range, "min", 0))
cv2.createTrackbar("s_min", "mask", filters["min"][1], 255, partial(update_range, "min", 1))
cv2.createTrackbar("v_min", "mask", filters["min"][2], 255, partial(update_range, "min", 2))
cv2.createTrackbar("h_max", "mask", filters["max"][0], 179, partial(update_range, "max", 0))
cv2.createTrackbar("s_max", "mask", filters["max"][1], 255, partial(update_range, "max", 1))
cv2.createTrackbar("v_max", "mask", filters["max"][2], 255, partial(update_range, "max", 2))

cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cv2.namedWindow("bgr", flags=cv2.WINDOW_GUI_EXPANDED)

while cap.isOpened():
    _, bgr = cap.read()
    cv2.imshow('bgr', bgr)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
    cv2.imshow("mask", mask)
    key = cv2.waitKey(10)
    if key & 0xFF == ord("s"):
        save()
    if key & 0xFF == ord("q"):
        break

cap.release()
