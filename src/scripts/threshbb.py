import cv2
import json
from functools import partial
import numpy as np
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
        "min": 0,
        "max": 255
    }

def save():
    saved_colors[color] = filters

    with open("colors.json", "w") as f:
        f.write(json.dumps(saved_colors))

def update_range(channel, value):
    filters[channel] = value


cv2.namedWindow("mask")

cv2.createTrackbar("min", "mask", filters['min'], 255, partial(update_range, "min"))
cv2.createTrackbar("max", "mask", filters['max'], 255, partial(update_range, "max"))

cap = cv2.VideoCapture(4)

t_mask = np.zeros((480, 640), np.uint8)
t_mask[160:320] = 1
t_mask[280:,370:450] = 0

while cap.isOpened():
    _, bgr = cap.read()
    cv2.imshow('bgr', bgr)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = np.multiply(t_mask, gray)
    mask = cv2.inRange(gray, filters['min'], filters['max']) 
    cv2.imshow("mask", mask)
    cv2.imshow('gray', gray)
    key = cv2.waitKey(10)
    if key & 0xFF == ord("s"):
        save()
    if key & 0xFF == ord("q"):
        break

cap.release()
