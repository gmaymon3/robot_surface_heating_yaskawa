import numpy as np
import cv2 as cv

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv.resize(frame, dim, interpolation =cv.INTER_AREA)

cap = cv.VideoCapture(0)
if not cap.isOpened():
  print("Cannot open camera")
  exit()
while True:
  # Capture frame-by-frame
  ret, frame = cap.read()
  frame_big = rescale_frame(frame, percent=1000)
  # if frame is read correctly ret is True
  if not ret:
    print("Can't receive frame (stream end?). Exiting ...")
    break
  # Our operations on the frame come here
  color = cv.cvtColor(frame_big, cv.COLOR_BGR2BGRA)
  # Display the resulting frame
  cv.imshow('frame_big', color)
  if cv.waitKey(1) == ord('q'):

    break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()