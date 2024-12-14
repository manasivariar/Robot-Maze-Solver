import cv2
import maze_solver as ms

image = cv2.imread('mazeTest.png', cv2.IMREAD_GRAYSCALE)
ratio = 640 / image.shape[1]
dim = (640, int(image.shape[0] * ratio))
image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
img = cv2.resize(image, dim, interpolation=cv2.INTER_LINEAR)

interpolationPoints = [(392, 172), (392, 121), (281, 121), (281, 122), (280, 122), (280, 123), (279, 123), (279, 180), (336, 180), (336, 181), (337, 181), (337, 182), (338, 182), (338, 236), (204, 236)]
for i, j in zip(interpolationPoints[:-1], interpolationPoints[1:]):
    cv2.line(img, (i[0], i[1]), (j[0], j[1]), (0, 255, 0), 1)

cv2.imshow('image', img)
cv2.waitKey(0)