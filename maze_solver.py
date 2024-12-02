import cv2
import numpy as np
import threading
import colorsys


class Point(object):

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


rw = 2
p = 0
start = Point()
end = Point()

dir4 = [Point(0, -1), Point(0, 1), Point(1, 0), Point(-1, 0)]

def getCountOfPoints(ls):
    if not ls:
        return []

    result = []
    current_element = ls[0]
    count = 1

    for element in ls[1:]:
        if element == current_element:
            count += 1
        else:
            result.append((current_element, count))
            current_element = element
            count = 1

    result.append((current_element, count))
    return result

def getInterPolationPoints(path, pathArrayWithCount):
    global img

    interpolationPoints = [path[0]]
    count = 0

    for point, c in pathArrayWithCount[:-1]:
        pointX = point.x
        pointY = point.y

        count += c
        
        if pointX == -1 and pointY == 0:
            # Moving left
            if img[path[count].y][path[count].x-1][0] != 0 or img[path[count].y][path[count].x-1][1] != 0 or img[path[count].y][path[count].x-1][2] != 0:
                # print("-----No Wall Found when moving left")
                interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))
            else:
                # print("-----Wall Found when moving left")
                interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))

        elif pointX == 1 and pointY == 0:
            # Moving right
            if img[path[count].y][path[count].x+1][0] != 0 or img[path[count].y][path[count].x+1][1] != 0 or img[path[count].y][path[count].x+1][2] != 0:
                # print("-----No Wall Found when moving right")
                interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))

            else:
                # print("-----Wall Found when moving right")
                interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))

        elif pointX == 0 and pointY == -1:
            # Moving up
            if img[path[count].y-1][path[count].x][0] != 0 or img[path[count].y-1][path[count].x][1] != 0 or img[path[count].y-1][path[count].x][2] != 0:
                # print("-----No Wall Found when moving up")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))

            else:
                # print("-----Wall Found when moving up")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))

        elif pointX == 0 and pointY == 1:
            # Moving down
            if img[path[count].y+1][path[count].x][0] != 0 or img[path[count].y+1][path[count].x][1] != 0 or img[path[count].y+1][path[count].x][2] != 0:
                # print("-----No Wall Found when moving down")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))
            else:
                # print("-----Wall Found when moving down")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))
    
    # When the second last point is either moving up or down on either side of the point, we'll match the y.
    if point.x == 0 and (point.y == 1 or pointY == -1):
        interpolationPoints[-1] = Point(interpolationPoints[-1].x, path[-1].y)
    
    # When the second last point is either moving left or right on top/bottom side of the point, we'll match the x.
    if point.y == 0 and (point.x == 1 or pointX == -1):
        interpolationPoints[-1] = Point(path[-1].x, interpolationPoints[-1].y)

    interpolationPoints.append(path[-1])
    return interpolationPoints

def centeredPath(path):
    # if difference between two points is (-1, 0) then moving left
    # if difference between two points is (1, 0) then moving right
    # if difference between two points is (0, -1) then moving up
    # if difference between two points is (0, 1) then moving down
    global pathArrayWithCount

    pathArray = list(map(lambda p1, p2: Point(p2.x - p1.x, p2.y - p1.y), path[:-1], path[1:]))
    pathArrayWithCount = getCountOfPoints(pathArray)

    interpolationPoints = getInterPolationPoints(path, pathArrayWithCount)
    print("Interpolation points = ",[(p.x,p.y) for p in interpolationPoints])

    for i, j in zip(interpolationPoints[:-1], interpolationPoints[1:]):
        cv2.line(img, (i.x, i.y), (j.x, j.y), (0, 255, 0), 1)

    return path

def BFS(s, e):

    global img, h, w
    const = 10000

    found = False
    q = []
    v = [[0 for j in range(w)] for i in range(h)]
    parent = [[Point() for j in range(w)] for i in range(h)]

    q.append(s)
    v[s.y][s.x] = 1
    while len(q) > 0:
        p = q.pop(0)
        for d in dir4:
            cell = p + d
            if (cell.x >= 0 and cell.x < w and cell.y >= 0 and cell.y < h and v[cell.y][cell.x] == 0 and
                    (img[cell.y][cell.x][0] != 0 or img[cell.y][cell.x][1] != 0 or img[cell.y][cell.x][2] != 0)):
                q.append(cell)
                v[cell.y][cell.x] = v[p.y][p.x] + 1  # Later

                # img[cell.y][cell.x] = list(reversed(
                #     [i * 255 for i in colorsys.hsv_to_rgb(v[cell.y][cell.x] / const, 1, 1)])
                # )
                parent[cell.y][cell.x] = p
                if cell == e:
                    found = True
                    del q[:]
                    break

    path = []
    if found:
        p = e
        while p != s:
            path.append(p)
            p = parent[p.y][p.x]
        path.append(p)
        path.reverse()
        centeredPath(path)

        # for i, j in zip(path[:-1], path[1:]):
        #     cv2.line(img, (i.x, i.y), (j.x, j.y), (255, 255, 0), 1)
        print("Path Found")
    else:
        print("Path Not Found")

def mouse_event(event, pX, pY, flags, param):

    global img, start, end, p

    if event == cv2.EVENT_LBUTTONUP:
        if p == 0:
            cv2.rectangle(img, (pX - rw, pY - rw),
                          (pX + rw, pY + rw), (0, 0, 255), -1)
            start = Point(pX, pY)
            print("start = ", start.x, start.y)
            p += 1
        elif p == 1:
            cv2.rectangle(img, (pX - rw, pY - rw),
                          (pX + rw, pY + rw), (0, 200, 50), -1)
            end = Point(pX, pY)
            print("end = ", end.x, end.y)
            p += 1

def disp():
    global img
    cv2.imshow("Image", img)
    cv2.setMouseCallback('Image', mouse_event)
    while True:
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        # if cv2.waitKey(0) & 0xFF == ord('q'):
        #     quit()
        #     cv2.destroyAllWindows()

image = cv2.imread("maze3.png", cv2.IMREAD_GRAYSCALE)
_, image = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY)
ratio = 640 / image.shape[1]
dim = (640, int(image.shape[0] * ratio))
image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
img = cv2.resize(image, dim, interpolation=cv2.INTER_LINEAR)
# img = cv2.GaussianBlur(img,(5,5),0)
h, w = img.shape[:2]

print("Select start and end points : ")

t = threading.Thread(target=disp, args=())
t.daemon = True
t.start()

while p < 2:
    pass

BFS(start, end)

cv2.waitKey(0)
cv2.destroyAllWindows()
