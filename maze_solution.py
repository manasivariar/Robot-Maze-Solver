import cv2
import threading
import savePointsCSV
import getRobotCoordinates

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

    for index, (point, c) in enumerate(pathArrayWithCount[:-1]):
        pointX = point.x
        pointY = point.y

        count += c
        
        if pointX == -1 and pointY == 0:
            if index == 0: 
                pixelX = path[count + pathArrayWithCount[index+1][1]].x + 1
                pixelY = path[count + pathArrayWithCount[index+1][1]].y - pathArrayWithCount[index+1][0].y
                if img[pixelY][pixelX][0] == 0 and img[pixelY][pixelX][1] == 0 and img[pixelY][pixelX][2] == 0:
                    interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))
                else:
                    interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))
                continue
            # Moving left
            for i in range(1,16):
                if (
                    (img[path[count].y][path[count].x-i][0] == 0 and img[path[count].y][path[count].x-i][1] == 0 and img[path[count].y][path[count].x-i][2] == 0) or
                    (img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x-i][0] == 0 and img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x-i][1] == 0 and img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x-i][2] == 0)
                    ):
                    # print("-----No Wall Found when moving left")
                    interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))
                    break
            else:
                # print("-----Wall Found when moving left")
                interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))

        elif pointX == 1 and pointY == 0:
            if index == 0:
                pixelX = path[count + pathArrayWithCount[index+1][1]].x - 1
                pixelY = path[count + pathArrayWithCount[index+1][1]].y - pathArrayWithCount[index+1][0].y
                if img[pixelY][pixelX][0] == 0 and img[pixelY][pixelX][1] == 0 and img[pixelY][pixelX][2] == 0:
                    interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))
                else:
                    interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))
                continue
            # Moving right
            for i in range(1,16):
                if (
                    (img[path[count].y][path[count].x+i][0] == 0 and img[path[count].y][path[count].x+i][1] == 0 and img[path[count].y][path[count].x+i][2] == 0) or
                    (img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x+i][0] == 0 and img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x+i][1] == 0 and img[path[count].y+pathArrayWithCount[index+1][0].y][path[count].x+i][2] == 0)
                    ):
                    # print("-----No Wall Found when moving right")
                    
                    interpolationPoints.append(Point(path[count].x - 25, interpolationPoints[-1].y))
                    break

            else:
                # print("-----Wall Found when moving right")
                interpolationPoints.append(Point(path[count].x + 25, interpolationPoints[-1].y))

        elif pointX == 0 and pointY == -1:
            if index == 0:
                pixelY = path[count + pathArrayWithCount[index+1][1]].y + 1
                pixelX = path[count + pathArrayWithCount[index+1][1]].x - pathArrayWithCount[index+1][0].x
                if img[pixelY][pixelX][0] == 0 and img[pixelY][pixelX][1] == 0 and img[pixelY][pixelX][2] == 0:
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))
                else:
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))
                continue
            # Moving up
            for i in range(1, 16):
                if (
                    (img[path[count].y-i][path[count].x][0] == 0 and img[path[count].y-i][path[count].x][1] == 0 and img[path[count].y-i][path[count].x][2] == 0) or
                    (img[path[count].y-i][path[count].x+pathArrayWithCount[index+1][0].x][0] == 0 and img[path[count].y-i][path[count].x+pathArrayWithCount[index+1][0].x][1] == 0 and img[path[count].y-i][path[count].x+pathArrayWithCount[index+1][0].x][2] == 0)
                    ):
                    # print("-----Mo Wall Found when moving up")
                    
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))
                    break

            else:
                # print("-----Wall Found when moving up")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))

        elif pointX == 0 and pointY == 1:
            if index == 0:
                pixelY = path[count + pathArrayWithCount[index+1][1]].y - 1
                pixelX = path[count + pathArrayWithCount[index+1][1]].x - pathArrayWithCount[index+1][0].x
                if img[pixelY][pixelX][0] == 0 and img[pixelY][pixelX][1] == 0 and img[pixelY][pixelX][2] == 0:
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))
                else:
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))
                continue
            # Moving down
            for i in range(1,16):
                if (
                    (img[path[count].y+i][path[count].x][0] == 0 and img[path[count].y+i][path[count].x][1] == 0 and img[path[count].y+i][path[count].x][2] == 0) or
                    (img[path[count].y+i][path[count].x+pathArrayWithCount[index+1][0].x][0] == 0 and img[path[count].y+i][path[count].x+pathArrayWithCount[index+1][0].x][1] == 0 and img[path[count].y+i][path[count].x+pathArrayWithCount[index+1][0].x][2] == 0)
                    ):
                    # print("-----No Wall Found when moving down")
                    interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y - 25))
                    break
            else:
                # print("-----Wall Found when moving down")
                interpolationPoints.append(Point(interpolationPoints[-1].x, path[count].y + 25))
    
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
    interpolatedRobotCoordinates = [getRobotCoordinates.getRobotCoordinates(p.x, p.y) for p in interpolationPoints]

    print("Interpolated robot coordinates = ",[(p[0],p[1]) for p in interpolatedRobotCoordinates])

    savePointsCSV.savePointsInCSV(interpolatedRobotCoordinates)

    imageWithLine = img.copy()

    for i, j in zip(interpolationPoints[:-1], interpolationPoints[1:]):
        cv2.line(imageWithLine, (i.x, i.y), (j.x, j.y), (0, 255, 0), 1)
    else:
        cv2.imshow('image with line', imageWithLine)

    return path

def BFS(s, e):

    global img, h, w

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

        for i, j in zip(path[:-1], path[1:]):
            cv2.line(img, (i.x, i.y), (j.x, j.y), (255, 255, 0), 1)
        cv2.imshow("Image path of BFS", img)
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
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

while True:
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    cv2.imshow('ImageFrame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        break
    elif key == ord('q'):
        break
        
# image = cv2.imread("images/maze9.jpg", cv2.IMREAD_GRAYSCALE)
_, image = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY)
ratio = 640 / image.shape[1]
dim = (640, int(image.shape[0] * ratio))
image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
img = cv2.resize(image, dim, interpolation=cv2.INTER_LINEAR)
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