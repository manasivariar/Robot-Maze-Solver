def savePointsInCSV(points):
    with open("Robot\InterpolationPoints.csv", 'w') as f:
        for point in points:
            f.write(f'{point[0]},{point[1]}\n')