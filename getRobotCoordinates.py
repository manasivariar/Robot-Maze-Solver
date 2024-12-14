def getRobotCoordinates(cameraX, cameraY):
    ax = 0.409636
    bx = -361.017

    ay = -0.480382
    by = -175.516

    return (cameraX*ax + bx, cameraY*ay + by)