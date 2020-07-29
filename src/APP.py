import numpy as np

firstDegrees = 50
currentDistance = 100
InFOV = 30
targetDistance = 0
distanceInBetween = 0


while currentDistance > targetDistance:
    if currentDistance == InFOV:
        visionSensorDistance = 60
        distanceInBetween = currentDistance/visionSensorDistance
        
    currentDistance-=1
