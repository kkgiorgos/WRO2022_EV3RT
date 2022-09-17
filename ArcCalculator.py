from unicodedata import decimal


speedLimit = 1150
wheelCircumference = 3.14 * 6.24
chassisRadius = 16.51/2

def abs(number):
    if number >= 0:
        return  number
    else:
        return -number

def sign(number):
    if number == 0:
        return  0
    return abs(number) / number

def clamp(number, lowerBound, upperBound):
    if number > upperBound:
        return upperBound
    elif number < lowerBound:
        return lowerBound
    else:
        return number

def cmToTacho(centimeters):
    return centimeters / wheelCircumference * 360

def tachoToCm(tachoCount):
    return tachoCount / 360 * wheelCircumference

def calculateWheelDistances(center, angle):
    return (abs(2 * 3.14 * angle * ((chassisRadius * -1) - center) / wheelCircumference), abs(2 * 3.14 * angle * ((chassisRadius * 1) - center) / wheelCircumference))

def calculateArcSpeeds(velocity, distances, center, angle):
    wheelSpeeds = [0, 0]
    if distances[1] > distances[0]:
        wheelSpeeds[1] = velocity
        wheelSpeeds[0] = velocity * (distances[0] / distances[1])
    else:
        wheelSpeeds[0] = velocity
        wheelSpeeds[1] = velocity * (distances[1] / distances[0])

    if 0 <= center and center <= chassisRadius:
        if angle < 0:
            wheelSpeeds[0] *= -1
        else:
            wheelSpeeds[1] *= -1
    elif -1 * chassisRadius <= center and center <= 0:
        if angle < 0:
            wheelSpeeds[1] *= -1
        else:
            wheelSpeeds[0] *= -1
    elif angle < 0:
        wheelSpeeds[0] *= -1
        wheelSpeeds[1] *= -1

    return  wheelSpeeds

def reverseCalculate(velocity, distances, center, angle):
    wheelSpeeds = [0, 0]

    if distances[1] > distances[0]:
        wheelSpeeds[0] = velocity
        wheelSpeeds[1] = velocity * (distances[1] / distances[0])
    else:
        wheelSpeeds[1] = velocity
        wheelSpeeds[0] = velocity * (distances[0] / distances[1])

    if 0 <= center and center <= chassisRadius:
        if angle < 0:
            wheelSpeeds[0] *= -1
        else:
            wheelSpeeds[1] *= -1
    elif -1 * chassisRadius <= center and center <= 0:
        if angle < 0:
            wheelSpeeds[1] *= -1
        else:
            wheelSpeeds[0] *= -1
    elif angle < 0:
        wheelSpeeds[0] *= -1
        wheelSpeeds[1] *= -1

    return  wheelSpeeds

def main():
    minVelocity = float(input("Enter minimum desired linear velocity in cm/s (of min speed wheel): "))
    velocity = float(input("Enter desired LinearVelocity in cm/s (of max speed wheel): "))
    angle = float(input("Enter desired turn angle in degrees: "))
    center = float(input("Enter center around which we arc (cm from center of robot): "))

    velocity *= sign(angle)
    minVelocity *= sign(angle)
    angle = abs(angle)
    
    velocity = tachoToCm(clamp(cmToTacho(velocity), -speedLimit, speedLimit))
    minVelocity = tachoToCm(clamp(cmToTacho(minVelocity), -speedLimit, speedLimit))

    distances = calculateWheelDistances(center, angle)
    speeds = calculateArcSpeeds(velocity, distances, center, angle)
    minSpeeds = reverseCalculate(minVelocity, distances, center, angle)

    print(distances, speeds, minSpeeds)



if __name__ == "__main__":
    main()
