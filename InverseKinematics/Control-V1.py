import math
import numpy
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

center = {
    "x": 0,
    "y": 0,
    "z": 10,
}

cube = {
    "x": 0,
    "y": 200,
    "z": 10,
    "z-angle": None,
}

dimension = {
    "L0": 20,
    "L1": 150,
    "L2": 150,
    "L3": 100,
}

angle = {
    # Vinkel mellem Robotarm og overflade
    "V0": 90,
    # Aktuel vinkel af led
    "V1": None,
    "V2": None,
    "V3": None,
    "V4": None,
    "V5": None,
    "V6": None,
}
step1 = {
    "type": "Nema 17 planetary 51:1",
    "gear-ratio": 50 + 4397 / 4913,
    "step-angle": 1.80
}

step2 = {
    "type": "Nema 17 planetary 51:1",
    "gear-ratio": 50 + 4397 / 4913,
    "step-angle": 1.80
}

step3 = {
    "type": "Nema 17 planetary 19:1",
    "gear-ratio": 19 + 38 / 187,
    "step-angle": 1.80
}

step4 = {
    "type": "Nema 17 planetary 19:1",
    "gear-ratio": 19 + 38 / 187,
    "step-angle": 1.80
}

step5 = {
    "type": "Nema 17",
    "gear-ratio": 1,
    "step-angle": 1.80
}

step6 = {
    "type": "Nema 17",
    "gear-ratio": 1,
    "step-angle": 1.80
}

def calibrate():
    return

# Udreng offset af sidste akse til emne
# https://en.wikipedia.org/wiki/Rotation_matrix
def calculateOffset(rx_offset, ry_offset, rz_offset, x_offset, y_offset, z_offset):
    # Konverter til radianer
    rx_offset = numpy.radians(rx_offset)
    ry_offset = numpy.radians(ry_offset)
    rz_offset = numpy.radians(rz_offset)

    # Rotation om z-aksen
    rz = numpy.array([[math.cos(rz_offset), -math.sin(rz_offset), 0],
                      [math.sin(rz_offset), math.cos(rz_offset), 0],
                      [0, 0, 1]])
    # Rotation om y-aksen
    ry = numpy.array([[math.cos(ry_offset), 0, math.sin(ry_offset)],
                      [0, 1, 0],
                      [-math.sin(ry_offset), 0, math.cos(ry_offset)]])
    # Rotation om x-aksen
    rx = numpy.array([[1, 0, 0],
                      [0, math.cos(rx_offset), -math.sin(rx_offset)],
                      [0, math.sin(rx_offset), math.cos(rx_offset)]])
    # Koordinater af sidste akse, med objekt i orego
    coordinates = numpy.array([[x_offset],
                               [y_offset],
                               [z_offset]])
    # Beregn offset ved matrixmultiplikation
    R = numpy.matmul(rz, ry)
    R = numpy.matmul(R, rx)
    offset = numpy.matmul(R, coordinates)

    # Formater array til float og afrund
    offset_x = round((offset[0][0]), 6)
    offset_y = round((offset[1][0]), 6)
    offset_z = round((offset[2][0]), 6)
    offset_xyz = [offset_x, offset_y, offset_z]
    return offset_xyz

def calculatep3(offset):
    p3_x = cube["x"] + offset[0]
    p3_y = cube["y"] + offset[1]
    p3_z = cube["z"] + offset[2]
    p3 = [p3_x, p3_y, p3_z]
    return p3

def distancep1p3(p3):
    # Afstandsformel 3-dimensioner
    d = ((p3[0] - center["x"])**2 + (p3[1] - center["y"])**2 + (p3[2] - center["z"])**2)**0.5
    return d

def calculateV123(p1p3, p3):
    # vinkel i revinklet trekant set på x plan
    angle["V1"] = math.degrees(math.asin(p3[0] / p1p3))
    # Cosinus relation
    angle["V2"] = math.degrees(math.acos((dimension["L1"]**2 + p1p3**2 - dimension["L2"]**2) / (2 * dimension["L1"] * p1p3)))
    angle["V3"] = math.degrees(math.acos((dimension["L2"]**2 + dimension["L1"]**2 - p1p3**2) / (2 * dimension["L2"] * dimension["L1"])))

def calculatep2(p123offset):
    offset = calculateOffset(-(90-angle["V2"] - p123offset), 0, -angle["V1"], 0, 0, dimension["L1"])
    p2_x = offset[0] + center["x"]
    p2_y = offset[1] + center["y"]
    p2_z = offset[2] + center["z"]
    p2 = [p2_x, p2_y, p2_z]
    return p2

def calculatep123offset(p3, p1p3):
    # Vinkel mellem plan og teoretisk retvinklet trekant dannet af vinkel 1,2,3
    p123offset = math.degrees(math.asin((p3[2] - center["z"]) / p1p3))
    return p123offset

def render(p2, p3):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    X, Y, Z = [p3[0], cube["x"]], [p3[1], cube["y"]], [p3[2], cube["z"]]
    ax.plot(X, Y, Z, color="blue")
    X, Y, Z = [p2[0], center["x"]], [p2[1], center["y"]], [p2[2], center["z"]]
    ax.plot(X, Y, Z, color="red")
    X, Y, Z = [p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]]
    ax.plot(X, Y, Z, color="green")
    X, Y, Z = [-10, 10, 10, -10, -10, -10, 10, 10, 10, 10, 10, 10, -10, -10, -10, -10], [190, 190, 210, 210, 190, 190, 190, 190, 190, 210, 210, 210, 210, 210, 210, 190], [0, 0, 0, 0, 0, 20, 20, 0, 20, 20, 0, 20, 20, 0, 20, 20]
    ax.plot(X, Y, Z, linestyle='dashed', color="black")

    ax.set_aspect('equal', 'box')
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    calibrate()

    offset = calculateOffset(-45, 0, 90, 0, -dimension["L3"], 0)
    p3 = calculatep3(offset)
    p1p3 = distancep1p3(p3)
    calculateV123(p1p3, p3)
    p123offset = calculatep123offset(p3, p1p3)
    p2 = calculatep2(p123offset)
    p1p2 = ((center["x"] - p2[0])**2 + (center["y"] - p2[1])**2 + (center["z"] - p2[2])**2)**0.5
    p2p3 = ((p2[0] - p3[0])**2 + (p2[1] - p3[1])**2 + (p2[2] - p3[2])**2)**0.5
    print(p1p2)
    print(p2p3)
    render(p2, p3)