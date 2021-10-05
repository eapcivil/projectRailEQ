import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import xlwt


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def writeIrre(irregularityX, irregularityY):
    filePath = Path(__file__)
    fileName = os.path.join(filePath.parent.absolute(), "Irregularity.txt")
    with open(fileName, "w") as file2write:
        for i in range(len(irregularityX)):
            file2write.write(str(irregularityX[i]) + " " + str(irregularityY[i]) + "\n")
    file2write.close()


def readIrregularityFromFile(fileAbsPath):
    with open(fileAbsPath, "r") as irrFile:
        lines = irrFile.readlines()
        irregularityX = [float(line.split()[0]) for line in lines]
        irregularityY = [float(line.split()[1]) for line in lines]
    irrFile.close()

    return irregularityX, irregularityY


def writeExcelSheet(xxln, yyln, zzln, xxln1, yyln1, zzln1, xxrn, yyrn, zzrn, xxrn1, yyrn1, zzrn1):
    book = xlwt.Workbook(encoding="utf-8")

    sheet1 = book.add_sheet("Sheet 1")

    sheet1.write(0, 0, "xLeft - n")
    sheet1.write(0, 1, "yLeft - n")
    sheet1.write(0, 2, "zLeft - n")
    sheet1.write(0, 3, "xLeft - n+1")
    sheet1.write(0, 4, "yLeft - n+1")
    sheet1.write(0, 5, "zLeft - n+1")
    sheet1.write(0, 6, "xRight - n")
    sheet1.write(0, 7, "yRight - n")
    sheet1.write(0, 8, "zRight - n")
    sheet1.write(0, 9, "xRight - n+1")
    sheet1.write(0, 10, "yRight - n+1")
    sheet1.write(0, 11, "zRight - n+1")

    for iii in range(int(len(xxln))):
        sheet1.write(iii + 1, 0, xxln[iii])
        sheet1.write(iii + 1, 1, yyln[iii])
        sheet1.write(iii + 1, 2, zzln[iii])
        sheet1.write(iii + 1, 3, xxln1[iii])
        sheet1.write(iii + 1, 4, yyln1[iii])
        sheet1.write(iii + 1, 5, zzln1[iii])
        sheet1.write(iii + 1, 6, xxrn[iii])

        sheet1.write(iii + 1, 7, yyrn[iii])
        sheet1.write(iii + 1, 8, zzrn[iii])
        sheet1.write(iii + 1, 9, xxrn1[iii])
        sheet1.write(iii + 1, 10, yyrn1[iii])
        sheet1.write(iii + 1, 11, zzrn1[iii])

    book.save("trial.xls")


def convertSecondsToHours(time):
    seconds = time % (24 * 3600)
    hour = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
    return "%d:%02d:%02d" % (hour, minutes, seconds)


def writeOutputsT4(saveOutputFile, pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX, abutmntY,
                   eqDirection):
    with open(saveOutputFile, "w") as saveFile:
        # Write Piers
        pier_top_nodes = [6, 7, 8]
        for j in pier_top_nodes:
            for i in range(len(pierTopsDict["Node{}".format(j)])):
                if i == len(pierTopsDict["Node{}".format(j)]) - 1:
                    saveFile.write("{}\n".format(pierTopsDict["Node{}".format(j)][i]))
                else:
                    saveFile.write("{} ".format(pierTopsDict["Node{}".format(j)][i]))

            for i in range(len(pierBaseDict["Node{}".format(j + 3)])):
                if i == len(pierBaseDict["Node{}".format(j + 3)]) - 1:
                    saveFile.write("{}\n".format(pierBaseDict["Node{}".format(j + 3)][i]))
                else:
                    saveFile.write("{} ".format(pierBaseDict["Node{}".format(j + 3)][i]))

        # Write Bearings
        bearingTopNodes = [136, 138, 140, 142, 144, 146, 148, 150, 152, 154]
        for j in bearingTopNodes:
            for i in range(len(bearingTop["Node{}".format(j)])):
                if i == len(bearingTop["Node{}".format(j)]) - 1:
                    saveFile.write("{}\n".format(bearingTop["Node{}".format(j)][i]))
                else:
                    saveFile.write("{} ".format(bearingTop["Node{}".format(j)][i]))

            for i in range(len(bearingBase["Node{}".format(j + 1)])):
                if i == len(bearingBase["Node{}".format(j + 1)]) - 1:
                    saveFile.write("{}\n".format(bearingBase["Node{}".format(j + 1)][i]))
                else:
                    saveFile.write("{} ".format(bearingBase["Node{}".format(j + 1)][i]))

        # Write Abutments
        if eqDirection == 1:
            dictkeys = abutmntX.keys()
            for i in dictkeys:
                for j in range(len(abutmntX["{}".format(str(i))])):
                    if j == len(abutmntX["{}".format(str(i))]) - 1:
                        saveFile.write("{}\n".format(abutmntX["{}".format(str(i))][j]))
                    else:
                        saveFile.write("{} ".format(abutmntX["{}".format(str(i))][j]))
        else:
            dictkeys = abutmntY.keys()
            for i in dictkeys:
                for j in range(len(abutmntY["{}".format(str(i))])):
                    if j == len(abutmntY["{}".format(str(i))]) - 1:
                        saveFile.write("{}\n".format(abutmntY["{}".format(str(i))][j]))
                    else:
                        saveFile.write("{} ".format(abutmntY["{}".format(str(i))][j]))
    saveFile.close()


def writeOutputsT4wForces(saveOutputFile, pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX, abutmntY,
                          forces, vehicleAccelerations, bogieAccelerations,
                          eqDirection):
    with open(saveOutputFile, "w") as saveFile:
        # Write Piers
        pier_top_nodes = [6, 7, 8]
        for j in pier_top_nodes:
            for i in range(len(pierTopsDict["Node{}".format(j)])):
                if i == len(pierTopsDict["Node{}".format(j)]) - 1:
                    saveFile.write("{}\n".format(pierTopsDict["Node{}".format(j)][i]))
                else:
                    saveFile.write("{} ".format(pierTopsDict["Node{}".format(j)][i]))

            for i in range(len(pierBaseDict["Node{}".format(j + 3)])):
                if i == len(pierBaseDict["Node{}".format(j + 3)]) - 1:
                    saveFile.write("{}\n".format(pierBaseDict["Node{}".format(j + 3)][i]))
                else:
                    saveFile.write("{} ".format(pierBaseDict["Node{}".format(j + 3)][i]))

        # Write Bearings
        bearingTopNodes = [136, 138, 140, 142, 144, 146, 148, 150, 152, 154]
        for j in bearingTopNodes:
            for i in range(len(bearingTop["Node{}".format(j)])):
                if i == len(bearingTop["Node{}".format(j)]) - 1:
                    saveFile.write("{}\n".format(bearingTop["Node{}".format(j)][i]))
                else:
                    saveFile.write("{} ".format(bearingTop["Node{}".format(j)][i]))

            for i in range(len(bearingBase["Node{}".format(j + 1)])):
                if i == len(bearingBase["Node{}".format(j + 1)]) - 1:
                    saveFile.write("{}\n".format(bearingBase["Node{}".format(j + 1)][i]))
                else:
                    saveFile.write("{} ".format(bearingBase["Node{}".format(j + 1)][i]))

        # Write Abutments
        if eqDirection == 1:
            dictkeys = abutmntX.keys()
            for i in dictkeys:
                for j in range(len(abutmntX["{}".format(str(i))])):
                    if j == len(abutmntX["{}".format(str(i))]) - 1:
                        saveFile.write("{}\n".format(abutmntX["{}".format(str(i))][j]))
                    else:
                        saveFile.write("{} ".format(abutmntX["{}".format(str(i))][j]))
        else:
            dictkeys = abutmntY.keys()
            for i in dictkeys:
                for j in range(len(abutmntY["{}".format(str(i))])):
                    if j == len(abutmntY["{}".format(str(i))]) - 1:
                        saveFile.write("{}\n".format(abutmntY["{}".format(str(i))][j]))
                    else:
                        saveFile.write("{} ".format(abutmntY["{}".format(str(i))][j]))

        # Write Forces
        for i in range(len(forces)):
            if i == len(forces) - 1:
                saveFile.write("{}\n".format(forces[i]))
            else:
                saveFile.write("{} ".format(forces[i]))

        # Write vehicle Accelerations

        for i in range(len(vehicleAccelerations)):
            if i == len(vehicleAccelerations) - 1:
                saveFile.write("{}\n".format(vehicleAccelerations[i]))
            else:
                saveFile.write("{} ".format(vehicleAccelerations[i]))
        # Write Bogie Accelerations

        for i in range(len(bogieAccelerations)):
            if i == len(bogieAccelerations) - 1:
                saveFile.write("{}\n".format(bogieAccelerations[i]))
            else:
                saveFile.write("{} ".format(bogieAccelerations[i]))
    saveFile.close()


def writeExtraOutputsT45wForces(saveOutputFile, verticalAccel, R):
    newsaveOutputFile = os.path.join("extra_", saveOutputFile)
    with open(newsaveOutputFile, "w") as saveFile:
        for i in range(len(verticalAccel)):
            if i == len(verticalAccel) - 1:
                saveFile.write("{}\n".format(verticalAccel[i]))
            else:
                saveFile.write("{} ".format(verticalAccel[i]))

        for i in range(len(R)):
            if i == len(R) - 1:
                saveFile.write("{}\n".format(R[i]))
            else:
                saveFile.write("{} ".format(R[i]))
    saveFile.close()


def irregularity(tag, roughness, L, v, plots=False):
    """
    @param tag: can be only 1 or 2. 1 corresponds to German regulations and 2 corresponds to USA regulations
    @param roughness:   for tag = 1 roughness values are:
                                                        0 no roughness
                                                        1 very good roughness
                                                        2 good roughness
                        for tag = 2 roughness values are:
                                                        0 no roughness
                                                        1 level 1 roughness
                                                        2 level 2 roughness - very poor
                                                        3 level 3 roughness
                                                        4 level 4 roughness
                                                        5 level 5 roughness
                                                        6 level 6 roughness - very good
    @param L: Rail Length
    @param v: Vehicle Velocity
    @param plots: Plot the Irregularity
    @return: Vertical displacement irregularity profile (m)
    """
    VALID_TAG = {1, 2}
    VALID_ROUGH1 = {0, 1, 2}
    VALID_ROUGH2 = {0, 1, 2, 3, 4, 5, 6}
    if tag not in VALID_TAG:
        raise ValueError("tag instance must be one of {}.".format(VALID_TAG))
    if tag == 1:
        if roughness not in VALID_ROUGH1:
            raise ValueError("roughness instance for tag {} must be one of {}.".format(tag, VALID_ROUGH1))
    elif tag == 2:
        if roughness not in VALID_ROUGH2:
            raise ValueError("roughness instance for tag {} must be one of {}.".format(tag, VALID_ROUGH2))

    if tag == 1:
        nl = 2 * np.pi / 80  # rad/m, 0.5m
        nh = 2 * np.pi / 0.5  # rad/m, 80m
        if roughness == 0:
            Av = 0
            wr = 0.0206
            wc = 0.8246
        elif roughness == 1:
            Av = 4.032e-7
            wr = 0.0206
            wc = 0.8246
        elif roughness == 2:
            Av = 10.80e-7
            wr = 0.0206
            wc = 0.8246

    elif tag == 2:
        nl = 2 * np.pi / 300  # rad/m, 1.5m
        nh = 2 * np.pi / 1.5  # rad/m, 300m
        if roughness == 0:
            Av = 0
            wc = 0.8245
        elif roughness == 1:
            Av = 1.2107e-4
            wc = 0.8245
        elif roughness == 2:
            Av = 1.0181e-4
            wc = 0.8245
        elif roughness == 3:
            Av = 0.6816e-4
            wc = 0.8245
        elif roughness == 4:
            Av = 0.5376e-4
            wc = 0.8245
        elif roughness == 5:
            Av = 0.2095e-4
            wc = 0.8245
        elif roughness == 6:
            Av = 0.0339e-4
            wc = 0.8245

    N = 2048
    deta_f = (nh - nl) / N
    nk = np.linspace((nl + (nh - nl) / (2 * N)), (nh - (nh - nl) / (2 * N)), num=N)
    faik = []

    for i in range(N):
        faik.append(2 * np.pi * np.random.random())
    ak2 = np.zeros(N)

    for i in range(1, N):
        wn = nk[i]
        if tag == 1:
            Sn = ((Av * wc ** 2) / (wn ** 2 + wr ** 2) / (wn ** 2 + wc ** 2))
        elif tag == 2:
            Sn = ((0.25 * Av * wc ** 2) / (wn ** 2) / (wn ** 2 + wc ** 2))
        ak2[i] = np.sqrt(2 * deta_f * Sn)

    step = 1 / (nh / 2 / np.pi) / v / 2
    number_of_spaces = int(L / v / step)
    t = []
    for i in np.linspace(0, L / v, num=number_of_spaces):
        t.append(i)
    Lt = len(t)

    qkdd = np.zeros((N, Lt))
    for i in range(N):
        for j in range(Lt):
            qkdd[i, j] = -(nk[i]) ** 2 * ak2[i] * np.cos(nk[i] * v * t[j] + faik[i])
    qtdd = np.sum(qkdd, axis=0).tolist()

    qkd = np.zeros((N, Lt))
    for i in range(N):
        for j in range(Lt):
            qkd[i, j] = -(nk[i]) * ak2[i] * np.sin(nk[i] * v * t[j] + faik[i])
    qtd = np.sum(qkd, axis=0).tolist()

    qk = np.zeros((N, Lt))
    for i in range(N):
        for j in range(Lt):
            qk[i, j] = ak2[i] * np.cos(nk[i] * v * t[j] + faik[i])
    qt = np.sum(qk, axis=0).tolist()
    v_t = [i * v for i in t]  # generate discrete positioning

    ax = plt.subplot()
    if tag == 1:
        tag = "German"
    else:
        tag = "USA"

    if plots:
        line1 = ax.plot(v_t, qt, linewidth=0.5, label='Irregularity {}, roughness level {}'.format(tag, roughness))
        plt.xlabel("Length (m)")
        plt.ylabel("Vertical Irregularity (m)")
        ax.legend()
        plt.show()
    return v_t, qt


def createEQstartsV2(eqFilePath):
    """
    @param eqFilePath: Path of Earthquake file
    @return: Return the value of the time when the maximum acceleration occurs. 3 seconds subtracted.
    """

    with open(eqFilePath, "r") as eqfile:
        lines = eqfile.readlines()
        time = [float(line.split()[0]) for line in lines]
        eqValues = [float(line.split()[1]) for line in lines]
    index_max = max(range(len(eqValues)), key=eqValues.__getitem__)
    index_min = min(range(len(eqValues)), key=eqValues.__getitem__)
    if abs(eqValues[index_max]) < abs(eqValues[index_min]):
        index = index_min
    else:
        index = index_max

    if time[index] - 3.0 <= 0.0:
        startTime = 0.0
    else:
        startTime = time[index] - 3.0

    return startTime


def getSplineExtension(vehicleSpeed, eqTime, eqValues):
    index_max = max(range(len(eqValues)), key=eqValues.__getitem__)
    index_min = min(range(len(eqValues)), key=eqValues.__getitem__)
    if abs(eqValues[index_max]) < abs(eqValues[index_min]):
        index = index_min
    else:
        index = index_max
    splineExtension = eqTime[index] * vehicleSpeed
    return splineExtension


def LoadRecordTimeandValues(filename, plot=False):
    with open('{}'.format(filename), 'r') as file:
        lines = file.readlines()
        eq_time = [float(line.split()[0]) for line in lines]
        eq_values = [float(line.split()[1]) for line in lines]

    if plot:
        plt.plot(eq_time, eq_values, label="Earhquake Timeseries")
        plt.xlabel("Time (sec)")
        plt.ylabel("Acceleration (*9.81 m/sec^2)")
        plt.title("Earthquake {}".format(filename))
        plt.legend()
        plt.show()

    return eq_time, eq_values


def test_irregularity():
    irregularity(1, 2, 200, 4)


def tests():
    test_irregularity()


if __name__ == '__main__':
    tests()
