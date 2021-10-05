import os

import matplotlib.pyplot as plt


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



def writeNewEqFile(filePath, eqTime, eqValues):

    file = open(filePath, "w")
    for index in range(len(eqValues)):
        file.write("{:.4f} {}\n".format(eqTime[index], eqValues[index]))
    file.close()


def main():
    eqFolderName = "/Users/KostasM/REDI ENG. Dropbox/Konstantinos Mixios/SS_KM_PN/Earthquakes/G10"
    for files in sorted(os.listdir(eqFolderName)):
        print(files)
        fileName = os.path.join(eqFolderName, files)
        eqTime, eqValues = LoadRecordTimeandValues(fileName)
        maxEqValues = max(eqValues)
        minEqValues = min(eqValues)
        if abs(maxEqValues) > abs(minEqValues):
            maxIndex = eqValues.index(max(eqValues))
            time = eqTime[maxIndex]
        else:
            minIndex = eqValues.index(min(eqValues))
            time = eqTime[minIndex]

        start_time = time - 10.0
        finish_time = time + 10.0
        if finish_time > eqTime[-1]:
            finish_time = eqTime[-1]
        dt = eqTime[1] - eqTime[0]
        newTimeVal = 0
        newEqTime = []
        newEqValues = []
        for i in range(len(eqValues)):
            if eqTime[i] >= start_time and eqTime[i] <= finish_time:
                newEqTime.append(newTimeVal)
                newEqValues.append(eqValues[i])
                newTimeVal += dt

        if newEqTime[-1] < 20.0:
            while newTimeVal <= 20.0:
                newEqTime.append(newTimeVal)
                newEqValues.append(0.0)
                newTimeVal += dt

        new_path = "/Users/KostasM/REDI ENG. Dropbox/Konstantinos Mixios/SS_KM_PN/Earthquakes/G10new"
        newEqfile = os.path.join(new_path, files)
        writeNewEqFile(newEqfile, newEqTime, newEqValues)





if __name__ == '__main__':
    main()
