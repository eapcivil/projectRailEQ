import numpy as np


def getValues(cp, tag):
    """ Method that splits the lists that Vehicle return into smaller ones. Smaller ones multiply"""
    x = []
    y = []
    z = []
    if tag == 'position':
        for i in range(0, len(cp), 3):
            x.append(cp[i])
        for i in range(1, len(cp), 3):
            y.append(cp[i])
        for i in range(2, len(cp), 3):
            z.append(cp[i])
        return x, y, z

    if tag == 'velocities':
        for i in range(0, len(cp), 3):
            x.append(cp[i])
        for i in range(1, len(cp), 3):
            y.append(cp[i])
        for i in range(2, len(cp), 3):
            z.append(cp[i])
        return x, y, z

    if tag == 'forces':
        for i in range(0, len(cp), 3):
            x.append(cp[i])
        for i in range(1, len(cp), 3):
            y.append(cp[i])
        for i in range(2, len(cp), 3):
            z.append(cp[i])
        return x, y, z


def generalMatrixToDirection(cp_position, cp_velocities, cp_forces):
    cp_posx, cp_posy, cp_posz = getValues(cp_position, 'position')
    cp_velx, cp_vely, cp_velz = getValues(cp_velocities, 'velocities')
    cp_fx, cp_fy, cp_fz = getValues(cp_forces, 'forces')

    return cp_posx, cp_posy, cp_posz, cp_velx, cp_vely, cp_velz, cp_fx, cp_fy, cp_fz


def calculateForceVector(cp_posx, cp_posy, cp_posz, cp_fx, cp_fy, cp_fz) -> list:
    """
    Calculate the force vector

    Parameters:
    @param cp_posx: Vector with X coords of all Contact Points
    @param cp_posy: Vector with Y coords of all Contact Points
    @param cp_posz: Vector with Z coords of all Contact Points
    @param cp_fx: Vector with X component of the Force at Contact Points
    @param cp_fy: Vector with Y component of the Force at Contact Points
    @param cp_fz: Vector with Z component of the Force at Contact Points
    @return: The LoadVector(2D-array) located at the axis of the beam for every control point. Matrix's dimensions are (6 x Number Of Control Points)
    """
    fullM = np.zeros((3, len(cp_posx)))
    forceVector = np.zeros((6, len(cp_posx)))
    for i in range(len(cp_posx)):
        r = [cp_posx[i], cp_posy[i], cp_posz[i]]
        F = [cp_fx[i], cp_fy[i], cp_fz[i]]
        M = np.cross(r, F)
        for j in range(3):
            fullM[j][i] = M[j]

    for i in range(len(cp_posx)):
        for j in range(6):
            if j == 0:
                forceVector[j][i] = cp_fx[i]
            elif j == 1:
                forceVector[j][i] = cp_fy[i]
            elif j == 2:
                forceVector[j][i] = cp_fz[i]
            elif j == 3:
                forceVector[j][i] = fullM[0][i]
            elif j == 4:
                forceVector[j][i] = fullM[1][i]
            elif j == 5:
                forceVector[j][i] = fullM[2][i]
    return forceVector


def fromMomentCalculations(n1, n2, xp, load):
    """Equations to calculate node reactions from force"""
    L1 = np.sqrt(np.dot((xp - n1), (xp - n1)))
    L2 = np.sqrt(np.dot((n2 - xp), (n2 - xp)))
    L = np.sqrt(np.dot((n2 - n1), (n2 - n1)))
    xpn1 = np.subtract(xp, n1)
    n2xp = np.subtract(n2, xp)
    n2n1 = np.subtract(n2, n1)
    if np.dot((xpn1), (n2xp)) >= 0:
        Q1 = -6 * load[0] * L1 * L2 / L ** 3
        Q2 = (load[0] * L2 / L ** 2) * (L2 - 2 * L1)
        Q3 = (6 * load[0] * L1 * L2) / L ** 3
        Q4 = ((-load[0] * L1) / L ** 2) * (L1 - 2 * L2)
    else:
        Q1 = 0.0
        Q2 = 0.0
        Q3 = 0.0
        Q4 = 0.0
    return Q1, Q2, Q3, Q4


def fromForcesCalculaltions(n1, n2, xp, load):
    """Equations to calculate node reactions from moment"""
    L1 = np.sqrt(np.dot((xp - n1), (xp - n1)))
    L2 = np.sqrt(np.dot((n2 - xp), (n2 - xp)))
    L = np.sqrt(np.dot((n2 - n1), (n2 - n1)))
    xpn1 = np.subtract(xp, n1)
    n2xp = np.subtract(n2, xp)
    n2n1 = np.subtract(n2, n1)
    if np.dot((xpn1), (n2xp)) >= 0:
        Q1 = ((load[0] * L2 ** 2) / L ** 3) * (3 * L1 + L2)
        Q2 = (load[0] * L1 * L2 ** 2) / (L ** 2)
        Q3 = ((load[0] * L1 ** 2) / L ** 3) * (L1 + 3 * L2)
        Q4 = (-load[0] * L1 ** 2 * L2) / L ** 2
    else:
        Q1 = 0.0
        Q2 = 0.0
        Q3 = 0.0
        Q4 = 0.0
    return Q1, Q2, Q3, Q4


def extraForces(xp, n1, n2, Fyxp, Fzxp, Myxp, Mzxp):
    """Extra forces at element ends(i, j) due to point load and point moment"""

    for i in range(2):
        if i == 0:
            dof20i, dof60i, dof20j, dof60j = fromForcesCalculaltions(n1, n2, xp, Fyxp)
        else:
            dof30i, dof50i, dof30j, dof50j = fromForcesCalculaltions(n1, n2, xp, Fzxp)
    for i in range(2):
        if i == 0:
            dof31i, dof51i, dof31j, dof51j = fromMomentCalculations(n1, n2, xp, Myxp)
        else:
            dof21i, dof61i, dof21j, dof61j = fromMomentCalculations(n1, n2, xp, Mzxp)

    extraFyi = dof20i + dof21i
    extraFzi = dof30i - dof31i
    extraMyi = -dof50i + dof51i
    extraMzi = dof60i + dof61i

    extraFyj = dof20j + dof21j
    extraFzj = dof30j - dof31j
    extraMyj = -dof50j + dof51j
    extraMzj = dof60j + dof61j

    return extraFyi, extraFzi, extraMyi, extraMzi, extraFyj, extraFzj, extraMyj, extraMzj


def nodalForces(nodeID, nodeX, nodeY, nodeZ, cp_position, cp_velocities, cp_forces):
    """
    @param nodeID: List of the Bridge Node Labels
    @param nodeX: List of the Bridge Node x coordinate
    @param nodeY: List of the Bridge Node y coordinate
    @param nodeZ: List of the Bridge Node z coordinate
    @param cp_position: XYZ coordinates of the contact points per wheel
    @param cp_velocities: XYZ velocity components of the contact points per wheel
    @param cp_forces: XYZ force components of the contact points per wheel
    @return: A 2D array named nodalLoadArray with dimensions (len(cp_points) x 8), where every line has the following Values: (n1ID, n2ID, Fx, Fy, Fz, Mx, My, Mz)
    """
    cp_posx, cp_posy, cp_posz, cp_velx, cp_vely, cp_velz, cp_fx, cp_fy, cp_fz = generalMatrixToDirection(cp_position,
                                                                                                         cp_velocities,
                                                                                                         cp_forces)
    # forceVector = calculateForceVector(cp_posx, cp_posy, cp_posz, cp_fx, cp_fy, cp_fz)

    nodalLoadArrayi = np.zeros((len(nodeID), 7))
    nodalLoadArrayj = np.zeros((len(nodeID), 7))
    for i in range(len(cp_posx)):
        count1 = 0
        n1 = np.zeros(3)
        n2 = np.zeros(3)
        xp = np.zeros(3)
        mod = 0.0
        for ii in range(len(nodeX)):
            if cp_posx[i] > nodeX[ii]:
                mod = 1.0
                count1 += 1
            else:
                break
        if count1 == 0:
            count1 = 1
        if count1 == len(nodeX):
            count1 = len(nodeX) - 1

        n1[0] = nodeX[count1 - 1]
        n1[1] = nodeY[count1 - 1]
        n1[2] = nodeZ[count1 - 1]
        n2[0] = nodeX[count1]
        n2[1] = nodeY[count1]
        n2[2] = nodeZ[count1]

        # find projection
        cp = np.zeros(3)
        cp[0] = cp_posx[i]
        cp[1] = cp_posy[i]
        cp[2] = cp_posz[i]

        cpn1 = np.subtract(cp, n1)
        n2n1 = np.subtract(n2, n1)
        try:
            projcoef = np.inner(cpn1, n2n1) / np.inner(n2n1, n2n1)
            if (projcoef <= 1.0) and (projcoef >= 0.0):
                proj = np.subtract(cpn1, projcoef * (np.subtract(n2, n1)))
                xp = np.add(n1, projcoef * n2n1)
                mod = 1.0
            else:
                proj = np.subtract(cpn1, projcoef * (np.subtract(n2, n1)))
                mod = 0.0
        except:
            print("Force n1==n2")

        forceVector = calculateForceVector([proj[0]], [proj[1]], [proj[2]], [mod * cp_fx[i]], [mod * cp_fy[i]],
                                           [mod * cp_fz[i]])
        #       print("Forces @ Projection on beam centroid axis: \n", forceVector)

       #nodalLoadArrayi = np.zeros((len(cp_posx), 7))
       #nodalLoadArrayj = np.zeros((len(cp_posx), 7))

        xp = np.add(n1, projcoef * n2n1)

        extraFyi, extraFzi, extraMyi, extraMzi, extraFyj, extraFzj, extraMyj, extraMzj = extraForces(xp, n1, n2,
                                                                                                     forceVector[1],
                                                                                                     forceVector[2],
                                                                                                     forceVector[4],
                                                                                                     forceVector[5])

        nodalLoadArrayi[count1-1][0] = nodeID[count1 - 1]
        nodalLoadArrayi[count1-1][1] += forceVector[0][0] * (1 - projcoef)
        nodalLoadArrayi[count1-1][2] += extraFyi
        nodalLoadArrayi[count1-1][3] += extraFzi
        nodalLoadArrayi[count1-1][4] += forceVector[3][0] * (1 - projcoef)
        nodalLoadArrayi[count1-1][5] += extraMyi
        nodalLoadArrayi[count1-1][6] += extraMzi

        nodalLoadArrayj[count1][0] = nodeID[count1]
        nodalLoadArrayj[count1][1] += forceVector[0][0] * projcoef
        nodalLoadArrayj[count1][2] += extraFyj
        nodalLoadArrayj[count1][3] += extraFzj
        nodalLoadArrayj[count1][4] += forceVector[3][0] * projcoef
        nodalLoadArrayj[count1][5] += extraMyj
        nodalLoadArrayj[count1][6] += extraMzj
    #        print("Forces @  on beam i: \n", nodalLoadArrayi)
    #        print("Forces @  on beam i: \n", nodalLoadArrayj)
    #print(nodalLoadArrayi)
    return nodalLoadArrayi, nodalLoadArrayj


if __name__ == '__main__':
    nodeID = [0, 1, 2, 3]
    nodeX = [0, 5, 10, 15]
    nodeY = [0, 0, 0, 0]
    nodeZ = [0, 0, 0, 0]
    cp_position = []
    cp_velocities = []
    cp_forces = []
    nodalForces()
