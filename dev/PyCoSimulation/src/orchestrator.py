import numpy as np
from scipy import interpolate
from Vehicle_dll import FixedVehicleDll
from utilities import *
from pathlib import Path
from Bridge import Bridge
import os
import time
import shutil
import sys
import ctypes as ct
from _ctypes import FreeLibrary as FreeLibrary


class Orch(object):
    def __init__(self, dllDir, xmlDir,spline_extension, CosimSteps, bridgeLenght, eqTime, eqValues, irregularityX,
                 irregularityY, Ec,
                 gap, stiffnessFactor, phiybz, Mybz, phiubz, Mubz, phiyby, Myby, phiuby, Μυby, eqStartTime,
                 eqDirection, vehicle_speed):

        self.dllDir = dllDir
        self.xmlDir = xmlDir
        self.spline_extension = spline_extension
        self.num_of_spline_extension = 20
        self.CosimSteps = CosimSteps
        self.bridgeLength = bridgeLenght
        self.eqTime = eqTime
        self.eqValues = eqValues
        self.irregularityX = irregularityX
        self.irregularityY = irregularityY
        self.eqStartTime = eqStartTime
        self.eqDirection = eqDirection
        self.vehicle_speed = vehicle_speed

        self.Ec = Ec
        self.gap = gap
        self.stiffnessFactor = stiffnessFactor
        self.phiybz = phiybz
        self.Mybz = Mybz
        self.phiubz = phiubz
        self.Mubz = Mubz
        self.phiyby = phiyby
        self.Myby = Myby
        self.phiuby = phiuby
        self.Muby = Μυby

        self.x_ext = []
        for i in range(self.num_of_spline_extension):
            self.x_ext.append(
                (i - self.num_of_spline_extension) * (self.spline_extension / self.num_of_spline_extension))
        self.x_ext1 = []
        for i in range(self.num_of_spline_extension):
            self.x_ext1.append(self.bridgeLength + (i + 1) *
                               (self.spline_extension / self.num_of_spline_extension))

    def time(self):
        self.globalTime = 0.0
        self.dtCosim    = 0.1
        self.dtBridge   = 0.025
        self.dtBoogie   = 0.0025

    def update(self, bridge):
        self.pierTopsDict, self.pierBaseDict, self.bearingTop, self.bearingBase, \
        self.abutmntX, self.abutmntY = bridge.update(self.eqDirection, self.pierTopsDict, self.pierBaseDict,
                                                     self.bearingTop, self.bearingBase,
                                                     self.abutmntX, self.abutmntY)

    def solution(self):
        x_new = [i - 3000 for i in self.irregularityX]
        f = interpolate.interp1d(x_new, self.irregularityY)
        ncp = 120
        cp_positions = np.zeros(ncp)
        cp_velocities = np.zeros(ncp)
        cp_forces = np.zeros(ncp)

        cp_positions1 = np.zeros(ncp)
        cp_velocities1 = np.zeros(ncp)
        cp_forces1 = np.zeros(ncp)

        cp_positions10 = np.zeros(ncp)
        cp_velocities10 = np.zeros(ncp)
        cp_forces10 = np.zeros(ncp)

        # Remains as it is
        x_v_l_n = []
        y_v_l_n = []
        z_v_l_n = []
        x_v_l_n1 = []
        y_v_l_n1 = []
        z_v_l_n1 = []
        x_v_r_n = []
        y_v_r_n = []
        z_v_r_n = []
        x_v_r_n1 = []
        y_v_r_n1 = []
        z_v_r_n1 = []

        for i in range(44):
            x_v_l_n.append(0.0)
            y_v_l_n.append(0.0)
            z_v_l_n.append(0.0)
            x_v_l_n1.append(0.0)
            y_v_l_n1.append(0.0)
            z_v_l_n1.append(0.0)
            x_v_r_n.append(0.0)
            y_v_r_n.append(0.0)
            z_v_r_n.append(0.0)
            x_v_r_n1.append(0.0)
            y_v_r_n1.append(0.0)
            z_v_r_n1.append(0.0)

        tol_in = 0.01
        tol_out = 1
        tol_dphi = 1000000
        tol_phi = 0.01
        nn = 10.0
        krr_explicit = 1.0
        crr_explicit = 1.048
        xml0_path =  self.xmlDir+"\\veh_0.xml"
        xml1_path =  self.xmlDir+"\\veh_1.xml"
        xml2_path =  self.xmlDir+"\\veh_2.xml"
        vx = self.vehicle_speed
        iiter = 0
        convergence = 100
        tol = 0.01
        tag = 0
        # vehicle
        cosim_converge = False
        first_time = True
        time = []
        force = []
        # bridge lists for outputs)
        self.pierTopsDict = {"Node6": [],
                             "Node7": [],
                             "Node8": []}

        self.pierBaseDict = {"Node9": [],
                             "Node10": [],
                             "Node11": []}

        self.bearingTop = {"Node136": [],
                           "Node138": [],
                           "Node140": [],
                           "Node142": [],
                           "Node144": [],
                           "Node146": [],
                           "Node148": [],
                           "Node150": [],
                           "Node152": [],
                           "Node154": []}

        self.bearingBase = {"Node137": [],
                            "Node139": [],
                            "Node141": [],
                            "Node143": [],
                            "Node145": [],
                            "Node147": [],
                            "Node149": [],
                            "Node151": [],
                            "Node153": [],
                            "Node155": []}

        self.abutmntX = {"Node156": [],
                         "Node157": [],
                         "Node162": [],
                         "Node163": [],
                         "Node1560": [],
                         "Node1620": []}

        self.abutmntY = {"Node182": [],
                         "Node182000": [],
                         "Node183": [],
                         "Node183000": [],
                         "Node184": [],
                         "Node184000": [],
                         "Node185": [],
                         "Node185000": []}

        bridgeTag = 0
        vehicleTag = True
        vehicleTag1 = 0
        count = 0  # REMOVE IT
        forces = []
        vehicleAccelerations = []
        bogieAccelerations = []
        verticalAccelControlPoints = []
        r = []
        # load dll
        vehicleLoaddll = ct.CDLL(self.dllDir)
        handle = vehicleLoaddll._handle
        bridgeSubSteps= int(self.dtCosim/self.dtBridge)

        for istep in range(self.CosimSteps):
            try:
                while iiter < 1 and convergence > tol:
                    bridge = Bridge(self.Ec, self.gap, self.stiffnessFactor, self.phiybz, self.Mybz, self.phiubz,
                                      self.Muby, self.phiyby, self.Myby,
                                      self.phiuby, self.Muby,
                                      self.dtBridge, self.globalTime, self.eqTime, self.eqValues, self.eqDirection)
                    bridge.initialize_model()
                    bridge.construct_model()
                    if bridgeTag == 0:
                        bridge.solve_gravity()
                        bridgeTag = 1
                    bridge.restore()
                    bridge.add_nodalLoad(0, cp_positions, cp_velocities, cp_forces,bridgeSubSteps)
                    bridge.add_nodalLoad(1, cp_positions1, cp_velocities1, cp_forces1,bridgeSubSteps)
                    x_p_l_n, y_p_l_n, z_p_l_n_woIr, x_p_l_n1, y_p_l_n1, z_p_l_n1_woIr, x_p_r_n, y_p_r_n, z_p_r_n_woIr, x_p_r_n1, y_p_r_n1, z_p_r_n1_woIr, \
                    xx_v_l_n, yy_v_l_n, zz_v_l_n, xx_v_l_n1, yy_v_l_n1, zz_v_l_n1, xx_v_r_n, yy_v_r_n, zz_v_r_n, xx_v_r_n1, yy_v_r_n1, zz_v_r_n1, verticalAccel, r2return = bridge.solve_transient()
                    # only for explicit------------
                    cp_positions = cp_positions1
                    cp_velocities = cp_velocities1
                    cp_forces = cp_forces1
                    # ------------------------------
                    vel1 = np.zeros(self.num_of_spline_extension).tolist()
                    vel2 = np.zeros(self.num_of_spline_extension).tolist()
                    x_v_l_n = vel1 + xx_v_l_n + vel2
                    y_v_l_n = vel1 + yy_v_l_n + vel2
                    z_v_l_n = vel1 + zz_v_l_n + vel2
                    x_v_l_n1 = vel1 + xx_v_l_n1 + vel2
                    y_v_l_n1 = vel1 + yy_v_l_n1 + vel2
                    z_v_l_n1 = vel1 + zz_v_l_n1 + vel2
                    x_v_r_n = vel1 + xx_v_r_n + vel2
                    y_v_r_n = vel1 + yy_v_r_n + vel2
                    z_v_r_n = vel1 + zz_v_r_n + vel2
                    x_v_r_n1 = vel1 + xx_v_r_n1 + vel2
                    y_v_r_n1 = vel1 + yy_v_r_n1 + vel2
                    z_v_r_n1 = vel1 + zz_v_r_n1 + vel2

                    z_irreg = f(x_p_l_n)
                    y_begin_l = y_p_l_n[0]
                    y_begin_l1 = y_p_l_n1[0]
                    y_begin_r = y_p_r_n[0]
                    y_begin_r1 = y_p_r_n1[0]
                    y_last_l = y_p_l_n[-1]
                    y_last_l1 = y_p_l_n1[-1]
                    y_last_r = y_p_r_n[-1]
                    y_last_r1 = y_p_r_n1[-1]
                    z_begin_l = z_p_l_n_woIr[0]
                    z_begin_l1 = z_p_l_n1_woIr[0]
                    z_begin_r = z_p_r_n_woIr[0]
                    z_begin_r1 = z_p_r_n1_woIr[0]
                    z_last_l = z_p_l_n_woIr[-1]
                    z_last_l1 = z_p_l_n1_woIr[-1]
                    z_last_r = z_p_r_n_woIr[-1]
                    z_last_r1 = z_p_r_n1_woIr[-1]
                    y_ext_l = np.zeros(self.num_of_spline_extension)
                    y_ext_l1 = np.zeros(self.num_of_spline_extension)
                    y_ext_r = np.zeros(self.num_of_spline_extension)
                    y_ext_r1 = np.zeros(self.num_of_spline_extension)
                    z_ext_l = np.zeros(self.num_of_spline_extension)
                    z_ext_l1 = np.zeros(self.num_of_spline_extension)
                    z_ext_r = np.zeros(self.num_of_spline_extension)
                    z_ext_r1 = np.zeros(self.num_of_spline_extension)
                    y_ext1_l = np.zeros(self.num_of_spline_extension)
                    y_ext1_l1 = np.zeros(self.num_of_spline_extension)
                    y_ext1_r = np.zeros(self.num_of_spline_extension)
                    y_ext1_r1 = np.zeros(self.num_of_spline_extension)
                    z_ext1_l = np.zeros(self.num_of_spline_extension)
                    z_ext1_l1 = np.zeros(self.num_of_spline_extension)
                    z_ext1_r = np.zeros(self.num_of_spline_extension)
                    z_ext1_r1 = np.zeros(self.num_of_spline_extension)
                    for i in range(self.num_of_spline_extension):
                        y_ext_l[i] = y_begin_l
                        y_ext_l1[i] = y_begin_l1
                        z_ext_l[i] = z_begin_l + f(self.x_ext[i]) * 0.0
                        z_ext_l1[i] = z_begin_l1 + f(self.x_ext[i]) * 0.0
                        y_ext_r[i] = y_begin_r
                        y_ext_r1[i] = y_begin_r1
                        z_ext_r[i] = z_begin_r + f(self.x_ext[i]) * 0.0
                        z_ext_r1[i] = z_begin_r1 + f(self.x_ext[i]) * 0.0
                        y_ext1_l[i] = y_last_l
                        y_ext1_l1[i] = y_last_l1
                        z_ext1_l[i] = z_last_l + f(self.x_ext[i])
                        z_ext1_l1[i] = z_last_l1 + f(self.x_ext[i])
                        y_ext1_r[i] = y_last_r
                        y_ext1_r1[i] = y_last_r1
                        z_ext1_r[i] = z_last_r + f(self.x_ext[i])
                        z_ext1_r1[i] = z_last_r1 + f(self.x_ext[i])
                    z_p_l_n = [values + z_irreg[i]
                               for i, values in enumerate(z_p_l_n_woIr)]
                    z_p_l_n1 = [values + z_irreg[i]
                                for i, values in enumerate(z_p_l_n1_woIr)]
                    z_p_r_n = [values + z_irreg[i]
                               for i, values in enumerate(z_p_r_n_woIr)]
                    z_p_r_n1 = [values + z_irreg[i]
                                for i, values in enumerate(z_p_r_n1_woIr)]

                    t0 = self.globalTime
                    t1 = t0 + self.dtCosim
                    nintervals = self.dtCosim / self.dtBoogie
                    res = [i * (-1) for i in cp_forces1]
                    count += 1  # To choose in dt to insert the train
                    if vehicleTag:  # if self.eqStartTime <= t0:
                        if vehicleTag1 == 0:
                            self.x_o_vehicle = -(self.spline_extension - 20)
                            self.y_o_vehicle = (y_begin_l + y_begin_r) / 2
                            self.z_o_vehicle = 0.5 + (z_begin_l + z_begin_r) / 2
                            vehicleTag1 = 1

                        xln = (self.x_ext + x_p_l_n + self.x_ext1)
                        xxln = [i - self.x_o_vehicle for i in xln]
                        yln = (y_ext_l.tolist() + y_p_l_n + y_ext1_l.tolist())
                        yyln = [i - self.y_o_vehicle for i in yln]
                        zln = (z_ext_l.tolist() + z_p_l_n + z_ext1_l.tolist())
                        zzln = [i - self.z_o_vehicle for i in zln]
                        xln1 = (self.x_ext + x_p_l_n1 + self.x_ext1)
                        xxln1 = [i - self.x_o_vehicle for i in xln1]
                        yln1 = (y_ext_l1.tolist() + y_p_l_n1 + y_ext1_l1.tolist())
                        yyln1 = [i - self.y_o_vehicle for i in yln1]
                        zln1 = (z_ext_l1.tolist() + z_p_l_n1 + z_ext1_l1.tolist())
                        zzln1 = [i - self.z_o_vehicle for i in zln1]
                        xrn = (self.x_ext + x_p_r_n + self.x_ext1)
                        xxrn = [i - self.x_o_vehicle for i in xrn]
                        yrn = (y_ext_r.tolist() + y_p_r_n + y_ext1_r.tolist())
                        yyrn = [i - self.y_o_vehicle for i in yrn]
                        zrn = (z_ext_r.tolist() + z_p_r_n + z_ext1_r.tolist())
                        zzrn = [i - self.z_o_vehicle for i in zrn]
                        xrn1 = (self.x_ext + x_p_r_n1 + self.x_ext1)
                        xxrn1 = [i - self.x_o_vehicle for i in xrn1]
                        yrn1 = (y_ext_r1.tolist() + y_p_r_n1 + y_ext1_r1.tolist())
                        yyrn1 = [i - self.y_o_vehicle for i in yrn1]
                        zrn1 = (z_ext_r1.tolist() + z_p_r_n1 + z_ext1_r1.tolist())
                        zzrn1 = [i - self.z_o_vehicle for i in zrn1]

                        t00 = 0.0
                        vehicle = FixedVehicleDll(vehicleLoaddll, cosim_converge, first_time, t00, self.dtCosim,
                                                  nintervals,
                                                  tol_in,
                                                  tol_out,
                                                  tol_dphi,
                                                  tol_phi, nn, krr_explicit, crr_explicit,
                                                  xml0_path, xml1_path, xml2_path, int(
                                len(xxln)), vx,
                                                  xxln, yyln, zzln,
                                                  xxln1, yyln1, zzln1,
                                                  xxrn, yyrn, zzrn,
                                                  xxrn1, yyrn1, zzrn1,
                                                  x_v_l_n, y_v_l_n, z_v_l_n,
                                                  x_v_l_n1, y_v_l_n1, z_v_l_n1,
                                                  x_v_r_n, y_v_r_n, z_v_r_n,
                                                  x_v_r_n1, y_v_r_n1, z_v_r_n1, ncp)
                        cosim_converge = False
                        cp_positions10, cp_velocities10, cp_forces10, vehicle_accelerations, bogie_accelerations, bogie_yaws = vehicle.solve()
                        cp_forces10 = [values / 1000.0 for i,
                                                           values in enumerate(cp_forces10)]
                        del vehicle
                        if np.linalg.norm(cp_positions10) > 0.01:
                            cp_positions1 = [values for i,
                                                        values in enumerate(cp_positions10)]
                            cp_velocities1 = [values for i,
                                                         values in enumerate(cp_velocities10)]
                            cp_forces1 = [values for i,
                                                     values in enumerate(cp_forces10)]
                            for i in range(int(len(cp_positions1) / 3)):
                                cp_positions1[3 * i] += self.x_o_vehicle
                                cp_positions1[3 * i + 1] += self.y_o_vehicle
                                cp_positions1[3 * i + 2] += self.z_o_vehicle

                            res = [res[i] + values for i,
                                                       values in enumerate(cp_forces1)]

                        try:
                            convergence = np.linalg.norm(
                                res) / np.linalg.norm(cp_forces1)
                        except:
                            convergence = 100
                            cp_positions1 = [values for i,
                                                        values in enumerate(cp_positions)]
                            cp_velocities1 = [values for i,
                                                         values in enumerate(cp_velocities)]
                            cp_forces1 = [values for i,
                                                     values in enumerate(cp_forces)]
                        iiter += 1
                    else:
                        cp_positions = np.zeros(ncp)
                        cp_velocities = np.zeros(ncp)
                        cp_forces = np.zeros(ncp)
                        vehicle_accelerations = np.zeros(ncp)
                        convergence = tol / 10.0
            except:
                print("analysis failed")
                break
            cp_positions1 = [values for i, values in enumerate(cp_positions1)]
            cp_velocities1 = [values for i, values in enumerate(cp_velocities1)]
            cp_forces1 = [values for i, values in enumerate(cp_forces1)]


            if cp_positions1[0] >= self.bridgeLength:
                vehicleTag = False
                cp_positions1 = [0.0 * values for i, values in enumerate(cp_positions1)]
                cp_velocities1 = [0.0 * values for i, values in enumerate(cp_velocities1)]
                cp_forces1 = [0.0 * values for i, values in enumerate(cp_forces1)]
            self.globalTime = self.globalTime + self.dtCosim
            print('Time: {}, iter: {}'.format(self.globalTime, iiter))
            first_time = False
            cosim_converge = True
            iiter = 0
            time.append(self.globalTime)

            self.update(bridge)
            forces.append(cp_forces10)
            vehicleAccelerations.append(list(vehicle_accelerations))
            bogieAccelerations.append(list(bogie_accelerations))
            verticalAccelControlPoints.append(verticalAccel)
            r.append(r2return)
            convergence = 100
        # unload dll
        del vehicleLoaddll
        FreeLibrary(handle)
        del handle
        # return
        return self.pierTopsDict, self.pierBaseDict, self.bearingTop, self.bearingBase, \
               self.abutmntX, self.abutmntY, forces, vehicleAccelerations, bogieAccelerations, verticalAccelControlPoints, r


def multiple_analysis(argv):
    start_time = time.time()
    eqDirection = argv[3]  # argv[1]
    orchPath = Path(__file__)
    srcFolderPath = orchPath.parent.absolute()
    projectPath = orchPath.parent.parent.absolute()
    PyCosimPath = srcFolderPath.parent.absolute()
    ApiPath=orchPath.parent.parent.parent
    eqPath = os.path.join(projectPath, "Earthquakes")
    dllDir = os.path.join(ApiPath, "api_vehicle\\Release\\vehicle.dll")
    xmlDir = os.path.join(ApiPath, "api_vehicle\\vehicle_xmls")



    if os.path.exists(os.path.join(PyCosimPath, "OpenSees_DB")):
        shutil.rmtree(os.path.join(PyCosimPath, "OpenSees_DB"))

    numberofCarbodies=5
    vehicleLength = 26.0 * numberofCarbodies
    spline_extension = 3000
    CosimSteps = 200
    bridgeLength = 168.0
    vehicleSpeed = 55.0
    # irregularityX, irregularityY = irregularity(1, 1, bridgeLength + 2 * 3000, vehicleSpeed)
    # writeIrre(irregularityX, irregularityY)
    irregularityX, irregularityY = readIrregularityFromFile(
        os.path.join(srcFolderPath, "Irregularity.txt"))
    parametersFile = os.path.join(
        Path(__file__).parent.absolute(), "Parameters.txt")
    with open(parametersFile, "r") as parameterFile:
        lines = parameterFile.readlines()
        Ec = [float(line.split()[0]) for line in lines]
        gap = [float(line.split()[1]) for line in lines]
        stiffnessFactor = [float(line.split()[2]) for line in lines]
        phiybz = [float(line.split()[3]) for line in lines]
        Mybz = [float(line.split()[4]) for line in lines]
        phiubz = [float(line.split()[5]) for line in lines]
        Mubz = [float(line.split()[6]) for line in lines]
        phiyby = [float(line.split()[7]) for line in lines]
        Myby = [float(line.split()[8]) for line in lines]
        phiuby = [float(line.split()[9]) for line in lines]
        Muby = [float(line.split()[10]) for line in lines]
        eqFolderName = [int(line.split()[11]) for line in lines]

    if not os.path.exists(os.path.join(projectPath, "Results")):
        resultDir = os.path.join(projectPath, "Results")
        os.makedirs(resultDir)
    else:
        resultDir = os.path.join(projectPath, "Results")

    if int(eqDirection) == 1:
        resultDirPerDirection = os.path.join(resultDir, "x")
        if not os.path.exists(os.path.join(resultDir, "x")):
            os.makedirs(resultDirPerDirection)
    else:
        resultDirPerDirection = os.path.join(resultDir, "y")
        if not os.path.exists(os.path.join(resultDir, "y")):
            os.makedirs(resultDirPerDirection)

    for indx, folderName in enumerate(eqFolderName):
        eqFiles = os.path.join(eqPath, str(folderName))
        case_indx = 0
        for files in os.listdir(eqFiles):
            case_indx += 1
            fileName = os.path.join(eqFiles, files)
            eqStartTime = 0  # eqStartTime = createEQstartsV2(fileName)
            eqTime, eqValues = LoadRecordTimeandValues(fileName)
            # CosimSteps = int(eqTime[-1] / 0.2)
            spline_extension = getSplineExtension(
                vehicleSpeed, eqTime, eqValues) + vehicleLength + 20
            orch = Orch(dllDir, xmlDir,spline_extension, CosimSteps, bridgeLength, eqTime, eqValues, irregularityX,
                        irregularityY,
                        Ec[indx],
                        gap[indx], stiffnessFactor[indx], phiybz[indx], Mybz[indx], phiubz[indx], Mubz[indx],
                        phiyby[indx], Myby[indx], phiuby[indx], Muby[indx], eqStartTime, eqDirection, vehicleSpeed)
            orch.time()
            vehicleAccel = []
            bogieAccel = []
            pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX, abutmntY, forces, vehicleAccelerations, bogieAccelerations, verticalAccelControlPoints, r = orch.solution()
            forceZonly = []
            for i in forces:
                for j in range(2, len(i), 3):
                    forceZonly.append(i[j])
            for i in range(len(vehicleAccelerations)):
                for j in range(len(vehicleAccelerations[i])):
                    vehicleAccel.append(vehicleAccelerations[i][j])
            for i in range(len(bogieAccelerations)):
                for j in range(len(bogieAccelerations[i])):
                    bogieAccel.append(bogieAccelerations[i][j])
            saveOutputFile = os.path.join(
                resultDirPerDirection, "line" + str(indx) + '_' + files)
            saveOutputFile_extra = os.path.join(
                resultDirPerDirection, "extra_line" + str(indx) + '_' + files)
            writeOutputsT4wForces(saveOutputFile, pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX,
                                  abutmntY, forceZonly, vehicleAccel, bogieAccel,
                                  int(eqDirection))
            writeExtraOutputsT45wForces(saveOutputFile_extra, verticalAccelControlPoints, r)
            del orch
            if os.path.exists(os.path.join(PyCosimPath, "OpenSees_DB")):
                shutil.rmtree(os.path.join(PyCosimPath, "OpenSees_DB"))


if __name__ == "__main__":
    multiple_analysis(sys.argv)
