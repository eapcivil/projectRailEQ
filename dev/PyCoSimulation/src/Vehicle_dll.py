import ctypes
from pathlib import Path
import os
import ctypes as ct
from _ctypes import FreeLibrary as FreeLibrary
import time

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


def isLoaded(self, lib):
    libp = os.path.abspath(lib)
    ret = os.system("listdlls -p pid | grep %s > /dev/null" % (os.getpid(), libp))
    return (ret == 0)


def free_library(handle):
    kernel32 = ct.WinDLL('kernel32', use_last_error=True)
    kernel32.FreeLibrary(handle)


class FixedVehicleDll(object):

    def __init__(self, dllDir, cosim_converge: bool, first_time: bool,
                 t0: float, t1: float, nintervals: float,
                 tol_in: float, tol_out: float, tol_dphi: float, tol_phi: float,
                 nn: float, krr_explicit: float, crr_explicit: float,
                 xml0_path: str, xml1_path: str, xml2_path: str,
                 npoints: int, vx: float,
                 x_p_l_n: float, y_p_l_n: float, z_p_l_n: float,
                 x_p_l_n1: float, y_p_l_n1: float, z_p_l_n1: float,
                 x_p_r_n: float, y_p_r_n: float, z_p_r_n: float,
                 x_p_r_n1: float, y_p_r_n1: float, z_p_r_n1: float,
                 x_v_l_n: float, y_v_l_n: float, z_v_l_n: float,
                 x_v_l_n1: float, y_v_l_n1: float, z_v_l_n1: float,
                 x_v_r_n: float, y_v_r_n: float, z_v_r_n: float,
                 x_v_r_n1: float, y_v_r_n1: float, z_v_r_n1: float, ncp: int):
        """
        Source code


        @param dllDir : Directory of dll
        @param cosim_converge: Cosimulation Converge
        @param first_time: Is the first time you are calling the Vehicle dll
        @param t0: Previous CoSimulation time
        @param t1: Current CoSimulation time
        @param nintervals: Number of intervals for vehicle solution, EXAMPLE: time_step = (t1 - t0) / nintervals
        @param tol_in: Velocity tolerance, SUGGESTED VALUES: 1e-3 or lower
        @param tol_out: Residual tolerance, MUST EXPRESSION: tol_in < tol_out, SUGGESTED VALUES: 1e-2 or lower
        @param tol_dphi: Tolerance of dphi_dt, SUGGESTED VALUES: 1e-3 or lower
        @param tol_phi: Tolerance of phi, SUGGESTED VALUES: 1e-2 or lower
        @param nn: For crr, krr (10, 100, 1000 , ...). If nn = 0 the krrs = crrs = 0
        @param krr_explicit: Explicit values for all krrs, works only when nn = -1
        @param crr_explicit: Explicit values for all crrs, works only when nn = -1
        @param xml0_path: Absolute Path for xml0
        @param xml1_path: Absolute Path for xml1
        @param xml2_path: Absolute Path for xml2
        @param npoints: Number of interpolation points for the Position and Velocity Splines
        @param vx: Constant Velocity of all bodies
        @param x_p_l_n: x coords of interpolation points, for position spline, left rail, spline at n
        @param y_p_l_n: y coords of interpolation points, for position spline, left rail, spline at n
        @param z_p_l_n: z coords of interpolation points, for position spline, left rail, spline at n
        @param x_p_l_n1: x coords of interpolation points, for position spline, left rail, spline at n1
        @param y_p_l_n1: y coords of interpolation points, for position spline, left rail, spline at n1
        @param z_p_l_n1: z coords of interpolation points, for position spline, left rail, spline at n1
        @param x_p_r_n: x coords of interpolation points, for position spline, right rail, spline at n
        @param y_p_r_n: y coords of interpolation points, for position spline, right rail, spline at n
        @param z_p_r_n: z coords of interpolation points, for position spline, right rail, spline at n
        @param x_p_r_n1: x coords of interpolation points, for position spline, right rail, spline at n1
        @param y_p_r_n1: y coords of interpolation points, for position spline, right rail, spline at n1
        @param z_p_r_n1: z coords of interpolation points, for position spline, right rail, spline at n1
        @param x_v_l_n: x coords of interpolation points, for velocity spline, left rail, spline at n
        @param y_v_l_n: y coords of interpolation points, for velocity spline, left rail, spline at n
        @param z_v_l_n: z coords of interpolation points, for velocity spline, left rail, spline at n
        @param x_v_l_n1: x coords of interpolation points, for velocity spline, left rail, spline at n1
        @param y_v_l_n1: y coords of interpolation points, for velocity spline, left rail, spline at n1
        @param z_v_l_n1: z coords of interpolation points, for velocity spline, left rail, spline at n1
        @param x_v_r_n: x coords of interpolation points, for velocity spline, right rail, spline at n
        @param y_v_r_n: y coords of interpolation points, for velocity spline, right rail, spline at n
        @param z_v_r_n: z coords of interpolation points, for velocity spline, right rail, spline at n
        @param x_v_r_n1: x coords of interpolation points, for velocity spline, right rail, spline at n1
        @param y_v_r_n1: y coords of interpolation points, for velocity spline, right rail, spline at n1
        @param z_v_r_n1: z coords of interpolation points, for velocity spline, right rail, spline at n1
        @param cp_positions: XYZ coordinates of the contact points per wheel
        @param cp_velocities: XYZ velocity components of the contact points per wheel
        @param cp_forces: XYZ force components of the contact points per wheel
        """
        self.dllDir = dllDir
        self.cosim_converge = cosim_converge
        self.first_time = first_time
        self.t0 = t0
        self.t1 = t1
        self.nintervals = nintervals
        self.tol_in = tol_in
        self.tol_out = tol_out
        self.tol_dphi = tol_dphi
        self.tol_phi = tol_phi
        self.nn = nn
        self.krr_explicit = krr_explicit
        self.crr_explicit = crr_explicit
        self.xml0_path = xml0_path
        self.xml1_path = xml1_path
        self.xml2_path = xml2_path
        self.npoints = npoints
        self.vx = vx
        self.x_p_l_n = x_p_l_n
        self.x_p_l_n1 = x_p_l_n1
        self.y_p_l_n = y_p_l_n
        self.y_p_l_n1 = y_p_l_n1
        self.z_p_l_n = z_p_l_n
        self.z_p_l_n1 = z_p_l_n1
        self.x_p_r_n = x_p_r_n
        self.x_p_r_n1 = x_p_r_n1
        self.y_p_r_n = y_p_r_n
        self.y_p_r_n1 = y_p_r_n1
        self.z_p_r_n = z_p_r_n
        self.z_p_r_n1 = z_p_r_n1
        self.x_v_l_n = x_v_l_n
        self.x_v_l_n1 = x_v_l_n1
        self.y_v_l_n = y_v_l_n
        self.y_v_l_n1 = y_v_l_n1
        self.z_v_l_n = z_v_l_n
        self.z_v_l_n1 = z_v_l_n1
        self.x_v_r_n = x_v_r_n
        self.x_v_r_n1 = x_v_r_n1
        self.y_v_r_n = y_v_r_n
        self.y_v_r_n1 = y_v_r_n1
        self.z_v_r_n = z_v_r_n
        self.z_v_r_n1 = z_v_r_n1
        self.ncp = ncp
        print(bcolors.HEADER + "Vehicle Initialization DONE." + bcolors.ENDC)

    def solve(self):
        # vehicleLoaddll = ct.cdll.LoadLibrary(self.dllDir)
        # vehicleLoaddll = ct.WinDLL(self.dllDir)
        # vehicleLoaddll = ct.OleDLL(self.dllDir)

        #try:
        #    vehicleLoaddll = ct.CDLL(self.dllDir)
        #    handle = vehicleLoaddll._handle
        #    vehicle = vehicleLoaddll.Vehicle
        #except:
        #    print("Failed to Load Library with CDLL")
        #    try:
        #        vehicleLoaddll = ct.WinDll(self.dllDir)
        #        handle = vehicleLoaddll.Connect
        #    except:
        #        print("Failed to Load Library with WinDll")
        #        try:
        #            vehicleLoaddll = ct.PyDLL(self.dllDir)
        #            vehicle1 = ctypes.CFUNCTYPE(ct.c_bool, ct.c_bool,
        #                                ct.c_double, ct.c_double, ct.c_double,
        #                                ct.c_double, ct.c_double,
        #                                ct.c_double, ct.c_double,
        #                                ct.c_double, ct.c_double, ct.c_double,
        #                                ct.c_char_p, ct.c_char_p, ct.c_char_p,
        #                                ct.c_int, ct.c_double,
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
        #                                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double))
        #            vehicle = vehicle1(("Vehicle", vehicleLoaddll))
        #        except:
        #            print("FAILED TO LOAD LIBRARY")
#
        vehicle = self.dllDir.Vehicle
        vehicle.argtypes = [ct.c_bool, ct.c_bool,
                            ct.c_double, ct.c_double, ct.c_double,
                            ct.c_double, ct.c_double,
                            ct.c_double, ct.c_double,
                            ct.c_double, ct.c_double, ct.c_double,
                            ct.c_char_p, ct.c_char_p, ct.c_char_p,
                            ct.c_int, ct.c_double,
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                            ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_double)]
        vehicle.restype = ct.POINTER(ct.POINTER(ct.c_double))
        xml0_path = self.xml0_path
        xml0_path = xml0_path.encode('UTF-8')

        xml1_path = self.xml1_path
        xml1_path = xml1_path.encode('UTF-8')
        xml2_path = self.xml2_path
        xml2_path = xml2_path.encode('UTF-8')

        x_p_l_n = (ct.c_double * len(self.x_p_l_n))(*self.x_p_l_n)
        y_p_l_n = (ct.c_double * len(self.y_p_l_n))(*self.y_p_l_n)
        z_p_l_n = (ct.c_double * len(self.z_p_l_n))(*self.z_p_l_n)

        for i in range(len(self.x_p_l_n)):
            x_p_l_n[i] = self.x_p_l_n[i]
            y_p_l_n[i] = self.y_p_l_n[i]
            z_p_l_n[i] = self.z_p_l_n[i]

        x_p_l_n1 = (ct.c_double * len(self.x_p_l_n1))(*self.x_p_l_n1)
        y_p_l_n1 = (ct.c_double * len(self.y_p_l_n1))(*self.y_p_l_n1)
        z_p_l_n1 = (ct.c_double * len(self.z_p_l_n1))(*self.z_p_l_n1)

        for i in range(len(self.x_p_l_n)):
            x_p_l_n1[i] = self.x_p_l_n1[i]
            y_p_l_n1[i] = self.y_p_l_n1[i]
            z_p_l_n1[i] = self.z_p_l_n1[i]

        x_p_r_n = (ct.c_double * len(self.x_p_r_n))(*self.x_p_r_n)
        y_p_r_n = (ct.c_double * len(self.y_p_r_n))(*self.y_p_r_n)
        z_p_r_n = (ct.c_double * len(self.z_p_r_n))(*self.z_p_r_n)

        for i in range(len(self.x_p_l_n)):
            x_p_r_n[i] = self.x_p_r_n[i]
            y_p_r_n[i] = self.y_p_r_n[i]
            z_p_r_n[i] = self.z_p_r_n[i]

        x_p_r_n1 = (ct.c_double * len(self.x_p_r_n1))(*self.x_p_r_n1)
        y_p_r_n1 = (ct.c_double * len(self.y_p_r_n1))(*self.y_p_r_n1)
        z_p_r_n1 = (ct.c_double * len(self.z_p_r_n1))(*self.z_p_r_n1)

        for i in range(len(self.x_p_l_n)):
            x_p_r_n1[i] = self.x_p_r_n1[i]
            y_p_r_n1[i] = self.y_p_r_n1[i]
            z_p_r_n1[i] = self.z_p_r_n1[i]

        x_v_l_n = (ct.c_double * len(self.x_v_l_n))(*self.x_v_l_n)
        y_v_l_n = (ct.c_double * len(self.y_v_l_n))(*self.y_v_l_n)
        z_v_l_n = (ct.c_double * len(self.z_v_l_n))(*self.z_v_l_n)

        for i in range(len(self.x_p_l_n)):
            x_v_l_n[i] = self.x_v_l_n[i]
            y_v_l_n[i] = self.y_v_l_n[i]
            z_v_l_n[i] = self.z_v_l_n[i]

        x_v_l_n1 = (ct.c_double * len(self.x_v_l_n1))(*self.x_v_l_n1)
        y_v_l_n1 = (ct.c_double * len(self.y_v_l_n1))(*self.y_v_l_n1)
        z_v_l_n1 = (ct.c_double * len(self.z_v_l_n1))(*self.z_v_l_n1)

        for i in range(len(self.x_p_l_n)):
            x_v_l_n1[i] = self.x_v_l_n1[i]
            y_v_l_n1[i] = self.y_v_l_n1[i]
            z_v_l_n1[i] = self.z_v_l_n1[i]

        x_v_r_n = (ct.c_double * len(self.x_v_r_n))(*self.x_v_r_n)
        y_v_r_n = (ct.c_double * len(self.y_v_r_n))(*self.y_v_r_n)
        z_v_r_n = (ct.c_double * len(self.z_v_r_n))(*self.z_v_r_n)

        for i in range(len(self.x_p_l_n)):
            x_v_r_n[i] = self.x_v_r_n[i]
            y_v_r_n[i] = self.y_v_r_n[i]
            z_v_r_n[i] = self.z_v_r_n[i]

        x_v_r_n1 = (ct.c_double * len(self.x_v_r_n1))(*self.x_v_r_n1)
        y_v_r_n1 = (ct.c_double * len(self.y_v_r_n1))(*self.y_v_r_n1)
        z_v_r_n1 = (ct.c_double * len(self.z_v_r_n1))(*self.z_v_r_n1)

        for i in range(len(self.x_p_l_n)):
            x_v_r_n1[i] = self.x_v_r_n1[i]
            y_v_r_n1[i] = self.y_v_r_n1[i]
            z_v_r_n1[i] = self.z_v_r_n1[i]

        cp_positions = (ct.c_double * self.ncp)()
        cp_velocities = (ct.c_double * self.ncp)()
        cp_forces = (ct.c_double * self.ncp)()
        vehicle_accelerations = (ct.c_double * 40)()
        bogie_accelerations = (ct.c_double * 80)()
        bogie_yaws = (ct.c_double * 10)()

        #(bcolors.OKGREEN + "Vehicle Solution Starting..." + bcolors.ENDC)
        try:
            vehicle(ct.c_bool(self.cosim_converge), ct.c_bool(self.first_time),
                         ct.c_double(self.t0), ct.c_double(self.t1), ct.c_double(self.nintervals),
                         ct.c_double(self.tol_in), ct.c_double(self.tol_out),
                         ct.c_double(self.tol_dphi), ct.c_double(self.tol_phi),
                         ct.c_double(self.nn), ct.c_double(self.krr_explicit), ct.c_double(self.crr_explicit),
                         ct.c_char_p(xml0_path), ct.c_char_p(xml1_path), ct.c_char_p(xml2_path),
                         ct.c_int(self.npoints), ct.c_double(self.vx),
                         x_p_l_n, y_p_l_n, z_p_l_n,
                         x_p_l_n1, y_p_l_n1, z_p_l_n1,
                         x_p_r_n, y_p_r_n, z_p_r_n,
                         x_p_r_n1, y_p_r_n1, z_p_r_n1,
                         x_v_l_n, y_v_l_n, z_v_l_n,
                         x_v_l_n1, y_v_l_n1, z_v_l_n1,
                         x_v_r_n, y_v_r_n, z_v_r_n,
                         x_v_r_n1, y_v_r_n1, z_v_r_n1,
                         cp_positions, cp_velocities, cp_forces, vehicle_accelerations, bogie_accelerations, bogie_yaws)
        except:
            pass
        del vehicle
        print(bcolors.OKGREEN + "Vehicle Solution FINISHED..." + bcolors.ENDC)
        return list(cp_positions), list(cp_velocities), list(cp_forces), list(vehicle_accelerations), list(bogie_accelerations), list(bogie_yaws)


def VehicleFIXEDTEST():
    p1 = Path(__file__).parents[2]
    dllDir = "{}".format(p1) + "\\api_vehicle\\Release\\vehicle.dll"
    cosim_converge = False
    first_time = True
    t0 = 0.0
    t1 = 0.001
    nintervals = 1
    tol_in = 0.001
    tol_out = 0.01
    tol_dphi = 10.00
    tol_phi = 0.01
    nn = 100.00
    krr_explicit = 0.0
    crr_explicit = 0.0

    p2 = Path(__file__).parents[3]
    xml0_path = "{}".format(p2) + "\\Reference\\Code\\Train\\vehicle_xmls\\veh_1\\veh_0.xml"
    xml1_path = "{}".format(p2) + "\\Reference\\Code\\Train\\vehicle_xmls\\veh_1\\veh_1.xml"
    xml2_path = "{}".format(p2) + "\\Reference\\Code\\Train\\vehicle_xmls\\veh_1\\veh_2.xml"

    vx = 50.0
    x_p_l_n = [-100, -50, 50, 100]
    y_p_l_n = [-1.0, -1.0, -1.0, -1.0]
    z_p_l_n = [0.0, 0.0, 0.0, 0.0]
    x_p_l_n1 = [-100, -50, 50, 100]
    y_p_l_n1 = [-1.0, -1.0, -1.0, -1.0]
    z_p_l_n1 = [0.0, 0.0, 0.0, 0.0]
    x_p_r_n = [-100, -50, 50, 100]
    y_p_r_n = [1.0, 1.0, 1.0, 1.0]
    z_p_r_n = [0.0, 0.0, 0.0, 0.0]
    x_p_r_n1 = [-100, -50, 50, 100]
    y_p_r_n1 = [1.0, 1.0, 1.0, 1.0]
    z_p_r_n1 = [0.0, 0.0, 0.0, 0.0]
    x_v_l_n = [0.0, 0.0, 0.0, 0.0]
    y_v_l_n = [0.0, 0.0, 0.0, 0.0]
    z_v_l_n = [0.0, 0.0, 0.0, 0.0]
    x_v_l_n1 = [0.0, 0.0, 0.0, 0.0]
    y_v_l_n1 = [0.0, 0.0, 0.0, 0.0]
    z_v_l_n1 = [0.0, 0.0, 0.0, 0.0]
    x_v_r_n = [0.0, 0.0, 0.0, 0.0]
    y_v_r_n = [0.0, 0.0, 0.0, 0.0]
    z_v_r_n = [0.0, 0.0, 0.0, 0.0]
    x_v_r_n1 = [0.0, 0.0, 0.0, 0.0]
    y_v_r_n1 = [0.0, 0.0, 0.0, 0.0]
    z_v_r_n1 = [0.0, 0.0, 0.0, 0.0]
    cp_positions = [0.0] * 24
    cp_velocities = [0.0] * 24
    cp_forces = [0.0] * 24
    ncp = 24
    vehicle = FixedVehicleDll(dllDir, cosim_converge, first_time, t0, t1, nintervals, tol_in, tol_out, tol_dphi,
                              tol_phi, nn, krr_explicit, crr_explicit,
                              xml0_path, xml1_path, xml2_path, int(len(x_p_l_n)), vx,
                              x_p_l_n, y_p_l_n, z_p_l_n,
                              x_p_l_n1, y_p_l_n1, z_p_l_n1,
                              x_p_r_n, y_p_r_n, z_p_r_n,
                              x_p_r_n1, y_p_r_n1, z_p_r_n1,
                              x_v_l_n, y_v_l_n, z_v_l_n,
                              x_v_l_n1, y_v_l_n1, z_v_l_n1,
                              x_v_r_n, y_v_r_n, z_v_r_n,
                              x_v_r_n1, y_v_r_n1, z_v_r_n1,
                              ncp)
    for i in range(200):
        vehicle = FixedVehicleDll(dllDir, cosim_converge, first_time, t0, t1, nintervals, tol_in, tol_out, tol_dphi,
                                  tol_phi, nn, krr_explicit, crr_explicit,
                                  xml0_path, xml1_path, xml2_path, int(len(x_p_l_n)), vx,
                                  x_p_l_n, y_p_l_n, z_p_l_n,
                                  x_p_l_n1, y_p_l_n1, z_p_l_n1,
                                  x_p_r_n, y_p_r_n, z_p_r_n,
                                  x_p_r_n1, y_p_r_n1, z_p_r_n1,
                                  x_v_l_n, y_v_l_n, z_v_l_n,
                                  x_v_l_n1, y_v_l_n1, z_v_l_n1,
                                  x_v_r_n, y_v_r_n, z_v_r_n,
                                  x_v_r_n1, y_v_r_n1, z_v_r_n1,
                                  ncp)
        cp_position, cp_velocity, cp_force = vehicle.solve()
        print(i)

    print(cp_position)
    print(cp_velocity)
    print(cp_force)


if __name__ == "__main__":
    VehicleFIXEDTEST()
