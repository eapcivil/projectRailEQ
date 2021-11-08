import os

import numpy as np
import openseespy.opensees as ops
import openseespy.postprocessing.Get_Rendering as opsplt
from force_transform import nodalForces
from utilities import bcolors


def relativePosition(ry, rz, rotation):
    list = [0, ry, rz]
    cross_prod = np.cross(rotation, list)
    returnpos = np.zeros(3)
    for i in range(len(returnpos)):
        returnpos[i] = cross_prod[i] + list[i]
    return returnpos


def relativeVelocity(ry, rz, rotation):
    list = [0, ry, rz]
    cross_prod = np.cross(rotation, list)
    returnpos = np.zeros(3)
    for i in range(len(returnpos)):
        returnpos[i] = cross_prod[i]
    return returnpos


def relativeAccel(ry, rz, rotation):
    list = [0, ry, rz]
    cross_prod = np.cross(rotation, list)
    returnpos = np.zeros(3)
    for i in range(len(returnpos)):
        returnpos[i] = cross_prod[i]
    return returnpos


class Bridge(object):

    def __init__(self, Ec, gap, stiffnessFactor, phiybz, Mybz, phiubz, Mubz, phiyby, Myby, phiuby, Μυby, dt, globalT,
                 eqTime, eqValues, eqDirection):
        self.deckNodeID = [1, 13, 28, 29, 30, 31, 32, 33, 34, 35, 36, 14, 15, 2, 16, 17, 37, 38, 39, 40, 41, 42, 43, 44,
                           45, 18, 19, 3, 20, 21, 46, 47, 48, 49, 50, 51, 52, 53, 54, 22, 23, 4, 24, 25, 55, 56, 57, 58,
                           59, 60, 61,
                           62, 63, 26, 5]
        self.controlPoints = [1, 28, 31, 34, 14, 2, 17, 40, 42, 18, 3, 21, 49, 51, 22, 4, 25, 57, 60, 63, 5] 
        self.pier_top_nodes = [6, 7, 8]
        self.pier_base_nodes = [9, 10, 11]
        self.bearing_top_nodes = [136, 138, 140, 142, 144, 146, 148, 150, 152, 154]
        self.bearing_base_nodes = [137, 139, 141, 143, 145, 147, 149, 151, 153, 155]
        self.abtX_nodes = [156, 157, 162, 163, 1560, 1620]
        self.abtY_nodes = [182, 182000, 183, 183000, 184, 184000, 185, 185000]
        self.dt = dt
        self.globalT = globalT
        self.eqTime = eqTime
        self.eqValues = eqValues
        self.eqDirection = eqDirection

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

    def initialize_model(self):
        ops.wipe()
        ops.setPrecision(5)
        ops.model('basic', '-ndm', 3, '-ndf', 6)
        ops.setTime(self.globalT)
        dir_of_database = "./OpenSees_DB"
        try:
            if not os.path.exists(dir_of_database):
                os.mkdir(dir_of_database, mode=0o777)
        except OSError:
            print("Directory creation failed")
        ops.database("File", "{}/OpenSees_Model".format(dir_of_database))
        print(bcolors.HEADER + "Bridge Initialization DONE." + bcolors.ENDC)

    def construct_model(self):
        ops.node(1, 0.0, 0.0, 0.0)
        ops.node(2, 39.0, 0.0, 0.0)
        ops.node(3, 84.0, 0.0, 0.0)
        ops.node(4, 129.0, 0.0, 0.0)
        ops.node(5, 168.0, 0.0, 0.0)

        ops.node(6, 39.0, 0.0, -2.803)
        ops.node(7, 84.0, 0.0, -2.803)
        ops.node(8, 129.0, 0.0, -2.803)

        ops.node(9, 39.0, 0.0, -21.153)
        ops.node(10, 84.0, 0.0, -24.603)
        ops.node(11, 129.0, 0.0, -15.153)

        ops.node(13, 3.75, 0.0, 0.0)
        ops.node(14, 35.0, 0.0, 0.0)
        ops.node(15, 38.0, 0.0, 0.0)
        ops.node(16, 40.0, 0.0, 0.0)
        ops.node(17, 43.0, 0.0, 0.0)
        ops.node(18, 80.0, 0.0, 0.0)
        ops.node(19, 83.0, 0.0, 0.0)
        ops.node(20, 85.0, 0.0, 0.0)
        ops.node(21, 88.0, 0.0, 0.0)
        ops.node(22, 125.0, 0.0, 0.0)
        ops.node(23, 128.0, 0.0, 0.0)
        ops.node(24, 130.0, 0.0, 0.0)
        ops.node(25, 133.0, 0.0, 0.0)
        ops.node(26, 164.25, 0.0, 0.0)
        ops.node(28, 6.875, 0.0, 0.0)
        ops.node(29, 10.0, 0.0, 0.0)
        ops.node(30, 13.125, 0.0, 0.0)
        ops.node(31, 16.25, 0.0, 0.0)
        ops.node(32, 19.375, 0.0, 0.0)
        ops.node(33, 22.5, 0.0, 0.0)
        ops.node(34, 25.625, 0.0, 0.0)
        ops.node(35, 28.75, 0.0, 0.0)
        ops.node(36, 31.875, 0.0, 0.0)
        ops.node(37, 46.7, 0.0, 0.0)
        ops.node(38, 50.4, 0.0, 0.0)
        ops.node(39, 54.1, 0.0, 0.0)
        ops.node(40, 57.8, 0.0, 0.0)
        ops.node(41, 61.5, 0.0, 0.0)
        ops.node(42, 65.2, 0.0, 0.0)
        ops.node(43, 68.9, 0.0, 0.0)
        ops.node(44, 72.6, 0.0, 0.0)
        ops.node(45, 76.3, 0.0, 0.0)
        ops.node(46, 91.7, 0.0, 0.0)
        ops.node(47, 95.4, 0.0, 0.0)
        ops.node(48, 99.1, 0.0, 0.0)
        ops.node(49, 102.8, 0.0, 0.0)
        ops.node(50, 106.5, 0.0, 0.0)
        ops.node(51, 110.2, 0.0, 0.0)
        ops.node(52, 113.9, 0.0, 0.0)
        ops.node(53, 117.6, 0.0, 0.0)
        ops.node(54, 121.3, 0.0, 0.0)
        ops.node(55, 136.125, 0.0, 0.0)
        ops.node(56, 139.25, 0.0, 0.0)
        ops.node(57, 142.375, 0.0, 0.0)
        ops.node(58, 145.5, 0.0, 0.0)
        ops.node(59, 148.625, 0.0, 0.0)
        ops.node(60, 151.75, 0.0, 0.0)
        ops.node(61, 154.875, 0.0, 0.0)
        ops.node(62, 158.0, 0.0, 0.0)
        ops.node(63, 161.125, 0.0, 0.0)
        ops.node(124, 39.0, 0.0, -6.473)
        ops.node(125, 39.0, 0.0, -10.143)
        ops.node(126, 39.0, 0.0, -13.813)
        ops.node(127, 39.0, 0.0, -17.483)
        ops.node(128, 84.0, 0.0, -7.163)
        ops.node(129, 84.0, 0.0, -11.523)
        ops.node(130, 84.0, 0.0, -15.883)
        ops.node(131, 84.0, 0.0, -20.243)
        ops.node(132, 129.0, 0.0, -5.273)
        ops.node(133, 129.0, 0.0, -7.743)
        ops.node(134, 129.0, 0.0, -10.213)
        ops.node(135, 129.0, 0.0, -12.683)

        # KOMVOI EFEDRANWN
        ops.node(136, 0.0, -3.5, -2.52)
        ops.node(137, 0.0, -3.5, -2.52)

        ops.node(138, 0.0, 3.5, -2.52)
        ops.node(139, 0.0, 3.5, -2.52)

        ops.node(140, 168.0, -3.5, -2.52)
        ops.node(141, 168.0, -3.5, -2.52)

        ops.node(142, 168.0, 3.5, -2.52)
        ops.node(143, 168.0, 3.5, -2.52)

        ops.node(144, 39.0, -1.8, -2.803)
        ops.node(145, 39.0, -1.8, -2.803)

        ops.node(146, 39.0, 1.8, -2.803)
        ops.node(147, 39.0, 1.8, -2.803)

        ops.node(148, 84.0, -1.8, -2.803)
        ops.node(149, 84.0, -1.8, -2.803)

        ops.node(150, 84.0, 1.8, -2.803)
        ops.node(151, 84.0, 1.8, -2.803)

        ops.node(152, 129.0, -1.8, -2.803)
        ops.node(153, 129.0, -1.8, -2.803)

        ops.node(154, 129.0, 1.8, -2.803)
        ops.node(155, 129.0, 1.8, -2.803)

        # KOMVOI GIA RIGID EFEDRANWN-MESOVATHRA
        ops.node(168, 39.0, -1.8, 0.0)
        ops.node(169, 39.0, 1.8, 0.0)
        ops.node(170, 84.0, -1.8, 0.0)
        ops.node(171, 84.0, 1.8, 0.0)
        ops.node(172, 129.0, -1.8, 0.0)
        ops.node(173, 129.0, 1.8, 0.0)

        # KOMVOI GIA ZERO LENGTH THEMELIWSHS VATHRWN
        ops.node(90, 39.0, 0.0, -21.153)
        ops.node(100, 84.0, 0.0, -24.603)
        ops.node(110, 129.0, 0.0, -15.153)

        # KOMVOI AKROVATHRWN
        ops.node(12, 0.0, 0.0, 1.58)
        ops.node(156, -5.0, 0.0, 1.58)
        ops.node(157, -5.0, 0.0, 1.58)
        ops.node(158, -5.0, 0.0, -2.89)
        ops.node(159, -2.25, 0.0, -2.89)
        ops.node(160, -2.25, 0.0, -4.39)
        ops.node(190, 0.0, 0.0, -2.52)
        ops.node(192, 0.0, 0.0, -2.89)
        ops.node(27, 168.0, 0.0, 1.58)

        ops.node(162, 173.0, 0.0, 1.58)
        ops.node(163, 173.0, 0.0, 1.58)

        ops.node(164, 173.0, 0.0, -3.11)
        ops.node(165, 170.25, 0.0, -3.11)
        ops.node(166, 170.25, 0.0, -4.61)
        ops.node(191, 168.0, 0.0, -2.52)
        ops.node(193, 168.0, 0.0, -3.11)

        # KOMVOI GIA RIGID EFEDRANWN-AKROVATHRA
        ops.node(174, 0.0, 0.0, -2.52)
        ops.node(175, 168.0, 0.0, -2.52)

        # KOMVOI GIA ZERO LENGTH ARISTEROU AKROVATHROU
        # diamhkhs
        ops.node(1560, -5.0, 0.0, 1.58)

        # KOMVOI GIA ZERO LENGTH DEKSIOU AKROVATHROU
        # diamhkhs
        ops.node(1620, 173.0, 0.0, 1.58)

        # KOMVOI GIA DESMEYSH EGARSIAS DIEU8YNSHS STA AKROVATHRA
        ops.node(182, 0.0, -4.55, -2.52)
        ops.node(182000, 0.0, -4.55, -2.52)
        ops.node(183, 0.0, 4.55, -2.52)
        ops.node(183000, 0.0, 4.55, -2.52)
        ops.node(184, 168.0, -4.55, -2.52)
        ops.node(184000, 168.0, -4.55, -2.52)
        ops.node(185, 168.0, 4.55, -2.52)
        ops.node(185000, 168.0, 4.55, -2.52)

        # KOMVOI GIA APOSVESTHRES
        ops.node(161, -5.0, 0.0, -1.595)
        ops.node(186, -1.25, 0.0, -2.52)

        ops.node(167, 173.0, 0.0, -1.595)
        ops.node(187, 169.25, 0.0, -2.52)

        ops.node(180, -1.25, 0.0, -1.595)
        ops.node(181, 169.25, 0.0, -1.595)
        ops.node(1800, -3.75, 0.0, -1.595)
        ops.node(1810, 171.75, 0.0, -1.595)

        ops.node(176, -1.25, -1.0, -1.595)
        ops.node(1760, -3.75, -1.0, -1.595)
        ops.node(177, -1.25, 1.0, -1.595)
        ops.node(1770, -3.75, 1.0, -1.595)
        ops.node(178, 169.25, -1.0, -1.595)
        ops.node(1780, 171.75, -1.0, -1.595)
        ops.node(179, 169.25, 1.0, -1.595)
        ops.node(1790, 171.75, 1.0, -1.595)

        self.deckX = []
        self.deckY = []
        self.deckZ = []
        for nodes in self.deckNodeID:
            self.deckX.append(ops.nodeCoord(nodes, 1))
            self.deckY.append(ops.nodeCoord(nodes, 2))
            self.deckZ.append(ops.nodeCoord(nodes, 3))


        self.deckXCon = []
        self.deckYCon = []
        self.deckZCon = []
        for nodes in self.controlPoints:
            self.deckXCon.append(ops.nodeCoord(nodes, 1))
            self.deckYCon.append(ops.nodeCoord(nodes, 2))
            self.deckZCon.append(ops.nodeCoord(nodes, 3))

        #######################################
        #######################################
        # 3. MASSES
        #######################################
        #######################################

        ops.mass(1,  105.0172, 105.0172, 105.0172, 200.00, 0.0, 0.0)
        ops.mass(13, 192.5315, 192.5315, 192.5315, 200.00, 0.0, 0.0)
        ops.mass(28, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(29, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(30, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(31, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(32, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(33, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(34, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(35, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(36, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(14, 171.5281, 171.5281, 171.5281, 200.00, 0.0, 0.0)
        ops.mass(15, 112.0183, 112.0183, 112.0183, 200.00, 0.0, 0.0)
        ops.mass(2,   56.0092,  56.0092,  56.0092, 200.00, 0.0, 0.0)
        ops.mass(16, 187.6307, 187.6307, 187.6307, 200.00, 0.0, 0.0)
        ops.mass(17, 187.6307, 187.6307, 187.6307, 200.00, 0.0, 0.0)
        ops.mass(37, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(38, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(39, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(40, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(41, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(42, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(43, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(44, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(45, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(18, 187.6307, 187.6307, 187.6307, 200.00, 0.0, 0.0)
        ops.mass(19, 112.0183, 112.0183, 112.0183, 200.00, 0.0, 0.0)
        ops.mass(21, 187.6307, 187.6307, 187.6307, 200.00, 0.0, 0.0)
        ops.mass(46, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(47, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(48, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(49, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(50, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(51, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(52, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(53, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(54, 207.2339, 207.2339, 207.2339, 200.00, 0.0, 0.0)
        ops.mass(22, 187.6307, 187.6307, 187.6307, 200.00, 0.0, 0.0)
        ops.mass(23, 112.0183, 112.0183, 112.0183, 200.00, 0.0, 0.0)
        ops.mass(4,   56.0092,  56.0092,  56.0092, 200.00, 0.0, 0.0)
        ops.mass(24, 112.0183, 112.0183, 112.0183, 200.00, 0.0, 0.0)
        ops.mass(25, 171.5281, 171.5281, 171.5281, 200.00, 0.0, 0.0)
        ops.mass(55, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(56, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(57, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(58, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(59, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(60, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(61, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(62, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(63, 175.0287, 175.0287, 175.0287, 200.00, 0.0, 0.0)
        ops.mass(26, 192.5315, 192.5315, 192.5315, 200.00, 0.0, 0.0)
        ops.mass(5,  105.0172, 105.0172, 105.0172, 200.00, 0.0, 0.0)
        ops.mass(6,     31.9862, 31.9862, 31.9862, 0.00, 0.0, 0.0)
        ops.mass(124,   63.9725, 63.9725, 63.9725, 0.00, 0.0, 0.0)
        ops.mass(125,   63.9725, 63.9725, 63.9725, 0.00, 0.0, 0.0)
        ops.mass(129,   76.0000, 76.0000, 76.0000, 0.00, 0.0, 0.0)
        ops.mass(130,   76.0000, 76.0000, 76.0000, 0.00, 0.0, 0.0)
        ops.mass(131,   76.0000, 76.0000, 76.0000, 0.00, 0.0, 0.0)
        ops.mass(10,    38.0000, 38.0000, 38.0000, 0.00, 0.0, 0.0)
        ops.mass(8,     21.5275, 21.5275, 21.5275, 0.00, 0.0, 0.0)
        ops.mass(132,   43.0550, 43.0550, 43.0550, 0.00, 0.0, 0.0)
        ops.mass(133,   43.0550, 43.0550, 43.0550, 0.00, 0.0, 0.0)
        ops.mass(134,   43.0550, 43.0550, 43.0550, 0.00, 0.0, 0.0)
        ops.mass(135,   43.0550, 43.0550, 43.0550, 0.00, 0.0, 0.0)
        ops.mass(11,    21.5275, 21.5275, 21.5275, 0.00, 0.0, 0.0)
        ops.mass(156,   76.3226, 76.3226, 76.3226, 0.00, 0.0, 0.0)
        ops.mass(162,   80.0790, 80.0790, 80.0790, 0.00, 0.0, 0.0)
        ops.mass(159,153.6697, 153.6697, 153.6697, 0.00, 0.0, 0.0)
        ops.mass(165,153.6697, 153.6697, 153.6697, 0.00, 0.0, 0.0)

        #######################################
        #######################################
        # 4. BOUNDARY CONDITIONS
        #######################################
        #######################################

        # fixY 0.0 1 1 1 0 1 0		#pin all Y=0.0 nodes

        # FIXITY
        ops.fix(160, 1, 1, 1, 1, 1, 1)
        ops.fix(166, 1, 1, 1, 1, 1, 1)
        ops.fix(1560, 1, 1, 1, 1, 1, 1)
        ops.fix(1620, 1, 1, 1, 1, 1, 1)
        ops.fix(182000, 1, 1, 1, 1, 1, 1)
        ops.fix(183000, 1, 1, 1, 1, 1, 1)
        ops.fix(184000, 1, 1, 1, 1, 1, 1)
        ops.fix(185000, 1, 1, 1, 1, 1, 1)

        # FIXITY VATHRA
        ops.fix(90, 1, 1, 1, 1, 1, 1)
        ops.fix(100, 1, 1, 1, 1, 1, 1)
        ops.fix(110, 1, 1, 1, 1, 1, 1)

        #############################################################
        #############################################################
        # 5. ElasticSection GEOMETRIES (Deck, Foundation, Rigid)
        #############################################################
        #############################################################

        AANWD_AKROV = 15.3695
        JANWD_AKROV = 23.8947  # pollaplasiasmeno epi 0,50
        IzANWD_AKROV = 25.414
        IyANWD_AKROV = 122.0886

        AANWD_ANOIGMA = 10.2332
        JANWD_ANOIGMA = 16.5389  # pollaplasiasmeno epi 0,50
        IzANWD_ANOIGMA = 17.7917
        IyANWD_ANOIGMA = 106.0131

        AANWD_KONTA_MESOV = 14.0715
        JANWD_KONTA_MESOV = 22.2906  # pollaplasiasmeno epi 0,50
        IzANWD_KONTA_MESOV = 22.7915
        IyANWD_KONTA_MESOV = 120.866

        AANWD_MESOV = 17.662
        JANWD_MESOV = 26.8701  # pollaplasiasmeno epi 0,50
        IzANWD_MESOV = 25.8682
        IyANWD_MESOV = 131.483

        ARIGID = 1000000.0
        JRIGID = 1000000.0
        IzRIGID = 1000000.0
        IyRIGID = 1000000.0

        Aabtm = 80.4
        Jabtm = 693.5517 * 0.2
        Izabtm = 241.2 * 0.5
        Iyabtm = 1203.052 * 0.5

        Athwrakio = 6.7
        Jthwrakio = 0.5452
        Izthwrakio = 0.1396
        Iythwrakio = 100.2543

        #############################################################
        #############################################################
        # 6. Elastic Section (Bridge Deck and Foundation)
        #############################################################
        #############################################################

        Edeck = 35000000.0  # kPa
        Eth = 30000000.0  # kPa
        Evathro = 33000000.0  # kPa
        Gdeck = (Edeck / (2 * (0.2 + 1)))  # kPa
        Gth = (Eth / (2 * (0.2 + 1)))  # kPa
        Gvathro = (Evathro / (2 * (0.2 + 1)))  # kPa
        numIntgrPtsNLBC = 5  # number of integration points for nonlinear Beam Column

        # section tags diatomwn
        secTagANWD_AKROV = 100
        secTagANWD_ANOIGMA = 101
        secTagANWD_KONTA_MESOV = 102
        secTagANWD_MESOV = 103
        secTagABUTMENT = 104
        secTagTHWRAKIO = 105
        secTagRIGID = 106

        # Diatomes katastrwmatos-abtm-thwrakio-rigid
        # section Elastic $secTag $E $A $Iz <$Iy $G $J> gia na synperilhfthoun sto ('nonlinearBeamColumn',

        ops.section('Elastic', secTagANWD_AKROV, Edeck, AANWD_AKROV, IzANWD_AKROV, IyANWD_AKROV, Gdeck, JANWD_AKROV)
        ops.section('Elastic', secTagANWD_ANOIGMA, Edeck, AANWD_ANOIGMA, IzANWD_ANOIGMA, IyANWD_ANOIGMA, Gdeck,
                    JANWD_ANOIGMA)
        ops.section('Elastic', secTagANWD_KONTA_MESOV, Edeck, AANWD_KONTA_MESOV, IzANWD_KONTA_MESOV, IyANWD_KONTA_MESOV,
                    Gdeck, JANWD_KONTA_MESOV)
        ops.section('Elastic', secTagANWD_MESOV, Edeck, AANWD_MESOV, IzANWD_MESOV, IyANWD_MESOV, Gdeck, JANWD_MESOV)
        ops.section('Elastic', secTagABUTMENT, Eth, Aabtm, Izabtm, Iyabtm, Gth, Jabtm)
        ops.section('Elastic', secTagTHWRAKIO, Eth, Athwrakio, Izthwrakio, Iythwrakio, Gth, Jthwrakio)
        ops.section('Elastic', secTagRIGID, Eth, ARIGID, IzRIGID, IyRIGID, Gdeck, JRIGID)

        IDColTransf = 1  # all columns
        IDBeamTransf = 2  # all beams
        IDR1Transf = 3  # Rigid 1
        IDR2Transf = 4  # Rigid 2

        ops.geomTransf('Linear', IDBeamTransf, *(0, -1, 0))
        ops.geomTransf('Linear', IDR1Transf, *(1, 0, 0))
        ops.geomTransf('Linear', IDR2Transf, *(-1, 0, 0))
        ops.geomTransf('Linear', IDColTransf, *(0, 1, 0))

        #####################################################################
        #####################################################################
        # 8. DECK FOUNDATION + RIGID ELEMENTS + SHEAR KEY (ELASTIC)
        #####################################################################
        #####################################################################

        # ELEMENTS KATASTRWMATOS
        ops.element('nonlinearBeamColumn', 2, 1, 13, numIntgrPtsNLBC, secTagANWD_AKROV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 3, 13, 28, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 4, 28, 29, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 5, 29, 30, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 6, 30, 31, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 7, 31, 32, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 8, 32, 33, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 9, 33, 34, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 10, 34, 35, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 11, 35, 36, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 12, 36, 14, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 13, 14, 15, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 14, 15, 2, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 15, 2, 16, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 16, 16, 17, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 17, 17, 37, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 18, 37, 38, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 19, 38, 39, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 20, 39, 40, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 21, 40, 41, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 22, 41, 42, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 23, 42, 43, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 24, 43, 44, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 25, 44, 45, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 26, 45, 18, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 27, 18, 19, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 28, 19, 3, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 29, 3, 20, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 30, 20, 21, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 31, 21, 46, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 32, 46, 47, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 33, 47, 48, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 34, 48, 49, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 35, 49, 50, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 36, 50, 51, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 37, 51, 52, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 38, 52, 53, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 39, 53, 54, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 40, 54, 22, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 41, 22, 23, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 42, 23, 4, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 43, 4, 24, numIntgrPtsNLBC, secTagANWD_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 44, 24, 25, numIntgrPtsNLBC, secTagANWD_KONTA_MESOV, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 45, 25, 55, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 46, 55, 56, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 47, 56, 57, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 48, 57, 58, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 49, 58, 59, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 50, 59, 60, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 51, 60, 61, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 52, 61, 62, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 53, 62, 63, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 54, 63, 26, numIntgrPtsNLBC, secTagANWD_ANOIGMA, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 55, 26, 5, numIntgrPtsNLBC, secTagANWD_AKROV, IDBeamTransf)

        # RIGID KATASTRWMATOS-MESOVATHRA
        ops.element('nonlinearBeamColumn', 116, 168, 2, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 117, 2, 169, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 118, 145, 168, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 119, 147, 169, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 120, 144, 6, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 121, 6, 146, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 122, 170, 3, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 123, 3, 171, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 124, 149, 170, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 125, 151, 171, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 126, 148, 7, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 127, 7, 150, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 128, 172, 4, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 129, 4, 173, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 130, 153, 172, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 131, 155, 173, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 132, 152, 8, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 133, 8, 154, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)

        # RIGID AKROVATHROY A1
        ops.element('nonlinearBeamColumn', 134, 157, 12, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        # ops.element('nonlinearBeamColumn', 135, 161, 1800, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        # ops.element('nonlinearBeamColumn', 136, 186, 174, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 137, 158, 159, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        # ops.element('nonlinearBeamColumn', 156, 186, 180, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 157, 1, 12, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 170, 192, 190, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 171, 159, 192, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)

        # RIGID AKROVATHROY A2
        ops.element('nonlinearBeamColumn', 158, 27, 163, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 160, 165, 164, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 162, 5, 27, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 172, 193, 191, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 173, 193, 165, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)

        # RIGID AKROVATHRWN-EFEDRANA
        ops.element('nonlinearBeamColumn', 138, 136, 190, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 174, 190, 138, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 139, 137, 174, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 140, 174, 139, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 141, 174, 1, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 142, 140, 191, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 175, 191, 142, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 143, 141, 175, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 144, 175, 143, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 145, 175, 5, numIntgrPtsNLBC, secTagRIGID, IDColTransf)

        # RIGID AKROVATHRWN GIA DESMEYSH EGARSIAS DIEY8YNSHS
        ops.element('nonlinearBeamColumn', 152, 182, 137, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 153, 139, 183, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 154, 184, 141, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 155, 143, 185, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)

        # RIGID APOSVESTHRWN
        ops.element('nonlinearBeamColumn', 146, 176, 180, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 147, 180, 177, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 149, 178, 181, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 150, 181, 179, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 250, 1760, 1800, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 251, 1800, 1770, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 252, 1780, 1810, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 253, 1810, 1790, numIntgrPtsNLBC, secTagRIGID, IDR1Transf)
        ops.element('nonlinearBeamColumn', 135, 161, 1800, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 136, 186, 174, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 156, 186, 180, numIntgrPtsNLBC, secTagRIGID, IDColTransf)
        ops.element('nonlinearBeamColumn', 159, 1810, 167, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 161, 175, 187, numIntgrPtsNLBC, secTagRIGID, IDBeamTransf)
        ops.element('nonlinearBeamColumn', 163, 187, 181, numIntgrPtsNLBC, secTagRIGID, IDColTransf)

        #####################################################################
        #####################################################################
        # 9. PIER SECTION AND ELEMENTS (BEAM WITH HINGES)
        #####################################################################
        #####################################################################

        pi = 3.141592653589793
        numIntgrPtsNLBC = 5

        # calculated geometry parameters
        ACol = 6.8  # cross-sectional area, make stiff
        IyCol = 24.56  # Column moment of inertia
        IzCol = 8.825  # Column moment of inertia
        JCol = 4.19564  # 0.2*20.9782,4.19564

        # MATERIAL parameters
        Ec = self.Ec  # kPa
        Gc = (Ec * 0.8) / (2 * (0.2 + 1))  # kPa

        EACol = Ec * ACol  # EA, for axial-force-strain relationship

        ColMatTagAxial = 3333  # assign a tag number to the column axial behavior
        ops.uniaxialMaterial('Elastic', ColMatTagAxial,
                             EACol)  # this is not used as a material, this is an axial-force-strain response  (U1)

        ###########
        # M1base
        ###########

        ColMatTagFlexM1by = 5001  # assign a tag number to the column flexural behavior
        ColSecTagM1b = 1001  # assign a tag number to the column section tag

        MyColM1by = (-1) * self.Myby  # yield moment
        MuColM1by = (-1) * self.Muby  # ultimate moment
        PhiYColM1by = self.phiyby  # yield curvature
        PhiuColM1by = self.phiuby  # ultimate curvature
        IcrM1by = MyColM1by / (Ec * PhiYColM1by)
        IeffM1by = 0.08 * IyCol + IcrM1by
        EIColeffM1by = Ec * IeffM1by  # cracked section inertia
        modIM1by = IeffM1by / IyCol

        phicorM1by = PhiYColM1by - (PhiYColM1by * (MyColM1by - 100) / MyColM1by)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM1by, (MyColM1by - 100), (PhiYColM1by - phicorM1by), MyColM1by,
                             PhiYColM1by, MuColM1by, PhiuColM1by, (-MyColM1by + 100), (-PhiYColM1by + phicorM1by),
                             (-MyColM1by), (-PhiYColM1by), (-MuColM1by), (-PhiuColM1by), 0.0, 0.0, 0.0, 0.0, 0.7)

        ColMatTagFlexM1bz = 5002
        MyColM1bz = (-1) * self.Mybz  # yield moment
        MuColM1bz = (-1) * self.Mubz  # ultimate moment
        PhiYColM1bz = self.phiybz  # yield curvature
        PhiuColM1bz = self.phiubz  # ultimate curvature
        IcrM1bz = MyColM1bz / (Ec * PhiYColM1bz)
        IeffM1bz = 0.08 * IzCol + IcrM1bz
        EIColeffM1bz = Ec * IeffM1bz  # cracked section inertia
        modIM1bz = IeffM1bz / IzCol

        phicorM1bz = PhiYColM1bz - (PhiYColM1bz * (MyColM1bz - 100) / MyColM1bz)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM1bz, (MyColM1bz - 100), (PhiYColM1bz - phicorM1bz), MyColM1bz,
                             PhiYColM1bz, MuColM1bz, PhiuColM1bz, (-MyColM1bz + 100), (-PhiYColM1bz + phicorM1bz),
                             (-MyColM1bz), (-PhiYColM1bz), (-MuColM1bz), (-PhiuColM1bz), 0.0, 0.0, 0.0, 0.0, 0.7)

        ops.section('Aggregator', ColSecTagM1b, ColMatTagAxial, 'P', ColMatTagFlexM1bz, 'Mz', ColMatTagFlexM1by, 'My')

        modIM1y = modIM1by
        modIM1z = modIM1bz

        ###########
        # M2base
        ###########

        ColMatTagFlexM2by = 5003  # assign a tag number to the column flexural behavior
        ColSecTagM2b = 1002  # assign a tag number to the column section tag
        MyColM2by = (-1) * self.Myby  # yield moment
        MuColM2by = (-1) * self.Muby  # ultimate moment
        PhiYColM2by = self.phiyby  # yield curvature
        PhiuColM2by = self.phiuby  # ultimate curvature
        IcrM2by = MyColM2by / (Ec * PhiYColM2by)
        IeffM2by = 0.08 * IyCol + IcrM2by
        EIColeffM2by = Ec * IeffM2by  # cracked section inertia
        modIM2by = IeffM2by / IyCol

        phicorM2by = PhiYColM2by - (PhiYColM2by * (MyColM2by - 100) / MyColM2by)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM2by, (MyColM2by - 100), (PhiYColM2by - phicorM2by), MyColM2by,
                             PhiYColM2by, MuColM2by, PhiuColM2by, (-MyColM2by + 100), (-PhiYColM2by + phicorM2by),
                             (-MyColM2by), (-PhiYColM2by), (-MuColM2by), (-PhiuColM2by), 0.0, 0.0, 0.0, 0.0, 0.7)

        ColMatTagFlexM2bz = 5004
        MyColM2bz = (-1) * self.Mybz  # yield moment
        MuColM2bz = (-1) * self.Mubz  # ultimate moment
        PhiYColM2bz = self.phiybz  # yield curvature
        PhiuColM2bz = self.phiubz  # ultimate curvature
        IcrM2bz = MyColM2bz / (Ec * PhiYColM2bz)
        IeffM2bz = 0.08 * IzCol + IcrM2bz
        EIColeffM2bz = Ec * IeffM2bz  # cracked section inertia
        modIM2bz = IeffM2bz / IzCol

        phicorM2bz = PhiYColM2bz - (PhiYColM2bz * (MyColM2bz - 100) / MyColM2bz)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM2bz, (MyColM2bz - 100), (PhiYColM2bz - phicorM2bz), MyColM2bz,
                             PhiYColM2bz, MuColM2bz, PhiuColM2bz, (-MyColM2bz + 100), (-PhiYColM2bz + phicorM2bz),
                             (-MyColM2bz), (-PhiYColM2bz), (-MuColM2bz), (-PhiuColM2bz), 0.0, 0.0, 0.0, 0.0, 0.7)

        ops.section('Aggregator', ColSecTagM2b, ColMatTagAxial, 'P', ColMatTagFlexM2bz, 'Mz', ColMatTagFlexM2by, 'My')

        modIM2y = modIM2by
        modIM2z = modIM2bz

        ###########
        # M3base
        ###########

        ColMatTagFlexM3by = 5005  # assign a tag number to the column flexural behavior
        ColSecTagM3b = 1003  # assign a tag number to the column section tag
        MyColM3by = (-1) * self.Myby  # yield moment
        MuColM3by = (-1) * self.Muby  # ultimate moment
        PhiYColM3by = self.phiyby  # yield curvature
        PhiuColM3by = self.phiuby  # ultimate curvature
        IcrM3by = MyColM3by / (Ec * PhiYColM3by)
        IeffM3by = 0.08 * IyCol + IcrM3by
        EIColeffM3by = Ec * IeffM3by  # cracked section inertia
        modIM3by = IeffM3by / IyCol

        phicorM3by = PhiYColM3by - (PhiYColM3by * (MyColM3by - 100) / MyColM3by)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM3by, (MyColM3by - 100), (PhiYColM3by - phicorM3by), MyColM3by,
                             PhiYColM3by, MuColM3by, PhiuColM3by, (-MyColM3by + 100), (-PhiYColM3by + phicorM3by),
                             (-MyColM3by), (-PhiYColM3by), (-MuColM3by), (-PhiuColM3by), 0.0, 0.0, 0.0, 0.0, 0.7)

        ColMatTagFlexM3bz = 5006
        MyColM3bz = (-1) * self.Mybz  # yield moment
        MuColM3bz = (-1) * self.Mubz  # ultimate moment
        PhiYColM3bz = self.phiybz  # yield curvature
        PhiuColM3bz = self.phiubz  # ultimate curvature
        IcrM3bz = MyColM3bz / (Ec * PhiYColM3bz)
        IeffM3bz = 0.08 * IzCol + IcrM3bz
        EIColeffM3bz = Ec * IeffM3bz  # cracked section inertia
        modIM3bz = IeffM3bz / IzCol

        phicorM3bz = PhiYColM3bz - (PhiYColM3bz * (MyColM3bz - 100) / MyColM3bz)  # gia ton ypologismo toy e1p

        ops.uniaxialMaterial('Hysteretic', ColMatTagFlexM3bz, (MyColM3bz - 100), (PhiYColM3bz - phicorM3bz), MyColM3bz,
                             PhiYColM3bz, MuColM3bz, PhiuColM3bz, (-MyColM3bz + 100), (-PhiYColM3bz + phicorM3bz),
                             (-MyColM3bz), (-PhiYColM3bz), (-MuColM3bz), (-PhiuColM3bz), 0.0, 0.0, 0.0, 0.0, 0.7)

        ops.section('Aggregator', ColSecTagM3b, ColMatTagAxial, 'P', ColMatTagFlexM3bz, 'Mz', ColMatTagFlexM3by, 'My')

        modIM3y = modIM3by
        modIM3z = modIM3bz

        secTagPier1 = 1050
        ops.section('Elastic', secTagPier1, Ec, ACol, (modIM1z * IzCol), (modIM1y * IyCol), Gc, JCol)

        secTagPier2 = 1051
        ops.section('Elastic', secTagPier2, Ec, ACol, (modIM2z * IzCol), (modIM2y * IyCol), Gc, JCol)

        secTagPier3 = 1052
        ops.section('Elastic', secTagPier3, Ec, ACol, (modIM3z * IzCol), (modIM3y * IyCol), Gc, JCol)

        ops.beamIntegration('HingeRadau', 9999999, ColSecTagM1b, 2.0225, ColSecTagM1b, 0, secTagPier1)
        ops.element('forceBeamColumn', 1030, 9, 127, IDColTransf, 9999999)
        ops.element('nonlinearBeamColumn', 1031, 127, 126, numIntgrPtsNLBC, secTagPier1, IDColTransf)
        ops.element('nonlinearBeamColumn', 1032, 126, 125, numIntgrPtsNLBC, secTagPier1, IDColTransf)
        ops.element('nonlinearBeamColumn', 1033, 125, 124, numIntgrPtsNLBC, secTagPier1, IDColTransf)
        ops.element('nonlinearBeamColumn', 1034, 124, 6, numIntgrPtsNLBC, secTagPier1, IDColTransf)

        ### Vathro M2 / H=21.8
        ops.beamIntegration('HingeRadau', 8888888, ColSecTagM2b, 2.3675, ColSecTagM2b, 0, secTagPier2)
        ops.element('forceBeamColumn', 1035, 10, 131, IDColTransf, 8888888)
        ops.element('nonlinearBeamColumn', 1036, 131, 130, numIntgrPtsNLBC, secTagPier2, IDColTransf)
        ops.element('nonlinearBeamColumn', 1037, 130, 129, numIntgrPtsNLBC, secTagPier2, IDColTransf)
        ops.element('nonlinearBeamColumn', 1038, 129, 128, numIntgrPtsNLBC, secTagPier2, IDColTransf)
        ops.element('nonlinearBeamColumn', 1039, 128, 7, numIntgrPtsNLBC, secTagPier2, IDColTransf)

        ### Vathro M3 / H=12.35
        ops.beamIntegration('HingeRadau', 7777777, ColSecTagM3b, 1.4225, ColSecTagM3b, 0, secTagPier3)
        ops.element('forceBeamColumn', 1040, 11, 135, IDColTransf, 7777777)
        ops.element('nonlinearBeamColumn', 1041, 135, 134, numIntgrPtsNLBC, secTagPier3, IDColTransf)
        ops.element('nonlinearBeamColumn', 1042, 134, 133, numIntgrPtsNLBC, secTagPier3, IDColTransf)
        ops.element('nonlinearBeamColumn', 1043, 133, 132, numIntgrPtsNLBC, secTagPier3, IDColTransf)
        ops.element('nonlinearBeamColumn', 1044, 132, 8, numIntgrPtsNLBC, secTagPier3, IDColTransf)

        #####################################################################
        #####################################################################
        # 10. BEARINGS
        #####################################################################
        #####################################################################

        ################################
        #### Efedrana M1,M2,M3
        ################################

        matTagKv123 = 4
        Kv123 = 7135084.118 * self.stiffnessFactor
        eta = 0.05

        ops.uniaxialMaterial('Elastic', matTagKv123, Kv123, eta)

        matTagKh123 = 5
        fy123 = 624
        ke123 = 28363.64 * self.stiffnessFactor
        alpha123 = 0.23
        ops.uniaxialMaterial('Steel01', matTagKh123, fy123, ke123, alpha123)

        matTagKbx123 = 6
        Kbx123 = 291310.8799 * self.stiffnessFactor

        ops.uniaxialMaterial('Elastic', matTagKbx123, Kbx123, eta)

        matTagKby123 = 7
        Kby123 = 291310.8799 * self.stiffnessFactor

        ops.uniaxialMaterial('Elastic', matTagKby123, Kby123, eta)

        ################################
        #### Efedrana Abutments
        ################################

        matTagKvA = 444
        KvA = 4096078.157 * self.stiffnessFactor
        eta = 0.05

        ops.uniaxialMaterial('Elastic', matTagKvA, KvA, eta)

        matTagKhA = 544
        fyA = 4400
        keA = 22510.82 * self.stiffnessFactor
        alphaA = 0.20
        ops.uniaxialMaterial('Steel01', matTagKhA, fyA, keA, alphaA)

        matTagKbxA = 644
        KbxA = 94498.10571 * self.stiffnessFactor

        ops.uniaxialMaterial('Elastic', matTagKbxA, KbxA, eta)

        matTagKbyA = 744
        KbyA = 94498.10571 * self.stiffnessFactor

        ops.uniaxialMaterial('Elastic', matTagKbyA, KbyA, eta)

        #####################################################################################
        ##########    ELEMENT BEARING   #######################################
        #####################################################################################

        #### ARISTERO AKROVATHRO
        ops.element('zeroLength', 991, 136, 137, '-mat', matTagKvA, matTagKhA, matTagKhA, matTagKbxA, matTagKbyA,
                    '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 992, 138, 139, '-mat', matTagKvA, matTagKhA, matTagKhA, matTagKbxA, matTagKbyA,
                    '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        #### DEKSIO AKROVATHRO
        ops.element('zeroLength', 993, 140, 141, '-mat', matTagKvA, matTagKhA, matTagKhA, matTagKbxA, matTagKbyA,
                    '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 994, 142, 143, '-mat', matTagKvA, matTagKhA, matTagKhA, matTagKbxA, matTagKbyA,
                    '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        #### VATHRA

        #### M1
        ops.element('zeroLength', 1991, 144, 145, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 1992, 146, 147, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        #### M2
        ops.element('zeroLength', 2991, 148, 149, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 2992, 150, 151, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        #### M3
        ops.element('zeroLength', 3991, 152, 153, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 3992, 154, 155, '-mat', matTagKv123, matTagKh123, matTagKh123, matTagKbx123,
                    matTagKby123, '-dir', 1, 2, 3, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        #####################################################################
        #####################################################################
        # 11. FOUNDATION SPRINGS
        #####################################################################
        #####################################################################

        matTagKz = 40
        Kz = 12670417  # kN/m
        ops.uniaxialMaterial('Elastic', matTagKz, Kz)

        matTagKx = 50
        Kx = 6536287  # kN/m
        ops.uniaxialMaterial('Elastic', matTagKx, Kx)

        matTagKy = 60
        Ky = 6566206  # kN/m
        ops.uniaxialMaterial('Elastic', matTagKy, Ky)

        matTagKrx = 70
        Krx = 188686649  # kN/m
        ops.uniaxialMaterial('Elastic', matTagKrx, Krx)

        matTagKry = 80
        Kry = 175326491  # kN/m
        ops.uniaxialMaterial('Elastic', matTagKry, Kry)

        matTagKrz = 90
        Krz = 146455770  # kN/m
        # set Krz 133297787 ; 	#kN/m
        ops.uniaxialMaterial('Elastic', matTagKrz, Krz)

        ops.element('zeroLength', 995, 90, 9, '-mat', matTagKz, matTagKx, matTagKy, matTagKrz, matTagKrx, matTagKry,
                    '-dir', 1, 2, 3, 4, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 996, 100, 10, '-mat', matTagKz, matTagKx, matTagKy, matTagKrz, matTagKrx, matTagKry,
                    '-dir', 1, 2, 3, 4, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 997, 110, 11, '-mat', matTagKz, matTagKx, matTagKy, matTagKrz, matTagKrx, matTagKry,
                    '-dir', 1, 2, 3, 4, 5, 6, '-orient', 0, 0, 1, 1, 0, 0)

        ######################
        # ---------------------
        #  12. ABUTMENTS
        # ---------------------
        ######################

        #####################################################################################
        ###############    LONGITUDINAL   ###################################################
        #####################################################################################

        matTagGap1X = 88
        E = 5732925.455  # kN/m
        # set E 0 ; 			#kN/m
        Fygap = -50000
        gap = (-1) * self.gap / 10
        ops.uniaxialMaterial('ElasticPPGap', matTagGap1X, E, Fygap, gap)

        matTagGap2X = 888
        E = 5732925.455  # kN/m
        # set E 0 ; 			#kN/m
        Fygap = 50000
        gap = self.gap / 10
        ops.uniaxialMaterial('ElasticPPGap', matTagGap2X, E, Fygap, gap)

        matTagAbtm_1X = 1001
        e1pX = 0.0452
        s1pX = 11805
        e2pX = 0.1582
        s2pX = 34904
        e3pX = 0.452
        s3pX = 53804
        pinchX = 1
        pinchY = 1
        damage1 = 0
        damage2 = 0

        ops.uniaxialMaterial('Hysteretic', matTagAbtm_1X, s1pX, e1pX, s2pX, e2pX, s3pX, e3pX, -s1pX, -e1pX, -s2pX,
                             -e2pX, -s3pX, -e3pX, pinchX, pinchY, damage1, damage2, 0.0)

        matTagAbtm_2X = 10011
        e1pX = 0.0452
        s1pX = 11805
        e2pX = 0.1582
        s2pX = 34904
        e3pX = 0.452
        s3pX = 53804
        pinchX = 1
        pinchY = 1
        damage1 = 0
        damage2 = 0

        ops.uniaxialMaterial('Hysteretic', matTagAbtm_2X, s1pX, e1pX, s2pX, e2pX, s3pX, e3pX, -s1pX, -e1pX, -s2pX,
                             -e2pX, -s3pX, -e3pX, pinchX, pinchY, damage1, damage2, 0.0)

        matTagAbtm_1Y = 10010000
        e1pY = 0.0452
        s1pY = 7869
        e2pY = 0.1582
        s2pY = 42997
        e3pY = 0.452
        s3pY = 71738
        pinchX = 1
        pinchY = 1
        damage1 = 0
        damage2 = 0

        ops.uniaxialMaterial('Hysteretic', matTagAbtm_1Y, s1pY, e1pY, s2pY, e2pY, s3pY, e3pY, -s1pY, -e1pY, -s2pY,
                             -e2pY, -s3pY, -e3pY, pinchX, pinchY, damage1, damage2, 0.0)

        matTagAbtm_2Y = 100110000
        e1pY = 0.0452
        s1pY = 7869
        e2pY = 0.1582
        s2pY = 42997
        e3pY = 0.452
        s3pY = 71738
        pinchX = 1
        pinchY = 1
        damage1 = 0
        damage2 = 0

        ops.uniaxialMaterial('Hysteretic', matTagAbtm_2Y, s1pY, e1pY, s2pY, e2pY, s3pY, e3pY, -s1pY, -e1pY, -s2pY,
                             -e2pY, -s3pY, -e3pY, pinchX, pinchY, damage1, damage2, 0.0)

        matTagDamp = 999
        KDamp = 100000.0  # kN/m
        Cd = 1175.0
        a = 0.15
        ops.uniaxialMaterial('Viscous Damper', matTagDamp, KDamp, Cd, a)

        #####################################################################################
        ##########    ELEMENTS ABUTMENT   #######################################
        #####################################################################################

        ### Elathrio Gap+Abtm (Series X)
        ops.element('zeroLength', 9991, 156, 157, '-mat', matTagGap1X, '-dir', 2, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 9992, 162, 163, '-mat', matTagGap2X, '-dir', 2, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 99991, 1560, 156, '-mat', matTagAbtm_1X, '-dir', 2, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 99992, 1620, 162, '-mat', matTagAbtm_2X, '-dir', 2, '-orient', 0, 0, 1, 1, 0, 0)

        ### Elathrio Abtm (Series Y)
        ops.element('zeroLength', 9991000, 182000, 182, '-mat', matTagAbtm_1Y, '-dir', 3, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 9992000, 183000, 183, '-mat', matTagAbtm_1Y, '-dir', 3, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 99991000, 184000, 184, '-mat', matTagAbtm_2Y, '-dir', 3, '-orient', 0, 0, 1, 1, 0, 0)
        ops.element('zeroLength', 99992000, 185000, 185, '-mat', matTagAbtm_2Y, '-dir', 3, '-orient', 0, 0, 1, 1, 0, 0)

        ### Elathria gia aposvesthres

        ops.element('zeroLength', 8881, 176, 1760, '-mat', matTagDamp, '-dir', 1)
        ops.element('zeroLength', 8882, 177, 1770, '-mat', matTagDamp, '-dir', 1)
        ops.element('zeroLength', 8883, 178, 1780, '-mat', matTagDamp, '-dir', 1)
        ops.element('zeroLength', 8884, 179, 1790, '-mat', matTagDamp, '-dir', 1)

        # ops.element('twoNodeLink', 8881, 176, 1760, '-mat', matTagDamp, '-dir', 1, '-orient', 0, 1, 0)
        # ops.element('twoNodeLink', 8882, 177, 1770, '-mat', matTagDamp, '-dir', 1, '-orient', 0, 1, 0)

        # ops.element('twoNodeLink', 8883, 178, 1780, '-mat', matTagDamp, '-dir', 1, '-orient', 0, 1, 0)
        # ops.element('twoNodeLink', 8884, 179, 1790, '-mat', matTagDamp, '-dir', 1, '-orient', 0, 1, 0)

        #####################################################################################
        ##########    ELEMENT THWRAKIO KAI TOIXOS ABUTMENT  #######################################
        #####################################################################################
        modIthwr0 = 1
        modIthwr0z = 1
        Ecthwr = 30000000  # kpa
        Gthwr = 0.8 * Ecthwr / 2 * (0.2 + 1)  # kPa
        secTagBackwall = 20003
        ops.section('Elastic', secTagBackwall, Ecthwr, Athwrakio, Izthwrakio * modIthwr0z, Iythwrakio * modIthwr0,
                    Gthwr, Jthwrakio * 0.2)

        Athwr = 0.5 * 13.4
        EAthwr = Ecthwr * Athwr  # EA, for axial-force-strain relationship
        thwrMatTagAxial = 30000  # assign a tag number to the column axial behavior
        ops.uniaxialMaterial('Elastic', thwrMatTagAxial,
                             EAthwr)  # this is not used as a material, this is an axial-force-strain response  (U1)

        ### Thwrakio 13.4x0.5
        thwrMatTagFlex0 = 20001  # assign a tag number to the column flexural behavior
        thwrSecTag = 20002  # assign a tag number to the column section tag
        Mythwr0 = 74245.79  # yield moment
        Muthwr0 = 77958.0795  # ultimate moment
        PhiYthwr0 = 0.000252  # yield curvature
        Phiuthwr0 = 0.005594  # ultimate curvature
        Icrthwry = Mythwr0 / (Ecthwr * PhiYthwr0)
        Ieffthwry = 0.08 * Iythwrakio + Icrthwry
        EIColeffthwr0 = Ec * Ieffthwry  # cracked section inertia

        ops.uniaxialMaterial('Hysteretic', thwrMatTagFlex0, Mythwr0 - 100, PhiYthwr0 - 0.00000033941, Mythwr0,
                             PhiYthwr0, Muthwr0, Phiuthwr0, -Mythwr0 + 100, -PhiYthwr0 + 0.00000033941, -Mythwr0,
                             -PhiYthwr0, -Muthwr0, -Phiuthwr0, 0.0, 0.0, 0.0, 0.0, 0.7)

        thwrMatTagFlex0z = 20002
        Mythwr0z = 2671.827  # yield moment
        Muthwr0z = 2826.83205  # ultimate moment
        PhiYthwr0z = 0.006263  # yield curvature
        Phiuthwr0z = 0.122494  # ultimate curvature
        Icrthwrz = Mythwr0z / (Ecthwr * PhiYthwr0z)
        Ieffthwrz = 0.08 * Izthwrakio + Icrthwrz
        EIColeffthwr0z = Ec * Ieffthwrz  # cracked section inertia

        ops.uniaxialMaterial('Hysteretic', thwrMatTagFlex0z, Mythwr0z - 100, PhiYthwr0z - 0.000234409, Mythwr0z,
                             PhiYthwr0z, Muthwr0z, Phiuthwr0z, -Mythwr0z + 100, -PhiYthwr0z + 0.000234409, -Mythwr0z,
                             -PhiYthwr0z, -Muthwr0z, -Phiuthwr0z, 0.0, 0.0, 0.0, 0.0, 0.7)

        ops.section('Aggregator', thwrSecTag, thwrMatTagAxial, 'P', thwrMatTagFlex0, 'Mz', thwrMatTagFlex0z,
                    'My')  # combine axial and flexural behavior into one section (no P-M interaction here)

        ops.beamIntegration('HingeRadau', 5555555, thwrSecTag, 0.3, secTagBackwall, 0, secTagBackwall)
        ops.element('forceBeamColumn', 926, 158, 161, IDColTransf, 5555555)

        ops.beamIntegration('HingeRadau', 4444444, thwrSecTag, 0.3, secTagBackwall, 0, secTagBackwall)
        ops.element('forceBeamColumn', 944, 164, 167, IDColTransf, 4444444)

        ops.element('nonlinearBeamColumn', 927, 161, 156, numIntgrPtsNLBC, secTagBackwall, IDColTransf)
        ops.element('nonlinearBeamColumn', 945, 167, 162, numIntgrPtsNLBC, secTagBackwall, IDColTransf)

        ### TOIXOS ABUTMENT (stem wall)

        #### TOIXOS AKROVATHROY

        ops.element('nonlinearBeamColumn', 913, 160, 159, numIntgrPtsNLBC, secTagABUTMENT, IDColTransf)
        ops.element('nonlinearBeamColumn', 933, 166, 165, numIntgrPtsNLBC, secTagABUTMENT, IDColTransf)
        #opsplt.plot_model()
    def restore(self):
        try:
            ops.restore(0)
        except:
            pass
        try:
            ops.remove('timeSeries', 0)
            ops.remove('timeSeries', 1)
            ops.remove('loadPattern', 0)
            ops.remove('loadPattern', 1)
            ops.remove('timeSeries', 10)
            ops.remove('loadPattern', 21313)
        except:
            pass

    def add_nodalLoad(self, tag, cp_position, cp_velocities, cp_forces,bridgeSubSteps):
        nodalLoadArrayi, nodalLoadArrayj = nodalForces(self.deckNodeID, self.deckX, self.deckY, self.deckZ, cp_position,
                                                       cp_velocities, cp_forces)
        if tag == 0:
            tn = ops.getTime()
            ops.timeSeries('Path', tag, '-values', 0, 1, 0, '-time', tn - self.dt, tn, tn + bridgeSubSteps*self.dt, '-prependZero')
            ops.pattern('Plain', tag, tag, '-fact', 1.0)
        else:
            tn = ops.getTime() + 4*self.dt
            ops.timeSeries('Path', tag, '-values', 0, 1, 0, '-time', tn - bridgeSubSteps*self.dt, tn, tn + self.dt, '-prependZero')
            ops.pattern('Plain', tag, tag, 1)

        for i, value in enumerate(self.deckNodeID):
            fx = 0
            fy = 0
            fz = 0
            mx = 0
            my = 0
            mz = 0
            #for j in range(int(nodalLoadArrayi.shape[0])):
                #if i == nodalLoadArrayi[j][0]:
            fx = nodalLoadArrayi[i][1]
            fy = nodalLoadArrayi[i][2]
            fz = nodalLoadArrayi[i][3]
            mx = nodalLoadArrayi[i][4]
            my = nodalLoadArrayi[i][5]
            mz = nodalLoadArrayi[i][6]
            #for j in range(int(nodalLoadArrayj.shape[0])):
                #if i == nodalLoadArrayj[j][0]:
            fx += nodalLoadArrayj[i][1]
            fy += nodalLoadArrayj[i][2]
            fz += nodalLoadArrayj[i][3]
            mx += nodalLoadArrayj[i][4]
            my += nodalLoadArrayj[i][5]
            mz += nodalLoadArrayj[i][6]
            ops.load(value, fx, fy, fz, mx, my, mz)
            #print(value, fx, fy, fz, mx, my, mz)

    def solve_gravity(self):
        ops.wipeAnalysis()
        ops.timeSeries('Linear', 3333333)
        ops.pattern('Plain', 101, 3333333)
        ops.load(1, 0.00, 0.00, (-150.0172 * 9.81), 0.0, 0.0, 0.0)
        ops.load(13, 0.00, 0.00, (-192.5315 * 9.81), 0.0, 0.0, 0.0)
        ops.load(28, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(29, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(30, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(31, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(32, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(33, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(34, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(35, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(36, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(14, 0.0, 0.0, (-171.5281 * 9.81), 0.0, 0.0, 0.0)
        ops.load(15, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(2, 0.0, 0.0, (-56.0092 * 9.81), 0.0, 0.0, 0.0)
        ops.load(16, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(17, 0.0, 0.0, (-187.6307 * 9.81), 0.0, 0.0, 0.0)
        ops.load(37, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(38, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(39, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(40, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(41, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(42, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(43, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(44, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(45, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(18, 0.0, 0.0, (-187.6307 * 9.81), 0.0, 0.0, 0.0)
        ops.load(19, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(3, 0.0, 0.0, (-56.0092 * 9.81), 0.0, 0.0, 0.0)
        ops.load(20, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(21, 0.0, 0.0, (-187.6307 * 9.81), 0.0, 0.0, 0.0)
        ops.load(46, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(47, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(48, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(49, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(50, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(51, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(52, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(53, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(54, 0.0, 0.0, (-207.2339 * 9.81), 0.0, 0.0, 0.0)
        ops.load(22, 0.0, 0.0, (-187.6307 * 9.81), 0.0, 0.0, 0.0)
        ops.load(23, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(4, 0.0, 0.0, (-56.0092 * 9.81), 0.0, 0.0, 0.0)
        ops.load(24, 0.0, 0.0, (-112.0183 * 9.81), 0.0, 0.0, 0.0)
        ops.load(25, 0.0, 0.0, (-171.5281 * 9.81), 0.0, 0.0, 0.0)
        ops.load(55, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(56, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(57, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(58, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(59, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(60, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(61, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(62, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(63, 0.0, 0.0, (-175.0287 * 9.81), 0.0, 0.0, 0.0)
        ops.load(26, 0.0, 0.0, (-192.5315 * 9.81), 0.0, 0.0, 0.0)
        ops.load(5, 0.0, 0.0, (-105.0172 * 9.81), 0.0, 0.0, 0.0)
        ops.load(6, 0.0, 0.0, (-31.9862 * 9.81), 0.0, 0.0, 0.0)
        ops.load(124, 0.0, 0.0, (-63.9725 * 9.81), 0.0, 0.0, 0.0)
        ops.load(125, 0.0, 0.0, (-63.9725 * 9.81), 0.0, 0.0, 0.0)
        ops.load(126, 0.0, 0.0, (-63.9725 * 9.81), 0.0, 0.0, 0.0)
        ops.load(127, 0.0, 0.0, (-63.9725 * 9.81), 0.0, 0.0, 0.0)
        ops.load(9, 0.0, 0.0, (-31.9862 * 9.81), 0.0, 0.0, 0.0)
        ops.load(7, 0.0, 0.0, (-38.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(128, 0.0, 0.0, (-76.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(129, 0.0, 0.0, (-76.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(130, 0.0, 0.0, (-76.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(131, 0.0, 0.0, (-76.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(10, 0.0, 0.0, (-38.0000 * 9.81), 0.0, 0.0, 0.0)
        ops.load(8, 0.0, 0.0, (-21.5275 * 9.81), 0.0, 0.0, 0.0)
        ops.load(132, 0.0, 0.0, 0, (-43.0550 * 9.81), 0.0, 0.0)
        ops.load(133, 0.0, 0.0, 0, (-43.0550 * 9.81), 0.0, 0.0)
        ops.load(134, 0.0, 0.0, 0, (-43.0550 * 9.81), 0.0, 0.0)
        ops.load(135, 0.0, 0.0, 0, (-43.0550 * 9.81), 0.0, 0.0)
        ops.load(11, 0.0, 0.0, (-21.5275 * 9.81), 0.0, 0.0, 0.0)
        ops.load(157, 0.0, 0.0, (-76.3226 * 9.81), 0.0, 0.0, 0.0)
        ops.load(163, 0.0, 0.0, (-80.0790 * 9.81), 0.0, 0.0, 0.0)
        ops.load(156, 0.0, 0.0, (-76.3226 * 9.81), 0.0, 0.0, 0.0)
        ops.load(162, 0.0, 0.0, (-80.0790 * 9.81), 0.0, 0.0, 0.0)
        ops.load(159, 0.0, 0.0, (-153 * 9.81), 0.0, 0.0, 0.0)
        ops.load(165, 0.0, 0.0, (-153 * 9.81), 0.0, 0.0, 0.0)

        ops.numberer('RCM')
        ops.constraints('Plain')
        ops.system('UmfPack')
        ops.test('RelativeNormDispIncr', 0.001, 500, 2)
        ops.algorithm('Newton')
        NstepGravity = 5
        DGravity = 1. / NstepGravity
        ops.integrator('LoadControl', DGravity)
        ops.analysis('Static')
        ops.analyze(NstepGravity)
        ops.loadConst('-time', 0.0)
        ops.save(0)

    def solve_transient(self):
        ops.wipeAnalysis()
        ops.wipeAnalysis()
        ops.remove('sp', 160, self.eqDirection)
        ops.remove('sp', 166, self.eqDirection)
        ops.remove('sp', 1560, self.eqDirection)
        ops.remove('sp', 1620, self.eqDirection)
        ops.remove('sp', 182000, self.eqDirection)
        ops.remove('sp', 183000, self.eqDirection)
        ops.remove('sp', 184000, self.eqDirection)
        ops.remove('sp', 185000, self.eqDirection)
        ops.remove('sp', 90, self.eqDirection)
        ops.remove('sp', 100, self.eqDirection)
        ops.remove('sp', 110, self.eqDirection)
        a0 = 0.0  # 0.2282461
        a1 = 0.01091939  #0.0
        ops.rayleigh(a0, 0.0, a1, 0.0)
        ops.constraints('Penalty', 1e+12, 1e+12)
        ops.test('RelativeNormDispIncr', 0.01, 100)
        ops.system('FullGeneral')
        ops.numberer('RCM')
        ops.integrator('Newmark', 0.5, 0.25)
        ops.algorithm('Newton')
        ops.analysis('Transient')

        ops.timeSeries('Path', 10, '-values', *self.eqValues, '-time', *self.eqTime, '-predendZero')
        ops.pattern('MultipleSupport', 21313)
        ops.groundMotion(1, 'Plain', '-accel', 10, 'Trapezoidal', '-fact', 9.81)
        ops.imposedMotion(160, self.eqDirection, 1)
        ops.imposedMotion(166, self.eqDirection, 1)
        ops.imposedMotion(1560, self.eqDirection, 1)
        ops.imposedMotion(1620, self.eqDirection, 1)
        ops.imposedMotion(182000, self.eqDirection, 1)
        ops.imposedMotion(183000, self.eqDirection, 1)
        ops.imposedMotion(184000, self.eqDirection, 1)
        ops.imposedMotion(185000, self.eqDirection, 1)
        ops.imposedMotion(90, self.eqDirection, 1)
        ops.imposedMotion(100, self.eqDirection, 1)
        ops.imposedMotion(110, self.eqDirection, 1)

        AnalTag = 0
        x_p_l_n = []
        y_p_l_n = []
        z_p_l_n = []
        x_p_l_n1 = []
        y_p_l_n1 = []
        z_p_l_n1 = []
        x_p_r_n = []
        y_p_r_n = []
        z_p_r_n = []
        x_p_r_n1 = []
        y_p_r_n1 = []
        z_p_r_n1 = []

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

        x_a_l_n = []
        y_a_l_n = []
        z_a_l_n = []
        x_a_r_n = []
        y_a_r_n = []
        z_a_r_n = []
        x_a_l_n1 = []
        y_a_l_n1 = []
        z_a_l_n1 = []
        x_a_r_n1 = []
        y_a_r_n1 = []
        z_a_r_n1 = []

        for index, nodeID in enumerate(self.controlPoints):
            rotations = np.zeros(3)
            rotations[0] = ops.nodeDisp(nodeID, 4)
            rotations[1] = ops.nodeDisp(nodeID, 5)
            rotations[2] = ops.nodeDisp(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativePosition(ry, rz, rotations)
            x_p_l_n.append(self.deckXCon[index] + ops.nodeDisp(nodeID, 1) + relative[0])
            y_p_l_n.append(self.deckYCon[index] + ops.nodeDisp(nodeID, 2) + relative[1])
            z_p_l_n.append(self.deckZCon[index] + ops.nodeDisp(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativePosition(ry, rz, rotations)
            x_p_r_n.append(self.deckXCon[index] + ops.nodeDisp(nodeID, 1) + relative[0])
            y_p_r_n.append(self.deckYCon[index] + ops.nodeDisp(nodeID, 2) + relative[1])
            z_p_r_n.append(self.deckZCon[index] + ops.nodeDisp(nodeID, 3) + relative[2])

        for index, nodeID in enumerate(self.controlPoints):
            rotations = np.zeros(3)
            rotations[0] = ops.nodeVel(nodeID, 4)
            rotations[1] = ops.nodeVel(nodeID, 5)
            rotations[2] = ops.nodeVel(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativeVelocity(ry, rz, rotations)
            x_v_l_n.append(ops.nodeVel(nodeID, 1) + relative[0])
            y_v_l_n.append(ops.nodeVel(nodeID, 2) + relative[1])
            z_v_l_n.append(ops.nodeVel(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativeVelocity(ry, rz, rotations)
            x_v_r_n.append(ops.nodeVel(nodeID, 1) + relative[0])
            y_v_r_n.append(ops.nodeVel(nodeID, 2) + relative[1])
            z_v_r_n.append(ops.nodeVel(nodeID, 3) + relative[2])

        for nodeID in self.controlPoints:
            rotations = np.zeros(3)
            rotations[0] = ops.nodeAccel(nodeID, 4)
            rotations[1] = ops.nodeAccel(nodeID, 5)
            rotations[2] = ops.nodeAccel(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativeAccel(ry, rz, rotations)
            x_a_l_n.append(ops.nodeAccel(nodeID, 1) + relative[0])
            y_a_l_n.append(ops.nodeAccel(nodeID, 2) + relative[1])
            z_a_l_n.append(ops.nodeAccel(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativeAccel(ry, rz, rotations)
            x_a_r_n.append(ops.nodeAccel(nodeID, 1) + relative[0])
            y_a_r_n.append(ops.nodeAccel(nodeID, 2) + relative[1])
            z_a_r_n.append(ops.nodeAccel(nodeID, 3) + relative[2])

        tCurrent = ops.getTime()
        tFinal = tCurrent + 4 * self.dt
        while AnalTag == 0 and tCurrent < (tFinal - self.dt/4):
            AnalTag = ops.analyze(1, self.dt)
            tCurrent = ops.getTime()
            #print(bcolors.OKCYAN + "Bridge Current Time: {:.2f}sec".format(tCurrent) + bcolors.ENDC)

        for index, nodeID in enumerate(self.controlPoints):
            rotations = np.zeros(3)
            rotations[0] = ops.nodeDisp(nodeID, 4)
            rotations[1] = ops.nodeDisp(nodeID, 5)
            rotations[2] = ops.nodeDisp(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativePosition(ry, rz, rotations)
            x_p_l_n1.append(self.deckXCon[index] + ops.nodeDisp(nodeID, 1) + relative[0])
            y_p_l_n1.append(self.deckYCon[index] + ops.nodeDisp(nodeID, 2) + relative[1])
            z_p_l_n1.append(self.deckZCon[index] + ops.nodeDisp(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativePosition(ry, rz, rotations)
            x_p_r_n1.append(self.deckXCon[index] + ops.nodeDisp(nodeID, 1) + relative[0])
            y_p_r_n1.append(self.deckYCon[index] + ops.nodeDisp(nodeID, 2) + relative[1])
            z_p_r_n1.append(self.deckZCon[index] + ops.nodeDisp(nodeID, 3) + relative[2])

        for nodeID in self.controlPoints:
            rotations = np.zeros(3)
            rotations[0] = ops.nodeVel(nodeID, 4)
            rotations[1] = ops.nodeVel(nodeID, 5)
            rotations[2] = ops.nodeVel(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativeVelocity(ry, rz, rotations)
            x_v_l_n1.append(ops.nodeVel(nodeID, 1) + relative[0])
            y_v_l_n1.append(ops.nodeVel(nodeID, 2) + relative[1])
            z_v_l_n1.append(ops.nodeVel(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativeVelocity(ry, rz, rotations)
            x_v_r_n1.append(ops.nodeVel(nodeID, 1) + relative[0])
            y_v_r_n1.append(ops.nodeVel(nodeID, 2) + relative[1])
            z_v_r_n1.append(ops.nodeVel(nodeID, 3) + relative[2])

        for nodeID in self.controlPoints:
            rotations = np.zeros(3)
            rotations[0] = ops.nodeAccel(nodeID, 4)
            rotations[1] = ops.nodeAccel(nodeID, 5)
            rotations[2] = ops.nodeAccel(nodeID, 6)
            ry, rz = -1.14, 2.44
            relative = relativeAccel(ry, rz, rotations)
            x_a_l_n1.append(ops.nodeAccel(nodeID, 1) + relative[0])
            y_a_l_n1.append(ops.nodeAccel(nodeID, 2) + relative[1])
            z_a_l_n1.append(ops.nodeAccel(nodeID, 3) + relative[2])
            ry, rz = -3.06, 2.44
            relative = relativeAccel(ry, rz, rotations)
            x_a_r_n1.append(ops.nodeAccel(nodeID, 1) + relative[0])
            y_a_r_n1.append(ops.nodeAccel(nodeID, 2) + relative[1])
            z_a_r_n1.append(ops.nodeAccel(nodeID, 3) + relative[2])

        # Calculate r
        R = []
        counter = 0
        mult = 1
        for i in range(0, len(self.controlPoints), 6):
            if i > 0:
                counter = i - 1 * mult
                mult += 1
            length = ops.nodeCoord(self.controlPoints[counter + 5],1) - ops.nodeCoord(self.controlPoints[counter],1)
            d2 = y_p_r_n1[counter + 2] - ((3 / 7) * y_p_r_n1[counter + 5]) - ((4 / 7) * y_p_r_n1[counter])
            d3 = y_p_r_n1[counter + 3] - ((3 / 7) * y_p_r_n1[counter]) - ((4 / 7) * y_p_r_n1[counter + 5])
            delta_max = max(abs(d2), abs(d3))
            if delta_max < 0.000001:
                delta_max = 0.000001
            r = (length ** 2) / (8 * delta_max)
            R.append(r)

        r2return = min(R)

        maxVerticalAccel = max(z_a_r_n1)
        minVerticalAccel = min(z_a_r_n1)
        verticalAccel = max(abs(maxVerticalAccel), abs(minVerticalAccel))
        ops.save(1)

        # print("Bridge SOLVED")
        return x_p_l_n, y_p_l_n, z_p_l_n, x_p_l_n1, y_p_l_n1, z_p_l_n1, x_p_r_n, y_p_r_n, z_p_r_n, x_p_r_n1, y_p_r_n1, z_p_r_n1, \
               x_v_l_n, y_v_l_n, z_v_l_n, x_v_l_n1, y_v_l_n1, z_v_l_n1, x_v_r_n, y_v_r_n, z_v_r_n, x_v_r_n1, y_v_r_n1, z_v_r_n1, verticalAccel, r2return

    def getGroundDisp(self):
        return [ops.nodeDisp(90, 1), ops.nodeDisp(90, 2), ops.nodeDisp(90, 3)]

    def update(self, eqDirection, pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX, abutmntY):
        ops.restore(1)
        ops.save(0)

        pierTopsDict["Node6"].append(ops.nodeDisp(6, eqDirection))
        pierTopsDict["Node7"].append(ops.nodeDisp(7, eqDirection))
        pierTopsDict["Node8"].append(ops.nodeDisp(8, eqDirection))

        pierBaseDict["Node9"].append(ops.nodeDisp(9, eqDirection))
        pierBaseDict["Node10"].append(ops.nodeDisp(10, eqDirection))
        pierBaseDict["Node11"].append(ops.nodeDisp(11, eqDirection))

        bearingTop["Node136"].append(ops.nodeDisp(136, eqDirection))
        bearingTop["Node138"].append(ops.nodeDisp(138, eqDirection))
        bearingTop["Node140"].append(ops.nodeDisp(140, eqDirection))
        bearingTop["Node142"].append(ops.nodeDisp(142, eqDirection))

        bearingTop["Node144"].append(ops.nodeDisp(144, eqDirection))  # M1
        bearingTop["Node146"].append(ops.nodeDisp(146, eqDirection))  # M1
        bearingTop["Node148"].append(ops.nodeDisp(148, eqDirection))  # M2
        bearingTop["Node150"].append(ops.nodeDisp(150, eqDirection))  # M2
        bearingTop["Node152"].append(ops.nodeDisp(152, eqDirection))  # M2
        bearingTop["Node154"].append(ops.nodeDisp(154, eqDirection))  # M2

        bearingBase["Node137"].append(ops.nodeDisp(137, eqDirection))
        bearingBase["Node139"].append(ops.nodeDisp(139, eqDirection))
        bearingBase["Node141"].append(ops.nodeDisp(141, eqDirection))
        bearingBase["Node143"].append(ops.nodeDisp(143, eqDirection))

        bearingBase["Node145"].append(ops.nodeDisp(145, eqDirection))  # M1
        bearingBase["Node147"].append(ops.nodeDisp(147, eqDirection))  # M1
        bearingBase["Node149"].append(ops.nodeDisp(149, eqDirection))  # M2
        bearingBase["Node151"].append(ops.nodeDisp(151, eqDirection))  # M2
        bearingBase["Node153"].append(ops.nodeDisp(153, eqDirection))  # M3
        bearingBase["Node155"].append(ops.nodeDisp(155, eqDirection))  # M3

        abutmntX["Node156"].append(ops.nodeDisp(156, eqDirection))
        abutmntX["Node157"].append(ops.nodeDisp(157, eqDirection))
        abutmntX["Node162"].append(ops.nodeDisp(162, eqDirection))
        abutmntX["Node163"].append(ops.nodeDisp(163, eqDirection))
        abutmntX["Node1560"].append(ops.nodeDisp(1560, eqDirection))
        abutmntX["Node1620"].append(ops.nodeDisp(1620, eqDirection))

        abutmntY["Node182"].append(ops.nodeDisp(182, eqDirection))
        abutmntY["Node182000"].append(ops.nodeDisp(182000, eqDirection))
        abutmntY["Node183"].append(ops.nodeDisp(183, eqDirection))
        abutmntY["Node183000"].append(ops.nodeDisp(183000, eqDirection))
        abutmntY["Node184"].append(ops.nodeDisp(184, eqDirection))
        abutmntY["Node184000"].append(ops.nodeDisp(184000, eqDirection))
        abutmntY["Node185"].append(ops.nodeDisp(185, eqDirection))
        abutmntY["Node185000"].append(ops.nodeDisp(185000, eqDirection))

        return pierTopsDict, pierBaseDict, bearingTop, bearingBase, abutmntX, abutmntY


if __name__ == '__main__':
    pass
