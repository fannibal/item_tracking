#!/usr/bin/env python
# coding: utf-8

import time
import numpy as np
from scipy.stats import circmean


class ItemHandler(object):

    NEW = 1
    UPDATE = 2
    LOST = 3

    def __init__(self, lastTimeSeen=None):
        self.ID = None
        self.lastTimeSeen = lastTimeSeen
        self.state = self.NEW


class Component(object):

    UNKNOWN = 0
    ON_SIGHT = 1

    def __init__(self, name=""):
        self.name = name
        self.x = None
        self.y = None
        self.z = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.size = None
        self.distWeight = 1.
        self.baryWeight = 1.
        self.speedWeight = 1.
        self.smooth_poses = False
        self.smoothing_coeff = 1.
        self.smoothed_x = None
        self.smoothed_y = None
        self.smoothed_z = None
        self.dx = None
        self.dy = None
        self.dz = None
        self.acceleration = None
        self.status = self.ON_SIGHT
        self.parents = []

    def updatePose(self, x=None, y=None, z=None):
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z

    def updateSmoothedPose(self, other):
        try:
            self.smoothed_x = self.smoothing_coeff*self.x + (1 - self.smoothing_coeff)*other.smoothed_x
            self.smoothed_y = self.smoothing_coeff*self.y + (1 - self.smoothing_coeff)*other.smoothed_y
            self.smoothed_z = self.smoothing_coeff*self.z + (1 - self.smoothing_coeff)*other.smoothed_z
        except TypeError as e:
            print("TypeError in updateSmoothedPose::{}".format(e))
            if self.x is None:
                self.smoothed_x, self.smoothed_y, self.smoothed_z = other.smoothed_x, other.smoothed_y, other.smoothed_z
                print("     Updating smoothed pose as former smoothed position")
            if other.smoothed_x is None:
                print("     Updating smoothed pose as current position")

    def updateOrientation(self, rx=None, ry=None, rz=None):
        if rx is not None:
            self.rx = rx
        if ry is not None:
            self.ry = ry
        if rz is not None:
            self.rz = rz

    def updateWeights(self, distWeight=None, baryWeight=None, speedWeight=None):
        if distWeight is not None:
            self.distWeight = distWeight
        if baryWeight is not None:
            self.baryWeight = baryWeight
        if speedWeight is not None:
            self.speedWeight = speedWeight

    def updateParents(self, parents=None):
        if parents is not None:
            self.parents = parents

    def fullUpdate(self, **kwargs):
        x, y, z = kwargs.get('x', None), kwargs.get('y', None), kwargs.get('z', None)
        rx, ry, rz = kwargs.get('rx', None), kwargs.get('ry', None), kwargs.get('rz', None)
        distWeight, baryWeight, speedWeight = kwargs.get('distWeight', None), kwargs.get('baryWeight', None), kwargs.get('speedWeight', None)
        parents = kwargs.get('parents', None)

        self.updatePose(x, y, z)
        self.updateOrientation(rx, ry, rz)
        self.updateWeights(distWeight, baryWeight, speedWeight)
        self.updateParents(parents)

    def fullGet(self):
        return self.x, self.y, self.z, self.rx, self.ry, self.rz

    def setSmoothedPose(self):
        if self.x is not None:
            self.smoothed_x = self.x
        if self.y is not None:
            self.smoothed_y = self.y
        if self.z is not None:
            self.smoothed_z = self.z

    def dist(self, other):
        if self.smooth_poses:
            dx = self.smoothed_x - other.smoothed_x
            dy = self.smoothed_y - other.smoothed_y
            dz = self.smoothed_z - other.smoothed_z
        else:
            dx = self.x - other.x
            dy = self.y - other.y
            dz = self.z - other.z
        return dx, dy, dz

    def dist2(self, other):
        dx, dy, dz = self.dist(other)
        x2 = pow(dx, 2)
        y2 = pow(dy, 2)
        z2 = pow(dz, 2)
        return np.sqrt(x2 + y2 + z2)

    def resetSpeed(self):
        self.dx, self.dy, self.dz = None, None, None  # no speed


class Item(Component):

    COMPONENTS = 1
    POINTCLOUD = 2

    def __init__(self, name="", lastTimeSeen=None):
        Component.__init__(self, name=name)
        self.itemHandler = ItemHandler(lastTimeSeen=lastTimeSeen)
        self.ref = self.COMPONENTS
        self.components = dict()
        self.pointCloud = None
        self.other = dict()  # dict of other characteristic (?)
        self.smooth_components_poses = False
        self.components_smoothing_coeff = False
        self.compute_speed_from_components_speeds = False

    def setID(self, ID):
        self.itemHandler.ID = ID

    def getID(self):
        return self.itemHandler.ID

    def setState(self, state):
        self.itemHandler.state = state

    def getState(self):
        return self.itemHandler.state

    def setTime(self, lastSeen):
        self.itemHandler.lastTimeSeen = lastSeen

    def getTime(self):
        return self.itemHandler.lastTimeSeen

    def setComponent(self, name, **kwargs):
        """
        :param name: name of the component to add
        :param kwargs: x=0.5, ry=0.11, ...
        """
        self.components.setdefault(name, Component(name=name)).fullUpdate(**kwargs)
        self.components[name].smoothing_coeff = self.components_smoothing_coeff
        self.components[name].smooth_poses = self.smooth_components_poses
        self.components[name].setSmoothedPose()

    def setBarycenter(self):
        if self.ref == self.COMPONENTS and len(self.components):
            _baryWeight = 0
            _smoothed_barycenter = 0
            _x, _y, _z = 0, 0, 0
            _sx, _sy, _sz = 0, 0, 0
            for name, component in self.components.items():
                if component.status == self.ON_SIGHT:
                    try:
                        if self.smooth_components_poses:
                            _scx, _scy, _scz = [p * component.baryWeight for p in (component.smoothed_x, component.smoothed_y, component.smoothed_z)]
                            _sx += _scx
                            _sy += _scy
                            _sz += _scz
                        _cx, _cy, _cz = [p * component.baryWeight for p in (component.x, component.y, component.z)]
                        _x += _cx
                        _y += _cy
                        _z += _cz
                        _baryWeight += component.baryWeight
                    except TypeError as e:
                        print("{} : forgot to setup position of component '{}'".format(e, name))
            if _baryWeight != 0:
                self.x, self.y, self.z = _x, _y, _z
                self.x /= _baryWeight
                self.y /= _baryWeight
                self.z /= _baryWeight
                self.baryWeight = _baryWeight
                if self.smooth_components_poses:
                    self.smoothed_x, self.smoothed_y, self.smoothed_z = _sx, _sy, _sz
                    self.smoothed_x /= _baryWeight
                    self.smoothed_y /= _baryWeight
                    self.smoothed_z /= _baryWeight
                if self.smooth_poses:
                    self.setSmoothedPose()
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def setOrientation(self):
        if self.ref == self.COMPONENTS and len(self.components):
            angleList = [[], [], []]
            for name, component in self.components.items():
                if component.status == self.ON_SIGHT:
                    try:
                        _crx, _cry, _crz = component.rx * 1., component.ry * 1., component.rz * 1.
                        angleList[0].append(_crx)
                        angleList[1].append(_cry)
                        angleList[2].append(_crz)
                    except TypeError as e:
                        print("{} : forgot to setup orientation of component '{}'".format(e, name))
            self.rx = circmean(angleList[0])
            self.ry = circmean(angleList[1])
            self.rz = circmean(angleList[2])
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def setSize(self):
        pass  # How?

    def setSpeed(self, old_body=None):
        if self.compute_speed_from_components_speeds:
            if self.ref == self.COMPONENTS and len(self.components):
                _speedWeight = 0
                _dx, _dy, _dz = 0, 0, 0
                for name, component in self.components.items():
                    if component.status == self.ON_SIGHT:
                        try:
                            _cdx, _cdy, _cdz = [v * component.speedWeight for v in (component.dx, component.dy, component.dz)]
                            _dx += _cdx
                            _dy += _cdy
                            _dz += _cdz
                            _speedWeight += component.speedWeight
                        except TypeError as e:
                            # print("{} : forgot to setup speed of component '{}'".format(e, name))
                            pass
                if _speedWeight != 0:
                    self.dx, self.dy, self.dz = _dx, _dy, _dz
                    self.dx /= _speedWeight
                    self.dy /= _speedWeight
                    self.dz /= _speedWeight
                    self.speedWeight = _speedWeight
            elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
                pass  # TODO
            else:
                print("empty item, fill it first")
        else:
            dx, dy, dz = self.dist(old_body)
            deltaTime = self.getTime() - old_body.getTime()
            if deltaTime != 0:
                self.dx = dx / deltaTime
                self.dy = dy / deltaTime
                self.dz = dz / deltaTime

    def resetItemSpeed(self):
        if self.ref == self.COMPONENTS and len(self.components):
            self.resetSpeed()
            for component in self.components.values():
                component.resetSpeed()
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def getParents(self):
        if self.ref == self.COMPONENTS and len(self.components):
            names = []
            L = dict()
            for component in self.components.values():
                if component.status == self.ON_SIGHT:
                    names.append(component.name)
            for compName in names:
                for parent in self.components[compName].parents:
                    if parent in names:
                        L.setdefault(compName, []).append(parent)
            return L
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def __eq__(self, other):
        """
        :param other: Item
        :return: distance between two items
        """
        if self.ref == self.COMPONENTS:
            d = 0  # init mean distance between items
            self.distWeight = 0  # init distWeight
            for selfComp in self.components.values():  # for each body part of the old skeletton
                if selfComp.name in other.components.keys():
                    otherComp = other.components[selfComp.name]
                    if selfComp.status == Component.ON_SIGHT and otherComp.status == Component.ON_SIGHT:
                        meanWeight = (selfComp.distWeight + otherComp.distWeight)/2.
                        d += selfComp.dist2(otherComp) * meanWeight
                        self.distWeight += meanWeight
            if self.distWeight != 0:
                d /= self.distWeight
                return d
            else:
                return np.inf
        elif self.ref == self.POINTCLOUD:
            pass  # TODO

    def __lt__(self, other):
        """
        :param other: Item
        Update self from other
        """
        self.setID(other.getID())
        if self.smooth_poses:
            self.updateSmoothedPose(other)
        for oldComp in other.components.values():
            name = oldComp.name
            if name in self.components.keys():  # speed relevant
                if oldComp.status == Component.ON_SIGHT:
                    if self.smooth_components_poses:
                        self.components[name].updateSmoothedPose(oldComp)
                    dx, dy, dz = self.components[name].dist(oldComp)
                    deltaTime = self.getTime() - other.getTime()
                    if deltaTime != 0:
                        self.components[name].dx = dx / deltaTime
                        self.components[name].dy = dy / deltaTime
                        self.components[name].dz = dz / deltaTime
                else:
                    self.components[name].dx = None
                    self.components[name].dy = None
                    self.components[name].dz = None
            else:  # keep track record even if lost
                oldComp.status = Component.UNKNOWN
                self.components[name] = oldComp

    def __gt__(self, other):
        """
        :param other: Item
        """
        _ = other < self
