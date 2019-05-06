#!/usr/bin/env python
# coding: utf-8

import time
import numpy as np
from scipy.stats import circmean


class ItemHandler(object):

    ADD = 1
    UPDATE = 2
    DELETE = 3

    def __init__(self):
        self.ID = None
        self.lastTimeSeen = time.time()
        self.state = self.ADD


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
        self.speed = None
        self.acceleration = None
        self.status = self.ON_SIGHT

    def updatePose(self, x=None, y=None, z=None):
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z

    def updateOrientation(self, rx=None, ry=None, rz=None):
        if rx is not None:
            self.rx = rx
        if ry is not None:
            self.ry = ry
        if rz is not None:
            self.rz = rz

    def fullUpdate(self, **kwargs):
        x, y, z = kwargs.get('x', None), kwargs.get('y', None), kwargs.get('z', None)
        rx, ry, rz = kwargs.get('rx', None), kwargs.get('ry', None), kwargs.get('rz', None)
        self.updatePose(x, y, z)
        self.updateOrientation(rx, ry, rz)

    def fullGet(self):
        return self.x, self.y, self.z, self.rx, self.ry, self.rz

    def dist(self, other):
        x2 = pow(other.x - self.x, 2)
        y2 = pow(other.y - self.y, 2)
        z2 = pow(other.z - self.z, 2)
        return np.sqrt(x2 + y2 + z2)


class Item(Component):

    COMPONENTS = 1
    POINTCLOUD = 2

    def __init__(self, name=""):
        Component.__init__(self, name=name)
        self.itemHandler = ItemHandler()
        self.ref = self.COMPONENTS
        self.components = dict()
        self.pointCloud = None
        self.other = dict()  # dict of other characteristic (?)

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

    def addComponent(self, name, **kwargs):
        """
        :param name: name of the component to add
        :param kwargs: x=0.5, ry=0.11, ...
        """
        self.components.setdefault(name, Component(name=name)).fullUpdate(**kwargs)

    def setBarycenter(self):
        if self.ref == self.COMPONENTS and len(self.components):
            self.baryWeight = 0
            self.x, self.y, self.z = 0, 0, 0
            for component in self.components.values():
                if component.status == self.ON_SIGHT:
                    try:
                        self.x += component.x
                        self.y += component.y
                        self.z += component.z
                        self.baryWeight += component.baryWeight
                    except TypeError as e:
                        print("{} : forgot to setup component".format(e))
            if self.baryWeight != 0:
                self.x /= self.baryWeight
                self.y /= self.baryWeight
                self.z /= self.baryWeight
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def setOrientation(self):
        if self.ref == self.COMPONENTS and len(self.components):
            angleList = [[], [], []]
            self.rx, self.ry, self.rz = 0, 0, 0
            for component in self.components.values():
                if component.status == self.ON_SIGHT:
                    angleList[0].append(component.rx)
                    angleList[1].append(component.ry)
                    angleList[2].append(component.rz)
            self.rx = circmean(angleList[0])
            self.ry = circmean(angleList[1])
            self.rz = circmean(angleList[2])
        elif self.ref == self.POINTCLOUD and self.pointCloud is not None:
            pass  # TODO
        else:
            print("empty item, fill it first")

    def setSize(self):
        pass  # How?

    def setSpeed(self):
        if self.ref == self.COMPONENTS and len(self.components):
            self.speedWeight = 0
            self.speed = 0
            for component in self.components.values():
                if component.status == self.ON_SIGHT and component.speed is not None:
                    self.speed += component.speed
                    self.speedWeight += component.speedWeight
            self.speed /= self.speedWeight
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
                        d += selfComp.dist(otherComp)
                        self.distWeight += selfComp.distWeight
            if self.distWeight != 0:
                d /= self.distWeight
                return d
            else:
                return np.nan
        elif self.ref == self.POINTCLOUD:
            pass  # TODO

    def __lt__(self, other):
        """
        :param other: Item
        Update self from other
        """
        for oldComp in other.components:
            name = oldComp.name
            if name in self.components.keys():  # speed relevant
                if oldComp.status == Component.ON_SIGHT:
                    deltaPose = self.components[name].dist(oldComp)
                    deltaTime = self.getTime() - other.getTime()
                    self.components[name].speed = deltaPose / deltaTime
                else:
                    self.components[name].speed = None
            else:  # keep track record even if lost
                oldComp.status = Component.UNKNOWN
                self.components[name] = oldComp


    def __gt__(self, other):
        """
        :param other: Item
        """
        _ = other < self
