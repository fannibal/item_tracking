#!/usr/bin/env python
# coding: utf-8

import time
import numpy as np
from .item import Item, ItemHandler


class Tracker(object):

    def __init__(self):
        self.trackedItems = []
        self.newItems = []
        self.distance = []
        self.maxID = 1
        # default args
        self.thresholdDist = 1.
        self.time_add = 1.
        self.time_del = 1.
        self.now = self.nowTime
        self.compute_speed_from_components_speeds = False
        self.with_items_pose_smoothing = False
        self.with_components_pose_smoothing = False
        self.components_pose_smoothing_coeff = 1.
        self.items_pose_smoothing_coeff = 1.

    def setParams(self, **kwargs):
        self.thresholdDist = kwargs.get('thresholdDist', self.thresholdDist)
        self.time_add = kwargs.get('time_add', self.time_add)
        self.time_del = kwargs.get('time_del', self.time_del)
        self.now = kwargs.get('nowTime', self.now)
        self.components_pose_smoothing_coeff = kwargs.get('components_pose_smoothing_coeff', self.components_pose_smoothing_coeff)
        self.items_pose_smoothing_coeff = kwargs.get('items_pose_smoothing_coeff', self.items_pose_smoothing_coeff)
        self.with_items_pose_smoothing = kwargs.get('with_items_pose_smoothing', self.with_items_pose_smoothing)
        self.with_components_pose_smoothing = kwargs.get('with_components_pose_smoothing', self.with_components_pose_smoothing)
        self.compute_speed_from_components_speeds = kwargs.get('compute_speed_from_components_speeds', self.compute_speed_from_components_speeds)

    def nowTime(self):
        return time.time()

    def addItem(self, item):
        item.smoothing_coeff = self.items_pose_smoothing_coeff
        item.smooth_poses = self.with_items_pose_smoothing
        item.smooth_components_poses = self.with_components_pose_smoothing
        item.components_smoothing_coeff = self.components_pose_smoothing_coeff
        item.speed_from_component_speed = self.compute_speed_from_components_speeds
        item.setSmoothedPose()
        self.newItems.append(item)

    def updateTracking(self):
        self.barycenters()
        self.distanceCompute()
        new, updating, lost = self.matchingDistanceDecider()
        self.addTracks(new)
        self.updateTracks(updating)
        self.deleteTracks(lost)
        self.trackedItems = self.newItems
        self.newItems = []

    def barycenters(self):
        for item in self.newItems:
            item.setBarycenter()

    def distanceCompute(self):
        self.distance = []
        for newItem in self.newItems:
            self.distance.append([])
            for oldItem in self.trackedItems:
                self.distance[-1].append(newItem == oldItem)

    def matchingDistanceDecider(self):
        toUpdate = []
        checked = [[], []]  # first list for old ones, second for new ones

        # min matching ("greedy" algorithm, not "optimal" algorithm)
        lSize = len(self.newItems)
        cSize = len(self.trackedItems)
        if lSize * cSize:
            mini = np.nanmin(self.distance)
            while mini < self.thresholdDist:  # not too far matching (later -> time/size relation?)
                p = np.argmin(self.distance)
                l, c = p//cSize, p % cSize  # new skeletton l, old skeletton c
                toUpdate.append([c, l])
                checked[0].append(c)
                checked[1].append(l)
                for i in range(cSize):  # avoid using same new skeletton twice
                    self.distance[l][i] = np.inf
                for j in range(lSize):  # avoid using same old skeletton twice
                    self.distance[j][c] = np.inf
                mini = np.nanmin(self.distance)

        # no matching skelettons
        toDelete = [elem for elem in range(cSize) if elem not in checked[0]]
        toAdd = [elem for elem in range(lSize) if elem not in checked[1]]

        return toAdd, toUpdate, toDelete

    def addTracks(self, toAdd):
        for newTrack in toAdd:
            self.newItems[newTrack].setID(self.maxID)
            self.newItems[newTrack].setTime(self.now())
            self.maxID += 1

    def updateTracks(self, toUpdate):
        for old, new in toUpdate:
            elder, youngster = self.trackedItems[old], self.newItems[new]
            if elder.getState() == ItemHandler.NEW:
                now = self.now()
                if (now - elder.getTime()) >= self.time_add:
                    youngster.setTime(now)
                    youngster.setState(ItemHandler.UPDATE)
                else:
                    youngster.setTime(elder.getTime())
            elif elder.getState() == ItemHandler.UPDATE:
                youngster.setTime(self.now())
                youngster.setState(ItemHandler.UPDATE)
            else:
                youngster.setState(ItemHandler.UPDATE)
                youngster.setTime(self.now())
            _ = elder > youngster
            youngster.setSpeed(old_body=elder)

    def deleteTracks(self, toDelete):
        for old in toDelete:
            delItem = self.trackedItems[old]
            delItem.resetItemSpeed()
            if delItem.getState() == ItemHandler.NEW:  # ADD case : suppress entry
                delItem.setState(ItemHandler.LOST)
                delItem.status = Item.UNKNOWN
            elif delItem.getState() == ItemHandler.UPDATE:  # UPDATE case : change status
                delItem.setState(ItemHandler.LOST)
                delItem.status = Item.UNKNOWN
                self.addItem(delItem)
            else:  # DELETE case : if gone for too long, suppress entry
                if (self.now() - delItem.getTime()) < self.time_del:
                    self.addItem(delItem)


if __name__ == "__main__":
    tracker = Tracker()
    itemList = []
    for t in np.arange(0,10,0.1):  # 2 items move on the x axis
        itemList.append([])
        i1, i2 = 2+t, 1-t
        itemList[-1].append(i1)
        itemList[-1].append(i2)

    for k in range(len(itemList)):
        for it in itemList[k]:
            item = Item()
            item.setComponent("onlyComp", x=it, y=0, z=0, rx=0, ry=0, rz=0)
            tracker.addItem(item)
        tracker.updateTracking()
        for trackedItem in tracker.trackedItems:
            print(trackedItem.getID(), trackedItem.x)
