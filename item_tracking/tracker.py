#!/usr/bin/env python
# coding: utf-8

import time
import numpy as np
from item import Item, ItemHandler


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

    def setParams(self, **kwargs):
        self.thresholdDist = kwargs.get('thresholdDist', self.thresholdDist)
        self.time_add = kwargs.get('time_add', self.time_add)
        self.time_del = kwargs.get('time_del', self.time_del)
        self.now = kwargs.get('nowTime', self.now)

    def nowTime(self):
        return time.time()

    def addItem(self, item):
        self.newItems.append(item)

    def updateTracking(self):
        self.barycenters()
        self.distanceCompute()
        toAdd, toUpdate, toDelete = self.matchingDistanceDecider()
        self.addTracks(toAdd)
        self.updateTracks(toUpdate)
        self.deleteTracks(toDelete)
        self.trackedItems = self.newItems

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
                    self.distance[l][i] = np.nan
                for j in range(lSize):  # avoid using same old skeletton twice
                    self.distance[j][c] = np.nan
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
            if self.trackedItems[old].getState() == ItemHandler.ADD:
                now = self.now()
                if (now - self.trackedItems[old].getTime()) >= self.time_add:
                    self.newItems[new].setTime(now)
                    self.newItems[new].setState(ItemHandler.UPDATE)
                else:
                    self.newItems[new].setTime(self.trackedItems[old].getTime())
            elif self.trackedItems[old].getState() == ItemHandler.UPDATE:
                self.newItems[new].setTime(self.now())
            else:
                self.newItems[new].setState(ItemHandler.UPDATE)
                self.newItems[new].setTime(self.now())
            _ = self.trackedItems[old] > self.newItems[new]

    def deleteTracks(self, toDelete):
        for old in toDelete:
            delItem = self.trackedItems[old]
            if delItem.getState() == ItemHandler.ADD:  # ADD case : suppress entry
                delItem.setState(ItemHandler.DELETE)
                delItem.status = Item.UNKNOWN
            elif delItem.getState() == ItemHandler.UPDATE:  # UPDATE case : change status
                delItem.setState(ItemHandler.DELETE)
                delItem.status = Item.UNKNOWN
                self.addItem(delItem)
            else:  # DELETE case : if gone for too long, suppress entry
                if (self.now() - delItem.getTime()) < self.time_del:
                    self.addItem(delItem)


if __name__ == "__main__":
    tracker = Tracker()
    item1 = Item()
    item1.addComponent("maincComp", x=0, y=0, z=0, rx=0, ry=0, rz=0)
    tracker.addItem(item1)
    tracker.updateTracking()
