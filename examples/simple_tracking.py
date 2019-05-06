#!/usr/bin/env python
# coding: utf-8

from item_tracking import Tracker, Item
import numpy as np

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
            item.addComponent("only1Comp", x=it, y=0, z=0, rx=0, ry=0, rz=0)
            tracker.addItem(item)
        tracker.updateTracking()
        for trackedItem in tracker.trackedItems:
            print(trackedItem.getID(), trackedItem.x)
