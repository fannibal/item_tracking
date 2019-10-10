#!/usr/bin/env python
# coding: utf-8

from item_tracking import Tracker, Item
import numpy as np
import time


def print_items(tracker):
    print("##########################################")
    print("Second time items are seen: \n")
    print("# new items")
    for Item in tracker.newItems:
        print(Item)
        print(Item.getID(), Item.x, Item.dx)
    print("# tracked items")
    for trackedItem in tracker.trackedItems:
        print(trackedItem.getID(), trackedItem.x, trackedItem.dx)
        print(trackedItem.components)


if __name__ == "__main__":
    tracker = Tracker()
    componentList = []
    components_names = [i for i in range(1,6)]

    for t in np.arange(0,10,0.1):  # 5 components moving on the x axis
        componentList.append([])
        xs = 2+t, 1+t, t, 1-t, 2-t
        for i in range(5):
            componentList[-1].append(xs[i])

    # 2 items with the same components, same movement
    now = time.time()
    for k in range(2):
        item = Item(lastTimeSeen=now)
        for xi in componentList:
            for i in range(5):
                item.setComponent(components_names[i], x=xi[i], y=0, z=0, rx=0, ry=0, rz=0)
        tracker.addItem(item)
    tracker.updateTracking()
    print_items(tracker)

    now += time.time()
    for k in range(2):
        item = Item(lastTimeSeen=now)
        for xi in componentList:
            for i in range(5):
                item.setComponent(components_names[i], x=xi[i]+1, y=0, z=0, rx=0, ry=0, rz=0)
        tracker.addItem(item)
    tracker.updateTracking()
    print_items(tracker)







