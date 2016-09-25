__author__ = 'Abhishek'
# This is an implementation of a priority queue where one can change the priority of a task
import itertools
import heapq

#import pacmanSearch


class customPriorityQueue():
    pq = []                         # list of entries arranged in a heap
    entry_finder = {}               # mapping of tasks to entries
    REMOVED = '<removed-task>'      # placeholder for a removed task
    counter = itertools.count()     # unique sequence count
    queue = []

    def put(self, priority, task):
        'Add a new task or update the priority of an existing task'
        if task in self.entry_finder:
            self.remove_task(task)
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heapq.heappush(self.pq, entry)
        self.queue = [x[2] for x in self.pq]

    def remove_task(self, task):
        'Mark an existing task as REMOVED.  Raise KeyError if not found.'
        entry = self.entry_finder.pop(task)
        entry[-1] = self.REMOVED

    def get(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            self.queue = [x[2] for x in self.pq]
            if task is not self.REMOVED:
                del self.entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        for element in self.pq:
            if element[2] != self.REMOVED:
                return False
        return True

'''    def queueWithLocs(self, dimensions):
        output = []
        for element in self.pq:
            if element[2] != self.REMOVED:
                output.append([element[0], pacmanSearch.pacmanID2Loc(element[2], dimensions)])
        return output
'''
if __name__ == '__main__':
    a = customPriorityQueue()
    a.put(1,'a')
    a.put(10,'a')
    a.put(2,'b')
    print a.queue