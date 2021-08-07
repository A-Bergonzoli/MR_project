#!/usr/bin/env python3

from collections import deque


class Queue:

    def __init__(self):
        self.queue = deque()

    def enQ(self, val):
        self.queue.appendleft(val)

    def deQ(self):
        return self.queue.pop()

    def peek_first(self):
        return self.queue[-1]

    def is_empty(self):
        return len(self.queue) == 0

    def size(self):
        return len(self.queue)
