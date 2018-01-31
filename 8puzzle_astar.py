import io
import os
import time
import timeit
from collections import deque
from heapq import heappush, heappop, heapify
from prettytable import PrettyTable

# Author: Kaustubh Hiware
# @kaustubhhiware
# python 8puzzle_astar.py
goal_state = [0, 1, 2, 3, 4, 5, 6, 7, 8]
# can change this goal state
# 4 heuristics used
# h = 0
# Manhattan Distance
# Euclidean Distance
# Misplaced Tiles
heurstics = {0: 'h = 0', 1: 'Manhattan', 2: 'Euclidean', 3:'Misplaced tiles'}

def arr_Equal(s1,s2):
    for i in range(len(s1)):
        if not s1[i] == s2[i]:
            return False
    return True

def print_State(state):
    s = [str(i) for i in state]
    print '+-+-+-+'
    for i in range(3):
        print  '|' + s[3*i] + '|' + s[3*i+1] + '|' + s[3*i+2] + '|'
    print '+-+-+-+'   

class Board:
    def __init__(self, state, parent, move, depth, g, f):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.g = g
        self.f = f

        if self.state:
            self.config = ''.join(str(e) for e in self.state)

    def __eq__(self, other):
        return self.config == other.config

    def __lt__(self, other):
        return self.config < other.config

    def __gt__(self, other):
        return self.config > other.config

goal = Board
initial_state = list()
moves = list()
cost = set()
examined = 0 # nodes examined so far
search_depth = 0
bound = 0

def move(state, direction):
    new_state = state[:]
    index = new_state.index(0)
    if direction == 1:  # Up
        if index in range(0, 3):
            return None
        else:
            new_state[index - 3], new_state[index] = new_state[index], new_state[index - 3]
            return new_state

    if direction == 2:  # Right
        if index in range(2, 9, 3):
            return None
        else:
            new_state[index + 1], new_state[index] = new_state[index], new_state[index + 1]
            return new_state

    if direction == -1:  # Down
        if index in range(6, 9):
            return None
        else:
            new_state[index + 3], new_state[index] = new_state[index], new_state[index + 3]
            return new_state

    if direction == -2:  # Left
        if index in range(0, 9, 3):
            return None
        else:
            new_state[index - 1], new_state[index] = new_state[index], new_state[index - 1]
            return new_state

def search(start_state, threshold, h_i):
    global bound, goal, search_depth, cost
    explored = set()
    stack = list([Board(start_state, None, None, 0, 0, threshold)])
    while stack:
        top = stack.pop()
        explored.add(top.config)
        if top.state == goal_state:
            goal = top
            return stack
        if top.f > threshold:
            cost.add(top.f)
        if top.depth < threshold:
            neighbors = reversed(expand(top))

            for neighbor in neighbors:
                if neighbor.config not in explored:
                    neighbor.f = neighbor.g + h(neighbor.state, h_i)
                    stack.append(neighbor)
                    explored.add(neighbor.config)

                    if neighbor.depth > search_depth:
                        search_depth += 1

            if len(stack) > bound:
                bound = len(stack)

    if cost:
        return min(cost)
    else:
        return 0

def expand(node):
    global examined
    examined += 1 # nodes examined so far
    neighbors = list()
    neighbors.append(Board(move(node.state, 1), node, 1, node.depth + 1, node.g + 1, 0))
    neighbors.append(Board(move(node.state, -1), node, 2, node.depth + 1, node.g + 1, 0))
    neighbors.append(Board(move(node.state, -2), node, 3, node.depth + 1, node.g + 1, 0))
    neighbors.append(Board(move(node.state, 2), node, 4, node.depth + 1, node.g + 1, 0))
    nodes = [neighbor for neighbor in neighbors if neighbor.state]
    return nodes

def Astar(start_state, h_i):
    global bound, goal, search_depth
    explored = set()
    heap = list()
    x = {} # insert into heap
    key = h(start_state, h_i)
    root = Board(start_state, None, None, 0, 0, key)
    entry = (key, 0, root)
    heappush(heap, entry)
    x[root.config] = entry

    while heap:
        head = heappop(heap)
        explored.add(head[2].config)
        if head[2].state == goal_state:
            goal = head[2]
            return heap
        neighbors = expand(head[2])

        for neighbor in neighbors:
            neighbor.f = neighbor.g + h(neighbor.state, h_i)
            entry = (neighbor.f, neighbor.move, neighbor)

            if neighbor.config not in explored:
                heappush(heap, entry)
                explored.add(neighbor.config)
                x[neighbor.config] = entry
                if neighbor.depth > search_depth:
                    search_depth += 1

            elif neighbor.config in x and neighbor.f < x[neighbor.config][2].f:
                hindex = heap.index((x[neighbor.config][2].f, x[neighbor.config][2].move, x[neighbor.config][2]))
                heap[int(hindex)] = entry
                x[neighbor.config] = entry
                heapify(heap)

        if len(heap) > bound:
            bound = len(heap)

# for h = 0, A* reduces to BFS
def BFS(start_state):
    global bound, goal, search_depth
    explored, queue = set(), deque([Board(start_state, None, None, 0, 0, 0)])
    while queue:
        left = queue.popleft()
        explored.add(left.config)

        if left.state == goal_state:
            goal = left
            return queue
        neighbors = expand(left)
        for neighbor in neighbors:
            if neighbor.config not in explored:
                queue.append(neighbor)
                explored.add(neighbor.config)

                if neighbor.depth > search_depth:
                    search_depth += 1

        if len(queue) > bound:
            bound = len(queue)

def IDAstar(start_state, h_i):
    global cost
    if h_i == 0:
        return BFS(start_state)

    threshold = h(start_state, h_i)
    while 1:
        result = search(start_state, threshold, h_i)
        if type(result) is list:
            return result
            break
        threshold = result
        cost = set()

# compute h
def h(state, h_i):
    value = 0
    if h_i == 0:      # Breadth First Search
        value = 0
    elif h_i == 1:    # Manhattan Distance
        for i in range(1,9):
            board_i = state.index(i)
            goal_i = goal_state.index(i)
            value += abs(board_i % 3  - goal_i % 3) + abs(board_i // 3 - goal_i // 3)

    elif h_i == 2:    # Euclidean Distance
        for i in range(1,9):
            board_i = state.index(i)
            goal_i = goal_state.index(i)
            value += abs(board_i % 3  - goal_i % 3)**2 + abs(board_i // 3 - goal_i // 3)**0.5

    elif h_i == 3:    # Misplaced tiles
        for i in range(1,9):
            board_i = state.index(i)
            goal_i = goal_state.index(i)
            value += int(board_i == goal_i)
    return value

# inversions is number of swaps from initial state, to goal state
def inversions(state):
    inversions = 0
    for i in range(len(state)):
        for j in range(i+1, len(state)):
            if state[i] and state[j] and state[i] > state[j]:
                inversions += 1
    return inversions

# check if a solution exists for this configuration or not
def is_solvable():    
    if (inversions(initial_state) - inversions(goal_state))%2:
        print "This puzzle cannot be solved"
        return 0
    return 1

algo = { 0: Astar, 1: IDAstar}
fname = {0:'a*', 1:'IDAstar*'}

if __name__ == '__main__':
    with open('input.txt') as f:
        s = f.read()

    astar = PrettyTable(['Start A*', 'N0', 't0', 'N1', 't1', 'N2', 't2', 'N3', 't3'])
    idastar = PrettyTable(['Start IDA*', 'N0', 't0', 'N1', 't1', 'N2', 't2', 'N3', 't3'])

    x = [list(y) for y in s.strip().split()]
    for each in x:
        initial_state = [int(i) for i in each]
        print 'Start state', initial_state

        for part in range(2):
            examined = 0 # nodes examined so far
            search_depth = 0
            bound = 0
            stats = list()
            stats.append(initial_state)
            if (is_solvable()==0):
                stats.extend((-1,-1,-1,-1,-1,-1,-1,-1))
                if part == 0:
                    astar.add_row(stats)
                else:
                    idastar.add_row(stats)
                continue
            
            for h_i in range(4):
                algo_i = algo[part]
                start = timeit.default_timer()
                solution = algo_i(initial_state, h_i)
                stop = timeit.default_timer()
                print fname[part], '>', heurstics[h_i], search_depth,'nodes expanded in ', format(stop-start, '.8f'), 's'
                stats.extend((search_depth, format(stop-start, '.8f')))

            print
            if part == 0:
                astar.add_row(stats)
            else:
                idastar.add_row(stats)

    print 'A*'
    print astar
    print 'IDA*'
    print idastar