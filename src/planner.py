#!/usr/bin/env python
# q learning planning 
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
start_point = 1
end_point = 10

real_start_point = start_point-1
real_end_point = end_point-1
# defines the reward/connection graph
r = np.ones((21,21), dtype = np.float32) #NESW and stay
r *= -1
r[0][3] = 0
r[0][4] = 0
r[0][1] = 0
r[1][2] = 0
r[1][0] = 0
r[1][9] = 0
r[2][7] = 0
r[2][1] = 0
r[2][3] = 0
r[2][8] = 0
r[3][6] = 0
r[3][0] = 0
r[3][4] = 0
r[3][2] = 0
r[4][5] = 0
r[4][0] = 0
r[4][3] = 0
r[5][10] = 0
r[5][4] = 0
r[5][6] = 0
r[6][11] = 0
r[6][3] = 0
r[6][5] = 0
r[6][7] = 0
r[7][2] = 0
r[7][6] = 0
r[7][8] = 0
r[8][9] = 0
r[8][2] = 0
r[8][7] = 0
r[9][12] = 0
r[9][1] = 0
r[9][8] = 0
r[10][15] = 0
r[10][5] = 0
r[10][11] = 0
r[11][6] = 0
r[11][10] = 0
r[11][13] = 0
r[12][17] = 0
r[12][9] = 0
r[12][13] = 0
r[13][17] = 0
r[13][11] = 0
r[13][16] = 0
r[13][12] = 0
r[14][18] = 0
r[14][14] = 0
r[15][18] = 0
r[15][10] = 0
r[15][16] = 0
r[16][13] = 0
r[16][15] = 0
r[16][17] = 0
r[17][13] = 0
r[17][16] = 0
r[17][12] = 0
r[18][19] = 0
r[18][15] = 0
r[18][14] = 0
r[18][20] = 0
r[19][20] = 0
r[19][18] = 0
r[20][19] = 0
r[20][18] = 0
for i in range(21):
	for j in range(21):
		if i==j:
			r[i][j]=0
for i in range(21):
	for j in range(21):
		if (j == real_end_point) and r[i][j]!=-1:
			r[i][j]=100

q = np.zeros_like(r)


def update_q(state, next_state, action, alpha, gamma):
    rsa = r[state, action]
    qsa = q[state, action]
    new_q = qsa + alpha * (rsa + gamma * max(q[next_state, :]) - qsa)
    q[state, action] = new_q
    # renormalize row to be between 0 and 1
    rn = q[state][q[state] > 0] / np.sum(q[state][q[state] > 0])
    q[state][q[state] > 0] = rn
    return r[state, action]


def show_traverse(i):
    # show all the greedy traversals
    current_state = i
    traverse = "%i -> " % current_state
    n_steps = 0
    while current_state != real_end_point and n_steps < 20:
        next_state = np.argmax(q[current_state])
        current_state = next_state
        traverse += "%i -> " % current_state
        n_steps = n_steps + 1
    # cut off final arrow
    traverse = traverse[:-4]
    print("Greedy traversal for starting state %i" % i)
    print(traverse)
    print("")



# Core algorithm
gamma = 0.8
alpha = 1.
n_episodes = 1E3
n_states = 21
n_actions = 21
epsilon = 0.05
random_state = np.random.RandomState(1999)
for e in range(int(n_episodes)):
    states = list(range(n_states))
    random_state.shuffle(states)
    current_state = states[0]
    goal = False
    if e % int(n_episodes / 10.) == 0 and e > 0:
        pass
        # uncomment this to see plots each monitoring
        #show_traverse()
        #show_q()
    while not goal:
        # epsilon greedy
        valid_moves = r[current_state] >= 0
        if random_state.rand() < epsilon:
            actions = np.array(list(range(n_actions)))
            actions = actions[valid_moves == True]
            if type(actions) is int:
                actions = [actions]
            random_state.shuffle(actions)
            action = actions[0]
            next_state = action
        else:
            if np.sum(q[current_state]) > 0:
                action = np.argmax(q[current_state])
            else:
                # Don't allow invalid moves at the start
                # Just take a random move
                actions = np.array(list(range(n_actions)))
                actions = actions[valid_moves == True]
                random_state.shuffle(actions)
                action = actions[0]
            next_state = action
        reward = update_q(current_state, next_state, action,
                          alpha=alpha, gamma=gamma)
        # Goal state has reward 100
        if reward > 1:
            goal = True
        current_state = next_state

print(q)
show_traverse(real_start_point)





