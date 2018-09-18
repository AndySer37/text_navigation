#!/usr/bin/env python
# q learning planning 
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import yaml

f = open('map.yaml')
map = yaml.load(f)
start_point = 4
end_point = 20

real_start_point = start_point-1
real_end_point = end_point-1
# defines the reward/connection graph
r = np.ones((21,5), dtype = np.float32) #NESW and stay
act = np.ones((21,5), dtype = np.int32)
map = map['node']
for i in range(21):
	for j in range(5):
		r[i][j] = map[i]['connect'][j]

for i in range(21):
	for j in range(5):
		act[i][j] = int(map[i]['action'][j])


for i in range(21):
	for j in range(5):
		if (act[i][j] == real_end_point) and act[i][j]!=-1:
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
    traverse = "%i -> " % (current_state+1)
    n_steps = 0
    while current_state != real_end_point and n_steps < 20:
        #next_state = np.argmax(q[current_state])
        acti = np.argmax(q[current_state])
        next_state = act[current_state][acti]
        current_state = next_state
        traverse += "%i -> " % (current_state+1)
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
n_actions = 5
epsilon = 0.05
random_state = np.random.RandomState(1999)
for e in range(int(n_episodes)):
    states = list(range(n_states))
    random_state.shuffle(states)
    current_state = states[0]
    #print current_state,r[current_state]
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
            next_state = act[current_state,action]
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
            next_state = act[current_state,action]
        reward = update_q(current_state, next_state, action,
                          alpha=alpha, gamma=gamma)
        # Goal state has reward 100
        if reward > 1:
            goal = True
        current_state = next_state

print(q)
show_traverse(real_start_point)





