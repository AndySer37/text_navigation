#!/usr/bin/env python
# license removed for brevity
import rospy
#from transitions import Machine
import transitions
import yaml

from transitions.extensions import GraphMachine as Machine
import pygraphviz
class topomap(object):
	states = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20', '21']
	actions = ['N', 'E', 'S', 'W']
	f = open('map.yaml')
	map = yaml.load(f)
	map = map['node']

	def __init__(self,init, initial_heading):	
		self.machine = Machine(model=self, states=topomap.states, initial=init)
		self.last_state = init
		self.current_heading = initial_heading
		for i in range(21):
			for j in range(4):
				num = self.map[i]['action'][j]
				if num != -1:
					self.machine.add_transition(trigger=self.actions[j], source = str(i+1) , dest = str(num+1) )

	def service_callback(self, req):
		# test comment out
		#action_str = "INIT"
		#action_str = req.action
		
		head = self.actions.index(self.current_heading)
		turn_arr = ['L', 'F', 'R']

		# test comment out
		#turn = turn_arr.index(action_str)-1
		
		# test need
		turn = turn_arr.index(req)-1

		absolute_dir = (head + turn)%4
		try:
			if absolute_dir == 0:
				self.N();
			elif absolute_dir == 1:
				self.E();
			elif absolute_dir == 2:
				self.S();
			elif absolute_dir == 3:
				self.W();
			absolute_dir += self.map[int(self.last_state)-1]['modify'][absolute_dir]
			print absolute_dir
			absolute_dir = (absolute_dir+4)%4
			self.current_heading = self.actions[absolute_dir]
			print "current state: ", self.machine.get_state(self.state).name
			print "last_state: ", self.last_state
			self.last_state = self.machine.get_state(self.state).name
			m.get_graph().draw('my_state_diagram.png', prog='dot')
			return self.machine.get_state(self.state).name
		except transitions.core.MachineError as e:
			print e

if __name__ == '__main__':
	#rospy.init_node('topo_map_node')
	initial_node = '1'
	initial_heading = 'W'
	m = topomap(initial_node, initial_heading)	

	# test comment out
	#s = rospy.Service('topo_map_action', actions, m.service_callback)


	print "initial state", initial_node , initial_heading

	# test need
	while(1):
		action_str = raw_input("input action...")
		m.service_callback(action_str)


	# test comment out
	#rospy.spin()


 




