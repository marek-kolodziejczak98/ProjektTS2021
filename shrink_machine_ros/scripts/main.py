#!/usr/bin/env python3
from shrink_machine import ShrinkMachine
from termcolor import colored
import sys
from machine_graph import MachineGraph
import rospy
from std_msgs.msg import String
from robot_control import RobotControl
import click
import roslib


def talker(command):
    pub = rospy.Publisher('commands', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(1)  # 10hz
    #while not rospy.is_shutdown():
    pub.publish(command)
        #rate.sleep()


@click.command()
@click.option('--graph', is_flag=True, help='Graph visualization.')
@click.option('--gazebo', is_flag=True, prompt='If you want to visualize in gazebo - robot here :).')
@click.option('-w', '--graph-width', 'graph_width', default=9, help='Width of the window with graph.')
@click.option('-h', '--graph-height', 'graph_height', default=8, help='Height of the window with graph.')
def main(graph, gazebo, graph_width, graph_height):
    if gazebo:
        rc = RobotControl()

    # Create a shrink machine object
    shrink = ShrinkMachine()

    # When start method is called, master is created
    shrink.start()
    states_master, states_slave1, states_slave2 = shrink.get_defined_states()
    edges_master, edges_slave1, edges_slave2 = shrink.get_defined_transitions()

    if graph:
        graph = MachineGraph((states_master, states_slave1, states_slave2), (edges_master, edges_slave1, edges_slave2),
                             graph_width, graph_height)

    print("Initial machine: " + str(shrink.get_current_machine()))
    print("Initial state: " + str(shrink.get_current_state()))

    while True:
        possible_trans = shrink.get_possible_transitions()
        print("INFO: if the state doesn't change, click enter :)")
        print("Possible transitions: ")
        #user_tran = input(f"Expected trans is{shrink.get_possible_transitions()}: ")
        for i in range(len(possible_trans)):
            print('\t' + str(i) + ": " + str(possible_trans[i]))
        user_tran = None

        if graph:
            graph.display(shrink)

        try:
            user_id = int(input("User transition (type a number only): "))
            if isinstance(user_id, int) and user_id < len(possible_trans):
                user_tran = possible_trans[user_id]
            else:
                print(colored('main Oj byczq sys32', 'red'))
                break
        except:
            print("It wasnt't a number\n")
            break

        if user_tran is not None:
            shrink.run_transition([user_tran])
        print("Current machine: " + str(shrink.get_current_machine()))
        print("Current state: " + str(shrink.get_current_state()))

        states = shrink.get_defined_states()
        trans = shrink.get_defined_transitions()
        s = states[0][0].transitions

        if gazebo:
            rc.move_robot(shrink.get_current_state().value)

    shrink.stop()


if __name__ == '__main__':
    main()


