from shrink_machine import ShrinkMachine
# import cv2
from termcolor import colored
import networkx as nx
import matplotlib.pyplot as plt
import sys


FIG_WIDTH, FIG_HEIGHT = 9, 8

# Create a shrink machine object
shrink = ShrinkMachine()

# create paths from transitions (exemplary)
path_1 = ["m_0_1", "m_1_2", "m_2_1", "m_1_3", "m_3_4"]
path_2 = ["m_0_2", "m_2_3", "m_3_2", "m_2_4"]
path_3 = ["m_0_3", "m_3_1", "m_1_2", "m_2_4"]
paths = [path_1, path_2, path_3]

# When start method is called, master is created
shrink.start()
states_master, states_slave1, states_slave2 = shrink.get_defined_states()
states_master_list = list()
states_slave1_list = list()
states_slave2_list = list()
for state in states_master:
    states_master_list.append(state.name)
for state in states_slave1:
    states_slave1_list.append(state.name)
for state in states_slave2:
    states_slave2_list.append(state.name)

graph_master = nx.DiGraph()
graph_slave1 = nx.DiGraph()
graph_slave2 = nx.DiGraph()
graph_master.add_nodes_from(states_master_list)
graph_slave1.add_nodes_from(states_slave1_list)
graph_slave2.add_nodes_from(states_slave2_list)

edges_master, edges_slave1, edges_slave2 = shrink.get_defined_transitions()
edges_master_list = list()
edges_slave1_list = list()
edges_slave2_list = list()
for edge in edges_master:
    for target in edge[1]:
        edges_master_list.append([states_master_list[edge[0]], states_master_list[target]])
for edge in edges_slave1:
    for target in edge[1]:
        edges_slave1_list.append([states_slave1_list[edge[0]], states_slave1_list[target]])
for edge in edges_slave2:
    for target in edge[1]:
        edges_slave2_list.append([states_slave2_list[edge[0]], states_slave2_list[target]])

graph_master.add_edges_from(edges_master_list)
graph_slave1.add_edges_from(edges_slave1_list)
graph_slave2.add_edges_from(edges_slave2_list)
plt.ion()

print("Initial machine: " + str(shrink.get_current_machine()))
print("Initial state: " + str(shrink.get_current_state()))
while True:
    possible_trans = shrink.get_possible_transitions()
    print("Possible transitions: ")
    #user_tran = input(f"Expected trans is{shrink.get_possible_transitions()}: ")
    for i in range(len(possible_trans)):
        print('\t' + str(i) + ": " + str(possible_trans[i]))
    user_tran = None
    plt.close()
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(int(FIG_WIDTH), int(FIG_HEIGHT)))
    ax = axes.flatten()
    nx.draw_networkx(graph_master, with_labels=True, pos=nx.spiral_layout(graph_master),\
                     node_size=300, node_color='#6666FF', font_color='#000000', ax=ax[0], linewidths=0.5)
    if str(shrink.get_current_machine()) == 'slave1':
        nx.draw_networkx(graph_master, nodelist=list(['Stacja wysłała sygnał i czeka na podanie butelki']), with_labels=True, pos=nx.spiral_layout(graph_master),\
                     node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])
    elif str(shrink.get_current_machine()) == 'slave2':
        nx.draw_networkx(graph_master, nodelist=list(['Awaria stacji']), with_labels=True, pos=nx.spiral_layout(graph_master),\
                     node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])
    elif str(shrink.get_current_machine()) == 'master':
        nx.draw_networkx(graph_master, nodelist=list([shrink.get_current_state().name]), with_labels=True, pos=nx.spiral_layout(graph_master),\
                     node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])

    ax[0].set_axis_off()

    nx.draw_networkx(graph_slave1, with_labels=True, pos=nx.spiral_layout(graph_slave1),\
                     node_size=300, node_color='#6666FF', font_color='#000000',  ax=ax[1])
    if str(shrink.get_current_machine()) == 'slave1':
        nx.draw_networkx(graph_slave1, nodelist=list([shrink.get_current_state().name]), with_labels=True, pos=nx.spiral_layout(graph_slave1),\
                     node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[1])

    ax[1].set_axis_off()
    nx.draw_networkx(graph_slave2, with_labels=True, pos=nx.spiral_layout(graph_slave2),\
                     node_size=300, node_color='#6666FF', font_color='#000000',  ax=ax[2])
    if str(shrink.get_current_machine()) == 'slave2':
        nx.draw_networkx(graph_slave2, nodelist=list([shrink.get_current_state().name]), with_labels=True, pos=nx.spiral_layout(graph_slave2),\
                     node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[2])
    ax[2].set_axis_off()
    #plt.show()
    plt.pause(0.5)

    try:
        user_id = int(input("User transition(type a number only): "))
        if isinstance(user_id, int) and user_id < len(possible_trans):
            user_tran = possible_trans[user_id]
        else:
            print(colored('main Oj byczq sys32', 'red'))

    except:
        print("")

    print()

    if user_tran == 'q':
        sys.exit(1)
    if user_tran is not None:
        shrink.run_transition([user_tran])
    print("Current machine: " + str(shrink.get_current_machine()))
    print("Current state: " + str(shrink.get_current_state()))

    states = shrink.get_defined_states()
    trans = shrink.get_defined_transitions()

    s = states[0][0].transitions
plt.close()
shrink.stop()

