import matplotlib.pyplot as plt
from typing import Tuple
import networkx as nx
from shrink_machine import ShrinkMachine


class MachineGraph:
    def __init__(self, states_tuple: Tuple, edges_tuple: Tuple, fig_width=9, fig_height=8) -> None:
        states_master, states_slave1, states_slave2 = states_tuple
        edges_master, edges_slave1, edges_slave2 = edges_tuple

        states_master_list = list()
        states_slave1_list = list()
        states_slave2_list = list()
        for state in states_master:
            states_master_list.append(state.name)
        for state in states_slave1:
            states_slave1_list.append(state.name)
        for state in states_slave2:
            states_slave2_list.append(state.name)

        self.graph_master = nx.DiGraph()
        self.graph_slave1 = nx.DiGraph()
        self.graph_slave2 = nx.DiGraph()
        self.graph_master.add_nodes_from(states_master_list)
        self.graph_slave1.add_nodes_from(states_slave1_list)
        self.graph_slave2.add_nodes_from(states_slave2_list)

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

        self.graph_master.add_edges_from(edges_master_list)
        self.graph_slave1.add_edges_from(edges_slave1_list)
        self.graph_slave2.add_edges_from(edges_slave2_list)
        plt.ion()

        self.fig, self.axes = plt.subplots(nrows=3, ncols=1, figsize=(int(fig_width), int(fig_height)))

    def display(self, shrink_machine: ShrinkMachine):
        ax = self.axes.flatten()
        nx.draw_networkx(self.graph_master, nodelist=list(['Stacja zgrzewająca pusta']), with_labels=True,
                         pos=nx.spiral_layout(self.graph_master),
                         node_size=500, node_color='#3636FF', font_color='#000000', ax=ax[0], linewidths=0.5)
        nx.draw_networkx(self.graph_master, with_labels=True, pos=nx.spiral_layout(self.graph_master),
                         node_size=300, node_color='#6666FF', font_color='#000000', ax=ax[0], linewidths=0.5)
        if str(shrink_machine.get_current_machine()) == 'slave1':
            nx.draw_networkx(self.graph_master, nodelist=list(['Stacja wysłała sygnał i czeka na podanie butelki']),
                             with_labels=True, pos=nx.spiral_layout(self.graph_master),
                             node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])
        elif str(shrink_machine.get_current_machine()) == 'slave2':
            nx.draw_networkx(self.graph_master, nodelist=list(['Awaria stacji']), with_labels=True,
                             pos=nx.spiral_layout(self.graph_master),
                             node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])
        elif str(shrink_machine.get_current_machine()) == 'master':
            nx.draw_networkx(self.graph_master, nodelist=list([shrink_machine.get_current_state().name]), with_labels=True,
                             pos=nx.spiral_layout(self.graph_master),
                             node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[0])

        nx.draw_networkx(self.graph_slave1, with_labels=True, nodelist=list(['Powrót do procesu zgrzewania']),
                         pos=nx.spiral_layout(self.graph_slave1),
                         node_size=500, node_color='#3636FF', font_color='#000000', ax=ax[1])
        nx.draw_networkx(self.graph_slave1, with_labels=True, pos=nx.spiral_layout(self.graph_slave1), \
                         node_size=300, node_color='#6666FF', font_color='#000000', ax=ax[1])
        if str(shrink_machine.get_current_machine()) == 'slave1':
            nx.draw_networkx(self.graph_slave1, nodelist=list([shrink_machine.get_current_state().name]), with_labels=True,
                             pos=nx.spiral_layout(self.graph_slave1),
                             node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[1])

        nx.draw_networkx(self.graph_slave2, with_labels=True, nodelist=list(['Powrót do procesu']),
                         pos=nx.spiral_layout(self.graph_slave2),
                         node_size=500, node_color='#3636FF', font_color='#000000', ax=ax[2])
        nx.draw_networkx(self.graph_slave2, with_labels=True, pos=nx.spiral_layout(self.graph_slave2), \
                         node_size=300, node_color='#6666FF', font_color='#000000', ax=ax[2])
        if str(shrink_machine.get_current_machine()) == 'slave2':
            nx.draw_networkx(self.graph_slave2, nodelist=list([shrink_machine.get_current_state().name]), with_labels=True,
                             pos=nx.spiral_layout(self.graph_slave2),
                             node_size=300, node_color='#33FF33', font_color='#000000', ax=ax[2])
        ax[0].set_axis_off()
        ax[1].set_axis_off()
        ax[2].set_axis_off()
        plt.show()
        plt.pause(2)
