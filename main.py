from shrink_machine import ShrinkMachine
from termcolor import colored
import sys
from machine_graph import MachineGraph

FIG_WIDTH, FIG_HEIGHT = 9, 8

# Create a shrink machine object
shrink = ShrinkMachine()

# When start method is called, master is created
shrink.start()
states_master, states_slave1, states_slave2 = shrink.get_defined_states()
edges_master, edges_slave1, edges_slave2 = shrink.get_defined_transitions()

graph = MachineGraph((states_master, states_slave1, states_slave2), (edges_master, edges_slave1, edges_slave2),
                     FIG_WIDTH, FIG_HEIGHT)

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

    graph.display(shrink)

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

