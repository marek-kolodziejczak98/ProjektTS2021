from shrink_machine import ShrinkMachine
import cv2
shrink = ShrinkMachine()
from termcolor import colored
# create paths from transitions (exemplary)
path_1 = ["m_0_1", "m_1_2", "m_2_1", "m_1_3", "m_3_4"]
path_2 = ["m_0_2", "m_2_3", "m_3_2", "m_2_4"]
path_3 = ["m_0_3", "m_3_1", "m_1_2", "m_2_4"]
paths = [path_1, path_2, path_3]

shrink.start()
print("Initial machine: " + str(shrink.get_current_machine()))
print("Initial state: " + str(shrink.get_current_state()))
while True:
    possible_trans = shrink.get_possible_transitions()
    print("Possible transition: ")
    #user_tran = input(f"Expected trans is{shrink.get_possible_transitions()}: ")
    for i in range(len(possible_trans)):
        print('\t' + str(i) + ": " + str(possible_trans[i]))
    user_tran = None
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
        break
    if user_tran is not None:
        shrink.run_transition([user_tran])
    print("Current machine: " + str(shrink.get_current_machine()))
    print("Current state: " + str(shrink.get_current_state()))

    states = shrink.get_defined_states()
    trans = shrink.get_defined_transitions()

    s = states[0][0].transitions
shrink.stop()

