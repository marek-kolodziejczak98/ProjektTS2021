from shrink_machine.generator_class import Generator
from statemachine import State, Transition
from termcolor import colored
from typing import Tuple


class ShrinkMachine:

    def __init__(self):
        # define states for a master (way of passing args to class)
        options1 = [
            {"name": "Stacja zgrzewająca pusta", "initial": True, "value": "Station_empty"},  # 0
            {"name": "Stacja czeka na komplet butelek", "initial": False, "value": "Station_waiting"},  # 1
            {"name": "Stacja wysłała sygnał i czeka na podanie butelki", "initial": False,
             "value": "Station_incomplete"},  # 2
            {"name": "Stacja ma zestaw butelek", "initial": False, "value": "Station_ready"},  # 3
            {"name": "Stacja ma zgrzewkę", "initial": False, "value": "Station_done"},  # 4
            {"name": "Awaria stacji", "initial": False, "value": "Station_failure"}]  # 5

        options2 = [
            {"name": "Manipulator w pozycji bazowej", "initial": True, "value": "Robot_in_base_position"},  # 0
            {"name": "Manipulator w pozycji gotowej do pochwycenia", "initial": False, "value": "Robot_ready"},  # 1
            {"name": "Manipulator trzymający butelkę", "initial": False, "value": "Robot_holding_beer"},  # 2
            {"name": "Manipulator w pozycji końcowej", "initial": False, "value": "Robot_in_final_position"},  # 3
            {"name": "Powrót do procesu zgrzewania", "initial": False, "value": "Return_to_shrink_process"}  # 4
        ]

        options3 = [
            {"name": "Operator oczekuje na sygnał błędu", "initial": True, "value": "Error_signal_received"},  # 0
            {"name": "Sygnalizacja błędu", "initial": False, "value": "Error_signalized"},  # 1
            {"name": "Usunięcie zgrzewki wody", "initial": False, "value": "Employee_reaction"},    # 2
            {"name": "Powrót do procesu", "initial": False, "value": "Return_to_process"}]  # 3

        # create State objects for a master
        # ** -> unpack dict to args
        self.master_states = [State(**opt) for opt in options1]
        self.slave1_states = [State(**opt) for opt in options2]
        self.slave2_states = [State(**opt) for opt in options3]

        # valid transitions for a master (indices of states from-to)
        self.from_to1 = [
            [0, [1]],
            [1, [2, 3]],
            [2, [1]],
            [3, [4]],
            [4, [5, 0]],
            [5, [0]]
        ]
        self.from_to2 = [
            [0, [1]],
            [1, [2]],
            [2, [3]],
            [3, [4]],
            [4, []]
        ]
        self.from_to3 = [
            [0, [1]],
            [1, [2]],
            [2, [3]],
            [3, []]
        ]

        # create transitions dict
        transitions = {"master": {}, "slave1": {}, "slave2": {}}

        # machines dict for a states and transitions creating
        machines = {"master": {"transitions": transitions["master"], "from_to": self.from_to1, "id": "m_{}_{}", "states": self.master_states},
                    "slave1": {"transitions": transitions["slave1"], "from_to": self.from_to2, "id": "s1_{}_{}", "states": self.slave1_states},
                    "slave2": {"transitions": transitions["slave2"], "from_to": self.from_to3, "id": "s2_{}_{}", "states": self.slave2_states}}

        for key in machines:
            for indices in machines[key]["from_to"]:
                from_idx, to_idx_tuple = indices  # unpack list of two elements into separate from_idx and to_idx_tuple
                for to_idx in to_idx_tuple:  # iterate over destinations from a source state
                    op_identifier = machines[key]["id"].format(from_idx, to_idx)  # parametrize identifier of a transition

                    # create transition object and add it to the master_transitions dict
                    transition = Transition(machines[key]["states"][from_idx], machines[key]["states"][to_idx],
                                            identifier=op_identifier)
                    machines[key]["transitions"][op_identifier] = transition

                    # add transition to source state
                    machines[key]["states"][from_idx].transitions.append(transition)

        # save states and transitions
        self.transitions = transitions
        self.states = {"master": self.master_states, "slave1": self.slave1_states, "slave2": self.slave2_states}
        self.current_machine = None
        self.machines = {}

    def start(self) -> None:
        '''
        Creates master
        '''
        self.machines["master"] = Generator(self.states["master"], self.transitions["master"])
        self.current_machine = "master"

    def run_transition(self, events: list) -> None:
        '''
        Move to the next state

        :param events: list of the transitions to be performed on the machine.
        '''
        for e in events:
            # Run another transition
            if e in self.get_possible_transitions():

                self.transitions[self.current_machine][e]._run(self.machines[self.current_machine])

                # Check if there's a time to change machine
                if self.current_machine == "master":
                    if self.machines[self.current_machine].current_state.value == "Station_incomplete":
                        self.machines["slave1"] = Generator(self.states["slave1"], self.transitions["slave1"])
                        self.current_machine = "slave1"
                    if self.machines[self.current_machine].current_state.value == "Station_failure":
                        self.machines["slave2"] = Generator(self.states["slave2"], self.transitions["slave2"])
                        self.current_machine = "slave2"

                if self.current_machine == "slave1" and \
                        self.machines[self.current_machine].current_state.value == "Return_to_shrink_process":
                    self.current_machine = "master"
                    self.transitions[self.current_machine]["m_2_1"]._run(self.machines[self.current_machine])
                    self.machines.pop("slave1")

                if self.current_machine == "slave2" and \
                        self.machines[self.current_machine].current_state.value == "Return_to_process":
                    self.current_machine = "master"
                    self.transitions[self.current_machine]["m_5_0"]._run(self.machines[self.current_machine])
                    self.machines.pop("slave2")
            else:
                print(colored('Oj byczq sys32', 'red'))

    def stop(self) -> None:
        '''
        Destroys all machines.
        '''
        self.machines = {}
        self.current_machine = None

    def get_current_state(self) -> State:
        return self.machines[self.current_machine].current_state

    def get_defined_states(self) -> Tuple:
        return self.master_states, self.slave1_states, self.slave2_states

    def get_defined_transitions(self) -> Tuple:
        return self.from_to1, self.from_to2, self.from_to3

    def get_possible_transitions(self) -> list:
        if self.machines:
            return [tran.identifier for tran in self.machines[self.current_machine].current_state.transitions]
        return []

    def get_current_machine(self) -> str:
        return self.current_machine
