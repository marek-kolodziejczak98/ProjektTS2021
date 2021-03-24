from shrink_machine import ShrinkMachine

shrink = ShrinkMachine()

# create paths from transitions (exemplary)
path_1 = ["m_0_1", "m_1_2", "m_2_1", "m_1_3", "m_3_4"]
path_2 = ["m_0_2", "m_2_3", "m_3_2", "m_2_4"]
path_3 = ["m_0_3", "m_3_1", "m_1_2", "m_2_4"]
paths = [path_1, path_2, path_3]

shrink.start()
shrink.run_transition(["m_0_1", "m_1_2", "s1_0_1", "s1_1_2", "s1_2_3", "s1_3_4", "m_1_3", "m_3_4", "m_4_5",
                       "s2_0_1", "s2_1_2", "s2_2_3"], verbose=True)
shrink.stop()
