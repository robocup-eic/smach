import numpy as np

MyType=np.dtype([('len', np.int64, ), ('digital_input_bits', np.int64, ), 
                ('digital_outputs', np.int64, ), ('robot_mode', np.int64, ), 
                ('controller_timer', np.int64, ),('run_time', np.int64, ), 
                ('test_value', np.int64, ), ('safety_mode', np.float64, ), 
                ('speed_scaling', np.float64, ), ('linear_momentum_norm', np.float64, ),
                ('v_main', np.float64, ), ('v_robot', np.float64, ), 
                ('i_robot', np.float64, ), ('program_state', np.float64, ), 
                ('safety_status', np.float64, ), ('tool_accelerometer_values', np.float64, (3,)), 
                ('elbow_position', np.float64, (3,)), ('elbow_velocity', np.float64, (3,)), 
                ('q_target', np.float64, (6,)), ('qd_target', np.float64,(6,)),
                ('qdd_target', np.float64, (6,)), ('i_target', np.float64,(6,)), 
                ('m_target', np.float64, (6,)), ('q_actual', np.float64, (6,)), 
                ('qd_actual', np.float64, (6,)), ('i_actual', np.float64, (6,)), 
                ('i_control', np.float64, (6,)), ('tool_vector_actual', np.float64, (6,)), 
                ('TCP_speed_actual', np.float64, (6,)), ('TCP_force', np.float64, (6,)),
                ('Tool_vector_target', np.float64, (6,)), ('TCP_speed_target', np.float64, (6,)), 
                ('motor_temperatures', np.float64, (6,)), ('joint_modes', np.float64, (6,)), 
                ('v_actual', np.float64, (6,)), ('dummy', np.float64, (9,6))
                ])

print(np.dtype(MyType['elbow_position']))