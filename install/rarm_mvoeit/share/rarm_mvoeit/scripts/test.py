import numpy as np
import trajectory_planing as tp
def calculate_motion(from_p_T,to_p_T):
    point1_pos=tp.inverse_kinematics(from_p_T).tolist()
    point2_pos=tp.inverse_kinematics(to_p_T).tolist()
    return tp.main(point1_pos,point2_pos)
point1=np.array([
    [1,     0,      0,       0.5    ],
    [0,    -1,      0,      -0.5    ],
    [0,     0,     -1,      0.3    ],
    [0,     0,      0,         1    ]
])
point2=np.array([
    [1,     0,      0,      -0.5    ],
    [0,    -1,      0,      -0.5    ],
    [0,     0,     -1,      0.3    ],
    [0,     0,      0,         1    ]
])
point3=np.array([
    [1,     0,      0,      -0.5    ],
    [0,    -1,      0,      0.4    ],
    [0,     0,     -1,      0.3    ],
    [0,     0,      0,         1    ]
])
point4=np.array([
    [1,     0,      0,      0.3    ],
    [0,    -1,      0,      0.4    ],
    [0,     0,     -1,      0.3    ],
    [0,     0,      0,         1    ]
])
point5=np.array([
    [1,     0,      0,      0.3    ],
    [0,    -1,      0,      0.4    ],
    [0,     0,     -1,      0.25    ],
    [0,     0,      0,         1    ]
])
pick_up_T6=np.array([
            [1,     0,      0,       0.5    ],
            [0,    -1,      0,      -0.5    ],
            [0,     0,     -1,      0.05    ],
            [0,     0,      0,         1    ]
        ])
motion=[]
motion.append(calculate_motion(pick_up_T6,point1))
motion.append(calculate_motion(point1,point2))
motion.append(calculate_motion(point2,point3))
motion.append(calculate_motion(point3,point4))
motion.append(calculate_motion(point4,point5))
print(motion[0][0][0])