import string
import numpy as np
import trajectory_planing_copy as tp
np.set_printoptions(precision=5, suppress=True) #so we can see better coment out at run

def calculate_motion(from_p_T,to_p_T):
    point1_pos=tp.inverse_kinematics(from_p_T).tolist()
    point2_pos=tp.inverse_kinematics(to_p_T).tolist()
    if len(motion)>0:
        print(f"motio count is {len(motion)}")
        point1_pos=tp.fix_trajectory_start(point1_pos,motion[-1][-1])
        #point2_pos=tp.fix_trajectory_start(point2_pos,point1_pos)
    return tp.main(point1_pos,point2_pos)
def wrap_continuous(a_current, a_previous):
    # This must be robust and generic for any joint
    k = np.round((a_previous - a_current) / (2 * np.pi))
    return a_current + 2 * np.pi * k
pick_up_T6_1=np.array([
    [1,     0,      0,       0.5    ],
    [0,    -1,      0,      -0.5    ],
    [0,     0,     -1,      0.08    ],
    [0,     0,      0,         1    ]
])
pick_up_T6_2=np.array([
    [1,     0,      0,       0.5    ],
    [0,    -1,      0,      -0.5    ],
    [0,     0,     -1,      0.05    ],
    [0,     0,      0,         1    ]
])
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
    [0,     0,     -1,      0.24    ],
    [0,     0,      0,         1    ]
])
point6=np.array([
    [1,     0,      0,      0.3    ],
    [0,    -1,      0,      0.4    ],
    [0,     0,     -1,      0.8    ],
    [0,     0,      0,         1    ]
])

motion=[]
# motion.append(calculate_motion(pick_up_T6,point1))
# motion.append(calculate_motion(point1,point2))
# motion.append(calculate_motion(point2,point3))
# motion.append(calculate_motion(point3,point4))
# motion.append(calculate_motion(point4,point5))
# # print(motion[0])
# theta6=[]
# for i in range(len(motion)):
#     for j in range(len(motion[i])):
#         # if j==0 and not i==0:
#         #     motion[i][j][3]=wrap_continuous(motion[i][j][3],motion[i-1][-1][3])
#         #     motion[i][j][5]=wrap_continuous(motion[i][j][5],motion[i-1][-1][5])
#         # else:
#         #     motion[i][j][3]=wrap_continuous(motion[i][j][3],motion[i][j-1][3])
#         #     motion[i][j][5]=wrap_continuous(motion[i][j][5],motion[i][j-1][5])
#         theta6.append(motion[i][j][5])
# print(np.array(theta6))
print(tp.inverse_kinematics(pick_up_T6_1).tolist())
print(tp.inverse_kinematics(pick_up_T6_2).tolist())

 
