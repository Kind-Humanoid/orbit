# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the ANYbotics robots.

The following configuration parameters are available:

* :obj:`ANYMAL_B_CFG`: The ANYmal-B robot with ANYdrives 3.0
* :obj:`ANYMAL_C_CFG`: The ANYmal-C robot with ANYdrives 3.0
* :obj:`ANYMAL_D_CFG`: The ANYmal-D robot with ANYdrives 3.0

Reference:

* https://github.com/ANYbotics/anymal_b_simple_description
* https://github.com/ANYbotics/anymal_c_simple_description
* https://github.com/ANYbotics/anymal_d_simple_description

"""

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import ActuatorNetLSTMCfg, DCMotorCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR

##
# Configuration - Actuators.
##

MONAV2_LEG_SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=["HpR","HpL",
                      "HrR","HrL",
                      "HwR","HwL",
                      "ApR","ApL",
                      "ArR","ArL",
                      "KpR","KpL",],
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
    stiffness={".*": 40.0},
    damping={".*": 5.0},
)

MONAV2_ARM_SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=["SpR","SpL", 
                      "SrR","SrL",
                      "SwR","SwL",
                      "WpR","WpL",
                      "WrR","WrL",
                      "WwR","WwL"],
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
    stiffness={".*": 40.0},
    damping={".*": 5.0},
)

MONAV2_TORSO_SIMPLE_ACTUATOR_CFG = DCMotorCfg(
    joint_names_expr=["FpC", "FrC", "FwC"],
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
    stiffness={".*": 40.0},
    damping={".*": 5.0},
)
"""Configuration for ANYdrive 3.x with DC actuator model."""

##
# Configuration - Articulation.
##

MONAV2_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/fnuabhimanyu/WholeBodyMotion/assets/hoa_full_body1/full_body1/URDF_description/meshes/mona_v2_simple/mona_v2_simple.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=10.0,
            max_angular_velocity=10.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.923),
        joint_pos={
            "SpR": -0.5,
            "SpL": 0.5    # all motors
        },
    ),
    actuators={"legs": MONAV2_LEG_SIMPLE_ACTUATOR_CFG, 
               "arms": MONAV2_ARM_SIMPLE_ACTUATOR_CFG,
               "torso": MONAV2_TORSO_SIMPLE_ACTUATOR_CFG,},

    soft_joint_pos_limit_factor=0.95,
)
