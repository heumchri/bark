# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
import os
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.pygame_viewer import PygameViewer
from modules.runtime.viewer.panda3d_viewer import Panda3dViewer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser


def read_tracks_from_csv(filename):
    # load trackfile
    original_trackfile = np.genfromtxt(filename, delimiter=',', skip_header=1)

    # append velocities
    velocities = np.sqrt(
        original_trackfile[:, 6]**2 + original_trackfile[:, 7]**2)
    velocities.shape = (velocities.shape[0], 1)
    tracks = np.hstack((original_trackfile, velocities))

    # determine split indices to split array into list of arrays for each agent
    track_id_changes = tracks[:, 0]-np.roll(tracks[:, 0], 1)
    split_indices = np.where(track_id_changes > 0)[0]

    # only extract neccessary columns
    # trajectories = np.zeros([tracks.shape[0], StateDefinition.MIN_STATE_SIZE])
    # trajectories[:, StateDefinition.TIME_POSITION] = tracks[:, 2]/1000
    # trajectories[:, StateDefinition.X_POSITION] = tracks[:, 4]
    # trajectories[:, StateDefinition.Y_POSITION] = tracks[:, 5]
    # trajectories[:, StateDefinition.THETA_POSITION] = tracks[:, 8]
    # trajectories[:, StateDefinition.VEL_POSITION] = tracks[:, 11]
    trajectories = np.zeros([tracks.shape[0], 5])
    trajectories[:, 0] = tracks[:, 2]/1000
    trajectories[:, 1] = tracks[:, 4]
    trajectories[:, 2] = tracks[:, 5]
    trajectories[:, 3] = tracks[:, 8]
    trajectories[:, 4] = tracks[:, 11]

    vehicle_dimensions = np.zeros([tracks.shape[0], 2])
    vehicle_dimensions[:, 0] = tracks[:, 9]  # length
    vehicle_dimensions[:, 1] = tracks[:, 10]  # width

    # split array into list of arrays for each agent
    trajectory_list = np.split(trajectories, split_indices)
    vehicle_dimension_list = np.split(vehicle_dimensions, split_indices)
    # only keep dimensions once for each vehicle
    vehicle_dimension_list = [a[0] for a in vehicle_dimension_list]

    return trajectory_list, vehicle_dimension_list


# track file definition
filename = "modules/runtime/tests/data/DR_DEU_Merging_MT/vehicle_tracks_000.csv"
trajectory_list, vehicle_dimension_list = read_tracks_from_csv(filename)

# Parameters Definitions
param_server = ParameterServer()

# World Definition
world = World(param_server)

# Model Definitions
behavior_models = []
execution_models = []
dynamic_models = []
nr_of_agents = len(vehicle_dimension_list)
for trajectory in trajectory_list:
    behavior_models.append(BehaviorDataDriven(param_server, trajectory))
    execution_models.append(ExecutionModelInterpolate(param_server))
    dynamic_models.append(SingleTrackModel(param_server))

# Map Definition
xodr_parser = XodrParser(
    "modules/runtime/tests/data/DR_DEU_Merging_MT_shifted.xodr")
map_interface = MapInterface()
map_interface.set_open_drive_map(xodr_parser.map)
world.set_map(map_interface)

# Agent Definition
for i, trajectory in enumerate(trajectory_list):
    agent_2d_shape = CarLimousine()
    init_state = trajectory[0]
    goal_polygon = Polygon2d(
        [0, 0, 0], [Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1)])
    goal_polygon = goal_polygon.translate(Point2d(-63, -61))
    agent_params = param_server.addChild("agent"+str(i))
    agent = Agent(init_state,
                  behavior_models[i],
                  dynamic_models[i],
                  execution_models[i],
                  agent_2d_shape,
                  agent_params,
                  GoalDefinitionPolygon(goal_polygon),
                  map_interface)
    # world.add_agent(agent)


# viewer
viewer = MPViewer(params=param_server,
                  x_range=[800, 1100],
                  y_range=[900, 1100],
                  )

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           1]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]

for i in range(0, 36):
    world.step(sim_step_time)
    viewer.drawWorld(world)
    viewer.drawTrajectory(trajectory_list[i], 'red')
    viewer.show(block=False)
    time.sleep(sim_step_time/sim_real_time_factor)

param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "params",
                               "od8_const_vel_one_agent_written.json"))
