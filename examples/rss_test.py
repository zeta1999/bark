# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import time
import os
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser
from bark.models.behavior import *
from bark.models.execution import ExecutionModelInterpolate
from bark.models.dynamic import SingleTrackModel
from bark.world import World
from bark.world.goal_definition import GoalDefinitionPolygon
from bark.world.agent import Agent
from bark.world.map import MapInterface
from bark.geometry.standard_shapes import CarLimousine
from bark.geometry import Point2d, Polygon2d

from bark.world.evaluation import EvaluatorRss


# Parameters Definitions
param_server = ParameterServer()

# World Definition
world = World(param_server)

# Model Definitions
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel(param_server)

behavior_model2 = BehaviorConstantVelocity(param_server)
execution_model2 = ExecutionModelInterpolate(param_server)
dynamic_model2 = SingleTrackModel(param_server)

# map_path="modules/runtime/tests/data/centered_city_highway_straight.xodr"
map_path="modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"

# Map Definition
xodr_parser = XodrParser(map_path)
map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
world.SetMap(map_interface)

# Agent Definition
agent_2d_shape = CarLimousine()
init_state = np.array([0, 1003, 1006, 0, 10])
goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
goal_polygon = goal_polygon.Translate(Point2d(883,1008))
agent_params = param_server.addChild("agent1")
agent1 = Agent(init_state,
               behavior_model,
               dynamic_model,
               execution_model,
               agent_2d_shape,
               agent_params,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent1)

agent_2d_shape2 = CarLimousine()
init_state2 = np.array([0, 1003, 1003, 0, 9])
agent_params2 = param_server.addChild("agent2")
agent2 = Agent(init_state2,
               behavior_model2,
               dynamic_model2,
               execution_model2,
               agent_2d_shape2,
               agent_params2,
               GoalDefinitionPolygon(goal_polygon),
               map_interface)
world.AddAgent(agent2)

# viewer
viewer = MPViewer(params=param_server, follow_agent_id=agent1.id)

# World Simulation
sim_step_time = param_server["simulation"]["step_time",
                                           "Step-time used in simulation",
                                           1]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                  "execution in real-time or faster",
                                                  1]
e=EvaluatorRss(agent1.id,map_path)
world.Step(0.5)
print("Initial safety",e.Evaluate(world))

for _ in range(0, 100):
  viewer.clear()
  world.Step(sim_step_time)
  print(e.Evaluate(world))
  viewer.drawWorld(world)
  # for l in agent1.road_corridor.lane_corridors:
  #   viewer.drawLaneCorridor(l)
  viewer.show(block=False)
  time.sleep(sim_step_time/sim_real_time_factor)

param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "params",
                  "city_highway_straight.json"))