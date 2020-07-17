// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/rss_interface.hpp"

using ::ad::map::point::ENUCoordinate;
using ::ad::map::route::FullRoute;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;

namespace modules {
namespace world {
namespace evaluation {

bool RssInterface::initializeOpenDriveMap(
    const std::string &opendrive_file_name) {
  std::ifstream opendrive_file(opendrive_file_name);
  std::string opendrive_file_content =
      std::string{std::istreambuf_iterator<char>(opendrive_file),
                  std::istreambuf_iterator<char>()};

  ::ad::map::access::cleanup();
  // TODO(chan): define IntersectionType
  // 2nd argument: the value to reduce overlapping between two lanes they are
  // intersected
  bool result = ::ad::map::access::initFromOpenDriveContent(
      opendrive_file_content, 0.2,
      ::ad::map::intersection::IntersectionType::TrafficLight,
      ::ad::map::landmark::TrafficLightType::UNKNOWN);

  if (result == false) {
    LOG(ERROR) << "Failed to initialize from OpenDrive map : "
               << opendrive_file_content << std::endl;
  }

  return result;
}

::ad::rss::world::RssDynamics RssInterface::GenerateVehicleDynamicsParameters(
    double lon_max_accel, double lon_max_brake, double lon_min_brake,
    double lon_min_brake_correct, double lat_max_accel, double lat_min_brake,
    double lat_fluctuation_margin, double response_time) {
  ::ad::rss::world::RssDynamics dynamics;
  dynamics.alphaLon.accelMax = Acceleration(lon_max_accel);
  dynamics.alphaLon.brakeMax = Acceleration(lon_max_brake);
  dynamics.alphaLon.brakeMin = Acceleration(lon_min_brake);
  dynamics.alphaLon.brakeMinCorrect = Acceleration(lon_min_brake_correct);
  dynamics.alphaLat.accelMax = Acceleration(lat_max_accel);
  dynamics.alphaLat.brakeMin = Acceleration(lat_min_brake);
  dynamics.lateralFluctuationMargin = Distance(lat_fluctuation_margin);
  dynamics.responseTime = Duration(response_time);

  return dynamics;
}

::ad::rss::world::RssDynamics
RssInterface::GenerateDefaultVehicleDynamicsParameters() {
  return GenerateVehicleDynamicsParameters(3.5, -8., -4., -3., 0.2, -0.8, 0.1,
                                           1.);
}

::ad::map::match::Object RssInterface::GetMatchObject(
    const models::dynamic::State &agent_state, const Polygon &agent_shape,
    const Distance &match_distance) {
  ::ad::map::match::Object matching_object;
  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));

  matching_object.enuPosition.centerPoint.x =
      ENUCoordinate(bg::get<0>(agent_center));
  matching_object.enuPosition.centerPoint.y =
      ENUCoordinate(bg::get<1>(agent_center));
  matching_object.enuPosition.centerPoint.z = ENUCoordinate(0);
  matching_object.enuPosition.heading =
      ::ad::map::point::createENUHeading(agent_state(THETA_POSITION));

  matching_object.enuPosition.dimension.length =
      Distance(agent_shape.front_dist_ + agent_shape.rear_dist_);
  matching_object.enuPosition.dimension.width =
      Distance(agent_shape.left_dist_ + agent_shape.right_dist_);
  matching_object.enuPosition.dimension.height = Distance(1.5);
  matching_object.enuPosition.enuReferencePoint =
      ::ad::map::access::getENUReferencePoint();

  ::ad::map::match::AdMapMatching map_matching;
  matching_object.mapMatchedBoundingBox = map_matching.getMapMatchedBoundingBox(
      matching_object.enuPosition, match_distance, Distance(2.));

  return matching_object;
}

FullRoute RssInterface::GenerateRoute(
    const Point2d &start, const Point2d &end,
    const std::vector<map::LaneCorridorPtr> &lane_corridors,
    const ::ad::map::match::Object &matched_object) {
  std::vector<::ad::map::point::ENUPoint> routing_targets;

  for (int i = 0; i < lane_corridors.size(); ++i) {
    geometry::Line center_line = lane_corridors[i]->GetCenterLine();
    float s_start, s_end;
    if (i == 0)
      s_start = GetNearestS(center_line, start);
    else
      s_start = 0;

    if (i != lane_corridors.size() - 1)
      s_end = GetNearestS(center_line,
                          center_line.obj_.at(center_line.obj_.size() - 1));
    else
      s_end = GetNearestS(center_line, end);

    while (s_start <= s_end) {
      geometry::Point2d traj_point = GetPointAtS(center_line, s_start);
      routing_targets.push_back(::ad::map::point::createENUPoint(
          bg::get<0>(traj_point), bg::get<1>(traj_point), 0));
      s_start += 3;
    }
  }

  std::vector<FullRoute> routes;
  std::vector<double> routes_probability;

  for (const auto &position :
       matched_object.mapMatchedBoundingBox.referencePointPositions[int32_t(
           ::ad::map::match::ObjectReferencePoints::Center)]) {
    auto starting_point = position.lanePoint.paraPoint;
    auto projected_starting_point = starting_point;

    if (!::ad::map::lane::isHeadingInLaneDirection(
            starting_point, matched_object.enuPosition.heading)) {
      ::ad::map::lane::projectPositionToLaneInHeadingDirection(
          starting_point, matched_object.enuPosition.heading,
          projected_starting_point);
    }

    auto route_starting_point = ::ad::map::route::planning::createRoutingPoint(
        projected_starting_point, matched_object.enuPosition.heading);

    if (!routing_targets.empty() &&
        ::ad::map::point::isValid(routing_targets)) {
      FullRoute route = ::ad::map::route::planning::planRoute(
          route_starting_point, routing_targets,
          ::ad::map::route::RouteCreationMode::AllRoutableLanes);
      // std::cout<<route<<std::endl;
      routes.push_back(route);
      routes_probability.push_back(position.probability);
    } else {
      std::vector<FullRoute> possible_routes =
          ::ad::map::route::planning::predictRoutesOnDistance(
              route_starting_point, Distance(50.),
              ::ad::map::route::RouteCreationMode::AllRoutableLanes);
      for (const auto &possible_route : possible_routes) {
        routes.push_back(possible_route);
        routes_probability.push_back(0);
      }
    }
  }

  FullRoute final_route;

  if (routes.empty()) {
    LOG(ERROR) << "Could not find any route to the targets" << std::endl;
  } else {
    int best_route_idx = std::distance(
        routes_probability.begin(),
        std::max_element(routes_probability.begin(), routes_probability.end()));
    final_route = routes[best_route_idx];
  }
  return final_route;
}

AgentState RssInterface::ConvertAgentState(
    const models::dynamic::State &agent_state,
    const ::ad::rss::world::RssDynamics &agent_dynamics) {
  AgentState rss_state;

  rss_state.timestamp = agent_state(TIME_POSITION);
  rss_state.center = ::ad::map::point::createENUPoint(
      agent_state(X_POSITION), agent_state(Y_POSITION), 0.);
  rss_state.heading =
      ::ad::map::point::createENUHeading(agent_state(THETA_POSITION));
  rss_state.speed = Speed(agent_state(VEL_POSITION));
  rss_state.min_stopping_distance =
      CalculateMinStoppingDistance(rss_state.speed, agent_dynamics);

  return rss_state;
}

Distance RssInterface::CalculateMinStoppingDistance(
    const Speed &speed, const ::ad::rss::world::RssDynamics &agent_dynamics) {
  Distance minStoppingDistance;
  Speed MaxSpeedAfterResponseTime;

  auto result = ::ad::rss::situation::calculateSpeedAfterResponseTime(
      ::ad::rss::situation::CoordinateSystemAxis::Longitudinal,
      std::fabs(speed), Speed::getMax(), agent_dynamics.alphaLon.accelMax,
      agent_dynamics.responseTime, MaxSpeedAfterResponseTime);

  result = result &&
           ::ad::rss::situation::calculateStoppingDistance(
               MaxSpeedAfterResponseTime,
               agent_dynamics.alphaLon.brakeMinCorrect, minStoppingDistance);

  minStoppingDistance +=
      MaxSpeedAfterResponseTime * agent_dynamics.responseTime;

  if (result == false) {
    LOG(ERROR)
        << "Failed to calculate maximum possible speed after response time "
        << agent_dynamics.responseTime << std::endl;
  }

  return minStoppingDistance;
}

::ad::rss::world::WorldModel RssInterface::CreateWorldModel(
    const AgentMap &agents, const AgentId &ego_id,
    const AgentState &ego_rss_state,
    const ::ad::map::match::Object &ego_matched_object,
    const ::ad::rss::world::RssDynamics &ego_dynamics,
    const ::ad::map::route::FullRoute &ego_route) {
  std::vector<AgentPtr> relevent_agents;
  double const relevant_distance = ego_rss_state.min_stopping_distance;

  geometry::Point2d ego_center(ego_rss_state.center.x, ego_rss_state.center.y);

  for (const auto &other_agent : agents) {
    if (other_agent.second->GetAgentId() != ego_id) {
      if (geometry::Distance(ego_center,
                             other_agent.second->GetCurrentPosition()) <
          relevant_distance) {
        relevent_agents.push_back(other_agent.second);
      }
    }
  }

  ::ad::rss::map::RssSceneCreation scene_creation(ego_rss_state.timestamp,
                                                  ego_dynamics);
  ::ad::map::landmark::LandmarkIdSet
      green_traffic_lights;  // we don't care about traffic lights right now

  for (const auto &relevent_agent : relevent_agents) {
    models::dynamic::State relevent_agent_state =
        relevent_agent->GetCurrentState();
    Polygon relevent_agent_shape = relevent_agent->GetShape();
    auto const other_matched_object = GetMatchObject(
        relevent_agent_state, relevent_agent_shape, Distance(2.0));
    Speed relevent_agent_speed = relevent_agent_state(VEL_POSITION);

    ::ad::rss::world::RssDynamics relevent_agent_dynamics =
        GenerateDefaultVehicleDynamicsParameters();

    scene_creation.appendScenes(
        ::ad::rss::world::ObjectId(ego_id), ego_matched_object,
        ego_rss_state.speed, ego_dynamics, ego_route,
        ::ad::rss::world::ObjectId(relevent_agent->GetAgentId()),
        ::ad::rss::world::ObjectType::OtherVehicle, other_matched_object,
        relevent_agent_speed, relevent_agent_dynamics,
        ::ad::rss::map::RssSceneCreation::RestrictSpeedLimitMode::
            IncreasedSpeedLimit10,
        green_traffic_lights);
  }

  return scene_creation.getWorldModel();
}

bool RssInterface::RssCheck(::ad::rss::world::WorldModel world_model) {
  ::ad::rss::core::RssCheck rss_check;
  ::ad::rss::situation::SituationSnapshot situation_snapshot;
  ::ad::rss::state::RssStateSnapshot rss_state_snapshot;
  ::ad::rss::state::ProperResponse proper_response;
  ::ad::rss::world::AccelerationRestriction acceleration_restriction;

  bool result = rss_check.calculateAccelerationRestriction(
      world_model, situation_snapshot, rss_state_snapshot, proper_response,
      acceleration_restriction);

  // std::map<AgentId, bool> relevent_agents_safety_check_result;

  bool is_agent_safe = true;

  if (result == true) {
    for (auto const state : rss_state_snapshot.individualResponses) {
      is_agent_safe = is_agent_safe && !::ad::rss::state::isDangerous(state);
    }
  } else {
    LOG(ERROR) << "Failed to perform RSS check" << std::endl;
  }
  return is_agent_safe;
}

std::vector<opendrive::XodrLaneId> FindIntersection(
    const std::vector<opendrive::XodrLaneId> &route_1,
    const std::vector<opendrive::XodrLaneId> &route_2) {
  std::vector<opendrive::XodrLaneId> _r1 = route_1;
  std::vector<opendrive::XodrLaneId> _r2 = route_2;
  std::vector<opendrive::XodrLaneId> intersection;

  std::sort(_r1.begin(), _r1.end());
  std::sort(_r2.begin(), _r2.end());

  std::set_intersection(_r1.begin(), _r1.end(), _r2.begin(), _r2.end(),
                        back_inserter(intersection));

  return intersection;
}

std::vector<opendrive::XodrLaneId> GetDriveableLanePathToGoal(
    const world::map::MapInterfacePtr &map, const AgentPtr &agent) {
  std::vector<opendrive::XodrLaneId> path;

  models::dynamic::Trajectory traj = agent->GetExecutionTrajectory();
  models::dynamic::State state = traj.row(traj.rows() - 1);
  Point2d center = Point2d(state(X_POSITION), state(Y_POSITION));

  auto goal_pose = agent->GetGoalDefinition()->GetShape().center_;
  Point2d goal_pt(goal_pose(0), goal_pose(1));

  auto start_lane = map->FindXodrLane(center);
  auto start_lane_idx = start_lane->GetId();
  auto goal_lane = map->FindXodrLane(goal_pt);
  auto goal_lane_idx = goal_lane->GetId();

  path =
      map->GetRoadgraph()->FindDrivableLanePath(start_lane_idx, goal_lane_idx);
  return path;
}

std::pair<float, float> GetLaneWidth(const map::LanePtr &lane) {
  // TODO get min&max width along lane
  geometry::Line left_boundary = lane->GetLeftBoundary().GetLine();
  geometry::Line right_boundary = lane->GetRightBoundary().GetLine();

  float d = geometry::Distance(left_boundary, right_boundary);
  return std::make_pair(d, d);
}

std::pair<float, float> GetLaneLength(const map::LanePtr &lane) {
  geometry::Line left_boundary = lane->GetLeftBoundary().GetLine();
  geometry::Line right_boundary = lane->GetRightBoundary().GetLine();

  return std::make_pair(
      std::min(left_boundary.Length(), right_boundary.Length()),
      std::max(left_boundary.Length(), right_boundary.Length()));
}

::ad::rss::world::RoadArea CreateRssRoadArea(
    const map::RoadPtr &road,
    const std::vector<opendrive::XodrLaneId> &intersect) {

  ::ad::rss::world::RoadArea rssRoadArea;

  map::RoadPtr next_road(road);

  bool isIntersect = false;
  bool inIntersect = false;
  while (next_road != nullptr) {
    ::ad::rss::world::RoadSegment rssRoadSegment;

    for (auto const &lane : next_road->GetLanes()) {
      if (lane.second->GetLaneType() == 1) {
        auto lane_id = lane.first;
        ::ad::rss::world::LaneSegment rssLaneSegment;
        rssLaneSegment.id = ::ad::rss::world::LaneSegmentId(lane_id);

        isIntersect = std::find(intersect.begin(), intersect.end(), lane_id) !=
                      intersect.end();
        if (isIntersect)
          rssLaneSegment.type = ::ad::rss::world::LaneSegmentType::Intersection;
        else
          rssLaneSegment.type = ::ad::rss::world::LaneSegmentType::Normal;

        switch (lane.second->GetDrivingDirection()) {
          case 0:
            rssLaneSegment.drivingDirection =
                ::ad::rss::world::LaneDrivingDirection::Positive;
          case 1:
            rssLaneSegment.drivingDirection =
                ::ad::rss::world::LaneDrivingDirection::Negative;
          case 2:
            rssLaneSegment.drivingDirection =
                ::ad::rss::world::LaneDrivingDirection::Bidirectional;
        }

        auto length_range = GetLaneLength(lane.second);
        rssLaneSegment.length.minimum = Distance(length_range.first);
        rssLaneSegment.length.maximum = Distance(length_range.second);

        auto width_range = GetLaneWidth(lane.second);
        rssLaneSegment.width.minimum = Distance(width_range.first);
        rssLaneSegment.width.maximum = Distance(width_range.second);

        // speed

        rssRoadSegment.push_back(rssLaneSegment);
        if (isIntersect && !inIntersect)
          inIntersect = true;
        else if (!isIntersect && inIntersect)
          break;
      }
    }
    if (!isIntersect && inIntersect) {
      break;
    }
    rssRoadArea.push_back(rssRoadSegment);
    next_road = next_road->GetNextRoad();
  }

  return rssRoadArea;
}

::ad::rss::world::OccupiedRegion CreateOccupiedRegion(){

}

std::pair<float,float> calculateParametricValue(const map::LanePtr &lane,const Point2d& pt){
  // using namespace geometry;
  auto center_line = lane->GetCenterLine();
  auto left_boundary=lane->GetLeftBoundary().GetLine();

  float s = geometry::GetNearestS(center_line, pt);
  float d = geometry::Distance(left_boundary, pt);

  auto width=GetLaneWidth(lane);
  auto length=GetLaneLength(lane);
  float w=(width.first+width.second)/2;
  float l=(length.first+length.second)/2;

  return std::make_pair(s/w,d/l);
}

bool RssInterface::IsAgentSafe(const World &world, const AgentId &agent_idx) {
  world::map::MapInterfacePtr map = world.GetMap();

  AgentPtr agent = world.GetAgent(agent_idx);
  models::dynamic::Trajectory traj = agent->GetExecutionTrajectory();
  models::dynamic::State agent_state = traj.row(traj.rows() - 1);
  Point2d ego_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));
  auto ego_road_idx = map->FindCurrentRoad(ego_center);

  Polygon agent_shape = agent->GetShape();
  ::ad::map::match::Object matched_object =
      GetMatchObject(agent_state, agent_shape, Distance(2.0));

  map::RoadCorridorPtr agent_road_corridor = agent->GetRoadCorridor();

  std::vector<opendrive::XodrLaneId> ego_lane_path_indices =
      GetDriveableLanePathToGoal(map, agent);

  map::LanePtr agent_lane =
      agent_road_corridor->GetCurrentLaneCorridor(ego_center)
          ->GetCurrentLane(ego_center);


  
  
  
  // std::vector<uint32_t> ego_lanes;
  // for (auto const& lane:agent_road_corridor->GetLaneCorridorMap()){
  //   std::cout<< "test 1: " << lane.first << std::endl;
  //   ego_lanes.push_back(lane.first);
  // }

  AgentMap other_agents = world.GetAgents();
  std::vector<opendrive::XodrLaneId> intersect;

  for (auto const &other_agent : other_agents) {
    if (other_agent.first == agent_idx) continue;

    std::vector<opendrive::XodrLaneId> other_lane_path_indices =
        GetDriveableLanePathToGoal(map, other_agent.second);

    intersect =
        FindIntersection(ego_lane_path_indices, other_lane_path_indices);
  }

  auto road = agent_road_corridor->GetRoad(ego_road_idx);
  ::ad::rss::world::RoadArea rssRoadArea = CreateRssRoadArea(road, intersect);

  // for (const auto &lane_id : ego_lane_path_indices) {
  //   std::cout <<"Road id: " <<ego_road_idx <<", lane_id: "
  //   <<lane_id<< std::endl;
  //   relevent_lane_corrs.push_back(agent_road_corridor->GetLaneCorridor(lane_id));
  // }

  return true;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules