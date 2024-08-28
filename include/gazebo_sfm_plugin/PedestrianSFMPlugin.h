/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
/**                                                                    */
/** Copyright (c) 2021, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_

// C++
#include <string>
#include <vector>
#include <algorithm>

#include <mutex>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

// Social Force Model
#include <lightsfm/sfm.hpp>

//ros 
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin
{
  /// \brief Constructor
public:
  PedestrianSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation Inherited.
public:
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
private:
  void OnUpdate(const common::UpdateInfo &_info);


  // private: void InitializePedestrians();

  /// \brief Helper function to detect the closest obstacles.
private:
  void HandleObstacles();

  /// \brief Helper function to detect the nearby pedestrians (other actors).
private:
  void HandlePedestrians();

public:
  void reset_start_end(double start_x,double start_y,double end_x,double end_y);

 //-------------------------------------------------

   /// \brief this actor as a SFM agent
private:
  sfm::Agent sfmActor;

  /// \brief names of the other models in my walking group.
private:
  std::vector<std::string> groupNames;

  /// \brief vector of pedestrians detected.
private:
  std::vector<sfm::Agent> otherActors;

  /// \brief Maximum distance to detect nearby pedestrians.
private:
  double peopleDistance;

  /// \brief Pointer to the parent actor.
private:
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
private:
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
private:
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
private:
  ignition::math::Vector3d velocity;

  /// \brief List of connections
private:
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update.
private:
  common::Time lastUpdate;

  /// \brief List of models to ignore. Used for vector field
private:
  std::vector<std::string> ignoreModels;

  /// \brief Custom trajectory info.
private:
  physics::TrajectoryInfoPtr trajectoryInfo;

private:
  std::mutex mtx;

private:
  ros::NodeHandle rosNode; // Use the global ROS node handle
  ros::Subscriber rosSub;  // ROS subscriber

private:
  void command_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (msg->data.size() < 4) {
        ROS_WARN("Array size is less than 4 elements.");
        return;
    }
    
    float num1 = msg->data[0];
    float num2 = msg->data[1];
    float num3 = msg->data[2];
    float num4 = msg->data[3];

    ROS_INFO("num1: %f, num2: %f, num3: %f, num4: %f", num1, num2, num3, num4);

    PedestrianSFMPlugin::reset_start_end(num1,num2,num3,num4);

  }

};
}
#endif
