//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

#pragma once

#include "../AbstractWorld.h"
#include "../../Group/Group.h"

#include <cstdlib>
#include <thread>
#include <vector>

#include <osgViewer/Viewer>

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>

class PhysicsWorld : public AbstractWorld {

public:
  static std::shared_ptr<ParameterLink<int>> modePL;
  static std::shared_ptr<ParameterLink<bool>> visualizationPL;
  static std::shared_ptr<ParameterLink<int>> numberOfOutputsPL;
  static std::shared_ptr<ParameterLink<double>> jointLengthPL;
  static std::shared_ptr<ParameterLink<int>> evaluationsPerGenerationPL;
  static std::shared_ptr<ParameterLink<double>> timeStepIntervalPL;
  static std::shared_ptr<ParameterLink<double>> simulationLengthPL;
  static std::shared_ptr<ParameterLink<double>> physicsInteractionIntervalPL;

  const double default_ground_width = 200;
  const double default_wall_thickness = 1;
  const double default_wall_height = 1;
  const double default_spawn_range = 0.9*default_ground_width/2;
  dart::simulation::WorldPtr physicsWorld;
  dart::dynamics::SkeletonPtr physicsObject;
  //bool visualization = false;
  // int mode;
  // int numberOfOutputs;
  // int evaluationsPerGeneration;

  static std::shared_ptr<ParameterLink<std::string>> groupNamePL;
  static std::shared_ptr<ParameterLink<std::string>> brainNamePL;
  // string groupName;
  // string brainName;

  PhysicsWorld(std::shared_ptr<ParametersTable> PT_ = nullptr);
  virtual ~PhysicsWorld() = default;
  dart::dynamics::SkeletonPtr createGround();
  dart::dynamics::SkeletonPtr createWall();
  void applyForce(dart::dynamics::SkeletonPtr skel, std::vector<double> brainOutputs);
  //void applyForce(dart::dynamics::BodyNode* body, double x, double y);
  void setupVisualization();
  void setupPhysics();
  void setupCaterpillar();
  void morphBody();
  void evaluateSolo(std::shared_ptr<Organism> org, int analyze, int visualize, int debug);
  virtual void evaluate(std::map<std::string, std::shared_ptr<Group>> &groups, int analyze, int visualize, int debug);

  dart::dynamics::BodyNode* makeRootBody(const dart::dynamics::SkeletonPtr& pendulum, const std::string& name);
  void setGeometry(const dart::dynamics::BodyNodePtr& bn);
  dart::dynamics::BodyNode* addBody(const dart::dynamics::SkeletonPtr& pendulum, dart::dynamics::BodyNode* parent, const std::string& name);

  virtual std::unordered_map<std::string, std::unordered_set<std::string>>
  requiredGroups() override {
    return {{groupNamePL->get(PT),
             {"B:" + brainNamePL->get(PT) + ",1," +
              std::to_string(numberOfOutputsPL->get(PT))}}};
    // requires a root group and a brain (in root namespace) and no addtional
    // genome,
    // the brain must have 1 input, and the variable numberOfOutputs outputs
  }
};
