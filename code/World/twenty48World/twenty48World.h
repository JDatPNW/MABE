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

#include <World/AbstractWorld.h>

#include <cstdlib>
#include <thread>
#include <vector>

//2048 Code:--------------------------------------------------------------------
#include <iostream>
#include <ctime>
#include <string>
//------------------------------------------------------------------------------

class twenty48World : public AbstractWorld {

public:
  static std::shared_ptr<ParameterLink<int>> modePL;
  static std::shared_ptr<ParameterLink<int>> numberOfOutputsPL;
  static std::shared_ptr<ParameterLink<int>> evaluationsPerGenerationPL;

  // int mode;
  // int numberOfOutputs;
  // int evaluationsPerGeneration;

  static std::shared_ptr<ParameterLink<std::string>> groupNamePL;
  static std::shared_ptr<ParameterLink<std::string>> brainNamePL;
  // string groupName;
  // string brainName;

  twenty48World(std::shared_ptr<ParametersTable> PT_ = nullptr);
  virtual ~twenty48World() = default;

  void evaluateSolo(std::shared_ptr<Organism> org, int analyze,
                            int visualize, int debug);
  void evaluate(std::map<std::string, std::shared_ptr<Group>> &groups,
                int analyze, int visualize, int debug);

  virtual std::unordered_map<std::string, std::unordered_set<std::string>>
    requiredGroups() override;

//2048 Code:--------------------------------------------------------------------
  int random_index_generate();
  int new_random_element();
  int is_win();
  int game_over();
  void display();
  void add_element();
  void move_left();
  void move_right();
  void move_up();
  void move_down();
  void ClearScreen();
//------------------------------------------------------------------------------
};
