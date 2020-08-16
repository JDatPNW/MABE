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

class tictactoeWorld : public AbstractWorld {

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

  tictactoeWorld(std::shared_ptr<ParametersTable> PT_ = nullptr);
  virtual ~tictactoeWorld() = default;

  void evaluateSolo(std::shared_ptr<Organism> org1, std::shared_ptr<Organism> org2, int analyze,
                            int visualize, int debug);
  void evaluate(std::map<std::string, std::shared_ptr<Group>> &groups,
                int analyze, int visualize, int debug);

  virtual std::unordered_map<std::string, std::unordered_set<std::string>>
    requiredGroups() override;

//tictactoe Code:--------------------------------------------------------------------

void resetField(int (&field)[3][3]);
void checkWinner(int (&field)[3][3], int player, int &winner, int(&scores)[2]);
void markBox(int (&field)[3][3], int choice, int player, int(&scores)[2]);
void drawField(int (&field)[3][3]);

//------------------------------------------------------------------------------
};
