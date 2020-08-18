//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

// Evaluates agents on how many 1s they can output. This is a purely fixed
// task
// that requires to reactivity to stimuli.
// Each correct 1 confers 1.0 point to score, or the decimal output determined
// by 'mode'.

#include "tictactoeWorld.h"

//tictactoe Code:--------------------------------------------------------------------

void tictactoeWorld::resetField(int (&field)[3][3]){
        for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                        //field[i][j] = int((3*i+j)+48); // +48 for ASCII conversion
                        field[i][j] = 5;
                }
        }
}


void tictactoeWorld::checkWinner(int (&field)[3][3], int player, int &winner, int(&scores)[2]){
        int counter = 0;
        for(int i=0; i<3; i++) {
                if(field[i][0] == field[i][1] && field[i][1] == field[i][2] && field[i][0] != 5) {
                        if(printField->get(PT)) { std::cout << "WIN side\n";} //DEBUG
                        winner = player;
                }
                if(field[0][i] == field[1][i] && field[1][i] == field[2][i] && field[0][i] != 5) {
                        if(printField->get(PT)) { std::cout << "WIN down\n";} //DEBUG
                        winner = player;
                }
        }
        if(field[0][0] == field[1][1] && field[1][1] == field[2][2] && field[0][0] != 5) {
                if(printField->get(PT)) { std::cout << "WIN dia down\n";} //DEBUG
                winner = player;
        }
        if(field[2][0] == field[1][1] && field[1][1] == field[0][2] && field[2][0] != 5) {
                if(printField->get(PT)) { std::cout << "WIN dia up\n";} //DEBUG
                winner = player;
        }
        for(int i=0; i<3; i++) {
                for(int j=0; j<3; j++) {
                        if(field[i][j] == 0 || field[i][j] == 1)
                                counter++;
                        if(counter == 9)
                                winner = 3;
                }
        }
}

void tictactoeWorld::markBox(int (&field)[3][3], int choice, int player, int(&scores)[2]){
        if(field[choice/3][choice%3] != 0 && field[choice/3][choice%3] != 1) {
                if(player == 0)
                        field[choice/3][choice%3] = 0;
                else
                        field[choice/3][choice%3] = 1;
                scores[player] = scores[player] + 1;
        }
        else{
                scores[player] = scores[player] - 1;
        }
        if(printField->get(PT)) { std::cout << "Score0 : " << scores[0] << "\n";} //DEBUG
        if(printField->get(PT)) { std::cout << "Score1 : " << scores[1] << "\n";} //DEBUG
}

void tictactoeWorld::drawField(int (&field)[3][3]){
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << " " << field[0][0] << " | " << field[0][1] << " | " << field[0][2] << "\n";
        std::cout << " - + - + -\n";
        std::cout << " " << field[1][0] << " | " << field[1][1] << " | " << field[1][2] << "\n";
        std::cout << " - + - + -\n";
        std::cout << " " << field[2][0] << " | " << field[2][1] << " | " << field[2][2] << "\n";
}

//------------------------------------------------------------------------------

std::shared_ptr<ParameterLink<int> > tictactoeWorld::modePL =
        Parameters::register_parameter(
                "WORLD_TICTACTOE-mode", 0, "0 = bit outputs before adding, 1 = add outputs");
std::shared_ptr<ParameterLink<int> > tictactoeWorld::numberOfOutputsPL =
        Parameters::register_parameter("WORLD_TICTACTOE-numberOfOutputs", 1,
                                       "number of outputs in this world");
std::shared_ptr<ParameterLink<int> > tictactoeWorld::evaluationsPerGenerationPL =
        Parameters::register_parameter("WORLD_TICTACTOE-evaluationsPerGeneration", 1,
                                       "Number of times to test each Genome per "
                                       "generation (useful with non-deterministic "
                                       "brains)");
std::shared_ptr<ParameterLink<std::string> > tictactoeWorld::groupNamePL =
        Parameters::register_parameter("WORLD_TICTACTOE_NAMES-groupNameSpace",
                                       (std::string) "root::",
                                       "namespace of group to be evaluated");
std::shared_ptr<ParameterLink<std::string> > tictactoeWorld::brainNamePL =
        Parameters::register_parameter(
                "WORLD_TICTACTOE_NAMES-brainNameSpace", (std::string) "root::",
                "namespace for parameters used to define brain");

std::shared_ptr<ParameterLink<int> > tictactoeWorld::printField =
        Parameters::register_parameter("WORLD_TICTACTOE-printField", 0,
                                       "1 prints the field (DEBUG), 0 does not");

std::shared_ptr<ParameterLink<int> > tictactoeWorld::extraPoinsForShortGame =
        Parameters::register_parameter("WORLD_TICTACTOE-extraPoinsForShortGame", 0,
                                       "1 gives a org more points if he wins more quickly"
                                       ", 0 pretty much means the opposite");

tictactoeWorld::tictactoeWorld(std::shared_ptr<ParametersTable> PT_)
        : AbstractWorld(PT_) {

        // columns to be added to ave file
        popFileColumns.clear();
        popFileColumns.push_back("score");
        popFileColumns.push_back("score_VAR"); // specifies to also record the
                                               // variance (performed automatically
                                               // because _VAR)
}



void tictactoeWorld::evaluateSolo(std::shared_ptr<Organism> org1, std::shared_ptr<Organism> org2, int analyze,
                                  int visualize, int debug) {
        auto brain1 = org1->brains[brainNamePL->get(PT)];
        auto brain2 = org2->brains[brainNamePL->get(PT)];
        for (int r = 0; r < evaluationsPerGenerationPL->get(PT); r++) {
                int steps = 12;
                int field[3][3];
                int choice = 0;
                int winner = -1;
                int scores[2] = {0,0};
                int brainScores[2] = {0,0};
                int adjustedInput = 5;
                int durationTotal = 0;
                int durationFirst = 0;
                int durationSecond = 0;
                brain1->resetBrain();
                brain2->resetBrain();
                if(printField->get(PT)) { std::cout << "New Match\n";} //DEBUG
                if(printField->get(PT)) { visualize = 1;} //DEBUG
                for(int turn = 0; turn<2; turn++) {
                        resetField(field);
                        scores[0] = 0;
                        scores[1] = 0;
                        winner = -1;
                        for(int i=0; i<steps; i++) {
                                if(turn == 0)
                                        durationFirst++;
                                else
                                        durationSecond++;
                                durationTotal++;

                                if(visualize) {drawField(field);}
                                if(i%2 == 0) {
                                        for(int j = 0; j < 9; j++) {
                                                adjustedInput = 5;
                                                if(field[j/3][j%3] == 0)
                                                        adjustedInput = 0;
                                                else if (field[j/3][j%3] == 1)
                                                        adjustedInput = 1;
                                                if(turn == 0)
                                                        brain1->setInput(j, adjustedInput);
                                                else if(turn == 1)
                                                        brain2->setInput(j, adjustedInput);

                                        }
                                        if(turn == 0) {
                                                brain1->update();
                                                choice = int(brain1->readOutput(0))%9;
                                        }
                                        else if(turn == 1) {
                                                brain2->update();
                                                choice = int(brain2->readOutput(0))%9;
                                        }
                                }
                                else if(i%2 == 1) {
                                        for(int j = 0; j < 9; j++) {
                                                adjustedInput = 5;
                                                if(field[j/3][j%3] == 0)
                                                        adjustedInput = 1;
                                                else if (field[j/3][j%3] == 1)
                                                        adjustedInput = 0;
                                                if(turn == 0)
                                                        brain2->setInput(j, adjustedInput);
                                                else if(turn == 1)
                                                        brain1->setInput(j, adjustedInput);
                                        }
                                        if(turn == 0) {
                                                brain2->update();
                                                choice = int(brain2->readOutput(0))%9;
                                        }
                                        else if(turn == 1) {
                                                brain1->update();
                                                choice = int(brain1->readOutput(0))%9;
                                        }
                                }
                                if(printField->get(PT)) {
                                        std::cout << "Player: " << i%2 << " Choice: " << choice << "\n"; //DEBUG
                                        std::cout << "Player: " << 0 << " Out: " << brain1->readOutput(0) << "\n"; //DEBUG
                                        std::cout << "Player: " << 1 << " Out: " << brain2->readOutput(0) << "\n"; //DEBUG
                                }

                                if(choice < 0 || choice > 8)
                                        choice = 4;
                                markBox(field, choice, i%2, scores);
                                checkWinner(field, i%2, winner, scores);

                                if(winner == 3) {
                                        scores[0] = scores[0]  + (steps - (2 * int(i/2)));
                                        scores[1] = scores[0];

                                        if(visualize && printField->get(PT)) {
                                                drawField(field);
                                                std::cout << "Draw: Player " << 0 << " with " << scores[0] << " Points\n";
                                                std::cout << "Draw: Player " << 1 << " with " << scores[1] << " Points\n";
                                        }
                                        break;
                                }

                                else if(winner != -1) {
                                        if(extraPoinsForShortGame->get(PT))
                                                scores[winner] = scores[winner]  + (steps - (2 * int(i/2)));
                                        scores[winner] = scores[winner] * 2;

                                        if(visualize && printField->get(PT)) {
                                                drawField(field);
                                                std::cout << "Winner: Player " << winner << " with " << scores[winner] << " Points\n";
                                                std::cout << "Loser: Player " << !winner << " with " << scores[!winner] << " Points\n";
                                        }
                                        break;
                                }
                        }
                        if(turn == 0) {
                                brainScores[0] = brainScores[0] + scores[0];
                                brainScores[1] = brainScores[1] + scores[1];
                        }
                        else if(turn == 1) {
                                brainScores[0] = brainScores[0] + scores[1];
                                brainScores[1] = brainScores[1] + scores[0];
                        }
                        if(visualize && printField->get(PT)) {
                                std::cout << "brainScore0: " << brainScores[0] << "\n"; //DEBUG
                                std::cout << "brainScore1: " << brainScores[1] << "\n"; //DEBUG
                        }
                }
                org1->dataMap.append("score", brainScores[0]);
                org2->dataMap.append("score", brainScores[1]);
                org1->dataMap.append("Player", 0);
                org2->dataMap.append("Player", 1);
                org1->dataMap.append("durationTotal", durationTotal);
                org2->dataMap.append("durationTotal", durationTotal);
                org1->dataMap.append("durationFirst", durationFirst);
                org2->dataMap.append("durationFirst", durationFirst);
                org1->dataMap.append("durationSecond", durationSecond);
                org2->dataMap.append("durationSecond", durationSecond);

                if(visualize) {
                        std::cout << "organism with ID " << org1->ID << " scored " << brainScores[0] << std::endl;
                        std::cout << "organism with ID " << org2->ID << " scored " << brainScores[1] << std::endl;
                }
        }
}

void tictactoeWorld::evaluate(std::map<std::string, std::shared_ptr<Group> > &groups,
                              int analyze, int visualize, int debug) {
        int popSize = groups[groupNamePL->get(PT)]->population.size();
        for (int i = 0; i < popSize; i = i+2) {
                evaluateSolo(groups[groupNamePL->get(PT)]->population[i], groups[groupNamePL->get(PT)]->population[i+1], analyze,
                             visualize, debug);
        }
}

std::unordered_map<std::string, std::unordered_set<std::string> >
tictactoeWorld::requiredGroups() {
        return {{groupNamePL->get(PT),
                 {"B:" + brainNamePL->get(PT) + ",9," +
                  std::to_string(numberOfOutputsPL->get(PT))}}};
        // requires a root group and a brain (in root namespace) and no addtional
        // genome,
        // the brain must have 1 input, and the variable numberOfOutputs outputs
}
