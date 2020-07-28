//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/Hintzelab/MABE/wiki/License

// Evaluates agents on how many '1's they can output. This is a purely fixed
// task
// that requires to reactivity to stimuli.
// Each correct '1' confers 1.0 point to score, or the decimal output determined
// by 'mode'.

#include "twenty48World.h"

//2048 Code:--------------------------------------------------------------------
const int MAX = 2048;
int board [4][4]={0};
static int step;
static int score;
bool GameVisualize = false;


int twenty48World::random_index_generate(){
        int random_index = rand() % 4;
        return random_index;
}

int twenty48World::new_random_element(){
        int random_element = rand() % 10;
        random_element = (random_element==0) ? 4 : 2;
        return random_element;
}
//win or lose check
int twenty48World::is_win(){
        for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                        if(board[i][j]==MAX) {
                                return 1;
                        }
                }
        }
        return 0;
}

int twenty48World::game_over(){
        int is_game_over = 1;
        for(int i=0; i<4; i++) {
                for(int j=0; j<3; j++) {
                        if(board[i][j]==0 || board[i][j+1]==0 || board[i][j] == board[i][j+1]) {
                                is_game_over = 0;
                                break;
                        }
                }
        }
        for(int j=0; j<4; j++) {
                for(int i=0; i<3; i++) {
                        if(board[i][j]==0 || board[i+1][j]==0 || board[i][j] == board[i+1][j]) {
                                is_game_over = 0;
                                break;
                        }
                }
        }
        return is_game_over;
}
//show game board
void twenty48World::display(){
        int index_i1, index_j1, index_i2, index_j2;
        //test duplication
        while(1) {
                index_i1 = random_index_generate();
                index_j1 = random_index_generate();
                index_i2 = random_index_generate();
                index_j2 = random_index_generate();
                if(index_i1==index_i2 && index_j1==index_j2) {
                        continue;
                }
                else
                        break;
        }
        //initialize
        if(step == 0) {
                if(GameVisualize) {std::cout<<"------2048------"<<std::endl;}
                for(int i=0; i<4; i++) {
                        if(GameVisualize) {std::cout<<"|-----------------------|"<<std::endl;}
                        for(int j=0; j<4; j++) {
                                if(GameVisualize) {std::cout<<"|";}
                                if(i==index_i1 && j==index_j1) {
                                        board[i][j]=2;
                                        if(GameVisualize) {std::cout<<"  "<<2<<"  ";}
                                }
                                else if(i==index_i2 && j==index_j2) {
                                        int temp = new_random_element();
                                        board[i][j] = temp;
                                        if(GameVisualize) {std::cout<<"  "<<temp<<"  ";}
                                }
                                else
                                if(GameVisualize) {std::cout<<"     ";}
                        }
                        if(GameVisualize) {std::cout<<"|"<<std::endl;}
                }
                if(GameVisualize) {std::cout<<"|-----------------------|"<<std::endl;}
        }
        //redisplay
        else{
                for(int i=0; i<4; i++) {
                        if(GameVisualize) {std::cout<<"|-----------------------|"<<std::endl;}
                        for(int j=0; j<4; j++) {
                                if(GameVisualize) {std::cout<<"|";}
                                if(board[i][j]!=0) {
                                        if(board[i][j]==1024 || board[i][j]==2048)
                                                if(GameVisualize) {std::cout<<" "<<board[i][j];}
                                        if(board[i][j]==128 || board[i][j]==256 || board[i][j]==512)
                                                if(GameVisualize) {std::cout<<" "<<board[i][j]<<" ";}
                                        if(board[i][j]==16 || board[i][j]==32 || board[i][j]==64)
                                                if(GameVisualize) {std::cout<<"  "<<board[i][j]<<" ";}
                                        if(board[i][j]==2 || board[i][j]==4 || board[i][j]==8)
                                                if(GameVisualize) {std::cout<<"  "<<board[i][j]<<"  ";}
                                }
                                else
                                if(GameVisualize) {std::cout<<"     ";}
                        }
                        if(GameVisualize) {std::cout<<"|"<<std::endl;}
                }
                if(GameVisualize) {std::cout<<"|-----------------------|"<<std::endl;}
        }
        //cout<<"step "<<step<<endl;
}
//insert random element
void twenty48World::add_element(){
        int index_i3, index_j3;
        int flag=0;
        while(1) {
                if(flag==1) break;
                index_i3 = random_index_generate();
                index_j3 = random_index_generate();
                if(board[index_i3][index_j3]==0) {
                        board[index_i3][index_j3] = new_random_element();
                        flag=1;
                }
        }
}

//move functions
void twenty48World::move_left(){
        int flag=0;
        for(int i=0; i<4; i++) {
                int n=0;
                int prev=0;
                for (int j=0; j<4; j++)
                {
                        if (n==board[i][j] && n!=0) {
                                board[i][prev] = 2*n;
                                board[i][j] = 0;
                                score+=2*n;
                                n = 0;
                                flag++;
                                continue;
                        }
                        if (board[i][j]!=0) {
                                n = board[i][j];
                                prev = j;
                        }
                }
        }
        for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                        for(int k=0; k<3; k++) {
                                if(board[i][k]==0 && board[i][k+1]!=0) {
                                        board[i][k]=board[i][k]^board[i][k+1];
                                        board[i][k+1]=board[i][k]^board[i][k+1];
                                        board[i][k]=board[i][k]^board[i][k+1];
                                        flag++;
                                }
                        }
                }
        }
        if(flag!=0) {
                add_element();
                step++;
        }
        display();
}

void twenty48World::move_right(){
        int flag=0;
        for(int i=0; i<4; i++) {
                int n=0;
                int prev=0;
                for (int j=3; j>=0; j--)
                {
                        if (n==board[i][j] && n!=0) {
                                board[i][prev] = 2*n;
                                board[i][j] = 0;
                                score+=2*n;
                                n = 0;
                                flag++;
                                continue;
                        }
                        if (board[i][j]!=0) {
                                n = board[i][j];
                                prev = j;
                        }
                }
        }
        for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                        for(int k=3; k>0; k--) {
                                if(board[i][k]==0 && board[i][k-1]!=0) {
                                        board[i][k]=board[i][k]^board[i][k-1];
                                        board[i][k-1]=board[i][k]^board[i][k-1];
                                        board[i][k]=board[i][k]^board[i][k-1];
                                        flag++;
                                }
                        }
                }
        }
        if(flag!=0) {
                add_element();
                step++;
        }
        display();
}

void twenty48World::move_up(){
        int flag=0;
        for(int i=0; i<4; i++) {
                int n=0;
                int prev=0;
                for (int j=0; j<4; j++)
                {
                        if (n==board[j][i] && n!=0) {
                                board[prev][i] = 2*n;
                                board[j][i] = 0;
                                score+=2*n;
                                n = 0;
                                flag++;
                                continue;
                        }
                        if (board[j][i]!=0) {
                                n = board[j][i];
                                prev = j;
                        }
                }
        }
        for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                        for(int k=0; k<3; k++) {
                                if(board[k][i]==0 && board[k+1][i]!=0) {
                                        board[k][i]=board[k][i]^board[k+1][i];
                                        board[k+1][i]=board[k][i]^board[k+1][i];
                                        board[k][i]=board[k][i]^board[k+1][i];
                                        flag++;
                                }
                        }
                }
        }
        if(flag!=0) {
                add_element();
                step++;
        }
        display();
}

void twenty48World::move_down(){
        int flag=0;
        for(int i=0; i<4; i++) {
                int n=0;
                int prev=0;
                for (int j=3; j>=0; j--)
                {
                        if (n==board[j][i] && n!=0) {
                                board[prev][i] = 2*n;
                                board[j][i] = 0;
                                score+=2*n;
                                n = 0;
                                flag++;
                                continue;
                        }
                        if (board[j][i]!=0) {
                                n = board[j][i];
                                prev = j;
                        }
                }
        }
        for(int i=0; i<4; i++) {
                for(int j=0; j<4; j++) {
                        for(int k=3; k>0; k--) {
                                if(board[k][i]==0 && board[k-1][i]!=0) {
                                        board[k][i]=board[k][i]^board[k-1][i];
                                        board[k-1][i]=board[k][i]^board[k-1][i];
                                        board[k][i]=board[k][i]^board[k-1][i];
                                        flag++;
                                }
                        }
                }
        }
        if(flag!=0) {
                add_element();
                step++;
        }
        display();
}


void twenty48World::ClearScreen(){
        std::cout << std::string( 25, '\n' );
}

//------------------------------------------------------------------------------

std::shared_ptr<ParameterLink<int> > twenty48World::modePL =
        Parameters::register_parameter(
                "WORLD_TWENTY48-mode", 0, "0 = bit outputs before adding, 1 = add outputs");
std::shared_ptr<ParameterLink<int> > twenty48World::numberOfOutputsPL =
        Parameters::register_parameter("WORLD_TWENTY48-numberOfOutputs", 10,
                                       "number of outputs in this world");
std::shared_ptr<ParameterLink<int> > twenty48World::evaluationsPerGenerationPL =
        Parameters::register_parameter("WORLD_TWENTY48-evaluationsPerGeneration", 1,
                                       "Number of times to test each Genome per "
                                       "generation (useful with non-deterministic "
                                       "brains)");
std::shared_ptr<ParameterLink<std::string> > twenty48World::groupNamePL =
        Parameters::register_parameter("WORLD_TWENTY48_NAMES-groupNameSpace",
                                       (std::string) "root::",
                                       "namespace of group to be evaluated");
std::shared_ptr<ParameterLink<std::string> > twenty48World::brainNamePL =
        Parameters::register_parameter(
                "WORLD_TWENTY48_NAMES-brainNameSpace", (std::string) "root::",
                "namespace for parameters used to define brain");

twenty48World::twenty48World(std::shared_ptr<ParametersTable> PT_)
        : AbstractWorld(PT_) {

        // columns to be added to ave file
        popFileColumns.clear();
        popFileColumns.push_back("score");
        popFileColumns.push_back("score_VAR"); // specifies to also record the
                                               // variance (performed automatically
                                               // because _VAR)
}

void twenty48World::evaluateSolo(std::shared_ptr<Organism> org, int analyze,
                             int visualize, int debug) {
        auto brain = org->brains[brainNamePL->get(PT)];
        for (int r = 0; r < evaluationsPerGenerationPL->get(PT); r++) {
                brain->resetBrain();

                //Loop here
                int controls_b1 = 0;
                int controls_b2 = 0;
                int controls = 0;
                int question = 0;
                int move = 0;
                step = 0;
                score = 0;
                display();
                for(int tick=0; tick<1000; tick++) {
                        if(is_win() && !question) {
                                question = !question;
                                break;
                        }
                        if(game_over()) {
                                if(visualize) {std::cout<<"~~~GAME OVER~~~"<<std::endl;}
                                break;
                        }
                        for(int axis1=0; axis1<4; axis1++) {
                                for(int axis2=0; axis2<4; axis2++) {
                                        brain->setInput((4*(axis1))+(axis2), board[axis1][axis2]); // give the brain a constant 1 (for wire brain)
                                        //brain->setInput(0,1);
                                }
                        }
                        brain->update();
                        //controls = brain->readOutput(tick);
                        controls_b1 = brain->readOutput(0);
                        controls_b2 = brain->readOutput(1);

                        //Debug - testing random vs brain
                        controls_b1 = randomValue = Random::getInt((int)0, (int)1);
                        controls_b2 = randomValue = Random::getInt((int)0, (int)1);

                        if(controls_b1 == 1 && controls_b2 == 1){
                          controls = 0; //up
                        }
                        else if(controls_b1 == 0 && controls_b2 == 0){
                          controls = 1; //down
                        }
                        else if(controls_b1 == 1 && controls_b2 == 0){
                          controls = 2; //left
                        }
                        else if(controls_b1 == 0 && controls_b2 == 1){
                          controls = 3; //right
                        }


                        if(GameVisualize) {ClearScreen();}
                        move = abs(controls*21)%4; //only Temp!!!
                        //std::cout << move << "\n";
                        switch (move) {
                        case 0:
                                move_up();
                                break;
                        case 1:
                                move_down();
                                break;
                        case 2:
                                move_left();
                                break;
                        case 3:
                                move_right();
                                break;
                        default:
                                break;
                        }
                        //show current scores
                        if(visualize) {
                                std::cout << "organism with ID " << org->ID <<" | step: " << step <<" | Current Game Score: "<<score<<std::endl;
                        }
                }

                int highestNum = 0;
                for(int axis1=0; axis1<4; axis1++) {
                        for(int axis2=0; axis2<4; axis2++) {
                                if(board[axis1][axis2] > highestNum){
                                  highestNum = board[axis1][axis2];
                                }
                        }
                }

                double brainScore = 0.0;
                brainScore = score;
                if (brainScore < 0.0)
                        brainScore = 0.0;
                org->dataMap.append("score", brainScore);
                org->dataMap.append("steps", step);
                org->dataMap.append("higestNum", highestNum);
                if(visualize)
                        std::cout << "organism with ID " << org->ID << " scored " << brainScore
                                  << std::endl;
        }
}

void twenty48World::evaluate(std::map<std::string, std::shared_ptr<Group> > &groups,
                         int analyze, int visualize, int debug) {
        int popSize = groups[groupNamePL->get(PT)]->population.size();
        for (int i = 0; i < popSize; i++) {
                evaluateSolo(groups[groupNamePL->get(PT)]->population[i], analyze,
                             visualize, debug);
        }
}

std::unordered_map<std::string, std::unordered_set<std::string> >
twenty48World::requiredGroups() {
        return {{groupNamePL->get(PT),
                 {"B:" + brainNamePL->get(PT) + ",16," +
                  std::to_string(numberOfOutputsPL->get(PT))}}};
        // requires a root group and a brain (in root namespace) and no addtional
        // genome,
        // the brain must have 1 input, and the variable numberOfOutputs outputs
}
