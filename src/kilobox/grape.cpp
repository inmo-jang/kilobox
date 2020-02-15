#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <bitset>
#include <math.h>
#include "utils.h" // A Collection of Custom-made utility functions

#include "kilolib.h"
#include "grape.h"

using namespace Kilolib;

bool IsRobotSatisfied(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid){
  // Choose the corresponding char from the char vector - agent_satisfied_flag_vector - , according to kilo_uid
  int idx = ceil(kilo_uid/8);

  
  unsigned char agent_satisfied_flag = agent_satisfied_flag_vector[idx];
  unsigned char flag = agent_satisfied_flag & (1 << kilo_uid);
  
  printf("agent_satisfied_flag (decimal representation): %d\n", agent_satisfied_flag); // std::cout << "Byte: " << str(byte) << "\n";
  printf("kilo_uid: %d\n", kilo_uid); // std::cout << "Byte: " << str(byte) << "\n";
  printf("(1 << kilo_uid): %d\n", (1 << kilo_uid)); // std::cout << "Byte: " << str(byte) << "\n";
  printf("flag (it becomes non-zero, when it is true): %d\n", flag); // std::cout << "Byte: " << str(byte) << "\n";

  
  bool satisfied = (flag == 0) ? false: true;
  return satisfied;
}

std::vector<unsigned char> CheckSatsified(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid){
    // Choose the corresponding char from the char vector - agent_satisfied_flag_vector - , according to kilo_uid
    int idx = ceil(kilo_uid/8);

    
    unsigned char agent_satisfied_flag = agent_satisfied_flag_vector[idx];
    unsigned char agent_satisfied_flag_modified = agent_satisfied_flag | (1 << kilo_uid);
    printf("agent_satisfied_flag_rcv (decimal representation): %d\n", agent_satisfied_flag); // std::cout << "Byte: " << str(byte) << "\n";
    printf("agent_satisfied_flag_modified (decimal representation): %d\n", agent_satisfied_flag_modified); // std::cout << "Byte: " << str(byte) << "\n";  
    agent_satisfied_flag_vector[idx] = agent_satisfied_flag_modified;
     
    return agent_satisfied_flag_vector;
    
}


std::vector<unsigned char> UpdateAgentDecisionVec(std::vector<unsigned char> agent_decision, unsigned int kilo_uid, int chosen_task, int num_task){ // Update agent_decision according to the robot's decision (chosen_task). Depending on num_task, the size of agent_decision is varied.  num_task includes void_task.
 
    
    int bits_for_each_robot = (int)ceil(log2(num_task)); // e.g. it becomes 2 when num_task is 3. 

    int bit_start = bits_for_each_robot*kilo_uid + 1; 
    int idx_start = (int)ceil(bit_start/8.0)-1; // To find the corresponding element in "agent_decision"
    int bit_start_idx = (bit_start - 1)%8; // bit_start_from_the_byte
  
    int bit_end = bits_for_each_robot*(kilo_uid+1);
    int idx_end = (int)ceil(bit_end/8.0)-1;// To find the corresponding element in "agent_decision"
    int bit_end_idx = (bit_end - 1)%8;  // bit_end_from_the_byte
    
    int num_robot = EstimateNumRobot(agent_decision.size(), num_task); 
    int num_bytes_agent_decision = (int)ceil(num_robot*bits_for_each_robot/8.0f);    
    

    
    std::vector<unsigned char> agent_decision_mypart  = slice(agent_decision,idx_start,idx_end); // It may be 1 element or multiples
    std::vector<unsigned char> agent_decision_mypart_before  = {};
    if (idx_start > 0){
        agent_decision_mypart_before  = slice(agent_decision,0,idx_start-1); // It may be 1 element or multiples
    }    
    std::vector<unsigned char> agent_decision_mypart_after  = slice(agent_decision,idx_end+1,agent_decision.size()-1); // It may be 1 element or multiples
    std::vector<unsigned char> agent_decision_mypart_modified = {};
    if(agent_decision_mypart.size() > 1){

        unsigned short int agent_decision_mypart_short = Vec2Int(agent_decision_mypart);
        unsigned short int agent_decision_neutraliser = 0;
        for (int i = bit_start_idx; i <= bit_end_idx+8; i++){
            agent_decision_neutraliser = agent_decision_neutraliser | (1 << i);
        }
    
        agent_decision_neutraliser = ~agent_decision_neutraliser;
        unsigned short int agent_decision_short_neutralised = agent_decision_mypart_short & agent_decision_neutraliser;
        unsigned short int agent_decision_short_modified = agent_decision_short_neutralised | (chosen_task << bit_start_idx);
        agent_decision_mypart_modified.resize(2);        
        agent_decision_mypart_modified = Int2Vec(agent_decision_short_modified);        
    }
    else{
        unsigned char agent_decision_mypart_byte = agent_decision_mypart[0];
        unsigned char agent_decision_neutraliser = 0;
        for (int i = bit_start_idx; i <= bit_end_idx; i++){
            agent_decision_neutraliser = agent_decision_neutraliser | (1 << i);
        }
        agent_decision_neutraliser = ~agent_decision_neutraliser;
        unsigned char agent_decision_byte_neutralised = agent_decision_mypart_byte & agent_decision_neutraliser;
        unsigned char agent_decision_byte_modified = agent_decision_byte_neutralised | (chosen_task << bit_start_idx);
        agent_decision_mypart_modified.resize(1);
        agent_decision_mypart_modified[0] = agent_decision_byte_modified;
        
    }

    std::vector<unsigned char> agent_decision_modified = AddVecToAnother(agent_decision_mypart_modified, agent_decision_mypart_before);
    agent_decision_modified = AddVecToAnother(agent_decision_mypart_after, agent_decision_modified);     
    return agent_decision_modified;
    
}


int GetMyChosenTaskID(unsigned int kilo_uid, std::vector<unsigned char> agent_decision, int num_task){ // num_task includes void_task.
    int bits_for_each_robot = (int)ceil(log2(num_task)); // e.g. it becomes 2 when num_task is 3. 
    
        int bit_start = bits_for_each_robot*kilo_uid + 1; 
        int idx_start = (int)ceil(bit_start/8.0)-1; // To find the corresponding element in "agent_decision"
        int bit_start_idx = (bit_start - 1)%8; // bit_start_from_the_byte
      
        int bit_end = bits_for_each_robot*(kilo_uid+1);
        int idx_end = (int)ceil(bit_end/8.0)-1;// To find the corresponding element in "agent_decision"
        int bit_end_idx = (bit_end - 1)%8;  // bit_end_from_the_byte      
        
  
        std::vector<unsigned char> agent_decision_mypart  = slice(agent_decision,idx_start,idx_end); // It may be 1 element or multiples
        unsigned char my_chosen_task = 0; // Initialisation
        if(agent_decision_mypart.size() > 1){
            
            unsigned short int agent_decision_mypart_short = Vec2Int(agent_decision_mypart);
            unsigned short int agent_decision_neutraliser = 0;
            for (int i = bit_start_idx; i <= bit_end_idx+8; i++){
                agent_decision_neutraliser = agent_decision_neutraliser | (1 << i);
            }
            
            unsigned short int agent_decision_short_filtered = agent_decision_mypart_short & agent_decision_neutraliser;
            my_chosen_task = (unsigned char)(agent_decision_short_filtered >> bit_start_idx);
    
        }
        else{
            unsigned char agent_decision_mypart_byte = agent_decision_mypart[0];
            unsigned char agent_decision_neutraliser = 0;
            for (int i = bit_start_idx; i <= bit_end_idx; i++){
                agent_decision_neutraliser = agent_decision_neutraliser | (1 << i);
            }
            
            unsigned char agent_decision_byte_filtered = agent_decision_mypart_byte & agent_decision_neutraliser;
            my_chosen_task = (agent_decision_byte_filtered >> bit_start_idx);
            
        }   
        return my_chosen_task;
}

std::vector<unsigned short int> GetSubpopulation(std::vector<unsigned char> agent_decision, int num_task, int num_agent){
    std::vector<unsigned short int> num_agent_in_task(num_task,0);
    
    for(int _kilo_uid = 0; _kilo_uid < num_agent; _kilo_uid++){
        unsigned char my_chosen_task = GetMyChosenTaskID(_kilo_uid, agent_decision, num_task);            
        num_agent_in_task[my_chosen_task]++;               
    }
  
    return num_agent_in_task;
}

int EstimateNumRobot(int num_bytes_agent_decision, int num_task){ // Estimate the number of robots from agent_decision.size(); num_task includes void_task
    int bits_for_each_robot = (int)ceil(log2(num_task)); // e.g. it becomes 2 when num_task is 3.
    int num_robot = num_bytes_agent_decision*8/bits_for_each_robot;
    return num_robot;    
}

int NumByteForAgentDecisionVec(int num_robot, int num_task){ // Given the number of robot/task, compute the required number of bytes for "agent_decision"
    int bits_for_each_robot = (int)ceil(log2(num_task)); // e.g. it becomes 2 when num_task is 3. 
    int num_bytes_agent_decision = (int)ceil(num_robot*bits_for_each_robot/8.0f); 
    return num_bytes_agent_decision;
}

std::vector<unsigned char> gen_content_from_partition(partition myPartition){
    
    
    std::vector<unsigned char> content = {}; // Initialisation of the content to broadcast

    // Num_Iterations (This needs to be tranformed to 2-element char vector)
    std::vector<unsigned char> num_iteration_vec = Int2Vec(myPartition.num_iterations);
    content.push_back(num_iteration_vec[0]);
    content.push_back(num_iteration_vec[1]);
    // Time Stamp
    content.push_back(myPartition.random_time_stamp);
    // Task Info (ID, Demand, Distance, Participants)
    content.push_back(myPartition.num_task);    
    for(int j=0; j < myPartition.num_task ; j++){ 
        content.push_back(myPartition.task_id[j]);
    }        
    for(int j=0; j < myPartition.num_task ; j++){ 
        content.push_back(myPartition.task_demand[j]);
    }    
    for(int j=0; j < myPartition.num_task ; j++){ 
        content.push_back(myPartition.task_distance[j]);
    }

    
    // Agent Decision Flag
    content = AddVecToAnother(myPartition.agent_decision, content);

    // Another idea (but doesn't work yet): Using num_agent_in_task
    // for(int j=0; j < myPartition.num_task ; j=j+1){ 
    //     content.push_back(myPartition.num_agent_in_task[j]);
    // }
    // // Agent Satisfied Flag
    // content = AddVecToAnother(myPartition.agent_satisfied_flag, content);

    return content;
   

}

partition get_partition_from_content(std::vector<unsigned char> decoded_content, uint16_t kilo_uid){ // Decoding the receved msg into a partition information
    partition neighbourPartition;

    // Num of Iterations involved in evoluation of this partition
    std::vector<unsigned char> num_iterations_vec = slice(decoded_content, 0,1);
    neighbourPartition.num_iterations = Vec2Int(num_iterations_vec);

    // Time stamp
    neighbourPartition.random_time_stamp = decoded_content[2];    
    // Num of task
    int num_task = decoded_content[3];
    neighbourPartition.num_task = num_task;


    // Task Demand and Number of Agents in each task
    neighbourPartition.task_id.resize(num_task);
    neighbourPartition.task_demand.resize(num_task);
    neighbourPartition.task_distance.resize(num_task);
    //neighbourPartition.num_agent_in_task.resize(num_task);
    for(int j=0; j < num_task ; j=j+1){ 
        neighbourPartition.task_id[j] = decoded_content[4+j];
        neighbourPartition.task_demand[j] = decoded_content[4+num_task+j];
        neighbourPartition.task_distance[j] = decoded_content[4+2*num_task+j];
        // neighbourPartition.num_agent_in_task[j] = decoded_content[4+3*num_task+j];
    }

    // Check Satisfied flag
    int start_byte_index = 4 + 3*num_task;
    std::vector<unsigned char> agent_decision = slice(decoded_content, start_byte_index, decoded_content.size());
    neighbourPartition.agent_decision = agent_decision;
    // neighbourPartition.satisfied = IsRobotSatisfied(agent_satisfied_flag, kilo_uid);

    return neighbourPartition;
}


unsigned int EstimateNumMsg(unsigned int content_size, unsigned int num_byte_for_content){ // Get Number of Msgs required for the content to be broadcast

    // Number of msgs to broadcast
    unsigned int num_msg;
    if(content_size % num_byte_for_content == 0){
        num_msg = content_size/num_byte_for_content; // Available bytes for communication except header
    }
    else{
        num_msg = content_size/num_byte_for_content + 1; 
    }
    return num_msg;
}

local_env_info UpdateLocalEnvAsNewTaskFound(local_env_info myLocalEnvInfo, int task_found_index, global_env _global_env, uint32_t kilo_ticks)
{
    // Get the index of the task in global info (This is just for simulation)
    int task_idx_in_globalinfo = GetIndexFromVec(_global_env.task_id, task_found_index);
    if (!IsIncludedInVec(myLocalEnvInfo.task_id, task_found_index)) // New Task Found
    { // If the task found is new one, add this task to my local information
        // Save the task info to myLocalInfo
        myLocalEnvInfo.num_task++;
        myLocalEnvInfo.task_id.push_back(_global_env.task_id[task_idx_in_globalinfo]);

        myLocalEnvInfo.time_stamp_when_task_updated.push_back(kilo_ticks);
        myLocalEnvInfo.task_demand.push_back(_global_env.task_demand[task_idx_in_globalinfo]);
        myLocalEnvInfo.task_distance.push_back(1); // 1 means that the robot found a task, which is 1 hop count distance from it self.

        // myLocalEnvInfo.num_agent_in_task.push_back(0); // Number of Participants for this task

        myLocalEnvInfo.task_freshness.push_back(0); // Initilisation

        myLocalEnvInfo.agent_decision = IniVec(myLocalEnvInfo.agent_decision); // Initialisation as this is a new GRAPE

    }
    else
    { // If the task found is already in my info, then just update some values
        int task_idx = GetIndexFromVec(myLocalEnvInfo.task_id, task_found_index);
        // Update myLocalInfo
        myLocalEnvInfo.time_stamp_when_task_updated[task_idx] = kilo_ticks;
        myLocalEnvInfo.task_demand[task_idx] = _global_env.task_demand[task_idx_in_globalinfo];
        myLocalEnvInfo.task_distance[task_idx] = 1; // 1 means that the robot found a task, which is 1 hop count distance from it self.
        myLocalEnvInfo.task_freshness[task_idx] = 0; // Initilisation
    }
    myLocalEnvInfo.needCheck = true;
    return myLocalEnvInfo;
}



local_env_info UpdateTaskFreshness(local_env_info myLocalEnvInfo, uint32_t kilo_ticks){ // Forgetting task information as time goes  (But, if the robot is very close to a task, don't forget this task's info)
    
    // User Parameters
    float forget_rate = 50.0; // NOTE: The user parameter to set how quickly a robot forgets its "task_distance" value as time goes. This paramter works along with "expire_time" below. (However, it works when a robot is alone. If there is another neighbour robot who updates task_distance info, then this parameter doesn't much matter)    
    unsigned int expire_time = 1000; // NOTE: When "task_freshness" reaches this value, then a robot does not consider this task as it is not valid any longer. 
    unsigned char distance_increase_rate = 1; // 4; // NOTE: Just by a robot itself, it increases each "task_distance". Otherwise, when this info is updated by inter-robot communication, an expired task is alive again. 

    // Function
    for (int i=0; i< myLocalEnvInfo.num_task; i++){ // For each locally known task
        // if (myLocalEnvInfo.task_distance[i] != 1 && myLocalEnvInfo.task_freshness[i] < expire_time){ // TODO: For only non-task robots; This condition is just for experiment purposes.
        if (myLocalEnvInfo.task_distance[i] != 1)
        {
            if (myLocalEnvInfo.task_freshness[i] < expire_time)
            { // TODO: For only non-task robots; This condition is just for experiment purposes.

                unsigned int _task_freshness = (kilo_ticks - myLocalEnvInfo.time_stamp_when_task_updated[i]) / 32 * forget_rate; // ++forget_rate per second; The increment should be less than half, I guess. Otherwise, distance_to_task is updated by neighbours who still has lower values, which eventually causes longer time for all the robots to forget this value.
                myLocalEnvInfo.task_freshness[i] = UpperClamp(_task_freshness, expire_time);
            }

            if (myLocalEnvInfo.task_freshness[i] >= expire_time)
            { // If the task info is expired, then reset the value
                myLocalEnvInfo.task_distance[i] = 255;
                // printf("Forgot Task %d Info, Now it is invalid\n", myLocalEnvInfo.task_id[i]);
            }
            else if (myLocalEnvInfo.task_distance[i] < 255)
            { // Gradually increase task_distance, otherwise multiple robots keep a task info alive
                unsigned char _distance_increase_rate = 255 - myLocalEnvInfo.task_distance[i];
                if (distance_increase_rate < _distance_increase_rate)
                { // To avoid overflow
                    _distance_increase_rate = distance_increase_rate;
                }

                myLocalEnvInfo.task_distance[i] += _distance_increase_rate;
            }
        }
    }
    
    return myLocalEnvInfo;    
}

int DecisionMaking(local_env_info myLocalEnvInfo){ // Decision making based on my local info, outputting the index of chosen task 
    

    // User Parameter
    unsigned int MaxCost = 255;

    // Output initisalisation
    int chosen_task = VOID_TASK; // 0 means void task 

    // Utility Computation
    std::vector<unsigned int> task_cost(myLocalEnvInfo.num_task, MaxCost);  // Initialisation; It needs to be arbitrarily large if this decision-making is for minimising
    // (1) - Just based on the task distance
    for(int i=0; i<myLocalEnvInfo.num_task; i++){
        unsigned int _task_cost = (unsigned int)myLocalEnvInfo.task_distance[i]; //  + myLocalEnvInfo.task_freshness[i];
        task_cost[i] = UpperClamp(_task_cost, MaxCost);
    }

    // Utility Comparison
    int min_cost = GetMinValue(task_cost);
    int preferred_task_index = GetMinIndex(task_cost)+1; // Note: 1,2,3. means Task 1,2,3,... for the robot's knowlege. 

    if (min_cost == MaxCost)
    {
        chosen_task = VOID_TASK;
    }
    else{
        chosen_task = preferred_task_index;
    }
    return chosen_task;
}



partition UpdatePartition(partition myPartition, uint16_t kilo_uid, int chosen_task, local_env_info myLocalEnvInfo){ // Update the exiting partition using the robot's new decision

    // Update Partition from Local Info
    myPartition.num_task = myLocalEnvInfo.num_task;
    myPartition.task_id = myLocalEnvInfo.task_id;
    myPartition.task_demand = myLocalEnvInfo.task_demand;
    myPartition.task_distance = myLocalEnvInfo.task_distance;
    myPartition.agent_decision = myLocalEnvInfo.agent_decision;



    // Number of Iteration
    myPartition.num_iterations = 256; 
    // Time stamp
    myPartition.random_time_stamp = (unsigned char)std::rand();    

    // TODO: Update Partition
    // Recognise my previously chosen task, and cancel my previous selection; Update the partition using chosen_task


        // unsigned short int mf;
        // memcpy(&mf, &(m->data[7]), 2); // memcpy(dest, src, count_byte)
        // num_iterations_neighbour = mf;

        // unsigned char mg;
        // memcpy(&mg, &(m->data[9]), 1); // memcpy(dest, src, count_byte)
        // random_time_stamp_neighbour = mg;


        // // D-Mutex Algorithm (T-RO paper, Algorithm 2)
        // if ((num_iterations_neighbour > num_iterations)||((num_iterations_neighbour == num_iterations)&&(random_time_stamp_neighbour > random_time_stamp)))
        // {
        //     for (int i=0; i < num_task; i++){
        //         memcpy(&mg, &(m->data[2*i+1]), 1); 
        //         num_agent_in_task[i] = mg; 

        //     }            

        //     num_iterations = num_iterations_neighbour;
        //     random_time_stamp = random_time_stamp_neighbour;

        //     satisfied = 0; 

        // }

    myPartition.satisfied = true;

    return myPartition;
}

local_env_info UpdateLocalInfo(local_env_info myLocalEnvInfo, uint16_t kilo_uid, int chosen_task){ // Update the exiting partition using the robot's new decision

    myLocalEnvInfo.chosen_task = chosen_task; // NOTE: chosen_task becomes 1,2,3,... or VOID_TASK if no task is selected 

    // Update Partition from Local Info
    partition newPartition;
    newPartition.num_task = myLocalEnvInfo.num_task;
    newPartition.task_id = myLocalEnvInfo.task_id;
    newPartition.task_demand = myLocalEnvInfo.task_demand;
    newPartition.task_distance = myLocalEnvInfo.task_distance;

    // Check the size of agent_decision. If not enough, then increase it. 
    int num_agent_ = EstimateNumRobot(myLocalEnvInfo.agent_decision.size(), myLocalEnvInfo.num_task + 1); // NOTE: here, num_task should include void_task
    if (kilo_uid + 1 > num_agent_){ // if this robot was not considered in agent_decision
        int num_byte_to_add = NumByteForAgentDecisionVec(kilo_uid + 1, myLocalEnvInfo.num_task + 1) - myLocalEnvInfo.agent_decision.size();
        for (int i = 0; i < num_byte_to_add; i++)
        {
            myLocalEnvInfo.agent_decision.push_back(0); // Increase 1 element;
        }
    }
    
    myLocalEnvInfo.agent_decision = UpdateAgentDecisionVec(myLocalEnvInfo.agent_decision, kilo_uid, chosen_task, myLocalEnvInfo.num_task + 1);

    newPartition.agent_decision = myLocalEnvInfo.agent_decision;



    // Number of Iteration
    newPartition.num_iterations = myLocalEnvInfo.myPartition.num_iterations + 1; 
    // Time stamp
    newPartition.random_time_stamp = (unsigned char)std::rand();    


    newPartition.satisfied = true;

    myLocalEnvInfo.myPartition = newPartition;
    return myLocalEnvInfo;
}


local_env_info UpdateLocalEnvInfoFromPartition(local_env_info myLocalEnvInfo, partition neighbourPartition, uint8_t dist_neighbour, uint32_t kilo_ticks, uint16_t kilo_uid){ // Update myLocalEnvInfo using neighbourPartition
    // User Parameter
    unsigned char unit_hop_dist = 15; // The communication radius will be modulated up to this value.
    unsigned char max_dist_neighbour = 130; // The maximum possible value from "estimate_distance()" function; For normalising purpose; Needs to be set after experiments 

    // (1) Task Information Update: This part is important as Kilobot cannot localise.
    bool IsNewTaskIncluded = false;  // Initialisation
    for(int _idx_Partition = 0; _idx_Partition < neighbourPartition.task_id.size(); _idx_Partition++){
        int _task_id = neighbourPartition.task_id[_idx_Partition];
        int _expected_task_distance = (int)neighbourPartition.task_distance[_idx_Partition] + (int)(unit_hop_dist*dist_neighbour/max_dist_neighbour);
        if(IsIncludedInVec(myLocalEnvInfo.task_id,_task_id)){
            int _idx_LocalInfo = GetIndexFromVec(myLocalEnvInfo.task_id, _task_id);
            if ((int)myLocalEnvInfo.task_distance[_idx_LocalInfo] > _expected_task_distance){
                if(_expected_task_distance > 255){
                    // printf("\n\n=========================== Robot %d OverFlow111 =========================== \n\n", kilo_uid);
                    myLocalEnvInfo.task_distance[_idx_LocalInfo] = 255;
                }
                else{
                    myLocalEnvInfo.task_distance[_idx_LocalInfo] = _expected_task_distance;
                }
                
                myLocalEnvInfo.time_stamp_when_task_updated[_idx_LocalInfo] = kilo_ticks;
                myLocalEnvInfo.task_demand[_idx_LocalInfo] = neighbourPartition.task_demand[_idx_Partition];     
                myLocalEnvInfo.task_freshness[_idx_LocalInfo] = 0; // Initialisation

                myLocalEnvInfo.needCheck = true;
                // printf("\nTask Info %d is newly updated\n\n", _task_id);
            }                   
        }
        else{ // New Task
            IsNewTaskIncluded = true;
            myLocalEnvInfo.num_task++;
            myLocalEnvInfo.task_id.push_back(neighbourPartition.task_id[_idx_Partition]);

            // neighbourPartition.task_distance[_idx_Partition] + (unsigned char)(unit_hop_dist*dist_neighbour/max_dist_neighbour)
            // To avoid overflow
            if (_expected_task_distance > 255)
            {
                // printf("\n\n=========================== Robot %d OverFlow 222 =========================== \n\n", kilo_uid);
                myLocalEnvInfo.task_distance.push_back(255);
            }
            else
            {
                myLocalEnvInfo.task_distance.push_back(_expected_task_distance);
            }
            
            myLocalEnvInfo.time_stamp_when_task_updated.push_back(kilo_ticks);
            

            myLocalEnvInfo.task_demand.push_back(neighbourPartition.task_demand[_idx_Partition]);
            myLocalEnvInfo.task_freshness.push_back(0); // Initialisation

            myLocalEnvInfo.needCheck = true;
            
            // myLocalEnvInfo.agent_decision = IniVec(myLocalEnvInfo.agent_decision); // As new task found, let's start new GRAPE from scratch

        }
        
    }

    // // (2) Partition Information Update: This part is for GRAPE
    if(myLocalEnvInfo.num_task == neighbourPartition.num_task && IsNewTaskIncluded == false){ // Only update partition if task info is updated without a new task newly included (i.e., not for a new GRAPE instance)
        myLocalEnvInfo.myPartition = D_Mutex(myLocalEnvInfo.myPartition, neighbourPartition);
        
    }
    
    
        
    return myLocalEnvInfo;
}

partition D_Mutex(partition myPartition, partition neighbourPartition){ // D-Mutex Algorithm (T-RO paper, Algorithm 2)

    if( (neighbourPartition.num_iterations > myPartition.num_iterations) || ( (neighbourPartition.num_iterations == myPartition.num_iterations) && (neighbourPartition.random_time_stamp > myPartition.random_time_stamp)  ) ){
        myPartition.agent_decision = neighbourPartition.agent_decision;
        myPartition.num_iterations = neighbourPartition.num_iterations;
        myPartition.random_time_stamp = neighbourPartition.random_time_stamp;
        myPartition.satisfied = false;
    }  
    return myPartition;
}

std::vector<unsigned char> test_gen_content_to_broadcast(uint16_t kilo_uid){ // For Testing Communication Function
    
    std::vector<unsigned char> content = {}; // The contents that I am going to broadcast
    content.push_back(kilo_uid);
    content.push_back(kilo_uid+1);
    content.push_back(kilo_uid+2);
    content.push_back(kilo_uid+3);
    content.push_back(kilo_uid+4);
    content.push_back(kilo_uid+5);
    content.push_back(kilo_uid+6);
    content.push_back(kilo_uid+7);
    content.push_back(kilo_uid+8);
    content.push_back(100);
    content.push_back(110);
    content.push_back(120);

    return content;
   

}