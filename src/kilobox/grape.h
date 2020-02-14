#ifndef GRAPE_H
#define GRAPE_H

#include "kilolib.h"
#include <vector>
#include <cstdlib>

using namespace Kilolib;

// Environment Information; This is for simulating a scenario

struct global_env{
        std::vector<unsigned char> task_id = {1,2,3}; // Task ID
        std::vector<unsigned char> task_demand = {100,100,100}; // Task Demand
};




// Locally-known information (This will not be shared)
struct local_env_info{
    // The first variables are updated when a task is newly found or myPartition is updated by communication
    unsigned char num_task = 0;
    std::vector<unsigned char> task_id = {};
    std::vector<uint32_t> time_stamp_when_task_updated = {};   

    std::vector<unsigned char> task_demand = {}; // Task Demand      
    std::vector<unsigned char> task_distance = {};   // Note: (Inmo) I keep it the same as partition's task_distance because we have task_freshness instead. 
    std::vector<unsigned char> num_agent_in_task = {}; // Number of Participants in each task
    
    // Partition info
    std::vector<unsigned char> agent_decision = {}; // Agents' decision

    // This variable is updated by a robot itself as time goes
    std::vector<uint32_t> task_freshness = {}; 
    
    int chosen_task = 0; // My previous decision for this given local information
    bool needCheck = false; // if it is true, a decision-making should be done based on this local info
};


// Locally-known Partiton Information (That will be shared to other robots)
struct partition{
        unsigned short int num_iterations = 0; 
        unsigned char random_time_stamp = 0;   
        unsigned char num_task = 0;
        std::vector<unsigned char> task_id = {}; // Task ID
        std::vector<unsigned char> task_demand = {}; // Task Demand
        std::vector<unsigned char> task_distance = {}; // Task Distance


        std::vector<unsigned char> agent_decision = {}; // Agents' decision
        bool satisfied = false; // Whether this agent is satisfied with the partition or not

        // Following will be removed soon
        std::vector<unsigned char> num_agent_in_task = {}; // Number of Participants in each task
        std::vector<unsigned char> agent_satisfied_flag = {}; // Agents' satisfied flag for the partition
        
};


bool IsRobotSatisfied(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid);
std::vector<unsigned char> CheckSatsified(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid);

std::vector<unsigned char> gen_content_from_partition(partition myPartition);
partition get_partition_from_content(std::vector<unsigned char> decoded_content, uint16_t kilo_uid);

unsigned int EstimateNumMsg(unsigned int content_size, unsigned int num_byte_for_content);

local_env_info UpdateLocalEnvAsNewTaskFound(local_env_info myLocalEnvInfo, int task_found_index, global_env _global_env, uint32_t kilo_ticks); // Update LocalInfo when a task is physically found

local_env_info UpdateTaskFreshness(local_env_info myLocalEnvInfo, uint32_t kilo_ticks);

#define VOID_TASK 0
#define NUM_BYTE_MSG 7 // Number of bytes for content per each message, except header

int DecisionMaking(local_env_info myLocalEnvInfo); // Decision making based on my local info, outputting the index of chosen task (Note: 0 means void task)

partition UpdatePartition(partition myPartition, uint16_t kilo_uid, int chosen_task, local_env_info myLocalEnvInfo);


local_env_info UpdateLocalEnvInfoFromPartition(local_env_info myLocalEnvInfo, partition neighbourPartition, uint8_t dist_neighbour, uint32_t kilo_ticks, uint16_t kilo_uid); // Update myLocalEnvInfo using neighbourPartition
std::vector<unsigned char> test_gen_content_to_broadcast(uint16_t kilo_uid); // For Testing Communication Function



#endif