#ifndef GRAPE_H
#define GRAPE_H

#include "kilolib.h"
#include <vector>
#include <cstdlib>

using namespace Kilolib;

// Environment Information; This is for simulating a scenario

struct global_env{
        std::vector<unsigned char> task_id = {1,2,3}; // Task ID
        std::vector<unsigned char> task_demand = {85,170,255}; // Task Demand
};







// Locally-known Partiton Information (That will be shared to other robots)
struct partition{
        unsigned short int num_iterations = 0; 
        unsigned char random_time_stamp = 0;   

        unsigned char num_task = 0;
        std::vector<unsigned char> task_id = {}; // Task ID
        std::vector<unsigned char> task_demand = {}; // Task Demand
        std::vector<unsigned char> task_distance = {}; // Task Distance


        std::vector<unsigned char> agent_decision = {0}; // Agents' decision
        bool satisfied = false; // Whether this agent is satisfied with the partition or not

        // Following will be removed soon
        std::vector<unsigned char> num_agent_in_task = {}; // Number of Participants in each task
        std::vector<unsigned char> agent_satisfied_flag = {}; // Agents' satisfied flag for the partition
        
};

// Locally-known information (This will not be shared)
struct local_env_info{
    // The first variables are updated when a task is newly found or myPartition is updated by communication
    // Partition info
    unsigned char num_task = 0;
    std::vector<unsigned char> task_id = {};
    std::vector<unsigned char> task_demand = {}; // Task Demand      
    std::vector<unsigned char> task_distance = {};   // Note: (Inmo) I keep it the same as partition's task_distance because we have task_freshness instead. 
    std::vector<unsigned char> agent_decision = {0}; // Agents' decision


    // This variable is updated by a robot itself
    std::vector<uint32_t> task_freshness = {}; 
    std::vector<uint32_t> time_stamp_when_task_updated = {};       

    // Variable facilitating the decision-making
    std::vector<unsigned char> num_agent_in_task = {}; // Number of Participants in each task
    unsigned char chosen_task_id = 0; // My previous decision for this given local information
    bool needCheck = false; // if it is true, a decision-making should be done based on this local info

    partition myPartition;
};

// The following two are for binary decision flag. 
bool IsRobotSatisfied(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid);
std::vector<unsigned char> CheckSatsified(std::vector<unsigned char> agent_satisfied_flag_vector, uint16_t kilo_uid);

// The following two are for complete-information partition
int GetMyChosenTaskID(unsigned int kilo_uid, std::vector<unsigned char> agent_decision);
std::vector<unsigned short int> GetSubpopulation(std::vector<unsigned char> agent_decision, int num_agent);
std::vector<unsigned char> UpdateAgentDecisionVec(std::vector<unsigned char> agent_decision, unsigned int kilo_uid, int chosen_task);
int EstimateNumRobot(int num_bytes_agent_decision);
int NumByteForAgentDecisionVec(int num_robot);

partition D_Mutex(partition myPartition, partition neighbourPartition); // D-Mutex Algorithm (T-RO paper, Algorithm 2)
// 
std::vector<unsigned char> gen_content_from_partition(partition myPartition);
partition get_partition_from_content(std::vector<unsigned char> decoded_content, uint16_t kilo_uid);

unsigned int EstimateNumMsg(unsigned int content_size, unsigned int num_byte_for_content);

local_env_info UpdateLocalEnvAsNewTaskFound(local_env_info myLocalEnvInfo, int task_found_index, global_env _global_env, uint32_t kilo_ticks); // Update LocalInfo when a task is physically found

local_env_info UpdateTaskFreshness(local_env_info myLocalEnvInfo, uint32_t kilo_ticks);

#define VOID_TASK 0
#define NUM_BYTE_MSG 7 // Number of bytes for content per each message, except header
#define NUM_MAX_TASK 4 // Number of maximum tasks (including the void task); This will be used for bit-modulation of "agent_decision" vector. 

#define DM_DISTANCE 1 
#define DM_BALANCE 2 

int DecisionMaking(local_env_info myLocalEnvInfo, unsigned char previous_task_id, int option); // Decision making based on my local info, outputting the index of chosen task (Note: 0 means void task)

partition UpdatePartition(partition myPartition, uint16_t kilo_uid, int chosen_task, local_env_info myLocalEnvInfo);
local_env_info UpdateLocalInfo(local_env_info myLocalEnvInfo, uint16_t kilo_uid, int chosen_task);

local_env_info UpdateLocalEnvInfoFromPartition(local_env_info myLocalEnvInfo, partition neighbourPartition, uint8_t dist_neighbour, uint32_t kilo_ticks, uint16_t kilo_uid); // Update myLocalEnvInfo using neighbourPartition
std::vector<unsigned char> test_gen_content_to_broadcast(uint16_t kilo_uid); // For Testing Communication Function

std::vector<unsigned short int> RearrangeNumAgentsInTasks(std::vector<unsigned short int> num_agent_in_task_global, local_env_info myLocalEnvInfo); // Transform to num_agent_in_task according to the robot's locally known task orders


#endif