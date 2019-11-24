//----------------------------------------------------------
// Evokilo1 - First experiment in evolved kilobot controller
// (c) Simon Jones 2015
//----------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>


#include "kilolib.h"
#include "evokilo1.h"

using namespace Kilolib;

//-------------------------------------------------------------
// Add your file logging pointer here
//-------------------------------------------------------------
// Single class variable for logging file pointer
FILE *Minimal_example::lfp  = NULL;
FILE *Orbit_star::lfp       = NULL;
FILE *Orbit_planet::lfp     = NULL;
FILE *Evokilo1::lfp         = NULL;
FILE *Evokilo2::lfp         = NULL;
FILE *Evokilo3::lfp         = NULL;
FILE *Evokilo4::lfp         = NULL;
FILE *Stigmergy_example::lfp= NULL;
FILE *Simple_example::lfp   = NULL;
FILE *Estimation_distance_to_task::lfp            = NULL;
FILE *Estimation_distance_to_task_forget::lfp            = NULL;
FILE *Grape::lfp            = NULL;


//-------------------------------------------------------------
void Minimal_example::setup()
{
    last_update     = kilo_ticks;
}
void Minimal_example::loop()
{
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        set_color(RGB((kilo_ticks>>4)%2,0,0));
    }
}
//-------------------------------------------------------------



//-------------------------------------------------------------
void Stigmergy_example::setup()
{
    last_update     = kilo_ticks;
    enable_pheromone();
}
void Stigmergy_example::loop()
{
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        set_color(RGB((kilo_ticks>>4)%2,0,0));
        int16_t e = get_environment();
        int16_t e1 = get_environment(false);
        
        printf("pheromone %d %d\n", e, e1);
    }
}
//-------------------------------------------------------------



//-------------------------------------------------------------
#define TASK_NULL 0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3
#define TASK_4 4
#define TASK_5 5

void Simple_example::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Simple_example::tx_message;
    kilo_message_rx         = (message_rx_t)&Simple_example::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;
}
void Simple_example::loop()
{
    
    int task_found_neighbour;
    int task_found_env;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        // Get Task Info from Local Communication
        task_found_neighbour = allocation;

        // Get Task Info from Environment
        task_found_env = get_environment(); 
        if (task_found_env!=0){
            preferred_task = task_found_env;
            allocation = TASK_NULL; // Initialise
        }
        else if ((task_found_env==0)&&(task_found_neighbour!=0)){
            preferred_task = task_found_neighbour;
        }

        
        
        printf("Robot %d found Task %d\n",kilo_uid, preferred_task);
        


        // Broadcast         
        memcpy(msg.data, &preferred_task, 4); // memcpy(dest, src, count_byte)
        msg.crc     = message_crc(&msg);

        switch (preferred_task){
            case TASK_NULL: 
                set_color(RGB(2,2,2));
                break;
            case TASK_1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case TASK_2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case TASK_3: // Task 3
                set_color(RGB(0,2,2));     
                break;
            case TASK_4: // Task 4
                set_color(RGB(2,0,0));     
                break;

            case TASK_5: // Task 5
                set_color(RGB(0,0,2));     
                break;
        }

        
        
    }

}
//-------------------------------------------------------------


//-------------------------------------------------------------
#define TASK_NULL 0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3

void Estimation_distance_to_task::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Estimation_distance_to_task::tx_message;
    kilo_message_rx         = (message_rx_t)&Estimation_distance_to_task::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;

    // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
    memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)    
}
void Estimation_distance_to_task::loop()
{
    // NB: All the variables will be reinitialised at each loop
    int task_found_neighbour;
    int task_found_env;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        // Get Task Info from Environment (if the robot found a new task by itself)
        task_found_env = get_environment(); 
        // printf("Robot %d found Task %d\n", kilo_uid, task_found_env);
        // printf("Robot %d Distance to Tasks (%d, %d, %d)\n", kilo_uid, distance_to_task[0], distance_to_task[1], distance_to_task[2]);
        if (task_found_env!=0){
            distance_to_task[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self. 
            
        }
        
        
        // Utility Comparison
        int min_cost;
        min_cost = (int)*std::min_element(distance_to_task.begin(), distance_to_task.end());
        int preferred_task; 
        preferred_task = std::min_element(distance_to_task.begin(), distance_to_task.end()) - distance_to_task.begin() + 1;
        if (min_cost < chosen_task_cost){
            printf("Robot %d joins to Task %d because its delta_cost is %d\n", kilo_uid, preferred_task, chosen_task_cost - min_cost);
            chosen_task = preferred_task;
            chosen_task_cost = min_cost;
            
            
        }
        // printf("Robot %d: Minimum Cost %d; Chosen task %d\n",kilo_uid, chosen_task_cost, chosen_task);
        

        num_iterations = (unsigned short int)std::rand();


        // Broadcast         
        memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
        memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)

        // memcpy(msg.data, &preferred_task, 4); // memcpy(dest, src, count_byte)        
        msg.crc     = message_crc(&msg);

        // LED Display
        switch (chosen_task){
            case TASK_NULL: 
                set_color(RGB(2,2,2));
                break;
            case TASK_1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case TASK_2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case TASK_3: // Task 3
                set_color(RGB(0,2,2));     
                break;
            
        }

        
        
    }

}
//-------------------------------------------------------------



//-------------------------------------------------------------
#define TASK_NULL 0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3

void Estimation_distance_to_task_forget::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Estimation_distance_to_task_forget::tx_message;
    kilo_message_rx         = (message_rx_t)&Estimation_distance_to_task_forget::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;

    // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
    memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)    
}
void Estimation_distance_to_task_forget::loop()
{
    // NB: All the variables will be reinitialised at each loop
    int task_found_neighbour;
    int task_found_env;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        // Get Task Info from Environment (if the robot found a new task by itself)
        task_found_env = get_environment(); 
        // printf("Robot %d found Task %d\n", kilo_uid, task_found_env);
        if (task_found_env!=0){
            // distance_to_task[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self. 
            distance_to_task_uint[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self.             
            task_info_time_stamp[task_found_env-1] = kilo_ticks; 
            
        }
        else{ // For forgetting untracked tasks
            for (int i=0; i< num_task; i++){ // For each task
                if (distance_to_task_uint[i] != 1 && distance_to_task_uint[i] < 255){ // For only non-task robots; This condition is just for experiment purposes. 
                    distance_to_task_uint[i] = distance_to_task_uint[i] + (kilo_ticks - task_info_time_stamp[i])/32*unit_hop_dist*parameter_forgetting; // ++parameter_forgetting*unit_hop_dist per second; The increment should be less than half, I guess. Otherwise, distance_to_task is updated by neighbours who still has lower values, which eventually causes longer time for all the robots to forget this value. 
                    if (distance_to_task_uint[i] > 230){ // 230, which is the value that is arbitrary large but below than 255. To cut off overflow. This value functions as bumber. 
                        distance_to_task_uint[i] = 255;
                    }
                }

            }
        }
  
        // Utility Computation
        for(int i=0; i<num_task; i++){
            task_cost[i] = distance_to_task_uint[i];
        }
        if (chosen_task != TASK_NULL){
            chosen_task_cost = task_cost[chosen_task-1];
        }
        
        // Utility Comparison
        int min_cost;
        min_cost = *std::min_element(task_cost.begin(), task_cost.end());
        int preferred_task; 
        preferred_task = std::min_element(task_cost.begin(), task_cost.end()) - task_cost.begin() + 1;
        if (min_cost == 255){
            chosen_task = TASK_NULL;
            chosen_task_cost = 255;
        }
        else if (min_cost < chosen_task_cost){
            // printf("Robot %d joins to Task %d because its delta_cost is %d\n", kilo_uid, preferred_task, chosen_task_cost - min_cost);
            chosen_task = preferred_task;
            chosen_task_cost = min_cost;
        }
        // printf("Robot %d dist_tasks (%d, %d, %d) at (%d, %d, %d) at t %d\n", kilo_uid, distance_to_task_uint[0], distance_to_task_uint[1], distance_to_task_uint[2], task_info_time_stamp[0], task_info_time_stamp[1], task_info_time_stamp[2], kilo_ticks);            
        // This is for analysis "Analysis Forgetting Task"
        // printf("%d \t\t %d \t %d \t %d \t\t %d \t %d \t %d \t\t %d \t \n", kilo_uid, distance_to_task_uint[0], distance_to_task_uint[1], distance_to_task_uint[2], task_info_time_stamp[0], task_info_time_stamp[1], task_info_time_stamp[2], kilo_ticks);            
        // printf("Robot %d: Minimum Cost (%d) preferred_task (%d) chosen_task_cost (%d) Finally chosen task (%d)\n",kilo_uid, min_cost, preferred_task, chosen_task_cost, chosen_task);
        

        num_iterations = (unsigned short int)std::rand();


        // Broadcast      
        for(int i=0; i<num_task; i++){
            if (distance_to_task_uint[i] > 255){
                distance_to_task[i] = 255;
            }
            else{
                distance_to_task[i] = distance_to_task_uint[i];
            }
            
        } 
        memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
        memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)

        // memcpy(msg.data, &preferred_task, 4); // memcpy(dest, src, count_byte)        
        msg.crc     = message_crc(&msg);

        // LED Display
        switch (chosen_task){
            case TASK_NULL: 
                set_color(RGB(2,2,2));
                break;
            case TASK_1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case TASK_2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case TASK_3: // Task 3
                set_color(RGB(0,1,2));     
                break;
            
        }

        
        
    }

}
//-------------------------------------------------------------




//-------------------------------------------------------------
#define TASK_NULL 0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3

void Grape::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Grape::tx_message;
    kilo_message_rx         = (message_rx_t)&Grape::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;

    // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
    memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
    memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
    memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)    
}
void Grape::loop()
{
    // NB: All the variables will be reinitialised at each loop
    int task_found_neighbour;
    int task_found_env;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        // Get Task Info from Environment (if the robot found a new task by itself)
        task_found_env = get_environment(); 
        // printf("Robot %d found Task %d\n", kilo_uid, task_found_env);
        if (task_found_env!=0){
            // distance_to_task[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self. 
            distance_to_task_uint[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self.             
            task_info_time_stamp[task_found_env-1] = kilo_ticks; 
            
        }
        else{ // For forgetting untracked tasks
            for (int i=0; i< num_task; i++){ // For each task
                if (distance_to_task_uint[i] != 1 && distance_to_task_uint[i] < 255){ // For only non-task robots; This condition is just for experiment purposes. 
                    distance_to_task_uint[i] = distance_to_task_uint[i] + (kilo_ticks - task_info_time_stamp[i])/32*unit_hop_dist*parameter_forgetting; // ++parameter_forgetting*unit_hop_dist per second; The increment should be less than half, I guess. Otherwise, distance_to_task is updated by neighbours who still has lower values, which eventually causes longer time for all the robots to forget this value. 
                    if (distance_to_task_uint[i] > 230){ // 230, which is the value that is arbitrary large but below than 255. To cut off overflow. This value functions as bumber. 
                        distance_to_task_uint[i] = 255;
                    }
                }

            }
        }
  
        // Utility Computation
        for(int i=0; i<num_task; i++){
            task_cost[i] = distance_to_task_uint[i];
        }
        if (chosen_task != TASK_NULL){
            chosen_task_cost = task_cost[chosen_task-1];
        }
        
        // Utility Comparison
        int min_cost;
        min_cost = *std::min_element(task_cost.begin(), task_cost.end());
        int preferred_task; 
        preferred_task = std::min_element(task_cost.begin(), task_cost.end()) - task_cost.begin() + 1;
        if (min_cost == 255){
            chosen_task = TASK_NULL;
            chosen_task_cost = 255;
        }
        else if (min_cost < chosen_task_cost){
            // printf("Robot %d joins to Task %d because its delta_cost is %d\n", kilo_uid, preferred_task, chosen_task_cost - min_cost);
            chosen_task = preferred_task;
            chosen_task_cost = min_cost;
        }
        // printf("Robot %d dist_tasks (%d, %d, %d) at (%d, %d, %d) at t %d\n", kilo_uid, distance_to_task_uint[0], distance_to_task_uint[1], distance_to_task_uint[2], task_info_time_stamp[0], task_info_time_stamp[1], task_info_time_stamp[2], kilo_ticks);            
        // This is for analysis "Analysis Forgetting Task"
        // printf("%d \t\t %d \t %d \t %d \t\t %d \t %d \t %d \t\t %d \t \n", kilo_uid, distance_to_task_uint[0], distance_to_task_uint[1], distance_to_task_uint[2], task_info_time_stamp[0], task_info_time_stamp[1], task_info_time_stamp[2], kilo_ticks);            
        // printf("Robot %d: Minimum Cost (%d) preferred_task (%d) chosen_task_cost (%d) Finally chosen task (%d)\n",kilo_uid, min_cost, preferred_task, chosen_task_cost, chosen_task);
        

        num_iterations = (unsigned short int)std::rand();


        // Broadcast      
        for(int i=0; i<num_task; i++){
            if (distance_to_task_uint[i] > 255){
                distance_to_task[i] = 255;
            }
            else{
                distance_to_task[i] = distance_to_task_uint[i];
            }
            
        } 
        memcpy(&msg.data[1], &num_agent_in_task[0], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[2], &distance_to_task[0], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[3], &num_agent_in_task[1], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[4], &distance_to_task[1], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[5], &num_agent_in_task[2], 1); // memcpy(dest, src, count_byte)
        memcpy(&msg.data[6], &distance_to_task[2], 1);  // memcpy(dest, src, count_byte)
        memcpy(&msg.data[7], &num_iterations, 2);      // memcpy(dest, src, count_byte)
        memcpy(&msg.data[9], &random_time_stamp, 1);   // memcpy(dest, src, count_byte)

        // memcpy(msg.data, &preferred_task, 4); // memcpy(dest, src, count_byte)        
        msg.crc     = message_crc(&msg);

        // LED Display
        switch (chosen_task){
            case TASK_NULL: 
                set_color(RGB(2,2,2));
                break;
            case TASK_1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case TASK_2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case TASK_3: // Task 3
                set_color(RGB(0,1,2));     
                break;
            
        }

        
        
    }

}
//-------------------------------------------------------------




//-------------------------------------------------------------
float sigmoid(float x) {return tanh(x);}

float *NN::nn_update(float *inputs)
{
    float *ptr=&nn_weights[0];
    for(int i=0; i<NN_NUM_HIDDEN; i++)
    {
        float sum = 0;
        // First the input nodes
        for(int j=0; j<NN_NUM_INPUTS; j++)
            sum += inputs[j] * (*ptr++);
        // Then the hidden nodes
        if (rec)
            for(int j=0; j<NN_NUM_HIDDEN; j++)
                sum += nn_hidden[j] * (*ptr++);
        // Then the bias
        sum += 1.0 * (*ptr++);
        // Update the neuron value with the weighted input pushed through the transfer function
        nn_hidden[i] = sigmoid(sum);
    }
    // Work out the output node values
    for(int i=0; i<NN_NUM_OUTPUTS; i++)
    {
        float sum = 0;
        for(int j=0; j<NN_NUM_HIDDEN; j++)
            sum += nn_hidden[j] * (*ptr++);
        // Bias
        sum += 1.0 * (*ptr++);
        nn_outputs[i] = sigmoid(sum);
    }
    return &nn_outputs[0];
    
}



//#define DEBUG

#define NEST 1
#define FOOD 2


void Evokilo1::setup()
{
    last_update     = kilo_ticks;
    last_region     = 0;
}

void Evokilo1::loop()
{
    // Run the NN at the same rate as the message send, roughly twice a second
    // Always send a message
    int region;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        region = get_environment();

        // Every cycle, build the inputs to the neuron net, compute the outputs
        // and set the actuators using the outputs

        // Little state machine for food transport: always collect food if in food area
        // and deposit it if in nest area. To reduce effect of noise, only trigger
        // change of state with two successive region values that are the same
        if (carrying && (region == NEST) && (last_region == NEST))
        {
            carrying = 0;
            total_food++;
        }
        else if (!carrying && (region == FOOD) && (last_region == FOOD))
        {
            carrying = 1;
            total_pickup++;
        }
        
        // Bias
        inputs[0]   = 1.0;
        // Nest
        inputs[1]   = (region == NEST) ? 1.0 : -1.0;
        // Food
        inputs[2]   = (region == FOOD) ? 1.0 : -1.0;
        // carrying food
        inputs[3]   = carrying ? 1.0 : -1.0;
        
        // Run the neural net
        outputs     = nn.nn_update(&inputs[0]);

        // Motor control
        int d = (outputs[0] >= 0 ? 1 : 0) | (outputs[1] >= 0 ? 2 : 0);
        switch(d)
        {
            case(0):
                set_motors(0,0);
                break;
            case(1):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_turn_left,0);
                break;
            case(2):
                if (last_output == 0) spinup_motors();
                set_motors(0,kilo_turn_right);
                break;
            case(3):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
        }

        // visualise the internal state
        set_color(RGB(carrying?3:0, (region==NEST)?3:0, (region==FOOD)?3:0));
                

        // remember where we were
        last_region = region;
        last_output = d;

#ifdef DEBUG
        printf("%lu %d %d\r\n", kilo_ticks,region,carrying);
#endif
    }

    
    
    //===========================================================================
    // Stage only, non kilobot logging
    {
        usec_t time = pos->GetWorld()->SimTimeNow();
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[1],inputs[2],inputs[3], outputs[0],outputs[1]);
            Evokilo1::log(buf);
        }
    }
    
}
//-------------------------------------------------------------

//-------------------------------------------------------------
void Evokilo2::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Evokilo2::tx_message;
    kilo_message_rx         = (message_rx_t)&Evokilo2::message_rx;
    
    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    
    last_region = 0;
}

void Evokilo2::loop()
{
    // Run the NN at the same rate as the message send, roughly twice a second
    // Always send a message
    
    int region;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        region      = get_environment();
        
        // Every cycle, build the inputs to the neuron net, compute the outputs
        // and set the actuators using the outputs
        
        // Little state machine for food transport: always collect food if in food area
        // and deposit it if in nest area
        if (carrying && (region == NEST) && (last_region == NEST))
        {
            carrying = 0;
            total_food++;
        }
        else if (!carrying && (region == FOOD) && (last_region == FOOD))
        {
            carrying = 1;
            total_pickup++;
        }
        
        // Nest
        inputs[0]   = (region == NEST) ? 1.0 : -1.0;
        // Food
        inputs[1]   = (region == FOOD) ? 1.0 : -1.0;
        // carrying food
        inputs[2]   = carrying ? 1.0 : -1.0;
        
        // Distance to nearest neighbours
        inputs[3]   = (float)min_dist / 100.0;
        // Number of neighbours
        inputs[4]   = messages;
        // Average message
        float avgmsg = messages > 0 ? msgsum / messages : 0.0;
        inputs[5]   = avgmsg;
        
        // Run the neural net
        outputs     = nn.nn_update(&inputs[0]);
        
        // Motor control
        int d = (outputs[0] >= 0 ? 1 : 0) | (outputs[1] >= 0 ? 2 : 0);
        switch(d)
        {
            case(0):
                set_motors(0,0);
                break;
            case(1):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_turn_left,0);
                break;
            case(2):
                if (last_output == 0) spinup_motors();
                set_motors(0,kilo_turn_right);
                break;
            case(3):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
        }
        
        // Message output direct from neuron
        float m = outputs[2];
        memcpy(msg.data, &m, 4);
        msg.crc     = message_crc(&msg);
        
        
        // visualise the internal state
        set_color(RGB(carrying?2:0, 0, 0));
        set_color_msg((outputs[2] + 1.0) / 2.0);
        //set_color_msg(carrying);
        
        // Clear the message variables
        messages    = 0;
        msgsum      = 0;
        min_dist    = 150;
        
        // remember last region visited
        last_region = region;
        last_output = d;
        
#ifdef DEBUG
        printf("%lu %d %d\r\n", kilo_ticks,region,carrying);
#endif
    }
    
    //===========================================================================
    // Stage only, non kilobot logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[0],inputs[1],inputs[2], outputs[0],outputs[1], outputs[2]);
            Evokilo2::log(buf);
        }
    }
    
}
//-------------------------------------------------------------

//-------------------------------------------------------------
void Evokilo3::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Evokilo3::tx_message;
    kilo_message_rx         = (message_rx_t)&Evokilo3::message_rx;
    
    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    
    last_region = 0;
}

void Evokilo3::loop()
{
    // Run the NN at the same rate as the message send, roughly twice a second
    // Always send a message
    
    int region;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        
        // Bias
        inputs[0]   = 1.0;
        // Distance to nearest neighbours
        inputs[1]   = (float)min_dist / 100.0;
        // Number of neighbours
        inputs[2]   = messages;
        
        // Run the neural net
        outputs     = nn.nn_update(&inputs[0]);
        
        // Motor control
        int d = (outputs[0] >= 0 ? 1 : 0) | (outputs[1] >= 0 ? 2 : 0);
        switch(d)
        {
            case(0):
                set_motors(0,0);
                break;
            case(1):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_turn_left,0);
                break;
            case(2):
                if (last_output == 0) spinup_motors();
                set_motors(0,kilo_turn_right);
                break;
            case(3):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
        }
        
       
        // visualise the internal state
        //set_color(RGB(carrying?2:0, (region==NEST)?3:0, (region==FOOD)?3:0));
        //set_color_msg((outputs[2] + 1.0) / 2.0);
        //set_color_msg(carrying);
        
        // Clear the message variables
        messages    = 0;
        min_dist    = 150;
        

    }
    
    //===========================================================================
    // Stage only, non kilobot logging
    {
        usec_t time = pos->GetWorld()->SimTimeNow();
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f, 0.0\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[1],inputs[2], 0.0, outputs[0],outputs[1]);
            Evokilo3::log(buf);
        }
    }
    
}
//-------------------------------------------------------------


//-------------------------------------------------------------
void Evokilo4::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Evokilo4::tx_message;
    kilo_message_rx         = (message_rx_t)&Evokilo4::message_rx;
    
    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    
    last_region = 0;
    // Clear record of message ids. There are no id==0
    for(int i=0; i<MMEM; msg_id[i++] = 0);
        
}


void Evokilo4::loop()
{
    // Run the NN at the same rate as the message send, roughly twice a second
    // Always send a message
    
    int region;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        region      = get_environment();
        
        // Every cycle, build the inputs to the neuron net, compute the outputs
        // and set the actuators using the outputs

        
        // Change in neighbourhood
        // This measures how the ids of the neighbours who have sent messages
        // has changed since the last time
        // if a message id is not in the circular buffer of recent message ids,
        // this is regarded as salient, and increases the input to that neuron
        
        // Bias
        inputs[0]   = 1.0;

        
        // Distance to nearest neighbours
        inputs[1]   = (float)min_dist / 100.0;
        // Number of neighbours
        inputs[2]   = messages;
        // Average message
        float avgmsg = messages > 0 ? msgsum / messages : 0.0;
        inputs[3]   = avgmsg;
        // Nest
        inputs[4]   = (region == NEST) ? 1.0 : -1.0;
        // Food
        inputs[5]   = (region == FOOD) ? 1.0 : -1.0;
        // New ids seen this time
        inputs[6]   = new_id;
        
        // Run the neural net
        outputs     = nn.nn_update(&inputs[0]);
        
        // Motor control
        int d = (outputs[0] >= 0 ? 1 : 0) | (outputs[1] >= 0 ? 2 : 0);
        switch(d)
        {
            case(0):
                set_motors(0,0);
                break;
            case(1):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_turn_left,0);
                break;
            case(2):
                if (last_output == 0) spinup_motors();
                set_motors(0,kilo_turn_right);
                break;
            case(3):
                if (last_output == 0) spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
        }
        
        // Message output direct from neuron
        *(float*)msg.data = outputs[2];
        msg.data[4] = kilo_uid & 0xff;
        msg.crc     = message_crc(&msg);
        
        
        // visualise the internal state
        //set_color(RGB(carrying?2:0, (region==NEST)?3:0, (region==FOOD)?3:0));
        set_colorf(messages > 0 ? (float)new_id/messages : 0);
        //set_colorf(0.5);
        set_color_msg((outputs[2] + 1.0) / 2.0);
        //set_color_msg(carrying);
        
        // Clear the message variables
        messages    = 0;
        msgsum      = 0;
        min_dist    = 150;
        new_id      = 0;
        
        // remember last region visited
        last_region = region;
        last_output = d;
        
#ifdef DEBUG
        printf("%lu %d %d\r\n", kilo_ticks,region,carrying);
#endif
    }
    
    //===========================================================================
    // Stage only, non kilobot logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[1],inputs[2],inputs[3], outputs[0],outputs[1], outputs[2]);
            Evokilo4::log(buf);
        }
    }
    
}
//-------------------------------------------------------------



void Kiloworld::update_regions()
{
    // Set region 0 position to centre of mass of robots
    float x = 0;
    float y = 0;
    for (int i = 0; i < bots.size(); i++)
    {
        x += bots[i]->pos->pose.x;
        y += bots[i]->pos->pose.y;
    }
    x /= bots.size();
    y /= bots.size();
    if (regions.size() > 0)
    {
        regions[0]->set_position(x, y);
    }
}



