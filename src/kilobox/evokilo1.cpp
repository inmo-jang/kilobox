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

#include "utils.h" // A Collection of Custom-made utility functions
#include "grape.h" // Functions for GRAPE

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
FILE *Comm_with_multiple_msgs::lfp            = NULL;
FILE *Estimation_distance_to_task_using_multiple_msgs::lfp            = NULL;
FILE *Grape_using_multiple_msgs::lfp            = NULL;
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
            case 1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case 2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case 3: // Task 3
                set_color(RGB(0,2,2));     
                break;
            case 4: // Task 4
                set_color(RGB(2,0,0));     
                break;

            case 5: // Task 5
                set_color(RGB(0,0,2));     
                break;
        }

        
        
    }

}
//-------------------------------------------------------------


//-------------------------------------------------------------
#define TASK_NULL 0


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
            case 1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case 2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case 3: // Task 3
                set_color(RGB(0,2,2));     
                break;
            
        }

        
        
    }

}
//-------------------------------------------------------------



//-------------------------------------------------------------
#define TASK_NULL 0


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
            case 1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case 2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case 3: // Task 3
                set_color(RGB(0,1,2));     
                break;
            
        }

        
        
    }

}
//-------------------------------------------------------------


//-------------------------------------------------------------
#define TASK_NULL 0

void Comm_with_multiple_msgs::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Comm_with_multiple_msgs::tx_message;
    kilo_message_rx         = (message_rx_t)&Comm_with_multiple_msgs::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;
    kilo_ticks_ini = kilo_ticks;

    // // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &kilo_uid, 1); // memcpy(dest, src, count_byte)

}

void Comm_with_multiple_msgs::message_rx(message_t *m, distance_measurement_t *d)
{
        // *m : Neighbour robot's msg pointer (Not sure)

        // dist_neighbour = estimate_distance(d);

        // Check who sent this msg
        unsigned char mf;
        memcpy(&mf, &(m->data[1]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)
        neighbour_kilo_uid = mf;

        // Check the msg sequence
        memcpy(&mf, &(m->data[2]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)
        rcv_num_seq_msg = mf;
        unsigned char neighbour_seq_msg = rcv_num_seq_msg % 10;
        unsigned char neighbour_num_msg = (rcv_num_seq_msg - neighbour_seq_msg)/10;


        printf("[%d] Robot %d rx: Data received %d-th msg of %d msgs from Robot %d\n", kilo_ticks, kilo_uid, neighbour_seq_msg + 1, neighbour_num_msg, neighbour_kilo_uid);
        // TODO: Check the msg contents


        // ======= Check if I already have a msg from this neighbour robot, and choose the memory to save the msg contents
        std::vector<unsigned char>::iterator it;
        it = find (msg_neighbour_robots.begin(), msg_neighbour_robots.end(), neighbour_kilo_uid); // Find the index corresponding "neighbour_kilo_uid" from the local memory. "it" is the resulatant.

        bool isNewRobot = false; // Inisialisation
        if (it == msg_neighbour_robots.end()){ // This becomes true if "it" was not in msg_neighbour_robots. 
            it = find (msg_neighbour_robots.begin(), msg_neighbour_robots.end(), 0); // Find the index of the memory that doesn't have any content. 
            isNewRobot = true;
        }
        int index = std::distance(msg_neighbour_robots.begin(), it); // the index of the memory that I am going to update using the received message. 


        // ====== Check if I am going to process this message.
        bool is_msg_valid = false; // Inisialisation 
        
        if(neighbour_num_msg == 0){ // Added this condition because in the simulator, the first msg recevied is from nobody. 
            printf("====== Just ignore the printf above =======\n");
        }   
        else{
            if(isNewRobot){ // For a new neighbour robot
                if(msg_seq_waiting[index] == neighbour_seq_msg){ // TODO: From a new robot, the first msg didn't come. 
                    is_msg_valid = true; // By default, this message is valid.  
                }        
                else{
                    printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- New Robot Error\n", kilo_ticks, kilo_uid, msg_seq_waiting[index]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;

                }                        
            }
            else{ // For a known neighbour robot
                if (msg_seq_waiting[index] == neighbour_seq_msg){ // Check if this sequence is the right sequence I was waiting for. Note that often the same msg can be received. That's why the robot needs to check this condition.
                    is_msg_valid = true;
                } 
                else if (msg_seq_waiting[index] < neighbour_seq_msg){ // This msg seq is wrong. It implies that I may have lost or missed some previous msgs. 
                    
                    // Note that, sometimes, the neighbour robot keeps broadcasting the same msg. That's why we have "<" here.  
                    printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- might lost some msgs\n", kilo_ticks, kilo_uid, msg_seq_waiting[index]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    // Re-initialisation
                    msg_neighbour_robots[index] = 0; 
                    msg_seq_waiting[index] = 0;                    
                    is_msg_valid = false;                
                }
                else{
                    printf("[%d] WAIT - Robot %d was wating for %d-th msg but received %d from Robot %d  -- the same msg\n", kilo_ticks, kilo_uid, msg_seq_waiting[index]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;

                }

            }
        }     


        // ====== Check whether now I receive all the message for decoding the contents
        bool is_ready_to_assemble = false; // Inisialisation -- This will be true if all the msgs are received.        
        if(is_msg_valid){
            printf("[%d] Robot %d storing the msg from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);

            msg_rcv_time_stamp[index] = kilo_ticks;
            msg_neighbour_robots[index] = neighbour_kilo_uid;             
            msg_seq_waiting[index] = msg_seq_waiting[index] + 1;

            
            // Store the msg contents into a local memory   
            // TODO: Here the 8 below should be changed        
            for(int i=1; i<8; i=i+1){ // 7-byte mode   
                memcpy(&mf, &(m->data[i+2]), 1); // memcpy(dest, src, count_byte)
                switch(index){
                    case 0:
                        msg_store_0.push_back(mf);
                        break;
                    case 1:
                        msg_store_1.push_back(mf);
                        break;
                    case 2:
                        msg_store_2.push_back(mf);
                        break;
                    case 3:
                        msg_store_3.push_back(mf);
                        break;
                    case 4:
                        msg_store_4.push_back(mf);
                        break;
                    case 5:
                        msg_store_5.push_back(mf);
                        break;
                    case 6:
                        msg_store_6.push_back(mf);
                        break;
                    case 7:  
                        msg_store_7.push_back(mf);
                        break;
                }
            }

            // Check whether I am ready to assemble
            if(msg_seq_waiting[index] == neighbour_num_msg){
                printf("[%d] *READY* - Robot %d is ready to assemble all the msgs from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);
                is_ready_to_assemble = true;
            }

            // printf("[%d] Robot %d storage (%d,%d,%d,%d,%d,%d,%d,%d) for (%d,%d,%d,%d,%d,%d,%d,%d)\n", kilo_ticks, kilo_uid, msg_store_0.size(), msg_store_1.size(), msg_store_2.size(), msg_store_3.size(), msg_store_4.size(), msg_store_5.size(), msg_store_6.size(), msg_store_7.size(), msg_neighbour_robots[0], msg_neighbour_robots[1], msg_neighbour_robots[2], msg_neighbour_robots[3], msg_neighbour_robots[4], msg_neighbour_robots[5], msg_neighbour_robots[6], msg_neighbour_robots[7]);            
            
        }

        
        // ====== Message assembling/decoding
        if(is_ready_to_assemble){
            // TODO: Message decode accroding to the protocol 
            std::vector<unsigned char> msg_decode = {};
            switch(index){
                case 0:
                    msg_decode = msg_store_0;
                    msg_store_0 = {}; // Empty the used storage
                    break;
                case 1:
                    msg_decode = msg_store_1;
                    msg_store_1 = {};
                    break;
                case 2:
                    msg_decode = msg_store_2;
                    msg_store_2 = {};
                    break;
                case 3:
                    msg_decode = msg_store_3;
                    msg_store_3 = {};
                    break;
                case 4:
                    msg_decode = msg_store_4;
                    msg_store_4 = {};
                    break;
                case 5:
                    msg_decode = msg_store_5;
                    msg_store_5 = {};
                    break;
                case 6:
                    msg_decode = msg_store_6;
                    msg_store_6 = {};
                    break;
                case 7:  
                    msg_decode = msg_store_7;
                    msg_store_7 = {};
                    break;
            }
            msg_neighbour_robots[index] = 0; // Initialise the used storage
            msg_seq_waiting[index] = 0;


            // printf("[%d] *** DECODED *** Robot %d decoded msg (size - %d); (msg - [", kilo_ticks, kilo_uid, msg_decode.size());
            // for(int i=0; i < msg_decode.size(); i=i+1){
            //     printf(" %d ",msg_decode[i]);
            // }
            // printf("])\n");

            
            // printf("[%d] Robot %d storage (%d,%d,%d,%d,%d,%d,%d,%d) for (%d,%d,%d,%d,%d,%d,%d,%d)\n", kilo_ticks, kilo_uid, msg_store_0.size(), msg_store_1.size(), msg_store_2.size(), msg_store_3.size(), msg_store_4.size(), msg_store_5.size(), msg_store_6.size(), msg_store_7.size(), msg_neighbour_robots[0], msg_neighbour_robots[1], msg_neighbour_robots[2], msg_neighbour_robots[3], msg_neighbour_robots[4], msg_neighbour_robots[5], msg_neighbour_robots[6], msg_neighbour_robots[7]);  

            // LED Display 
            if(check_msg(msg_decode, neighbour_kilo_uid)){
                LED_on(neighbour_kilo_uid);
            }
            
        }


        

        

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

        
        // printf("Robot %d rx: Distances of Tasks (Neighbour known) are (%d, %d, %d)\n", kilo_uid, distance_to_task_neighbour[0], distance_to_task_neighbour[1], distance_to_task_neighbour[2]);

        // // Estimating distances to tasks (Newly Added)
        // for (int i=0; i< num_task; i++){
        //     memcpy(&mg, &(m->data[2*i+2]), 1);
        //     if (mg != 0){ // NB: "mg" may be zero as m->data was initialised. So, we need to rule out this case when taking neighbour info.  
        //         distance_to_task_neighbour[i] = mg;
        //     } 
            
        //     // if (distance_to_task[i] > distance_to_task_neighbour[i] + unit_hop_dist*dist_neighbour/max_dist_neighbour){ // Update distance_task value 
        //     if (distance_to_task_uint[i] > distance_to_task_neighbour[i] + unit_hop_dist*dist_neighbour/max_dist_neighbour + correction_dist_to_task){ // Update distance_task value             
        //         // printf("Robot %d rx: Distance of Task %d is updated from %d to %d because of %d \n", kilo_uid, i+1, distance_to_task[i], distance_to_task_neighbour[i] + (unsigned char)(unit_hop_dist*dist_neighbour/max_dist_neighbour), distance_to_task_neighbour[i] + unit_hop_dist*dist_neighbour/max_dist_neighbour);
        //         // distance_to_task[i] = distance_to_task_neighbour[i] + (unsigned char)(unit_hop_dist*dist_neighbour/max_dist_neighbour);
        //         // printf("Robot %d rx: Distance of Task %d is updated from %d to %d \n", kilo_uid, i+1, distance_to_task_uint[i], distance_to_task_neighbour[i] + (unsigned char)(unit_hop_dist*dist_neighbour/max_dist_neighbour));
        //         distance_to_task_uint[i] = distance_to_task_neighbour[i] + (unsigned char)(unit_hop_dist*dist_neighbour/max_dist_neighbour + correction_dist_to_task);                
        //         task_info_time_stamp[i] = kilo_ticks;                 
        //     }
        // }



}

std::vector<unsigned char> Comm_with_multiple_msgs::generate_contents_to_broadcast(){
    
    printf("[%d] Robot %d generated a sequence of messages \n", kilo_ticks, kilo_uid);

    // TODO: Generation of information to share  - partition_info -> contents_to_broadcast; 
    std::vector<unsigned char> _contents_to_broadcast = {}; // The contents that I am going to broadcast
    _contents_to_broadcast.push_back(kilo_uid);
    _contents_to_broadcast.push_back(kilo_uid+1);
    _contents_to_broadcast.push_back(kilo_uid+2);
    _contents_to_broadcast.push_back(kilo_uid+3);
    _contents_to_broadcast.push_back(kilo_uid+4);
    _contents_to_broadcast.push_back(kilo_uid+5);
    _contents_to_broadcast.push_back(kilo_uid+6);
    _contents_to_broadcast.push_back(kilo_uid+7);
    _contents_to_broadcast.push_back(kilo_uid+8);
    _contents_to_broadcast.push_back(100);
    _contents_to_broadcast.push_back(110);
    _contents_to_broadcast.push_back(120);

    return _contents_to_broadcast;
   

}

void Comm_with_multiple_msgs::broadcast_msgs(){

        // Check if the generated contents are firstly to be broadcast 
        if (is_msg_to_broadcast == false){
            // Contents to msgs
            num_byte_contents = contents_to_broadcast.size(); // Check the byte size of the contents
            if(num_byte_contents > 0){
                // Number of msgs to broadcast
                if(num_byte_contents % 7 == 0){
                    num_msg = num_byte_contents/7; 
                }
                else{
                    num_msg = num_byte_contents/7 + 1; 
                }
                        
                // Initilisation - now the first msg will be sent
                is_msg_to_broadcast = true;
                seq_msg = 0; 

                printf("[%d] Robot %d Ready to broadcast (%d msgs)\n", kilo_ticks, kilo_uid, num_msg);
            }
            else{ // For debug
                printf("[%d] ERROR: Robot %d -- No contents to broadcast\n", kilo_ticks, kilo_uid);
            }  
        }


        if (seq_msg < num_msg){ // Meaning that the robot is in broadcasting a sequence of msgs
            unsigned int num_msg_seq_msg = num_msg*10 + seq_msg;
            memcpy(&msg.data[2], &num_msg_seq_msg, 1); // memcpy(dest, src, count_byte)
            // Msg Generation --- Main contents
            int num_byte_to_broadcast = std::min((int)contents_to_broadcast.size(), 7);
            printf("[%d] Robot %d num_byte_to_broadcast %d\n", kilo_ticks, kilo_uid, num_byte_to_broadcast);
            for (int i=3; i < num_byte_to_broadcast+3; i=i+1){    
                unsigned int byte_to_broadcast = contents_to_broadcast.front();                
                memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)                
                std::vector<unsigned char>::iterator first_index = contents_to_broadcast.begin();
                contents_to_broadcast.erase(first_index);
                // printf("[%d] Robot %d put %d to the msg, now the remaining bytes are %d\n", kilo_ticks, kilo_uid, byte_to_broadcast, contents_to_broadcast.size());
                // printf("[%d] Robot %d Msg Type %d\n", kilo_ticks, kilo_uid, msg.type);
            }
            // Fill dummy info indicating the end of the msg
            if(num_byte_to_broadcast < 7){    
                for (int i=num_byte_to_broadcast+3; i < 10; i=i+1){
                    unsigned int byte_to_broadcast = MSG_END;   
                    memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)  
                    printf("[%d] Robot %d put %d to the msg\n", kilo_ticks, kilo_uid, byte_to_broadcast);                                  
                }
            }
            // Msg Generation --- Header & CRC
            memcpy(&msg.data[1], &kilo_uid, 1); // memcpy(dest, src, count_byte)          
            msg.crc     = message_crc(&msg);

            printf("[%d] Robot %d broadcast (%d/%d) msg - Test %d, Msg Type %d\n", kilo_ticks, kilo_uid, seq_msg+1, num_msg, num_msg_seq_msg, msg.type);

            // Update the parameters for the next loop
            seq_msg = seq_msg + 1; // The sequence of the current msg to broadcast
            if(seq_msg == num_msg){ // Now broadcasting is finished, so do initilisation
                num_msg = 0;
                seq_msg = 0;
                is_msg_to_broadcast = false;
            }

        }

}

void Comm_with_multiple_msgs::LED_on(int indicator){
    // LED Display
    switch (indicator){
        case 0: 
            set_color(RGB(2,2,2));
            break;
        case 1: // Task 1
            set_color(RGB(0,2,0));
            break;
        case 2: // Task 2
            set_color(RGB(2,0,2));     
            break;
        case 3: // Task 3
            set_color(RGB(0,1,2));     
            break;
        case 4: 
            set_color(RGB(1,0,0));     
            break;
        case 5: 
            set_color(RGB(0,0,1));     
            break;
                
    }
}

bool Comm_with_multiple_msgs::check_msg(std::vector<unsigned char> msg_decode, unsigned char neighbour_kilo_uid){


    std::vector<unsigned char> preknown_contents = {}; 
    preknown_contents.push_back(neighbour_kilo_uid);
    preknown_contents.push_back(neighbour_kilo_uid+1);
    preknown_contents.push_back(neighbour_kilo_uid+2);
    preknown_contents.push_back(neighbour_kilo_uid+3);
    preknown_contents.push_back(neighbour_kilo_uid+4);
    preknown_contents.push_back(neighbour_kilo_uid+5);
    preknown_contents.push_back(neighbour_kilo_uid+6);
    preknown_contents.push_back(neighbour_kilo_uid+7);
    preknown_contents.push_back(neighbour_kilo_uid+8);
    preknown_contents.push_back(100);
    preknown_contents.push_back(110);
    preknown_contents.push_back(120);
    preknown_contents.push_back(255);
    preknown_contents.push_back(255);

    bool result;
    if(preknown_contents == msg_decode){
        result = true;
    }
    else{
        result = false;
    }
    return result;

}

void Comm_with_multiple_msgs::loop()
{
    // NB: All the variables will be reinitialised at each loop
    int task_found_env;
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;

        
        // // Get Task Info from Environment (if the robot found a new task by itself)
        // task_found_env = get_environment(); 
        // // printf("Robot %d found Task %d\n", kilo_uid, task_found_env);
        // if (task_found_env!=0){
        //     // distance_to_task[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self. 
        //     distance_to_task_uint[task_found_env-1] = 1; // Careful about Task Index; It means that the robot found a task, which is 1 hop count distance from it self.             
        //     task_info_time_stamp[task_found_env-1] = kilo_ticks; 
            
        // }
        // else{ // For forgetting untracked tasks
        //     for (int i=0; i< num_task; i++){ // For each task
        //         if (distance_to_task_uint[i] != 1 && distance_to_task_uint[i] < 255){ // For only non-task robots; This condition is just for experiment purposes. 
        //             distance_to_task_uint[i] = distance_to_task_uint[i] + (kilo_ticks - task_info_time_stamp[i])/32*unit_hop_dist*parameter_forgetting; // ++parameter_forgetting*unit_hop_dist per second; The increment should be less than half, I guess. Otherwise, distance_to_task is updated by neighbours who still has lower values, which eventually causes longer time for all the robots to forget this value. 
        //             if (distance_to_task_uint[i] > 230){ // 230, which is the value that is arbitrary large but below than 255. To cut off overflow. This value functions as bumber. 
        //                 distance_to_task_uint[i] = 255;
        //             }
        //         }

        //     }
        // }
  
        // // Utility Computation
        // for(int i=0; i<num_task; i++){
        //     task_cost[i] = distance_to_task_uint[i];
        // }
        // if (chosen_task != TASK_NULL){
        //     chosen_task_cost = task_cost[chosen_task-1];
        // }
        
        // // Utility Comparison
        // int min_cost;
        // min_cost = *std::min_element(task_cost.begin(), task_cost.end());
        // int preferred_task; 
        // preferred_task = std::min_element(task_cost.begin(), task_cost.end()) - task_cost.begin() + 1;
        // if (min_cost == 255){
        //     chosen_task = TASK_NULL;
        //     chosen_task_cost = 255;
        // }
        // else if (min_cost < chosen_task_cost){
        //     // printf("Robot %d joins to Task %d because its delta_cost is %d\n", kilo_uid, preferred_task, chosen_task_cost - min_cost);
        //     chosen_task = preferred_task;
        //     chosen_task_cost = min_cost;
        // }
        

        // num_iterations = (unsigned short int)std::rand();


        // // Broadcast      
        // for(int i=0; i<num_task; i++){
        //     if (distance_to_task_uint[i] > 255){
        //         distance_to_task[i] = 255;
        //     }
        //     else{
        //         distance_to_task[i] = distance_to_task_uint[i];
        //     }
            
        // } 
        

        // Generation of contents to broadcast
        if (is_msg_to_broadcast == false){
            contents_to_broadcast = generate_contents_to_broadcast();
        }

        // Broadcast the generated contents using multiple messages
        broadcast_msgs();
        
    }

}
//-------------------------------------------------------------



//-------------------------------------------------------------
void Grape_using_multiple_msgs::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Grape_using_multiple_msgs::tx_message;
    kilo_message_rx         = (message_rx_t)&Grape_using_multiple_msgs::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;

    // // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &kilo_uid, 1); // memcpy(dest, src, count_byte)
    unsigned char zero = 0;
    memcpy(&msg.data[2], &zero, 1); 
    memcpy(&msg.data[3], &zero, 1); 
    memcpy(&msg.data[4], &zero, 1); 
    memcpy(&msg.data[5], &zero, 1); 
    memcpy(&msg.data[6], &zero, 1); 
    memcpy(&msg.data[7], &zero, 1); 
    memcpy(&msg.data[8], &zero, 1); 
    memcpy(&msg.data[9], &zero, 1); 

}

void Grape_using_multiple_msgs::message_rx(message_t *m, distance_measurement_t *d) // Receive abd save msg to the local msg storage
{

        uint8_t dist_neighbour = estimate_distance(d);
        // Check who sent this msg
        memcpy(&neighbour_kilo_uid, &(m->data[1]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)

        // Check the msg sequence
        memcpy(&rcv_num_seq_msg, &(m->data[2]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)
        unsigned char neighbour_seq_msg = rcv_num_seq_msg % 10;
        unsigned char neighbour_num_msg = (rcv_num_seq_msg - neighbour_seq_msg)/10;

        // printf("[%d] Robot %d rx: Data received %d-th msg of %d msgs from Robot %d\n", kilo_ticks, kilo_uid, neighbour_seq_msg + 1, neighbour_num_msg, neighbour_kilo_uid);

        // ======= Check if I already have a msg from this neighbour robot, and choose the memory to save the msg contents
        int index_msg_store;
        bool isNewRobot;
        if(IsIncludedInVec(msg_neighbour_robots, neighbour_kilo_uid)){
            index_msg_store = GetIndexFromVec(msg_neighbour_robots, neighbour_kilo_uid);
            isNewRobot = false;
        }
        else{
            index_msg_store = GetIndexFromVec(msg_neighbour_robots, 0);
            isNewRobot = true;
        }
        

        // ====== Check if I am going to process this message.
        bool is_msg_valid = false; // Inisialisation         
        if(neighbour_num_msg == 0){ // Added this condition because in the simulator, the first msg recevied is from nobody. 
            printf("====== Just ignore the printf above =======\n");
        }   
        else{
            if(isNewRobot){ // For a new neighbour robot
                if(msg_seq_waiting[index_msg_store] == neighbour_seq_msg){ 
                    is_msg_valid = true; // By default, this message is valid.  
                }        
                else{
                    // printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- New Robot Error\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;
                }                        
            }
            else{ // For a known neighbour robot
                if (msg_seq_waiting[index_msg_store] == neighbour_seq_msg){ // Check if this sequence is the right sequence I was waiting for. Note that often the same msg can be received. That's why the robot needs to check this condition.
                    is_msg_valid = true;
                } 
                else if (msg_seq_waiting[index_msg_store] < neighbour_seq_msg){ // This msg seq is wrong. It implies that I may have lost or missed some previous msgs. 
                    
                    // Note that, sometimes, the neighbour robot keeps broadcasting the same msg. That's why we have "<" here.  
                    // printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- might lost some msgs\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    // Re-initialisation
                    msg_neighbour_robots[index_msg_store] = 0; 
                    msg_seq_waiting[index_msg_store] = 0;                    
                    is_msg_valid = false;                
                }
                else{
                    // printf("[%d] WAIT - Robot %d was wating for %d-th msg but received %d from Robot %d  -- the same msg\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;
                }
            }
        }     


        // ====== Check whether now I receive all the message for decoding the contents
        bool is_ready_to_assemble = false; // Inisialisation -- This will be true if all the msgs are received.        
        if(is_msg_valid){
            // printf("[%d] Robot %d storing the msg from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);
            msg_rcv_time_stamp[index_msg_store] = kilo_ticks;
            msg_neighbour_robots[index_msg_store] = neighbour_kilo_uid;             
            msg_seq_waiting[index_msg_store] = msg_seq_waiting[index_msg_store] + 1;
            
            // Store the msg contents into a local memory     
            for(int i=1; i<NUM_BYTE_MSG + 1; i=i+1){ // 
                unsigned char mf;
                memcpy(&mf, &(m->data[i+2]), 1); // memcpy(dest, src, count_byte)
                switch(index_msg_store){
                    case 0:
                        msg_store_0.push_back(mf);
                        break;
                    case 1:
                        msg_store_1.push_back(mf);
                        break;
                    case 2:
                        msg_store_2.push_back(mf);
                        break;
                    case 3:
                        msg_store_3.push_back(mf);
                        break;
                    case 4:
                        msg_store_4.push_back(mf);
                        break;
                    case 5:
                        msg_store_5.push_back(mf);
                        break;
                    case 6:
                        msg_store_6.push_back(mf);
                        break;
                    case 7:  
                        msg_store_7.push_back(mf);
                        break;
                }
            }

            // Check whether I am ready to assemble
            if(msg_seq_waiting[index_msg_store] == neighbour_num_msg){
                // printf("[%d] *READY* - Robot %d is ready to assemble all the msgs from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);
                is_ready_to_assemble = true;
            }                   
        }
        
        // ====== Message assembling/decoding
        if(is_ready_to_assemble){
            // Save the ready content into decoded_content
            std::vector<unsigned char> decoded_content = {};
            switch(index_msg_store){
                case 0:
                    decoded_content = msg_store_0;
                    msg_store_0 = {}; // Empty the used storage
                    break;
                case 1:
                    decoded_content = msg_store_1;
                    msg_store_1 = {};
                    break;
                case 2:
                    decoded_content = msg_store_2;
                    msg_store_2 = {};
                    break;
                case 3:
                    decoded_content = msg_store_3;
                    msg_store_3 = {};
                    break;
                case 4:
                    decoded_content = msg_store_4;
                    msg_store_4 = {};
                    break;
                case 5:
                    decoded_content = msg_store_5;
                    msg_store_5 = {};
                    break;
                case 6:
                    decoded_content = msg_store_6;
                    msg_store_6 = {};
                    break;
                case 7:  
                    decoded_content = msg_store_7;
                    msg_store_7 = {};
                    break;
            }
            msg_neighbour_robots[index_msg_store] = 0; // Initialise the used storage
            msg_seq_waiting[index_msg_store] = 0;

            if (decoded_content.size() == 0)
            {
                // printf("[%d] XXX DECODING ERROR --- Robot %d decoded msg (size - %d) ", kilo_ticks, kilo_uid, (int)(decoded_content.size()));
            }
            else if(decoded_content.size() != neighbour_num_msg * NUM_BYTE_MSG){ // To avoid unexpected comm error
                // printf("[%d] XXXXXXXXXXXXXXXXX DECODING ERROR --- 222 ",kilo_ticks );
            }
            else
            {
                // printf("[%d] *** DECODED *** Robot %d decoded msg (size - %d) from %d; (msg - [", kilo_ticks, kilo_uid, (int)(decoded_content.size()),neighbour_kilo_uid);
                // for (int i = 0; i < decoded_content.size(); i = i + 1)
                // {
                //     printf(" %d ", decoded_content[i]);
                // }
                // printf("])\n");

                // printf("[%d] Robot %d storage (%d,%d,%d,%d,%d,%d,%d,%d) for (%d,%d,%d,%d,%d,%d,%d,%d)\n", kilo_ticks, kilo_uid, msg_store_0.size(), msg_store_1.size(), msg_store_2.size(), msg_store_3.size(), msg_store_4.size(), msg_store_5.size(), msg_store_6.size(), msg_store_7.size(), msg_neighbour_robots[0], msg_neighbour_robots[1], msg_neighbour_robots[2], msg_neighbour_robots[3], msg_neighbour_robots[4], msg_neighbour_robots[5], msg_neighbour_robots[6], msg_neighbour_robots[7]);

                // Note: decoded_content only contains the main content without the header (i.e. neighbour kilobot ID, msg sequence)
                neighbourPartition = get_partition_from_content(decoded_content, kilo_uid);
                myLocalEnvInfo = UpdateLocalEnvInfoFromPartition(myLocalEnvInfo, neighbourPartition, dist_neighbour, kilo_ticks, kilo_uid);
            }
        }


}



void Grape_using_multiple_msgs::broadcast_msgs(){

        // Initisalisation when the content is first to be broadcast 
        if (is_msg_to_broadcast == false){
            if(contents_to_broadcast.size() > 0){ // The number of bytes for the content I am going to broadcast
                // Initilisation - now the first msg will be sent
                num_msg = EstimateNumMsg(contents_to_broadcast.size(), NUM_BYTE_MSG);                        
                is_msg_to_broadcast = true;
                seq_msg = 0; 

                // printf("[%d] Robot %d Ready to broadcast (%d msgs)\n", kilo_ticks, kilo_uid, num_msg);
            }
        }

        // Broadcast
        if (seq_msg < num_msg){ // Meaning that the robot is in broadcasting a sequence of msgs
            // Msg Generation --- Header
            memcpy(&msg.data[1], &kilo_uid, 1);  
            unsigned int num_msg_seq_msg = num_msg*10 + seq_msg;
            memcpy(&msg.data[2], &num_msg_seq_msg, 1); 

            // Msg Generation --- Main contents
            int num_byte_to_broadcast = std::min((int)contents_to_broadcast.size(), NUM_BYTE_MSG);
            // printf("[%d] Robot %d num_byte_to_broadcast %d\n", kilo_ticks, kilo_uid, num_byte_to_broadcast);
            for (int i=3; i < num_byte_to_broadcast+3; i=i+1){    
                
                std::vector<unsigned char>::iterator first_data = contents_to_broadcast.begin();
                unsigned int byte_to_broadcast = *first_data;                
                memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)                
                
                contents_to_broadcast.erase(first_data);
                // printf("[%d] Robot %d put %d to the msg, now the remaining bytes are %d\n", kilo_ticks, kilo_uid, byte_to_broadcast, contents_to_broadcast.size());
            }

            // Fill dummy info indicating the end of the msg
            if(num_byte_to_broadcast < NUM_BYTE_MSG){    
                for (int i=num_byte_to_broadcast+3; i < NUM_BYTE_MSG + 3; i=i+1){
                    unsigned int byte_to_broadcast = MSG_END;   
                    memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)  
                    // printf("[%d] Robot %d put %d to the msg\n", kilo_ticks, kilo_uid, byte_to_broadcast);                                  
                }
            }
            // Msg Generation --- CRC       
            msg.crc     = message_crc(&msg);

            //printf("[%d] Robot %d broadcast (%d/%d) msg - Test %d, Msg Type %d\n", kilo_ticks, kilo_uid, seq_msg+1, num_msg, num_msg_seq_msg, msg.type);

            // Update the parameters for the next loop
            seq_msg = seq_msg + 1; // The sequence of the current msg to broadcast
            if(seq_msg == num_msg){ // Now broadcasting is finished, so do initilisation
                num_msg = 0;
                seq_msg = 0;
                is_msg_to_broadcast = false;
            }

        }

}




void Grape_using_multiple_msgs::LED_on(int indicator){
    // LED Display
    switch (indicator){
        case VOID_TASK: 
            set_color(RGB(2,2,2));
            break;
        case 1: // Task 1
            set_color(RGB(0,2,0));
            break;
        case 2: // Task 2
            set_color(RGB(2,0,2));     
            break;
        case 3: // Task 3
            set_color(RGB(0,1,2));     
            break;
        case 4: 
            set_color(RGB(1,0,0));     
            break;
        case 5: 
            set_color(RGB(0,0,1));     
            break;
                
    }
}

void Grape_using_multiple_msgs::loop()
{
    // NB: All the variables will be reinitialised at each loop
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        
        // Get Task Info from Environment (if the robot found a new task by itself)
        int task_found_index = get_environment(); 
        if (task_found_index!=0){
            myLocalEnvInfo = UpdateLocalEnvAsNewTaskFound(myLocalEnvInfo, task_found_index, _global_env, kilo_ticks);    
        }
        
        // Forgetting task information as time goes        
        myLocalEnvInfo = UpdateTaskFreshness(myLocalEnvInfo, kilo_ticks);
        
    
        // Debug
        if (myLocalEnvInfo.num_task > 0)
        {
            printf("\n\n[%d] $$$$$$$$$$ Robot %d - Local Info:  Num_Task (w/o void_task) = %d;", kilo_ticks, kilo_uid, myLocalEnvInfo.num_task);
            printf("  Task ID: (");
            for (int i = 0; i < myLocalEnvInfo.num_task; i++)
            {
                printf(" %d, ", myLocalEnvInfo.task_id[i]);
            }
            printf("); Task Distance: (");
            for (int i = 0; i < myLocalEnvInfo.num_task; i++)
            {
                printf(" %d, ", myLocalEnvInfo.task_distance[i]);
            }
            printf(")\n");
        }

        if(myLocalEnvInfo.needCheck == true){  
            unsigned char previous_task_id = GetMyChosenTaskID(kilo_uid, myLocalEnvInfo.agent_decision); // For Debug
            // printf("-         given agent_decision          (size: %d) \n", myLocalEnvInfo.agent_decision.size());
            int estimated_max_num_agent = EstimateNumRobot(myLocalEnvInfo.agent_decision.size());
            std::vector<unsigned short int> num_agent_in_task = GetSubpopulation(myLocalEnvInfo.agent_decision, estimated_max_num_agent);
            printf("-         given num_agent_in_task        (size: %d) ===> ", num_agent_in_task.size());
            PrintVecInt(num_agent_in_task);
            printf("-         here, my previous TASK ID is %d \n", previous_task_id);
            if(kilo_uid <= 3){
                chosen_task = DecisionMaking(myLocalEnvInfo, previous_task_id, DM_DISTANCE); 
            }
            else{
                chosen_task = DecisionMaking(myLocalEnvInfo, previous_task_id, DM_BALANCE); 
            }
            
            if(chosen_task == VOID_TASK){
                // printf("\nRobot %d - Decision Making and chose VOID TASK\n\n ", kilo_uid);
                LED_on(VOID_TASK);
            }
            else{
                // printf("\nRobot %d - Decision Making and chose Task %d; chosen_task = %d \n\n ", kilo_uid, myLocalEnvInfo.task_id[chosen_task-1], chosen_task);
                LED_on(myLocalEnvInfo.task_id[chosen_task-1]);
            }
            
            myLocalEnvInfo = UpdateLocalInfo(myLocalEnvInfo, kilo_uid, chosen_task);        
            // myLocalEnvInfo.needCheck = false;
            // for debug
            estimated_max_num_agent = EstimateNumRobot(myLocalEnvInfo.agent_decision.size());
            num_agent_in_task = GetSubpopulation(myLocalEnvInfo.agent_decision, estimated_max_num_agent);
            printf("-         The robot selects  TASK ID %d from Pre-Task %d; assumes Num_Robot = %d  \n", myLocalEnvInfo.chosen_task_id, previous_task_id, estimated_max_num_agent);
            printf("-               num_agent_in_task                                                      (size: %d) ===> ", num_agent_in_task.size());
            PrintVecInt(num_agent_in_task);
            // printf("-         modified agent_decision        (size: %d) \n", myLocalEnvInfo.agent_decision.size());
            int reconfirm = GetMyChosenTaskID(kilo_uid, myLocalEnvInfo.agent_decision);
            printf("-            reconfirm ----> TASK %d \n", reconfirm);     
        }

        // Generation of contents to broadcast
        if (is_msg_to_broadcast == false){            
            contents_to_broadcast = gen_content_from_partition(myLocalEnvInfo.myPartition);
            // printf("\n\nRobot %d - Content Size %d\n\n", kilo_uid, contents_to_broadcast.size());
        }

        // Broadcast the generated contents using multiple messages
        broadcast_msgs(); 
        // Debug
        printf("[%d] >>>>>> Robot %d  broadcasted  agent_decision          (size: %d) \n", kilo_ticks, kilo_uid, myLocalEnvInfo.myPartition.agent_decision.size());    
    }
}
//-------------------------------------------------------------


//-------------------------------------------------------------
void Estimation_distance_to_task_using_multiple_msgs::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Estimation_distance_to_task_using_multiple_msgs::tx_message;
    kilo_message_rx         = (message_rx_t)&Estimation_distance_to_task_using_multiple_msgs::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    last_update     = kilo_ticks;

    // // Broadcast (Initialisation); NB: Otherwise, some random msg values spoil scnearios.          
    memcpy(&msg.data[1], &kilo_uid, 1); // memcpy(dest, src, count_byte)
    unsigned char zero = 0;
    memcpy(&msg.data[2], &zero, 1); 
    memcpy(&msg.data[3], &zero, 1); 
    memcpy(&msg.data[4], &zero, 1); 
    memcpy(&msg.data[5], &zero, 1); 
    memcpy(&msg.data[6], &zero, 1); 
    memcpy(&msg.data[7], &zero, 1); 
    memcpy(&msg.data[8], &zero, 1); 
    memcpy(&msg.data[9], &zero, 1); 

}

void Estimation_distance_to_task_using_multiple_msgs::message_rx(message_t *m, distance_measurement_t *d) // Receive abd save msg to the local msg storage
{

        uint8_t dist_neighbour = estimate_distance(d);
        // Check who sent this msg
        memcpy(&neighbour_kilo_uid, &(m->data[1]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)

        // Check the msg sequence
        memcpy(&rcv_num_seq_msg, &(m->data[2]), 1); // Using 7-bytes mode; memcpy(dest, src, count_byte)
        unsigned char neighbour_seq_msg = rcv_num_seq_msg % 10;
        unsigned char neighbour_num_msg = (rcv_num_seq_msg - neighbour_seq_msg)/10;

        printf("[%d] Robot %d rx: Data received %d-th msg of %d msgs from Robot %d\n", kilo_ticks, kilo_uid, neighbour_seq_msg + 1, neighbour_num_msg, neighbour_kilo_uid);

        // ======= Check if I already have a msg from this neighbour robot, and choose the memory to save the msg contents
        int index_msg_store;
        bool isNewRobot;
        if(IsIncludedInVec(msg_neighbour_robots, neighbour_kilo_uid)){
            index_msg_store = GetIndexFromVec(msg_neighbour_robots, neighbour_kilo_uid);
            isNewRobot = false;
        }
        else{
            index_msg_store = GetIndexFromVec(msg_neighbour_robots, 0);
            isNewRobot = true;
        }
        

        // ====== Check if I am going to process this message.
        bool is_msg_valid = false; // Inisialisation         
        if(neighbour_num_msg == 0){ // Added this condition because in the simulator, the first msg recevied is from nobody. 
            printf("====== Just ignore the printf above =======\n");
        }   
        else{
            if(isNewRobot){ // For a new neighbour robot
                if(msg_seq_waiting[index_msg_store] == neighbour_seq_msg){ 
                    is_msg_valid = true; // By default, this message is valid.  
                }        
                else{
                    printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- New Robot Error\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;
                }                        
            }
            else{ // For a known neighbour robot
                if (msg_seq_waiting[index_msg_store] == neighbour_seq_msg){ // Check if this sequence is the right sequence I was waiting for. Note that often the same msg can be received. That's why the robot needs to check this condition.
                    is_msg_valid = true;
                } 
                else if (msg_seq_waiting[index_msg_store] < neighbour_seq_msg){ // This msg seq is wrong. It implies that I may have lost or missed some previous msgs. 
                    
                    // Note that, sometimes, the neighbour robot keeps broadcasting the same msg. That's why we have "<" here.  
                    printf("[%d] LOSS - Robot %d was wating for %d-th msg but received %d from Robot %d  -- might lost some msgs\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    // Re-initialisation
                    msg_neighbour_robots[index_msg_store] = 0; 
                    msg_seq_waiting[index_msg_store] = 0;                    
                    is_msg_valid = false;                
                }
                else{
                    printf("[%d] WAIT - Robot %d was wating for %d-th msg but received %d from Robot %d  -- the same msg\n", kilo_ticks, kilo_uid, msg_seq_waiting[index_msg_store]+1, neighbour_seq_msg +1, neighbour_kilo_uid);
                    is_msg_valid = false;
                }
            }
        }     


        // ====== Check whether now I receive all the message for decoding the contents
        bool is_ready_to_assemble = false; // Inisialisation -- This will be true if all the msgs are received.        
        if(is_msg_valid){
            // printf("[%d] Robot %d storing the msg from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);
            msg_rcv_time_stamp[index_msg_store] = kilo_ticks;
            msg_neighbour_robots[index_msg_store] = neighbour_kilo_uid;             
            msg_seq_waiting[index_msg_store] = msg_seq_waiting[index_msg_store] + 1;
            
            // Store the msg contents into a local memory     
            for(int i=1; i<NUM_BYTE_MSG + 1; i=i+1){ // 
                unsigned char mf;
                memcpy(&mf, &(m->data[i+2]), 1); // memcpy(dest, src, count_byte)
                switch(index_msg_store){
                    case 0:
                        msg_store_0.push_back(mf);
                        break;
                    case 1:
                        msg_store_1.push_back(mf);
                        break;
                    case 2:
                        msg_store_2.push_back(mf);
                        break;
                    case 3:
                        msg_store_3.push_back(mf);
                        break;
                    case 4:
                        msg_store_4.push_back(mf);
                        break;
                    case 5:
                        msg_store_5.push_back(mf);
                        break;
                    case 6:
                        msg_store_6.push_back(mf);
                        break;
                    case 7:  
                        msg_store_7.push_back(mf);
                        break;
                }
            }

            // Check whether I am ready to assemble
            if(msg_seq_waiting[index_msg_store] == neighbour_num_msg){
                // printf("[%d] *READY* - Robot %d is ready to assemble all the msgs from %d \n", kilo_ticks, kilo_uid, neighbour_kilo_uid);
                is_ready_to_assemble = true;
            }                   
        }
        
        // ====== Message assembling/decoding
        if(is_ready_to_assemble){
            // Save the ready content into decoded_content
            std::vector<unsigned char> decoded_content = {};
            switch(index_msg_store){
                case 0:
                    decoded_content = msg_store_0;
                    msg_store_0 = {}; // Empty the used storage
                    break;
                case 1:
                    decoded_content = msg_store_1;
                    msg_store_1 = {};
                    break;
                case 2:
                    decoded_content = msg_store_2;
                    msg_store_2 = {};
                    break;
                case 3:
                    decoded_content = msg_store_3;
                    msg_store_3 = {};
                    break;
                case 4:
                    decoded_content = msg_store_4;
                    msg_store_4 = {};
                    break;
                case 5:
                    decoded_content = msg_store_5;
                    msg_store_5 = {};
                    break;
                case 6:
                    decoded_content = msg_store_6;
                    msg_store_6 = {};
                    break;
                case 7:  
                    decoded_content = msg_store_7;
                    msg_store_7 = {};
                    break;
            }
            msg_neighbour_robots[index_msg_store] = 0; // Initialise the used storage
            msg_seq_waiting[index_msg_store] = 0;

            if (decoded_content.size() == 0)
            {
                printf("[%d] XXX DECODING ERROR --- Robot %d decoded msg (size - %d) ", kilo_ticks, kilo_uid, (int)(decoded_content.size()));
            }
            else
            {
                printf("[%d] *** DECODED *** Robot %d decoded msg (size - %d) from %d; (msg - [", kilo_ticks, kilo_uid, (int)(decoded_content.size()),neighbour_kilo_uid);
                for (int i = 0; i < decoded_content.size(); i = i + 1)
                {
                    printf(" %d ", decoded_content[i]);
                }
                printf("])\n");

                // printf("[%d] Robot %d storage (%d,%d,%d,%d,%d,%d,%d,%d) for (%d,%d,%d,%d,%d,%d,%d,%d)\n", kilo_ticks, kilo_uid, msg_store_0.size(), msg_store_1.size(), msg_store_2.size(), msg_store_3.size(), msg_store_4.size(), msg_store_5.size(), msg_store_6.size(), msg_store_7.size(), msg_neighbour_robots[0], msg_neighbour_robots[1], msg_neighbour_robots[2], msg_neighbour_robots[3], msg_neighbour_robots[4], msg_neighbour_robots[5], msg_neighbour_robots[6], msg_neighbour_robots[7]);

                // Note: decoded_content only contains the main content without the header (i.e. neighbour kilobot ID, msg sequence)
                neighbourPartition = get_partition_from_content(decoded_content, kilo_uid);
                myLocalEnvInfo = UpdateLocalEnvInfoFromPartition(myLocalEnvInfo, neighbourPartition, dist_neighbour, kilo_ticks, kilo_uid);
            }
        }


}



void Estimation_distance_to_task_using_multiple_msgs::broadcast_msgs(){

        // Initisalisation when the content is first to be broadcast 
        if (is_msg_to_broadcast == false){
            if(contents_to_broadcast.size() > 0){ // The number of bytes for the content I am going to broadcast
                // Initilisation - now the first msg will be sent
                num_msg = EstimateNumMsg(contents_to_broadcast.size(), NUM_BYTE_MSG);                        
                is_msg_to_broadcast = true;
                seq_msg = 0; 

                // printf("[%d] Robot %d Ready to broadcast (%d msgs)\n", kilo_ticks, kilo_uid, num_msg);
            }
        }

        // Broadcast
        if (seq_msg < num_msg){ // Meaning that the robot is in broadcasting a sequence of msgs
            // Msg Generation --- Header
            memcpy(&msg.data[1], &kilo_uid, 1);  
            unsigned int num_msg_seq_msg = num_msg*10 + seq_msg;
            memcpy(&msg.data[2], &num_msg_seq_msg, 1); 

            // Msg Generation --- Main contents
            int num_byte_to_broadcast = std::min((int)contents_to_broadcast.size(), NUM_BYTE_MSG);
            // printf("[%d] Robot %d num_byte_to_broadcast %d\n", kilo_ticks, kilo_uid, num_byte_to_broadcast);
            for (int i=3; i < num_byte_to_broadcast+3; i=i+1){    
                
                std::vector<unsigned char>::iterator first_data = contents_to_broadcast.begin();
                unsigned int byte_to_broadcast = *first_data;                
                memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)                
                
                contents_to_broadcast.erase(first_data);
                // printf("[%d] Robot %d put %d to the msg, now the remaining bytes are %d\n", kilo_ticks, kilo_uid, byte_to_broadcast, contents_to_broadcast.size());
            }

            // Fill dummy info indicating the end of the msg
            if(num_byte_to_broadcast < NUM_BYTE_MSG){    
                for (int i=num_byte_to_broadcast+3; i < NUM_BYTE_MSG + 3; i=i+1){
                    unsigned int byte_to_broadcast = MSG_END;   
                    memcpy(&msg.data[i], &byte_to_broadcast, 1); // memcpy(dest, src, count_byte)  
                    // printf("[%d] Robot %d put %d to the msg\n", kilo_ticks, kilo_uid, byte_to_broadcast);                                  
                }
            }
            // Msg Generation --- CRC       
            msg.crc     = message_crc(&msg);

            //printf("[%d] Robot %d broadcast (%d/%d) msg - Test %d, Msg Type %d\n", kilo_ticks, kilo_uid, seq_msg+1, num_msg, num_msg_seq_msg, msg.type);

            // Update the parameters for the next loop
            seq_msg = seq_msg + 1; // The sequence of the current msg to broadcast
            if(seq_msg == num_msg){ // Now broadcasting is finished, so do initilisation
                num_msg = 0;
                seq_msg = 0;
                is_msg_to_broadcast = false;
            }

        }

}




void Estimation_distance_to_task_using_multiple_msgs::LED_on(int indicator){
    // LED Display
    switch (indicator){
        case VOID_TASK: 
            set_color(RGB(2,2,2));
            break;
        case 1: // Task 1
            set_color(RGB(0,2,0));
            break;
        case 2: // Task 2
            set_color(RGB(2,0,2));     
            break;
        case 3: // Task 3
            set_color(RGB(0,1,2));     
            break;
        case 4: 
            set_color(RGB(1,0,0));     
            break;
        case 5: 
            set_color(RGB(0,0,1));     
            break;
                
    }
}

void Estimation_distance_to_task_using_multiple_msgs::loop()
{
    // NB: All the variables will be reinitialised at each loop
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        
        // Get Task Info from Environment (if the robot found a new task by itself)
        int task_found_index = get_environment(); 
        if (task_found_index!=0){
            myLocalEnvInfo = UpdateLocalEnvAsNewTaskFound(myLocalEnvInfo, task_found_index, _global_env, kilo_ticks);    
        }
        
        // Forgetting task information as time goes        
        myLocalEnvInfo = UpdateTaskFreshness(myLocalEnvInfo, kilo_ticks);
        
    
        // Debug
        if (myLocalEnvInfo.num_task > 0)
        {
            printf("Robot %d - Local Info:  Num_Task = %d;", kilo_uid, myLocalEnvInfo.num_task);
            printf("  Task ID: (");
            for (int i = 0; i < myLocalEnvInfo.num_task; i++)
            {
                printf(" %d, ", myLocalEnvInfo.task_id[i]);
            }
            printf("); Task Distance: (");
            for (int i = 0; i < myLocalEnvInfo.num_task; i++)
            {
                printf(" %d, ", myLocalEnvInfo.task_distance[i]);
            }
            printf(")\n");
        }

        if(myLocalEnvInfo.needCheck == true){
            unsigned char previous_task_id = GetMyChosenTaskID(kilo_uid, myLocalEnvInfo.agent_decision); // For Debug            
            chosen_task = DecisionMaking(myLocalEnvInfo, previous_task_id, DM_BALANCE); // chosen_task becomes 1,2,3,... or VOID_TASK if no task is selected
            if(chosen_task == VOID_TASK){
                printf("\nRobot %d - Decision Making and chose VOID TASK\n\n ", kilo_uid);
                LED_on(VOID_TASK);
            }
            else{
                printf("\nRobot %d - Decision Making and chose Task %d; chosen_task = %d \n\n ", kilo_uid, myLocalEnvInfo.task_id[chosen_task-1], chosen_task);
                LED_on(myLocalEnvInfo.task_id[chosen_task-1]);
            }
            myLocalEnvInfo.chosen_task_id = chosen_task;
            myPartition = UpdatePartition(myPartition, kilo_uid, chosen_task, myLocalEnvInfo); // TODO
            // printf("Num of Known Task = %d \n", myPartition.num_task );            
            // myLocalEnvInfo.needCheck = false;
        }

        // Generation of contents to broadcast
        if (is_msg_to_broadcast == false){            
            contents_to_broadcast = gen_content_from_partition(myPartition);
            // printf("\n\nRobot %d - Content Size %d\n\n", kilo_uid, contents_to_broadcast.size());
        }

        // Broadcast the generated contents using multiple messages
        broadcast_msgs();     
    }
}
//-------------------------------------------------------------


//-------------------------------------------------------------
#define TASK_NULL 0


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
        // --- (1) Remind the prievous decision
        if (local_memory_time_stamp[random_time_stamp] == true){ // If this partition info is already stored
            chosen_task = local_memory_chosen_task[random_time_stamp];
            printf("Robot %d: Given a stored partition %d, where it was at task %d \n", kilo_uid, random_time_stamp, chosen_task);
        }
        else{ // This partition is new info
            chosen_task = TASK_NULL;   
            printf("Robot %d: Given a new partition \n", kilo_uid);
        }
        
        // --- (2) Utility computation
        for(int i=0; i<num_task; i++){
            if(distance_to_task_uint[i] != 255){ // Meaning that this task is known. 
                task_cost[i] = distance_to_task_uint[i];
                if ( (i+1) == chosen_task ){
                    individual_utility[i+1] = (float)task_demand[i]/num_agent_in_task[i] - (float)task_cost[i];
                }
                else{                
                    individual_utility[i+1] = (float)task_demand[i]/(num_agent_in_task[i]+1) - (float)task_cost[i];
                }                 
            }
            else{ // Meaning that this task is unknown yet. 
                individual_utility[i+1] = 0.0;
            }

          
        }
        printf("Robot %d: Task Demand (%d, %d, %d)\n", kilo_uid, task_demand[0], task_demand[1], task_demand[2]);
        printf("Robot %d: num_agent_in_task (%d, %d, %d)\n", kilo_uid, num_agent_in_task[0], num_agent_in_task[1], num_agent_in_task[2]);
        printf("Robot %d: Task cost (%d, %d, %d)\n", kilo_uid, task_cost[0], task_cost[1], task_cost[2]);
        printf("Robot %d: Utility (%f, %f, %f, %f)\n", kilo_uid, individual_utility[0], individual_utility[1], individual_utility[2], individual_utility[3]);
        
        // Utility Comparison (Maximisation)
        float max_utility;
        max_utility = *std::max_element(individual_utility.begin(), individual_utility.end());
        int preferred_task; 
        preferred_task = std::max_element(individual_utility.begin(), individual_utility.end()) - individual_utility.begin();
        if (max_utility <= 0){
            preferred_task = TASK_NULL;
            printf("Robot %d: Chose void task as task info is unknown \n", kilo_uid);
        }
        else if (max_utility > individual_utility[chosen_task]){ // Partition evolved
            
            if(chosen_task!=TASK_NULL){
                num_agent_in_task[chosen_task-1] = num_agent_in_task[chosen_task-1] - 1;
            }            
            num_agent_in_task[preferred_task-1] += 1;
            num_iterations += 1;

            // Stamping
            if (random_time_stamp == 0){ // If this is the first time evolution of a blank partition. Otherwise, keep the informed time stamp value. 
                random_time_stamp = (unsigned char)std::rand();  
            }            
            local_memory_time_stamp[random_time_stamp] = true; 

            printf("Robot %d: Partition ID %d evolved to (%d, %d, %d) from %d to %d, iteration %d \n", kilo_uid, random_time_stamp, num_agent_in_task[0], num_agent_in_task[1], num_agent_in_task[2], chosen_task, preferred_task, num_iterations);

        }
        else
        {
            printf("Robot %d: Partition ID %d remain at (%d, %d, %d) at %d \n", kilo_uid, random_time_stamp, num_agent_in_task[0], num_agent_in_task[1], num_agent_in_task[2], chosen_task);
        }

        
        // Save to local memory
        local_memory_chosen_task[random_time_stamp] = preferred_task;

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
        switch (preferred_task){
            case TASK_NULL: 
                set_color(RGB(2,2,2));
                break;
            case 1: // Task 1
                set_color(RGB(0,2,0));
                break;
            case 2: // Task 2
                set_color(RGB(2,0,2));     
                break;
            case 3: // Task 3
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



