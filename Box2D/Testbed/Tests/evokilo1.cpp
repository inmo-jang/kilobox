//----------------------------------------------------------
// Evokilo1 - First experiment in evolved kilobot controller
// (c) Simon Jones 2015
//----------------------------------------------------------

#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

//#include "stage.hh"
//using namespace Stg;

#include "kilolib.h"
#include "evokilo1.h"

//#include "bts.h"

using namespace Kilolib;


// Single class variable for logging file pointer
FILE *Minimal_example::lfp = NULL;
FILE *Orbit_star::lfp = NULL;
FILE *Orbit_planet::lfp = NULL;
FILE *Evokilo1::lfp = NULL;
FILE *Evokilo2::lfp = NULL;
FILE *Evokilo3::lfp = NULL;
FILE *Evokilo4::lfp = NULL;
FILE *Disperse::lfp = NULL;
FILE *Btdisperse::lfp = NULL;
FILE *NNdisperse::lfp = NULL;
FILE *Btsimple::lfp = NULL;
FILE *NNsimple::lfp = NULL;


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




float sigmoid(float x) {return tanh(x);}
float rect(float x) {if (x>0.0)return x; else return 0.0;}
//float rect(float x) {return x;}

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
#ifndef RECT
        nn_hidden[i] = sigmoid(sum);
#else
        nn_hidden[i] = rect(sum);
#endif
    }
    // Work out the output node values
    for(int i=0; i<NN_NUM_OUTPUTS; i++)
    {
        float sum = 0;
        for(int j=0; j<NN_NUM_HIDDEN; j++)
            sum += nn_hidden[j] * (*ptr++);
        // Bias
        sum += 1.0 * (*ptr++);
#ifndef RECT
        nn_outputs[i] = sigmoid(sum);
#else
        nn_outputs[i] = rect(sum);
#endif
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


void Evokilo4::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Evokilo4::tx_message;
    kilo_message_rx         = (message_rx_t)&Evokilo4::message_rx;
    
    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    
    last_region = 0;
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
        
        // Set up the inputs on the blackboard
        



        // Nest
        //bboard.inputs[0]    = (region == NEST) ? 1.0 : -1.0;
        // Food
        //bboard.inputs[1]    = (region == FOOD) ? 1.0 : -1.0;
        // Carrying food
        //bboard.inputs[2]    = carrying ? 1.0 : -1.0;
        // Distance to nearest neighbours
        //bboard.inputs[3]    = (float)min_dist / 100.0;
        // Number of neighbours
        //bboard.inputs[4]    = (float)messages / 10.0;
        // Average message
        float avgmsg = messages > 0 ? msgsum / messages : 0.0;
        //bboard.inputs[5]    = avgmsg;
        
        // Tick the behaviour tree
        bt->tick(&bboard);

        
        // Motor control
        //int d = (bboard.outputs[0] > 0 ? 1 : 0) | (bboard.outputs[1] > 0 ? 2 : 0);
        switch(0)
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
        //*(float*)msg.data = bboard.outputs[2];
        msg.data[4] = kilo_uid & 0xff;
        msg.crc     = message_crc(&msg);
        
        // visualise the internal state
        set_color(RGB(carrying?2:0, 0, 0));
        //set_color_msg((bboard.outputs[2] + 1.0) / 2.0);
        
        // Clear the message variables
        messages    = 0;
        msgsum      = 0;
        min_dist    = 150;
        
        // remember last region visited
        last_region = region;
        //last_output = d;
        
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
            /*snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, bboard.inputs[0],bboard.inputs[1],bboard.inputs[2], bboard.outputs[0],bboard.outputs[1], bboard.outputs[2]);*/
            Evokilo4::log(buf);
        }
    }
    
}




void Disperse::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Disperse::message_tx;
    kilo_message_rx         = (message_rx_t)&Disperse::message_rx;
    
    // Construct a valid message with our ID in the first two bytes
    msg.type    = NORMAL;
    msg.data[0] = kilo_uid & 0xff;
    msg.data[1] = (kilo_uid >> 8) & 0xff;
    msg.data[2] = 0;
    msg.crc     = message_crc(&msg);
    last_density = 0.0f;
    last_output = 0;
}

float Disperse::calc_density()
{
    // Density = sum(1/pi*r^2)
    // Scale so in kilobots/m^2
    float d = 0.0f;
    //printf("%3d: ", kilo_uid);
    for(ns_t::iterator i=neighbours_seen.begin(); i!=neighbours_seen.end(); ++i)
    {
        //printf("%4d ", i->second);
        d += 1/(M_PI * pow((float)i->second / 1000.0, 2));
    }
    //printf("\n");
    return d;
}

void Disperse::set_motion(int dir)
{
    switch(dir)
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
    last_output = dir;
}

void Disperse::loop()
{
    
    //int region;
    if (kilo_ticks > last_update + 32)
    {
        last_update = kilo_ticks;
        
        // Main loop, run through here approximately once a second
        
        // See if there is food
        if (get_environment() == FOOD)
        {
            set_color(RGB(0,3,0));
            found_food = 1;
        }
        if (told_about_food && !found_food)
        {
            set_color(RGB(3,3,0));
            found_food = 1;
        }
        
        // Get the density
        float density = calc_density();
        printf("%3d %f\n", kilo_uid, density);
        float ddelta = density - last_density;
        
        //if (fabs(density - dtarget) < dmargin)
        if (found_food || density < dtarget)
        {
            // Target met, stay still
            set_motion(0);
        }
        else if (ddelta > 0)
        {
            // Density going up, tumble
            int dir = rand_intrange(1,2);
            set_motion(dir);
        }
        else if (ddelta < 0)
        {
            // Density going down, run
            set_motion(3);
        }

        // Update the food part of the message
        msg.data[2] = found_food;
        msg.crc     = message_crc(&msg);

        
        last_density = density;
        // Reset the map of neighbour distances
        neighbours_seen.erase(neighbours_seen.begin(), neighbours_seen.end());
    }
    
    //===========================================================================
    // Stage only, non kilobot logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            //snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
            //         pos->GetPose().x, pos->GetPose().y, bboard.inputs[0],bboard.inputs[1],bboard.inputs[2], bboard.outputs[0],bboard.outputs[1], bboard.outputs[2]);
            //Evokilo4::log(buf);
        }
    }
    
}


//===========================================================================


void Btdisperse::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Btdisperse::message_tx;
    kilo_message_rx         = (message_rx_t)&Btdisperse::message_rx;
    
    // Construct a valid message with our ID in the first two bytes
    msg.type        = NORMAL;
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = 0;
    msg.data[3]     = max_hops;
    msg.data[6]     = max_hops;
    msg.crc         = message_crc(&msg);
    last_density    = 0.0f;
    for(int i=0; i<SMOOTHING; i++)
    {
        dist_to_food_smooth[i] = max_food_dist;
        dist_to_nest_smooth[i] = max_nest_dist;
        density_smooth[i]  = density;
    }
    for(int i=0; i<10; i++)
        bboard[i] = 0.0f;
    for(int i=0; i<ENV_BUF_SIZE; i++)
        env_buf[i] = 0;

}

int Btdisperse::get_filtered_environment()
{
    int counts[4] = {0,0,0,0};
    for(int i=0; i<ENV_BUF_SIZE; i++)
    {
        counts[env_buf[i]]++;
    }
    if (counts[0] > ENV_BUF_SIZE / 2) return 0;
    if (counts[1] > ENV_BUF_SIZE / 2) return 1;
    if (counts[2] > ENV_BUF_SIZE / 2) return 2;
    if (counts[3] > ENV_BUF_SIZE / 2) return 3;
    return 0;
}

float Btdisperse::calc_density()
{
    // Density = sum(1/pi*r^2)
    // Scale so in kilobots/m^2
    float d = 0.0f;
    for(int i = 0; i < ns_ptr; ++i)
    {
        d += 1/(M_PI * pow((float)neighbours_seen_dist[i] / 1000.0, 2));
    }
    return d;
}

void Btdisperse::set_motion(int dir)
{
    static int last_output = 0;
    switch(dir)
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
    last_output = dir;
}

Btdisperse::message_t *Btdisperse::message_tx()
{
    // Update the food part of the message
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = found_food;
    msg.data[3]     = hops_to_food;
    msg.data[4]     = dist_to_food & 0xff;
    msg.data[5]     = dist_to_food >> 8;
    msg.data[6]     = hops_to_nest;
    msg.data[7]     = dist_to_nest & 0xff;
    msg.data[8]     = dist_to_nest >> 8;
    msg.crc         = message_crc(&msg);
    return &msg;
}

void Btdisperse::message_rx(message_t *m, distance_measurement_t *d)
{
    new_message             += 1;
    int dist                = estimate_distance(d);
    int uid                 = m->data[0] | (m->data[1] << 8);
    
    int found = 0;
    for(int i = 0; i < ns_ptr; ++i)
    {
        if (neighbours_seen_uid[i] == uid)
        {
            neighbours_seen_dist[i] = dist;
            found                   = 1;
        }
    }
    if (!found && ns_ptr < MAX_NEIGHBOURS)
    {
        neighbours_seen_uid[ns_ptr]     = uid;
        neighbours_seen_dist[ns_ptr]    = dist;
        ns_ptr++;
    }
    
    int heard_about_food    = m->data[2] > 0;
    int hops                = m->data[3];
    
    told_about_food         |= heard_about_food;
    if (hops < min_hops_seen)
    {
        min_hops_seen       = hops;
        accum_dist_to_food  = (m->data[4] | (m->data[5] << 8)) + dist;
    }
    
    int nest_hops           = m->data[6];
    if (nest_hops < min_nest_hops_seen)
    {
        min_nest_hops_seen  = nest_hops;
        accum_dist_to_nest  = (m->data[7] | (m->data[8] << 8)) + dist;
    }
    
}


void Btdisperse::preamble()
{
    // Get the inputs needed for the BT
    int env = get_filtered_environment();
    detected_food   = env == FOOD;
    detected_nest   = env == NEST;
    float raw_density       = 0;
    if (new_message)
    {
        // We have messages, so we can recalul
        raw_density         = calc_density();
        
        if (min_hops_seen < max_hops)
        {
            hops_to_food    = min_hops_seen + 1;
            dist_to_food    = accum_dist_to_food;
        }
        else
        {
            hops_to_food    = max_hops;
            dist_to_food    = max_food_dist;
        }
        
        if (min_nest_hops_seen < max_hops)
        {
            hops_to_nest    = min_nest_hops_seen + 1;
            dist_to_nest    = accum_dist_to_nest;
        }
        else
        {
            hops_to_nest    = max_hops;
            dist_to_nest    = max_nest_dist;
        }
    }
    else
    {
        // We have no message, default to assuming max dist from everything, but revised
        // by environmental detection
        dist_to_food = max_food_dist;
        dist_to_nest = max_nest_dist;
    }
    if (detected_food)
    {
        hops_to_food    = 0;
        dist_to_food    = 0;
    }
    if (detected_nest)
    {
        hops_to_nest    = 0;
        dist_to_nest    = 0;
    }
    
    
    last_dfood      = dfood;
    last_dnest      = dnest;
    last_density    = density;

    
    dist_to_food_smooth[dtf_ptr] = dist_to_food;
    dist_to_nest_smooth[dtf_ptr] = dist_to_nest;
    density_smooth[dtf_ptr] = raw_density;

    dtf_ptr = (dtf_ptr + 1) % SMOOTHING;
    dfood = 0;
    dnest = 0;
    density = 0.0f;
    for(int i=0;i<SMOOTHING;i++)
    {
        dfood += dist_to_food_smooth[i];
        dnest += dist_to_nest_smooth[i];
        density += density_smooth[i];
    }
    dfood /= SMOOTHING;
    dnest /= SMOOTHING;
    density /= SMOOTHING;

    
    
    // Little state machine for food transport: always collect food if in food area
    // and deposit it if in nest area
    if (carrying_food && detected_nest)
    {
        carrying_food = 0;
        total_food++;
    }
    else if (!carrying_food && detected_food)
    {
        carrying_food = 1;
        total_pickup++;
    }
    
    
}

void Btdisperse::postamble()
{
    
    
    // Reset the map of neighbour distances
    ns_ptr              = 0;
    told_about_food     = 0;
    min_hops_seen       = max_hops;
    min_nest_hops_seen  = max_hops;
    new_message         = 0;
}

void Btdisperse::loop()
{
    // Always run the environment update
    if (kilo_ticks > last_env_update + 4)
    {
        last_env_update = kilo_ticks;
        env_buf[env_ptr] = get_environment();
        env_ptr = (env_ptr+1) % ENV_BUF_SIZE;
    }
    
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        // Main loop, run through here approximately once a second
        
        preamble();
        
        
        // Outputs or state variables:
        //  [0] motion
        //  [1] found_food      (to message subsystem)
        //  [2] harvester       (internal state)
        
        // Inputs:
        //  [3] detected_food   (get_environment() == FOOD)
        //  [4] told_about_food (from message subsystem)
        //  [5] density         (from message subsystem)
        //  [6] delta_density
        //  [7] delta_food_dist
        //  [8] delta_nest_dist
        //  [9] carrying_food
        
        // Motor values always reset to zero, then potentially altered
        bboard[0] = 0;
        
        // Values are always boolean or in range -1.0 to 1.0
        bboard[3]  = detected_food;
        bboard[4]  = told_about_food;
        bboard[5]  = density / 1000.0;
        bboard[6]  = (density - last_density) / 1000.0;
        bboard[7]  = (dfood - last_dfood) / 1000.0;
        bboard[8]  = (dnest - last_dnest) / 1000.0;
        bboard[9]  = carrying_food;
        
        DBPRINT("kb:%d\n",kilo_uid);
        set_vars(bboard);
        tick(bt);
        
        int motion = (int)bboard[0];
        found_food = bboard[1] > 0.5;
        
        set_motion(motion);

        set_color(carrying_food, density / 1000.0, bboard[7]);
       
        postamble();
        
    }
    
    //===========================================================================
    // Sim logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024,
                     //printf(
                     "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%4d,%4d,%4d,%4d,%4d,%4d\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y,
                     bboard[0], bboard[1], bboard[9], bboard[3], bboard[4], bboard[5], bboard[6],
                     bboard[7], bboard[8], bboard[2],
                     msg.data[2], msg.data[3], hops_to_food,  new_message, dist_to_food, dist_to_nest
                     );
            Btdisperse::log(buf);
        }
    }
    
}

void NNdisperse::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&NNdisperse::message_tx;
    kilo_message_rx         = (message_rx_t)&NNdisperse::message_rx;
    
    // Construct a valid message with our ID in the first two bytes
    msg.type        = NORMAL;
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = 0;
    msg.data[3]     = max_hops;
    msg.data[6]     = max_hops;
    msg.crc         = message_crc(&msg);
    last_density    = 0.0f;
    for(int i=0; i<SMOOTHING; i++)
    {
        dist_to_food_smooth[i] = max_food_dist;
        dist_to_nest_smooth[i] = max_nest_dist;
        density_smooth[i]  = density;
    }
    for(int i=0; i<10; i++)
        bboard[i] = 0.0f;
    for(int i=0; i<ENV_BUF_SIZE; i++)
        env_buf[i] = 0;
}

int NNdisperse::get_filtered_environment()
{
    int counts[4] = {0,0,0,0};
    for(int i=0; i<ENV_BUF_SIZE; i++)
    {
        counts[env_buf[i]]++;
    }
    if (counts[0] > ENV_BUF_SIZE / 2) return 0;
    if (counts[1] > ENV_BUF_SIZE / 2) return 1;
    if (counts[2] > ENV_BUF_SIZE / 2) return 2;
    if (counts[3] > ENV_BUF_SIZE / 2) return 3;
    return 0;
}


float NNdisperse::calc_density()
{
    // Density = sum(1/pi*r^2)
    // Scale so in kilobots/m^2
    float d = 0.0f;
    for(int i = 0; i < ns_ptr; ++i)
    {
        d += 1/(M_PI * pow((float)neighbours_seen_dist[i] / 1000.0, 2));
    }
    return d;
}

void NNdisperse::set_motion(int dir)
{
    static int last_output = 0;
    switch(dir)
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
    last_output = dir;
}

NNdisperse::message_t *NNdisperse::message_tx()
{
    // Update the food part of the message
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = found_food;
    msg.data[3]     = hops_to_food;
    msg.data[4]     = dist_to_food & 0xff;
    msg.data[5]     = dist_to_food >> 8;
    msg.data[6]     = hops_to_nest;
    msg.data[7]     = dist_to_nest & 0xff;
    msg.data[8]     = dist_to_nest >> 8;
    msg.crc         = message_crc(&msg);
    return &msg;
}


void NNdisperse::message_rx(message_t *m, distance_measurement_t *d)
{
    new_message             += 1;
    int dist                = estimate_distance(d);
    int uid                 = m->data[0] | (m->data[1] << 8);
    
    int found = 0;
    for(int i = 0; i < ns_ptr; ++i)
    {
        if (neighbours_seen_uid[i] == uid)
        {
            neighbours_seen_dist[i] = dist;
            found                   = 1;
        }
    }
    if (!found && ns_ptr < MAX_NEIGHBOURS)
    {
        neighbours_seen_uid[ns_ptr]     = uid;
        neighbours_seen_dist[ns_ptr]    = dist;
        ns_ptr++;
    }
    
    int heard_about_food    = m->data[2] > 0;
    int hops                = m->data[3];
    
    told_about_food         |= heard_about_food;
    if (hops < min_hops_seen)
    {
        min_hops_seen       = hops;
        accum_dist_to_food  = (m->data[4] | (m->data[5] << 8)) + dist;
    }
    
    int nest_hops           = m->data[6];
    if (nest_hops < min_nest_hops_seen)
    {
        min_nest_hops_seen  = nest_hops;
        accum_dist_to_nest  = (m->data[7] | (m->data[8] << 8)) + dist;
    }
    
}

void NNdisperse::preamble()
{
    // Get the inputs needed for the BT
    int env = get_filtered_environment();
    detected_food   = env == FOOD;
    detected_nest   = env == NEST;
    float raw_density       = 0;
    if (new_message)
    {
        // We have messages, so we can recalul
        raw_density         = calc_density();
        
        if (min_hops_seen < max_hops)
        {
            hops_to_food    = min_hops_seen + 1;
            dist_to_food    = accum_dist_to_food;
        }
        else
        {
            hops_to_food    = max_hops;
            dist_to_food    = max_food_dist;
        }
        
        if (min_nest_hops_seen < max_hops)
        {
            hops_to_nest    = min_nest_hops_seen + 1;
            dist_to_nest    = accum_dist_to_nest;
        }
        else
        {
            hops_to_nest    = max_hops;
            dist_to_nest    = max_nest_dist;
        }
    }
    else
    {
        // We have no message, default to assuming max dist from everything, but revised
        // by environmental detection
        dist_to_food = max_food_dist;
        dist_to_nest = max_nest_dist;
    }
    if (detected_food)
    {
        hops_to_food    = 0;
        dist_to_food    = 0;
    }
    if (detected_nest)
    {
        hops_to_nest    = 0;
        dist_to_nest    = 0;
    }
    
    
    last_dfood      = dfood;
    last_dnest      = dnest;
    last_density    = density;

    
    dist_to_food_smooth[dtf_ptr] = dist_to_food;
    dist_to_nest_smooth[dtf_ptr] = dist_to_nest;
    density_smooth[dtf_ptr] = raw_density;

    dtf_ptr = (dtf_ptr + 1) % SMOOTHING;
    dfood = 0;
    dnest = 0;
    density = 0.0f;
    for(int i=0;i<SMOOTHING;i++)
    {
        dfood += dist_to_food_smooth[i];
        dnest += dist_to_nest_smooth[i];
        density += density_smooth[i];
    }
    dfood /= SMOOTHING;
    dnest /= SMOOTHING;
    density /= SMOOTHING;

    
    
    // Little state machine for food transport: always collect food if in food area
    // and deposit it if in nest area
    if (carrying_food && detected_nest)
    {
        carrying_food = 0;
        total_food++;
    }
    else if (!carrying_food && detected_food)
    {
        carrying_food = 1;
        total_pickup++;
    }
    
    
}

void NNdisperse::postamble()
{
    
    
    // Reset the map of neighbour distances
    ns_ptr              = 0;
    told_about_food     = 0;
    min_hops_seen       = max_hops;
    min_nest_hops_seen  = max_hops;
    new_message         = 0;
}

void NNdisperse::loop()
{
    // Always run the environment update
    if (kilo_ticks > last_env_update + 4)
    {
        last_env_update = kilo_ticks;
        env_buf[env_ptr] = get_environment();
        env_ptr = (env_ptr+1) % ENV_BUF_SIZE;
    }
    
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        // Main loop, run through here approximately once a second
        
        preamble();
        
        
        // Outputs or state variables:
        //  [0] motion
        //  [1] found_food      (to message subsystem)
        //  [2] harvester       (internal state)
        
        // Inputs:
        //  [3] detected_food   (get_environment() == FOOD)
        //  [4] told_about_food (from message subsystem)
        //  [5] density         (from message subsystem)
        //  [6] delta_density
        //  [7] delta_food_dist
        //  [8] delta_nest_dist
        //  [9] carrying_food
        
        // Motor values always reset to zero, then potentially altered
        bboard[0] = 0;
        
        // Values are always boolean or in range -1.0 to 1.0
        bboard[3]  = detected_food;
        bboard[4]  = told_about_food;
        bboard[5]  = density / 1000.0;
        bboard[6]  = (density - last_density) / 1000.0;
        bboard[7]  = (dfood - last_dfood) / 1000.0;
        bboard[8]  = (dnest - last_dnest) / 1000.0;
        bboard[9]  = carrying_food;
        
        
        //set_vars(bboard);
        //tick(bt);
        
        //===============
        // Run the neural net
        float inputs[9];
        float *outputs;
        for(int i=0;i<9;i++)
            inputs[i] = bboard[i+1];
        outputs     = nn.nn_update(&inputs[0]);

        int motion  = (outputs[0] > 0 ? 1 : 0) + (outputs[1] > 0 ? 2 : 0);
        bboard[1]   = outputs[2];
        bboard[2]   = outputs[3];
        //================
        
        found_food  = bboard[1] > 0.5;
        
        set_motion(motion);
        
        set_color(carrying_food, density / 1000.0, bboard[7]);
        
        postamble();
        
    }
    
    //===========================================================================
    // Sim logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024,
                     //printf(
                     "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%4d,%4d,%4d,%4d,%4d,%4d\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y,
                     bboard[0], bboard[1], bboard[9], bboard[3], bboard[4], bboard[5], bboard[6],
                     bboard[7], bboard[8], bboard[2],
                     msg.data[2], msg.data[3], hops_to_food,  new_message, dist_to_food, dist_to_nest
                     );
            NNdisperse::log(buf);
        }
    }
    
}




void Btsimple::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Btsimple::message_tx;
    kilo_message_rx         = (message_rx_t)&Btsimple::message_rx;
    
    // Construct a valid message with our ID in the first two bytes
    msg.type        = NORMAL;
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = 0;
    msg.data[3]     = max_hops;
    msg.data[6]     = max_hops;
    msg.crc         = message_crc(&msg);
    last_density    = 0.0f;
    for(int i=0; i<SMOOTHING; i++)
    {
        dist_to_food_smooth[i] = max_food_dist;
        dist_to_nest_smooth[i] = max_nest_dist;
        density_smooth[i]  = density;
    }
    for(int i=0; i<bboard_size; i++)
        bboard[i] = 0.0f;
    for(int i=0; i<ENV_BUF_SIZE; i++)
        env_buf[i] = 0;
}


int Btsimple::get_filtered_environment()
{
    int counts[4] = {0,0,0,0};
    for(int i=0; i<ENV_BUF_SIZE; i++)
    {
        counts[env_buf[i]]++;
    }
    if (counts[0] > ENV_BUF_SIZE / 2) return 0;
    if (counts[1] > ENV_BUF_SIZE / 2) return 1;
    if (counts[2] > ENV_BUF_SIZE / 2) return 2;
    if (counts[3] > ENV_BUF_SIZE / 2) return 3;
    return 0;
}

float Btsimple::calc_density()
{
    // Density = sum(1/pi*r^2)
    // Scale so in kilobots/m^2
    float d = 0.0f;
    for(int i = 0; i < ns_ptr; ++i)
    {
        d += 1/(M_PI * pow((float)neighbours_seen_dist[i] / 1000.0, 2));
    }
    return d;

}

void Btsimple::set_motion(int dir)
{
    static int last_output = 0;
    switch(dir)
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
    last_output = dir;
}

void Btsimple::message_rx(message_t *m, distance_measurement_t *d)
{
    new_message             += 1;
    int dist                = estimate_distance(d);
    int uid                 = m->data[0] | (m->data[1] << 8);
    
    int found = 0;
    for(int i = 0; i < ns_ptr; ++i)
    {
        if (neighbours_seen_uid[i] == uid)
        {
            neighbours_seen_dist[i] = dist;
            found                   = 1;
        }
    }
    if (!found && ns_ptr < MAX_NEIGHBOURS)
    {
        neighbours_seen_uid[ns_ptr]     = uid;
        neighbours_seen_dist[ns_ptr]    = dist;
        ns_ptr++;
    }

    int signal              = m->data[2];
    int hops                = m->data[3];
    
    receive_signal         |= signal;
    if (hops < min_hops_seen)
    {
        min_hops_seen       = hops;
        accum_dist_to_food  = (m->data[4] | (m->data[5] << 8)) + dist;
    }
    
    int nest_hops           = m->data[6];
    if (nest_hops < min_nest_hops_seen)
    {
        min_nest_hops_seen  = nest_hops;
        accum_dist_to_nest  = (m->data[7] | (m->data[8] << 8)) + dist;
    }
    
    
}

Btsimple::message_t *Btsimple::message_tx()
{
    // Update the food part of the message
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = send_signal;
    msg.data[3]     = hops_to_food;
    msg.data[4]     = dist_to_food & 0xff;
    msg.data[5]     = dist_to_food >> 8;
    msg.data[6]     = hops_to_nest;
    msg.data[7]     = dist_to_nest & 0xff;
    msg.data[8]     = dist_to_nest >> 8;
    msg.crc         = message_crc(&msg);
    return &msg;
}



void Btsimple::preamble()
{
    // Get the inputs needed for the BT
    int env = get_filtered_environment();
    detected_food   = env == FOOD;
    detected_nest   = env == NEST;
    float raw_density       = 0;
    if (new_message)
    {
        // We have messages, so we can recalul
        raw_density         = calc_density();
        
        if (min_hops_seen < max_hops)
        {
            hops_to_food    = min_hops_seen + 1;
            dist_to_food    = accum_dist_to_food;
        }
        else
        {
            hops_to_food    = max_hops;
            dist_to_food    = max_food_dist;
        }
        
        if (min_nest_hops_seen < max_hops)
        {
            hops_to_nest    = min_nest_hops_seen + 1;
            dist_to_nest    = accum_dist_to_nest;
        }
        else
        {
            hops_to_nest    = max_hops;
            dist_to_nest    = max_nest_dist;
        }
    }
    else
    {
        // We have no message, default to assuming max dist from everything, but revised
        // by environmental detection
        dist_to_food = max_food_dist;
        dist_to_nest = max_nest_dist;
    }
    if (detected_food)
    {
        hops_to_food    = 0;
        dist_to_food    = 0;
    }
    if (detected_nest)
    {
        hops_to_nest    = 0;
        dist_to_nest    = 0;
    }
    
    
    last_dfood = dfood;
    last_dnest = dnest;
    last_density    = density;

    
    dist_to_food_smooth[dtf_ptr] = dist_to_food;
    dist_to_nest_smooth[dtf_ptr] = dist_to_nest;
    density_smooth[dtf_ptr] = raw_density;
    
    dtf_ptr = (dtf_ptr + 1) % SMOOTHING;
    
    dfood = 0;
    dnest = 0;
    density = 0.0f;
    for(int i=0;i<SMOOTHING;i++)
    {
        dfood += dist_to_food_smooth[i];
        dnest += dist_to_nest_smooth[i];
        density += density_smooth[i];
    }
    dfood /= SMOOTHING;
    dnest /= SMOOTHING;
    density /= SMOOTHING;
    
    
    // Little state machine for food transport: always collect food if in food area
    // and deposit it if in nest area
    if (carrying_food && detected_nest)
    {
        carrying_food = 0;
        total_food++;
    }
    else if (!carrying_food && detected_food)
    {
        carrying_food = 1;
        total_pickup++;
    }
    
    
}

void Btsimple::postamble()
{
    
    
    // Reset the map of neighbour distances
    ns_ptr              = 0;
    receive_signal      = 0;
    min_hops_seen       = max_hops;
    min_nest_hops_seen  = max_hops;
    new_message         = 0;
}

void Btsimple::loop()
{
    // Always run the environment update
    if (kilo_ticks > last_env_update + 4)
    {
        last_env_update = kilo_ticks;
        env_buf[env_ptr] = get_environment();
        env_ptr = (env_ptr+1) % ENV_BUF_SIZE;
    }

    
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        // Main loop, run through here approximately once a second
        
        preamble();
        
        
        // Outputs or state variables:
        //  [0] motion
        //  [1] send signal (to message subsystem)
        
        // Inputs:
        //  [2] carrying_food
        //  [3] receive signal (from message subsystem)
        //  [4] delta_density
        //  [5] delta_food_dist
        //  [6] delta_nest_dist
        
        // Motor values always reset to zero, then potentially altered
        bboard[0] = 0;
        
        // Values are always boolean or in range -1.0 to 1.0
        bboard[2]  = carrying_food;
        bboard[3]  = detected_food;
        bboard[4]  = receive_signal;
        bboard[5]  = (density - last_density) / 1000.0;
        bboard[6]  = (dfood - last_dfood) / 1000.0;
        bboard[7]  = (dnest - last_dnest) / 1000.0;
        
        DBPRINT("kb:%d\n",kilo_uid);
        set_vars(bboard);
        tick(bt);
        
        int motion = (int)bboard[0];
        send_signal = bboard[1] > 0.5;
        
        set_motion(motion);
        
        set_color(carrying_food, density/1000.0, 0);
        
        ns_ptr_save = ns_ptr;
        postamble();
        
    }
    
    //===========================================================================
    // Sim logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024,
            //printf(
                     "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%4d,%4d,%4d,%4d,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y,
                     bboard[0], bboard[1], bboard[2], bboard[3], bboard[4], bboard[5],
                     detected_food, detected_nest, hops_to_food,  ns_ptr_save, density, dfood, dnest, kilo_ticks_real
                     );
            Btsimple::log(buf);
        }
    }
    
}

void NNsimple::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&NNsimple::message_tx;
    kilo_message_rx         = (message_rx_t)&NNsimple::message_rx;
    
    // Construct a valid message with our ID in the first two bytes
    msg.type        = NORMAL;
    msg.data[0]     = kilo_uid & 0xff;
    msg.data[1]     = (kilo_uid >> 8) & 0xff;
    msg.data[2]     = 0;
    msg.data[3]     = max_hops;
    msg.data[6]     = max_hops;
    msg.crc         = message_crc(&msg);
    last_density    = 0.0f;
    for(int i=0; i<5; i++)
    {
        dist_to_food_smooth[i] = max_food_dist;
        dist_to_nest_smooth[i] = max_nest_dist;
    }
    for(int i=0; i<5; i++)
        bboard[i] = 0.0f;
}

float NNsimple::calc_density()
{
    // Density = sum(1/pi*r^2)
    // Scale so in kilobots/m^2
    float d = 0.0f;
    //printf("%3d: ", kilo_uid);
    for(ns_t::iterator i=neighbours_seen.begin(); i!=neighbours_seen.end(); ++i)
    {
        //printf("%4d ", i->second);
        d += 1/(M_PI * pow((float)i->second / 1000.0, 2));
    }
    //printf("\n");
    return d;
}

void NNsimple::set_motion(int dir)
{
    static int last_output = 0;
    switch(dir)
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
    last_output = dir;
}

void NNsimple::message_rx(message_t *m, distance_measurement_t *d)
{
    new_message             += 1;
    int dist                = estimate_distance(d);
    int uid                 = m->data[0] | (m->data[1] << 8);
    neighbours_seen[uid]    = dist;
    int heard_about_food    = m->data[2] > 0;
    int hops                = m->data[3];
    
    told_about_food         |= heard_about_food;
    if (hops < min_hops_seen)
    {
        min_hops_seen       = hops;
        accum_dist_to_food  = (m->data[4] | (m->data[5] << 8)) + dist;
    }
    
    int nest_hops           = m->data[6];
    if (nest_hops < min_nest_hops_seen)
    {
        min_nest_hops_seen  = nest_hops;
        accum_dist_to_nest  = (m->data[7] | (m->data[8] << 8)) + dist;
    }
    
    
}

NNsimple::message_t *NNsimple::message_tx()
{
    // Update the food part of the message
    msg.data[2]     = carrying_food;
    msg.data[3]     = hops_to_food;
    msg.data[4]     = dist_to_food & 0xff;
    msg.data[5]     = dist_to_food >> 8;
    msg.data[6]     = hops_to_nest;
    msg.data[7]     = dist_to_nest & 0xff;
    msg.data[8]     = dist_to_nest >> 8;
    msg.crc         = message_crc(&msg);
    return &msg;
}

int NNsimple::get_filtered_environment()
{
    return get_environment();
}

void NNsimple::preamble()
{
    // Get the inputs needed for the BT
    detected_food   = get_filtered_environment() == FOOD;
    detected_nest   = get_filtered_environment() == NEST;
    if (new_message)
    {
        density         = calc_density();
        
        if (min_hops_seen < max_hops)
        {
            hops_to_food    = min_hops_seen + 1;
            dist_to_food    = accum_dist_to_food;
        }
        else
        {
            hops_to_food    = max_hops;
            dist_to_food    = max_food_dist;
        }
        
        if (min_nest_hops_seen < max_hops)
        {
            hops_to_nest    = min_nest_hops_seen + 1;
            dist_to_nest    = accum_dist_to_nest;
        }
        else
        {
            hops_to_nest    = max_hops;
            dist_to_nest    = max_nest_dist;
        }
    }
    if (detected_food)
    {
        hops_to_food    = 0;
        dist_to_food    = 0;
    }
    if (detected_nest)
    {
        hops_to_nest    = 0;
        dist_to_nest    = 0;
    }
    
    
    last_dfood = dfood;
    last_dnest = dnest;
    
    dist_to_food_smooth[dtf_ptr] = dist_to_food;
    dist_to_nest_smooth[dtf_ptr] = dist_to_nest;
    dtf_ptr = (dtf_ptr + 1) % 5;
    for(int i=0;i<5;i++)
    {
        dfood += dist_to_food_smooth[i];
        dnest += dist_to_nest_smooth[i];
    }
    dfood /= 5;
    dnest /= 5;
    
    
    // Little state machine for food transport: always collect food if in food area
    // and deposit it if in nest area
    if (carrying_food && detected_nest)
    {
        carrying_food = 0;
        total_food++;
    }
    else if (!carrying_food && detected_food)
    {
        carrying_food = 1;
        total_pickup++;
    }
    
    
}

void NNsimple::postamble()
{
    last_density    = density;
    
    
    // Reset the map of neighbour distances
    neighbours_seen.erase(neighbours_seen.begin(), neighbours_seen.end());
    told_about_food     = 0;
    min_hops_seen       = max_hops;
    min_nest_hops_seen  = max_hops;
    new_message         = 0;
}

void NNsimple::loop()
{
    
    if (kilo_ticks > last_update + 32)
    {
        last_update = kilo_ticks;
        // Main loop, run through here approximately once a second
        
        preamble();
        
        
        // Outputs or state variables:
        //  [0] motion
        
        
        // Inputs:
        //  [1] carrying_food
        //  [2] told_about_food (from message subsystem)
        //  [3] delta_density
        //  [4] delta_food_dist
        //  [5] delta_nest_dist
        
        // Motor values always reset to zero, then potentially altered
        bboard[0] = 0;
        
        // Values are always boolean or in range -1.0 to 1.0
        bboard[1]  = carrying_food;
        bboard[2]  = told_about_food;
        bboard[3]  = density / 1000.0;
        bboard[4]  = (density - last_density) / 1000.0;
        //bboard[4]  = (dfood - last_dfood) / 1000.0;
        bboard[5]  = (dnest - last_dnest) / 1000.0;
        
        DBPRINT("kb:%d\n",kilo_uid);
        //set_vars(bboard);
        //tick(bt);
        //int motion = (int)bboard[0];

        
        //===============
        // Run the neural net
        float inputs[5];
        float *outputs;
        for(int i=0;i<5;i++)
            inputs[i] = bboard[i+1];
        outputs     = nn.nn_update(&inputs[0]);
        
        int motion  = (outputs[0] > 0 ? 1 : 0) + (outputs[1] > 0 ? 2 : 0);
        //================

        
        
        set_motion(motion);
        
        set_color(bboard[1], (float)dnest/max_nest_dist, bboard[2]);
        
        postamble();
        
    }
    
    //===========================================================================
    // Sim logging
    {
        usec_t time = world_us_simtime;
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024,
                     //printf(
                     "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%4d,%4d,%4d,%4d,%4d,%4d\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y,
                     bboard[0], bboard[2], bboard[1], bboard[3], bboard[4], bboard[5],
                     msg.data[2], msg.data[3], hops_to_food,  new_message, dist_to_food, dist_to_nest
                     );
            NNsimple::log(buf);
        }
    }
    
}


