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

//#include "stage.hh"
//using namespace Stg;

#include "kilolib.h"
#include "evokilo1.h"

using namespace Kilolib;


// Single class variable for logging file pointer
FILE *Evokilo1::lfp = NULL;
FILE *Evokilo2::lfp = NULL;


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
        for(int j=0; j<NN_NUM_HIDDEN; j++)
            sum += nn_hidden[j] * (*ptr++);
        // Update the neuron value with the weighted input pushed through the transfer function
        nn_hidden[i] = sigmoid(sum);
    }
    // Work out the output node values
    for(int i=0; i<NN_NUM_OUTPUTS; i++)
    {
        float sum = 0;
        for(int j=0; j<NN_NUM_HIDDEN; j++)
            sum += nn_hidden[j] * (*ptr++);
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
        
        // Bias
        inputs[0]   = 1.0;
        // Nest
        inputs[1]   = (region == NEST) ? 1.0 : -1.0;
        // Food
        inputs[2]   = (region == FOOD) ? 1.0 : -1.0;
        // carrying food
        inputs[3]   = carrying ? 1.0 : -1.0;
        
        // Distance to nearest neighbours
        inputs[4]   = (float)min_dist / 100.0;
        // Number of neighbours
        inputs[5]   = messages;
        // Average message
        float avgmsg = messages==0 ? 0.0 : ((float)msgsum/messages - 128.0) / 100.0;
        inputs[6]   = avgmsg;
        
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
        
        // Message output
        msg.data[0] = (outputs[2] + 1.0) * 250;
        msg.crc     = message_crc(&msg);

        
        // visualise the internal state
        set_color(RGB(carrying?2:0, (region==NEST)?3:0, (region==FOOD)?3:0));
        //set_color_msg((outputs[2] + 1.0) / 2.0);
        set_color_msg(carrying);
        
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
        usec_t time = pos->GetWorld()->SimTimeNow();
        if (time - last_time >= 1e6)
        {
            last_time += 1e6;
            char buf[1024];
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[1],inputs[2],inputs[3], outputs[0],outputs[1], outputs[2]);
            Evokilo2::log(buf);
        }
    }
    
}

