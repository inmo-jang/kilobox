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

std::vector<float> Neural_net::update(std::vector<float> inputs)
{
    if (inputs.size() != input_nodes)
    {
        printf("mismatch on number of inputs to NN\n");
        exit(1);
    }
    // Work out new values for hidden neurons. Because this is a recurrant
    // neural net, the hidden nodes previous value is also included in the
    // weighted sum
    for(int i=0; i<hidden_nodes; i++)
    {
        float sum = 0;
        // First the input nodes
        for(int j=0; j<input_nodes; j++)
            sum += inputs[j] * hidden.neurons[i].weights[j];
        // Then the hidden nodes
        for(int j=0; j<hidden_nodes; j++)
            sum += hidden.neurons[j].value * hidden.neurons[i].weights[j + input_nodes];
        // Update the neuron value with the weighted input pushed through the transfer function
        hidden.neurons[i].value = sigmoid(sum);
    }
    // Work out the output node values
    std::vector<float> out;
    for(int i=0; i<output_nodes; i++)
    {
        float sum = 0;
        for(int j=0; j<hidden_nodes; j++)
            sum += hidden.neurons[j].value * output.neurons[i].weights[j];
        out.push_back(sigmoid(sum));
    }
    return out;
}

void Neural_net::set_weights(std::vector<float> w)
{
    int idx = 0;
    for(int i=0; i<hidden_nodes; i++)
        for(int j=0; j<hidden.neurons[i].num_inputs; j++)
            hidden.neurons[i].weights[j] = w[idx++];
    for(int i=0; i<output_nodes; i++)
        for(int j=0; j<output.neurons[i].num_inputs; j++)
            output.neurons[i].weights[j] = w[idx++];
}

std::vector<float> Neural_net::get_weights()
{
    std::vector<float> w;
    for(int i=0; i<hidden_nodes; i++)
        for(int j=0; j<hidden.neurons[i].num_inputs; j++)
            w.push_back(hidden.neurons[i].weights[j]);
    for(int i=0; i<output_nodes; i++)
        for(int j=0; j<output.neurons[i].num_inputs; j++)
            w.push_back(output.neurons[i].weights[j]);
    return w;
}



void Evokilo1::setup()
{
    // Set the callbacks
    kilo_message_tx         = (message_tx_t)&Evokilo1::tx_message;
    kilo_message_tx_success = (message_tx_success_t)&Evokilo1::tx_message_success;
    kilo_message_rx         = (message_rx_t)&Evokilo1::message_rx;

    // Construct a valid message
    msg.type    = NORMAL;
    msg.crc     = message_crc(&msg);
    
    sent_message    = 0;
    send_message    = 1;
    messages        = 0;
    avg_dist        = 150;
    min_dist        = 150;
    avg_message     = 0;

    carrying    = 0;
    total_food  = 0;
    total_trail = 0;
    total_pickup= 0;
    last_update = 0;

    nest        = 0;
    food        = 0;
    pheromone   = 0;

    //printf("%3d evokilo setup\n", kilo_uid);
}

void Evokilo1::loop()
{
    // Run the NN at the same rate as the message send, roughly twice a second
    // Always send a message

    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
        
        // Every cycle, build the inputs to the neuron net, compute the outputs
        // and set the actuators using the outputs

        float light     = get_ambientlight();
        nest            = (light > 50)  && (light < 512)    ? 1.0 : -1.0;
        food            = (light > 512) && (light < 1000)   ? 1.0 : -1.0;
        
        // Little state machine for food transport: always collect food if in food area
        // and deposit it if in nest area
        if ((nest > 0) && carrying)
        {
            carrying = 0;
            total_food++;
            //printf("%s food:%d\n",pos->Token(), total_food);
        }
        else if ((food > 0) && !carrying)
        {
            carrying = 1;
            total_pickup++;
            
        }
        
        // Bias
        inputs[0]   = 1.0;
        // Random
        inputs[1]   = rand_gaussian(1.0);
        // Nest
        inputs[2]   = nest;
        // Food
        inputs[3]   = food;
        // carrying food
        inputs[4]   = carrying ? 1.0 : -1.0;
        
        
        // Average distance to neighbours
        //inputs[5]   = min_dist;
        // Number of neighbours
        //inputs[6]   = messages;
  


        
        // Run the neural net
        outputs     = nn.update(inputs);
        
        //printf("%s: %8.5f %8.5f %8.5f %8.5f\n", pos->Token(), outputs[0], outputs[1], outputs[2], outputs[3]);
        
        // Motor control
        int d = (outputs[0] >= 0 ? 1 : 0) | (outputs[1] >= 0 ? 2 : 0);
        switch(d)
        {
            case(0):
                set_motors(0,0);
                break;
            case(1):
                set_motors(kilo_turn_left,0);
                break;
            case(2):
                set_motors(0,kilo_turn_right);
                break;
            case(3):
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
        }

        
        // visualise the internal state
        set_color(RGB(carrying?3:0, nest>0?3:0, food>0?3:0));

        

        
        
        // Clear the message count
        messages = 0;
        avg_dist = 150;
        min_dist = 150;
        avg_message = 0;
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
                     pos->GetPose().x, pos->GetPose().y, inputs[2],inputs[3],inputs[4], outputs[0],outputs[1]);
            Evokilo1::log(buf);
        }
    }
    
}

