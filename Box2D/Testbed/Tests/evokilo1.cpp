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

        colour_t amb    = get_ambient();
        nest            = amb.g > 0.5 ? 1.0 : -1.0;
        food            = amb.b > 0.5 ? 1.0 : -1.0;
        pheromone       = amb.r;
        
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
        // Average distance to neighbours
        inputs[1]   = min_dist;
        // Number of neighbours
        inputs[2]   = messages;
        // Average message
        //inputs[3]   = avg_message;

        // Pheromone
        //inputs[3]   = pheromone;
        // Food
        //inputs[4]   = food;
        // Nest
        //inputs[5]   = nest;
        // Carrying
        //inputs[6]   = carrying ? 1.0 : -1.0;

        
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

        // Output 2 is the message to send, scaled from -1:+1 -> 0-255
        //int m = outputs[2] < -1 ? 0 : outputs[2] > 1 ? 255 : int((outputs[2]+1)*127);
        //msg.data[0] = m;
        //msg.crc     = message_crc(&msg);

        // Pheromone
        //set_pheromone(outputs[2]);
        
        // Generate a fitness rating based on distance travelled on pheromone trail,
        // favouring straight line
        if (d > 0)
            total_trail += (d == 3) ? pheromone : pheromone/100;
        //==================================================

        // Stage only - colour the kilobot body according to average distance
        float col = (150-min_dist)/120;
        //printf("%s %f %d\n", pos->Token(), min_dist, messages);
        pos->SetColor(Color(col));
        
        
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
            snprintf(buf, 1024, "%12s,%12f,%12f,%12f,%12f,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                     pos->GetPose().x, pos->GetPose().y, inputs[1],inputs[2],outputs[0],outputs[1]);
            Evokilo1::log(buf);
        }
    }
    
}

void Evokilo1::finish()
{
    //printf("[stats] %s %8d %8f %8d\n", pos->Token(), total_food, total_trail, total_pickup);
}

std::vector<std::string> split(std::string s)
{
    // Quite frankly, this is inpenetrable, found on stackoverflow, but at least it doesn't require boost
    // Apparently an istream iterator of type string regards whitespace as the separator..
    std::vector<std::string> words;
    std::istringstream iss(s);
    std::copy (std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(words));
    return words;
}

// Stage calls this when the model starts up, once for each model with this controller
extern "C" int Init( ModelPosition* mod, CtrlArgs* args )
{
    printf("Initialisation %s:%s\n", args->worldfile.c_str(), args->cmdline.c_str());
    // tokenize the argument string into words
    std::vector<std::string> words = split(args->worldfile);

    //boost::split( words, args->worldfile, boost::is_any_of(" \t"));
    
    // Parse arguments passed on the command line, this currently only consists of
    // "log <filename>" to log the robot states for later analysis
    std::vector<std::string> cargs = split(args->cmdline);
    std::string logfile = "";
    if (cargs.size() == 2 && cargs[0] == "log")
    {
        logfile = cargs[1];
        printf("Logging state to %s\n", logfile.c_str());
    }

    
    // Create a new controller and pass this to a new master. The master is called
    // by stage during the update phase of simulation. The master subsequently calls
    // the controller as a coroutine, which allows the controller to Detach back to the
    // master during the delay() function and resume running at that point once sufficent
    // time has passed. This allows the single-threaded code that would run on an actual
    // kilobot be used more-or-less unchanged
    
    Kilobot *ctrl = new Evokilo1(mod, words, logfile);
    new KBMaster(mod, (Kilobot*)ctrl, false);
    
    return 0;
}


