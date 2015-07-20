//----------------------------------------------------------
// Evokilo1 - First experiment in evolved kilobot controller
// (c) Simon Jones 2015
//----------------------------------------------------------

#ifndef EVOKILO1_H
#define EVOKILO1_H

#include "kilolib.h"
#include <vector>
#include <cstdlib>

using namespace Kilolib;

struct Neuron
{
    Neuron(int _num_inputs) : num_inputs(_num_inputs)
    {
        // Every neuron has weighted inputs and a bias
        for(int i=0; i<num_inputs; i++)
            weights.push_back(0);
        value = 0;
    }
    int num_inputs;
    std::vector<float> weights;
    float value;
};

struct Neuron_layer
{
    Neuron_layer(int _num_neurons, int num_inputs) : num_neurons(_num_neurons)
    {
        for(int i=0; i<num_neurons; i++)
            neurons.push_back(Neuron(num_inputs));
    }
    int num_neurons;
    std::vector<Neuron> neurons;
};

class Neural_net
{
public:
    // Fully recurrent neural net.
    Neural_net(int _input_nodes, int _hidden_nodes, int _output_nodes, bool _recurrent) :
    input_nodes     (_input_nodes),
    hidden_nodes    (_hidden_nodes),
    output_nodes    (_output_nodes),
    hidden          (Neuron_layer(hidden_nodes,
                        _recurrent ? hidden_nodes + input_nodes : input_nodes)),
    output          (Neuron_layer(output_nodes, hidden_nodes)),
    recurrent       (_recurrent)
    {
        int ni  = hidden.neurons[0].num_inputs;
        int nn  = hidden.num_neurons;
        num_weights = hidden_nodes * ni + output_nodes * nn;
    }
    int input_nodes;
    int hidden_nodes;
    int output_nodes;
    int num_weights;
    Neuron_layer hidden;
    Neuron_layer output;
    bool recurrent;
    
    std::vector<float> update(std::vector<float> inputs);
    float sigmoid(float x) {return tanh(x);}
    
    void set_weights(std::vector<float> w);
    std::vector<float> get_weights();
    
};



class Evokilo1 : public Kilobot
{
public:
    // Pass the model reference up to the parent constructor
    Evokilo1(ModelPosition *_pos, Settings *_settings,
            std::vector<std::string> _words, std::string _logfile = "") :
    Kilobot (_pos, _settings),
    words   (_words),
    logfile (_logfile),
    // Set the size of the neural net
    nn      (5, 5, 2, true),
    inputs  (5, 0),
    outputs (2)
    {
        std::vector<float> w;
        
        if (words.size()-1 == nn.num_weights)
            for(int i=0; i<nn.num_weights; i++)
                w.push_back(atof(words[i+1].c_str()));
        
        else
        {
            printf("Wrong number of weights in controller arguments, got %lu should be %d\n", words.size()-1, nn.num_weights );
            exit(1);
        }
        nn.set_weights(w);
        
        std::vector<float> x = nn.get_weights();
//        printf("%s:%d:",pos->Token(), pos->GetId());
//        for(int i=0; i<x.size(); i++)
//            printf("%f ", x[i]);
//        printf("\n");
        
        if (logfile != "")
        {
            //printf("Logfile is %s\n", logfile.c_str());
            log_open(logfile);
        }
        
        last_time = 0;
        setup();
    }
    ~Evokilo1()
    {
        if (lfp)
            log_close();
    }
    // Class methods to handle log file
    static FILE *lfp;
    static void log_open(std::string fname)
    {
        if (!lfp)
        {
        printf("Opening log file %s\n", fname.c_str());
            lfp = fopen(fname.c_str(),"w");
        }
    }
    static void log(char *s)
    {
        if (lfp)
            fputs(s, lfp);
    }
    static void log_close()
    {
        fclose(lfp);
        lfp = NULL;
    }
    
    //void finish();
    std::vector<std::string> words;
    std::string logfile;
    
    //------------------------------------------------------------
    // Kilobot user functions
    //------------------------------------------------------------
    
    void setup();
    void loop();

    Neural_net nn;
    
    message_t   msg;
    uint8_t     sent_message;
    int         messages;
    uint8_t     send_message;
    float       dist;
    uint8_t     message;
    uint8_t     carrying;
    int         total_food;
    float       total_trail;
    int         total_pickup;
    usec_t      last_time;
    float       avg_dist;
    float       avg_message;
    float       min_dist;
    uint32_t    last_update;

    float       nest;
    float       food;
    float       pheromone;
    std::vector<float> inputs;
    std::vector<float> outputs;

    
    // Message transmission callback
    message_t *tx_message()
    {
        if (send_message)
            return &msg;
        else
            return 0;
    }
    
    // Successful transmission callback
    void tx_message_success()
    {
        sent_message = 1;
    }
    
    void message_rx(message_t *m, distance_measurement_t *d)
    {
        //printf("in message_rx %s\n",__PRETTY_FUNCTION__);
        // Keep running average of message distance
        avg_dist    *= messages;
        avg_message *= messages;
        int dist = estimate_distance(d);
        if (dist < min_dist)
            min_dist = dist;
        avg_dist += dist;
        float msg = m->data[0];
        avg_message += (msg-128)/128;
        messages ++;
        avg_dist    /= messages;
        avg_message /= messages;
        //printf("%s got message at %d %d %f\n", pos->Token(), dist, messages, avg_dist);
    }
    
    
    
};

#endif

