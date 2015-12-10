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

#define W(i,h,o) (((i)+1)*(h)+(h)*(h)+((h)+1)*(o))
#define WF(i,h,o) (((i)+1)*(h)+((h)+1)*(o))


class NN
{
public:
    NN(int _nn_num_inputs, int _nn_num_hidden, int _nn_num_outputs, bool _rec = true)
    :   NN_NUM_INPUTS   (_nn_num_inputs),
        NN_NUM_HIDDEN   (_nn_num_hidden),
        NN_NUM_OUTPUTS  (_nn_num_outputs),
        NN_NUM_WEIGHTS  (_rec   ? W(_nn_num_inputs, _nn_num_hidden, _nn_num_outputs)
                                : WF(_nn_num_inputs, _nn_num_hidden, _nn_num_outputs)),
        nn_hidden       (NN_NUM_HIDDEN, 0.0),
        nn_outputs      (NN_NUM_OUTPUTS),
        nn_weights      (NN_NUM_WEIGHTS),
        rec             (_rec)
    {
        
    }
    int     NN_NUM_INPUTS;
    int     NN_NUM_HIDDEN;
    int     NN_NUM_OUTPUTS;
    int     NN_NUM_WEIGHTS;
    std::vector<float>  nn_hidden;
    std::vector<float>  nn_outputs;
    std::vector<float>  nn_weights;
    bool    rec;
    float *nn_update(float *inputs);
};




class Evokilo1 : public Kilobot
{
public:
    // Foraging kilobot using only local information, no messages
    //
    // Pass the model reference up to the parent constructor
    Evokilo1(ModelPosition *_pos, Settings *_settings,
             std::vector<std::string> _words, std::string _logfile = "") :
    Kilobot (_pos, _settings),
    words   (_words),
    logfile (_logfile),
    // Set the size of the neural net
    nn      (4, 4, 2),
    inputs  (4)
    {
        if (words.size()-1 == nn.NN_NUM_WEIGHTS)
            for(int i=0; i<nn.NN_NUM_WEIGHTS; i++)
                nn.nn_weights[i] = atof(words[i+1].c_str());
        else
        {
            printf("Wrong number of weights in controller arguments, got %lu should be %d\n", words.size()-1, nn.NN_NUM_WEIGHTS);
            exit(1);
        }
        
        if (logfile != "")
        {
            //printf("Logfile is %s\n", logfile.c_str());
            log_open(logfile);
        }
        kilo_message_tx         = (message_tx_t)&Evokilo1::message_tx_dummy;
        kilo_message_rx         = (message_rx_t)&Evokilo1::message_rx_dummy;
        kilo_message_tx_success = (message_tx_success_t)&Evokilo1::message_tx_success_dummy;
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

    // Hold usecs so we can log every second
    int last_time = 0;

    //------------------------------------------------------------
    // Kilobot user functions
    //------------------------------------------------------------
    
    void setup();
    void loop();
    
    NN          nn;
    
    uint32_t    last_update     = 0;
    int         last_region     = 0;
    int         last_output     = 0;

    uint8_t     carrying        = 0;
    int         total_food      = 0;
    int         total_pickup    = 0;
    
    std::vector<float> inputs;
    float       *outputs;

    
    
    
};

class Evokilo2 : public Kilobot
{
public:
    // Foraging robot using message system to produce swarm effects
    //
    //
    // Pass the model reference up to the parent constructor
    Evokilo2(ModelPosition *_pos, Settings *_settings,
             std::vector<std::string> _words, std::string _logfile = "") :
    Kilobot (_pos, _settings),
    words   (_words),
    logfile (_logfile),
    // Set the size of the neural net
    nn      (6, 6, 3),
    inputs  (6)
    {
        if (words.size()-1 == nn.NN_NUM_WEIGHTS)
            for(int i=0; i<nn.NN_NUM_WEIGHTS; i++)
                nn.nn_weights[i] = atof(words[i+1].c_str());
        else
        {
            printf("Wrong number of weights in controller arguments, got %lu should be %d\n", words.size()-1, nn.NN_NUM_WEIGHTS );
            exit(1);
        }
        
        if (logfile != "")
        {
            //printf("Logfile is %s\n", logfile.c_str());
            log_open(logfile);
        }
        kilo_message_tx         = (message_tx_t)&Evokilo2::message_tx_dummy;
        kilo_message_rx         = (message_rx_t)&Evokilo2::message_rx_dummy;
        kilo_message_tx_success = (message_tx_success_t)&Evokilo2::message_tx_success_dummy;
        setup();
    }
    ~Evokilo2()
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

    // Hold usecs so we can log every second
    int last_time = 0;
    
    float metric() {return total_food;}

    //------------------------------------------------------------
    // Kilobot user functions
    //------------------------------------------------------------
    
    void setup();
    void loop();
    
    NN          nn;
     
    uint32_t    last_update     = 0;
    int         last_region     = 0;   
    int         last_output     = 0;

    message_t   msg;
    int         messages        = 0;
    int         min_dist        = 150;
    float       msgsum          = 0;

    uint8_t     carrying        = 0;
    int         total_food      = 0;
    int         total_pickup    = 0;

    std::vector<float> inputs;
    float       *outputs;
    
    
    // Message transmission callback
    message_t *tx_message() 
    {
        return &msg;
    }
    

    void message_rx(message_t *m, distance_measurement_t *d)
    {
        //printf("in message_rx %s\n",__PRETTY_FUNCTION__);
        // Keep running average of message distance
        int dist = estimate_distance(d);
        if (dist < min_dist)
            min_dist = dist;
        float mf;
        memcpy(&mf, m->data, 4);
        msgsum += mf;
        messages ++;
    }
    
    
    
};

class Evokilo3 : public Kilobot
{
public:
    // Foraging robot using message system to produce swarm effects
    //
    //
    // Pass the model reference up to the parent constructor
    Evokilo3(ModelPosition *_pos, Settings *_settings,
             std::vector<std::string> _words, std::string _logfile = "") :
    Kilobot (_pos, _settings),
    words   (_words),
    logfile (_logfile),
    // Set the size of the neural net
    nn      (3, 3, 2, _words[1] == "rec"),
    inputs  (3)
    {

        if (words.size()-2 == nn.NN_NUM_WEIGHTS)
            for(int i=0; i<nn.NN_NUM_WEIGHTS; i++)
                nn.nn_weights[i] = atof(words[i+2].c_str());
        else
        {
            printf("Wrong number of weights in controller arguments, got %lu should be %d\n", words.size()-2, nn.NN_NUM_WEIGHTS );
            exit(1);
        }

        
        if (logfile != "")
        {
            //printf("Logfile is %s\n", logfile.c_str());
            log_open(logfile);
        }
        kilo_message_tx         = (message_tx_t)&Evokilo3::message_tx_dummy;
        kilo_message_rx         = (message_rx_t)&Evokilo3::message_rx_dummy;
        kilo_message_tx_success = (message_tx_success_t)&Evokilo3::message_tx_success_dummy;
        setup();
    }
    ~Evokilo3()
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
    
    // Hold usecs so we can log every second
    int last_time = 0;

    
    //------------------------------------------------------------
    // Kilobot user functions
    //------------------------------------------------------------
    
    void setup();
    void loop();
    
    NN          nn;
    
    uint32_t    last_update     = 0;
    int         last_region     = 0;
    int         last_output     = 0;
    
    message_t   msg;
    int         messages        = 0;
    int         min_dist        = 150;
    
    std::vector<float> inputs;
    float       *outputs;
    
    
    // Message transmission callback
    message_t *tx_message()
    {
        return &msg;
    }
    
    
    void message_rx(message_t *m, distance_measurement_t *d)
    {
        //printf("in message_rx %s\n",__PRETTY_FUNCTION__);
        // Keep running average of message distance
        int dist = estimate_distance(d);
        if (dist < min_dist)
            min_dist = dist;
        messages ++;
    }
    
    
    
};


class Evokilo4 : public Kilobot
{
public:
    // Foraging robot using message system to produce swarm effects
    //
    //
    // Pass the model reference up to the parent constructor
    Evokilo4(ModelPosition *_pos, Settings *_settings,
             std::vector<std::string> _words, std::string _logfile = "") :
    Kilobot (_pos, _settings),
    words   (_words),
    logfile (_logfile),
    // Set the size of the neural net
    nn      (7, 7, 3),
    inputs  (7)
    {
        if (words.size()-1 == nn.NN_NUM_WEIGHTS)
            for(int i=0; i<nn.NN_NUM_WEIGHTS; i++)
                nn.nn_weights[i] = atof(words[i+1].c_str());
        else
        {
            printf("Wrong number of weights in controller arguments, got %lu should be %d\n", words.size()-1, nn.NN_NUM_WEIGHTS );
            exit(1);
        }
        
        if (logfile != "")
        {
            //printf("Logfile is %s\n", logfile.c_str());
            log_open(logfile);
        }
        kilo_message_tx         = (message_tx_t)&Evokilo4::message_tx_dummy;
        kilo_message_rx         = (message_rx_t)&Evokilo4::message_rx_dummy;
        kilo_message_tx_success = (message_tx_success_t)&Evokilo4::message_tx_success_dummy;
        setup();
    }
    ~Evokilo4()
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
    
    // Hold usecs so we can log every second
    usec_t last_time = 0;
    
    float metric() {return total_food;}
    
    //------------------------------------------------------------
    // Kilobot user functions
    //------------------------------------------------------------
    
    void setup();
    void loop();
    
    NN          nn;
    
    uint32_t    last_update     = 0;
    int         last_region     = 0;
    int         last_output     = 0;
    
    message_t   msg;
    int         messages        = 0;
    int         min_dist        = 150;
    float       msgsum          = 0;
    int         mptr            = 0;
#define MMEM 100
    uint8_t     msg_id[MMEM];
    int         new_id          = 0;
    
    uint8_t     carrying        = 0;
    int         total_food      = 0;
    int         total_pickup    = 0;
    
    std::vector<float> inputs;
    float       *outputs;
    
    
    // Message transmission callback
    message_t *tx_message()
    {
        return &msg;
    }
    
    
    void message_rx(message_t *m, distance_measurement_t *d)
    {
        //printf("in message_rx %s\n",__PRETTY_FUNCTION__);
        // Keep running average of message distance
        int dist = estimate_distance(d);
        if (dist < min_dist)
            min_dist = dist;
        msgsum += *(float*)m->data;
        
        // Check memory to see if message in there
        uint8_t mid = m->data[4];
        int found = 0;
        for(int i=0; i<MMEM; i++)
            if (msg_id[i] == mid)
            {
                found = 1;
                break;
            }
        if (!found)
            // Salient, since not in memory
            new_id++;
        msg_id[mptr] = mid;
        mptr = (mptr+1) % MMEM;
        messages ++;
    }
    
    
    
};



#endif

