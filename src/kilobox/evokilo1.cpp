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
#include <cmath>


#include "kilolib.h"
#include "evokilo1.h"
//global variable:
bool delay_term;
extern bool AOI_reached;
extern bool all_out;
extern float offc_x;
extern float offc_y;


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
FILE *Disperse_magenta::lfp = NULL;
FILE *left_right::lfp	    = NULL;
FILE *Iterative_deep::lfp   = NULL;
FILE *forward_right::lfp    = NULL;
FILE *forward_right_left::lfp    = NULL;
FILE *forward_right_correct::lfp = NULL;
FILE *rand_static::lfp 		 = NULL;
//-------------------------------------------------------------
void Minimal_example::setup()
{
    last_update     = kilo_ticks;
    last_update2    = kilo_ticks;
	last_update3    = kilo_ticks;
	count = 0;
	wait_time = kilo_ticks;
	wait = 1;
	AOI_reached = 0;
}
void Minimal_example::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		delay_term = 1;
		wait = 1;
	}
    if(kilo_ticks > wait_time + 160){
	wait_time = kilo_ticks;
	wait = 0;
	
	}
	if(wait == 1){

	//do nothing
	
	}
else{
    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
	region = get_environment();

	if(region == MAGEN){
		//change colour of kilobot to red
		set_color(RGB(0,3,0));
			}
	else if(region == NON){
		//change colour of kilobot to black
		set_color(RGB(3,0,0));
		
			}
    }			
    if (kilo_ticks > last_update2 + 16){
	
	last_update2 = kilo_ticks;
	//region = get_environment();

	if(region == MAGEN){	
		count = 0;
		//random movement by kilobots
		//choose out of 0,1,2 and assign to variable e.
		
		rand_index = rand_soft() % 3;
		e = my_array[rand_index];
		
	}
	else if(region == NON && count < 1){
		//stationary
		count = count + 1;
	}
	else{
		e = 3;
	}
	
	if(e == 0){
		spinup_motors();		
		set_motors(kilo_straight_left,kilo_straight_right);
	}
	else if(e == 1){
		spinup_motors();
		set_motors(kilo_turn_left,0);
	}
	else if(e == 2){
		spinup_motors();
		set_motors(0,kilo_turn_right);
	}
	else{
		set_motors(0,0);	
	}
        }
}		
//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6 && AOI_reached ==1)
            {
                last_time += 1e6;
                char buf[1024];
                snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                         pos->GetPose().x, pos->GetPose().y);
               	Minimal_example::log(buf);
		
            }
        }
        //-----------------------------------------------------
    	
}
void rand_static::setup()
{
    last_update     = kilo_ticks;
    last_update2    = kilo_ticks;
    count = 0;
    all_out = 0;	
    
}



void rand_static::loop()
{
    

    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
	region = get_environment();

	if(region == MAGEN){
		//change colour of kilobot to red
		set_color(RGB(0,3,0));
			}
	else if(region == NON){
		//change colour of kilobot to black
		set_color(RGB(3,0,0));
		
			}
    }			
    if (kilo_ticks > last_update2 + 16){
	
	last_update2 = kilo_ticks;
	//region = get_environment();
	
	if(region == MAGEN){	
		count = 0;
		//random movement by kilobots
		//choose out of 0,1,2 and assign to variable e.
		
		rand_index = rand_soft() % 3;
		e = my_array[rand_index];
		
	}
	else if(region == NON && count < 1){
		//stationary
		count = count + 1;
	}
	else{
		e = 3;
	}
	
	if(e == 0 ){
		spinup_motors();		
		set_motors(kilo_straight_left,kilo_straight_right);
	}
	else if(e == 1){
		spinup_motors();
		set_motors(kilo_turn_left,0);
	}
	else if(e == 2){
		spinup_motors();
		set_motors(0,kilo_turn_right);
	}
	else{
		set_motors(0,0);	
	}
        }
	//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6)
            {
                last_time += 1e6;
                char buf[1024];
                snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                         pos->GetPose().x, pos->GetPose().y);
               	rand_static::log(buf);
            }
        }
        //-----------------------------------------------------

    	
}

void left_right::setup()
{
    last_update     = kilo_ticks;
    last_update2    = kilo_ticks;
	last_update3    = kilo_ticks;
	count = 0;
}
void left_right::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		//delay_term = 1;
	}
    

    if (kilo_ticks > last_update + 16)
    {
        last_update = kilo_ticks;
	region = get_environment();

	if(region == MAGEN){
		//change colour of kilobot to red
		set_color(RGB(0,3,0));
			}
	else if(region == NON){
		//change colour of kilobot to black
		set_color(RGB(3,0,0));
		
			}
    }			
    if (kilo_ticks > last_update2 + 16){
	
	last_update2 = kilo_ticks;
	//region = get_environment();

	if(region == MAGEN){	
		
		set_motors(kilo_straight_left,0);
	}
	
	else{
		set_motors(0,kilo_straight_right);	
	}
        }
		
//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6)
            {
                last_time += 1e6;
                char buf[1024];
                //snprintf(buf, 1024, "%12s,%12f,%12f,%12f, %12f,%12f\n", pos->Token(), time/1e6,
                         //pos->GetPose().x, pos->GetPose().y, offc_x, offc_y);
               	Minimal_example::log(buf);
            }
        }
        //-----------------------------------------------------
    	
}
//define minimal example as problem for random walk in magenta, stationary in none.

void Iterative_deep::setup()
{
    last_update     = kilo_ticks;
	update	    = kilo_ticks;
    last_update2    = kilo_ticks;
    last_update3    = kilo_ticks;
wait_time = kilo_ticks;
    count = 0;
    
}
void Iterative_deep::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		//delay_term = 1;
	}
    
    region = get_environment();

 //   if (kilo_ticks > last_update + 16)
 //  {
  //      last_update = kilo_ticks;
	

//	if(region == MAGEN){
		//change colour of kilobot to red
//		set_color(RGB(0,3,0));
//			}
//	else if(region == NON){
		//change colour of kilobot to black
//		set_color(RGB(3,0,0));
		
//			}
    //}	
	if(kilo_ticks > wait_time + 160){
	wait_time = kilo_ticks;
	if(region == MAGEN){
		
		if(go_left == 1){
			set_motors(0,kilo_turn_right);
			set_color(RGB(3,0,0));
			
		}
        	else{
			set_motors(kilo_turn_left,kilo_turn_right);
			set_color(RGB(0,0,3));
		}
		
		
    		if (kilo_ticks > update + a*16){
			
			update = kilo_ticks;
			if(go_left == 1){
				a = a + b;
				go_left = 0;
			}
			else{
				a = 2;
				go_left = 1;
			}
			b = b+1;
		}

	}
	else if(region == NON && count < 10){
		//stationary
		count = count + 1;
	}
	else{
		set_motors(0,0);
	}
	
	
	//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6)
            {
                last_time += 1e6;
                char buf[1024];
                //snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                  //       pos->GetPose().x, pos->GetPose().y);
                Minimal_example::log(buf);
            }
        }
        //-----------------------------------------------------

    }	
}
void forward_right::setup()
{
    last_update     = kilo_ticks;
	update	    = kilo_ticks;
    last_update2    = kilo_ticks;
    last_update3    = kilo_ticks;
	wait_time = kilo_ticks;
    count = 0;
	wait = 0;
	
    
}
void forward_right::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		//delay_term = 1;
		go_left = 0;
		
		
	}
    
    region = get_environment();

	if(kilo_ticks > wait_time + 160){
		wait_time = kilo_ticks;
		
	
	
	
	if(region == MAGEN){
		//count = 0;
		if(go_left == 1){
			set_motors(0,kilo_turn_right);
			set_color(RGB(3,0,0));
			
		}
        	else{
			set_motors(kilo_straight_left,kilo_straight_right);
			set_color(RGB(0,0,3));
		}
		
		
		
    		if (kilo_ticks > update + a*16){
			
			update = kilo_ticks;
			if(go_left == 1){
				a = 8;
				go_left = 0;
			}
			else{
				a = 1;
				go_left = 1;
			}
			
		}

	}
	else if(region == NON && count < 10){
		//stationary
		count = count + 1;
		//set_motors(kilo_turn_left,0);
		//set_color(RGB(0,3,0));
	}
	else{
		set_motors(0,0);
	}
	
	
	//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6)
            {
                last_time += 1e6;
                char buf[1024];
                snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                         pos->GetPose().x, pos->GetPose().y);
                Minimal_example::log(buf);
            }
        }
        //-----------------------------------------------------
}
    	
}
void forward_right_correct::setup()
{
    last_update     = kilo_ticks;
	update	    = kilo_ticks;
    last_update2    = kilo_ticks;
    last_update3    = kilo_ticks;
	wait_time = kilo_ticks;
    count = 0;
	wait = 0;
	
    
}
void forward_right_correct::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		//delay_term = 1;
		go_left = 0;
		wait = 1;	
	}
    
    region = get_environment();
	
		if(kilo_ticks > wait_time + 160){
			wait_time = kilo_ticks;
			wait = 0;
	
	
		}
		if(wait == 1){


		//do nothing

		}
		else{
		if(region == MAGEN){
			//count = 0;
			if(go_left == 1){
				set_motors(0,kilo_turn_right);
				set_color(RGB(3,0,0));
			
			}
			else{
				set_motors(kilo_straight_left,kilo_straight_right);
				set_color(RGB(0,0,3));
			}
		
		
		
	    		if (kilo_ticks > update + a*16){
			
				update = kilo_ticks;
				if(go_left == 1){
					a = 8;
					go_left = 0;
				}
				else{
					a = 1;
					go_left = 1;
				}
			
			}

		}
		else if(region == NON && count < 10){
			//stationary
			count = count + 1;
			//set_motors(kilo_turn_left,0);
			//set_color(RGB(0,3,0));
		}
		else{
			set_motors(0,0);
		}
	
	
		//-----------------------------------------------------
		// Logging example
		{
		    usec_t time = pos->GetWorld()->SimTimeNow();
		    if (time - last_time >= 1e6)
		    {
		        last_time += 1e6;
		        char buf[1024];
		        snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
		                 pos->GetPose().x, pos->GetPose().y);
		        Minimal_example::log(buf);
		    }
		}
		//-----------------------------------------------------
	}
    	
}




void ring::setup()
{
    last_update     = kilo_ticks;
	update	    = kilo_ticks;
    last_update2    = kilo_ticks;
    last_update3    = kilo_ticks;
	wait_time = kilo_ticks;
    count = 0;
	wait = 0;
	
    
}
#define BLUE 1
void ring::loop()
{
	if(kilo_ticks > last_update3 + 320){
		last_update3 = kilo_ticks;
		//delay_term = 1;
		go_left = 0;
		wait = 1;	
	}
    
    region = get_environment();
	
		if(kilo_ticks > wait_time + 160){
			wait_time = kilo_ticks;
			wait = 0;
	
	
		}
		if(wait == 1){


		//do nothing

		}
		else{
		if(region == BLUE){

		
			if(go_right == 1){
				set_motors(0,kilo_turn_right);
				set_color(RGB(3,0,0));
			
			}
			else{
				set_motors(kilo_straight_left,kilo_straight_right);
				set_color(RGB(0,0,3));
			}
		
		
		
	    		if (kilo_ticks > update + a*16){
			
				update = kilo_ticks;
				if(go_left == 1){
					a = 2;
					go_left = 0;
					go_right = 0;
				}
				else if(go_right == 1){
					a = 1;
					go_left = 0;
					go_right = 0;
				}
				else{

					a = 3;
					
					go_right = 0;
				}
			
			}




		}
		else if(region == MAGEN){
			//count = 0;
			if(go_left == 1){
				set_motors(kilo_turn_left,0);
				set_color(RGB(3,0,0));
			
			}
			else{
				set_motors(kilo_straight_left,kilo_straight_right);
				set_color(RGB(0,0,3));
			}
		
		
		
	    		if (kilo_ticks > update + a*16){
			
				update = kilo_ticks;
				if(go_left == 1){
					a = 8;
					go_left = 0;
				}
				else{
					a = 1;
					go_left = 1;
				}
			
			}

		}
		else if(region == NON && count < 10){
			//stationary
			count = count + 1;
			//set_motors(kilo_turn_left,0);
			//set_color(RGB(0,3,0));
		}
		else{
			set_motors(0,0);
		}
	
	}
		//-----------------------------------------------------
		// Logging example
		{
		    usec_t time = pos->GetWorld()->SimTimeNow();
		    if (time - last_time >= 1e6)
		    {
		        last_time += 1e6;
		        char buf[1024];
		        snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
		                 pos->GetPose().x, pos->GetPose().y);
		        Minimal_example::log(buf);
		    }
		}
		//-----------------------------------------------------
	
    	
}




void forward_right_left::setup()
{
    last_update     = kilo_ticks;
	update	    = kilo_ticks;
    last_update2    = kilo_ticks;
    last_update3    = kilo_ticks;
    count = 0;
	wait_time = kilo_ticks;
    
}
void forward_right_left::loop()
{
	if(kilo_ticks > last_update3 + 10000){
		last_update3 = kilo_ticks;
		//delay_term = 1;
		//go_right = 1;
		//go_left = 0;
		//go_forward = 0;
	}
    
    region = get_environment();

 //   if (kilo_ticks > last_update + 16)
 //  {
  //      last_update = kilo_ticks;
	

//	if(region == MAGEN){
		//change colour of kilobot to red
//		set_color(RGB(0,3,0));
//			}
//	else if(region == NON){
		//change colour of kilobot to black
//		set_color(RGB(3,0,0));
		
//			}
    //}	
	if(kilo_ticks > wait_time + 160){
	wait_time = kilo_ticks;
	if(region == MAGEN){
		
		if(go_right == 1){
			set_motors(0,kilo_turn_right);
			set_color(RGB(3,0,0));
			
		}
        	else if(go_forward == 1){
			set_motors(kilo_turn_left,kilo_turn_right);
			set_color(RGB(0,0,3));
		}
		else if(go_left == 1){
			set_motors(kilo_turn_left,0);
			set_color(RGB(0,3,0));
		}
		
		
    		if (kilo_ticks > update + a*32){
			
			update = kilo_ticks;
			if(go_forward == 1){
				a = 3;
				go_forward = 0;
				go_left = 1;
				
			}
			else if(go_left == 1){
				a = 3;
				go_left = 0;
				go_right = 1;
			}
			else if(go_right == 1){	
				a = 3;
				go_right = 0;
				go_forward = 1;
			
			}
			
		}

	}
	else if(region == NON && count < 10){
		//stationary
		count = count + 1;
		set_motors(kilo_straight_left,kilo_straight_right);

	}
	else{
		set_motors(0,0);
	}
	
	
	//-----------------------------------------------------
        // Logging example
        {
            usec_t time = pos->GetWorld()->SimTimeNow();
            if (time - last_time >= 1e6)
            {
                last_time += 1e6;
                char buf[1024];
                snprintf(buf, 1024, "%12s,%12f,%12f,%12f\n", pos->Token(), time/1e6,
                         pos->GetPose().x, pos->GetPose().y);
                Minimal_example::log(buf);
            }
        }
        //-----------------------------------------------------
}
    	
}
//-------------------------------------------------------------
void Disperse_magenta::setup()
{

    
  
	last_update = kilo_ticks;
	last_update2 = kilo_ticks;
	kilo_message_tx     = (message_tx_t)&Disperse_magenta::message_tx;
    	kilo_message_rx     = (message_rx_t)&Disperse_magenta::message_rx;
	kilo_message_tx_success = (message_tx_success_t)&Disperse_magenta::message_tx_success;
	
}

void Disperse_magenta::loop()
{	
	int region;
	int speed;
	if(kilo_ticks > last_update2 + 10000){
		last_update2 = kilo_ticks;
		//delay_term = 1;
	}
	if(kilo_ticks > last_update + 32){
		
		last_update = kilo_ticks;
		rand_num = rand_hard();
		dice = (rand_num % 3);
		
		region = get_environment();
		if(region == MAGEN && new_message == 1){
			new_message = 0;
			speed = 200;
			set_color(RGB(3,0,0));
		}
		else if(region == MAGEN && new_message == 0){
			speed = 100;
			set_color(RGB(0,3,0));
		}
		else if(region == NON && new_message == 1){
			new_message = 0;			
			speed = 70;
			set_color(RGB(0,0,3));
			
		}
		else if(region == NON && new_message == 0){
			speed = 0;
			set_color(RGB(3,3,0));
		}
		
		if(dice == 0){		//go foward
			spinup_motors();		
			set_motors(speed,speed);
		}
		else if(dice == 1){			//go left
			spinup_motors();
			set_motors(speed,0);	
		}
		else{			//go right
			spinup_motors();
			set_motors(0,speed);	
		}
		
	}

	
}


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



//void Kiloworld::update_regions_c()
//{
    // Set region 0 position to centre of mass of robots
//    float x = 0;
//    float y = 0;
//    for (int i = 0; i < bots.size(); i++)
//    {
//        x += bots[i]->pos->pose.x;
//        y += bots[i]->pos->pose.y;
//    }
//    x /= bots.size();
//    y /= bots.size();
//    if (regions.size() > 0)
//    {
//        regions[0]->set_position(x, y);
//    }
//}


