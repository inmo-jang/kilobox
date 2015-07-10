// Copyright Simon Jones 2015

#ifndef KILOWORLD_H
#define KILOWORLD_H

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "freeglut/freeglut.h"
#endif

#include "../Framework/Test.h"
#include "../Framework/Render.h"

#include "kilolib.h"
#include "worldfile.h"


namespace Kilolib
{

    class Kiloworld : public Test
    {
    public:
        Kiloworld(Settings *_settings)
        :
            settings(_settings),
            xsize(3.0),
            ysize(2.0),
            xgrid(1),
            ygrid(1),
            gridmargin(0.2)
        {
            // Turn off gravity
            m_world->SetGravity(b2Vec2(0,0));
            
            // Initialise the random number generator
            gen.seed(1);
            
            // Construct the world
            build_world();

            // Tell the engine that we have a contact callback
            m_world->SetContactListener(&contact_listener);
        }
        
        void Step(Settings* settings);
        
        void build_world();
        void parse_worldfile(float xoffset, float yoffset);
        void make_static_box(float xsize, float ysize, float xpos, float ypos);
        void render_arena();
        void make_kilobot(float xp, float yp, float th);
        
        // Function called by the testbench to create the world
        static Test* Create(Settings *settings)
        {
            return new Kiloworld(settings);
        }

    private:
        // These are the global settings
        Settings *settings;

        float   xsize;
        float   ysize;
        int     xgrid;
        int     ygrid;
        float   gridmargin;

        std::vector<Kilobot*>   bots;
        KBContactListener   contact_listener;

        float   rand(float low, float high)
        {
            std::uniform_real_distribution<float>   dist(low, high);
            return dist(gen);
        }
        
        // Set up a random number generator
        std::default_random_engine              gen;

        // arena
        //b2Body *arena;
        std::vector<b2Fixture *> arena_fixture;

        Worldfile *wf;

        std::vector<std::string> ctrlarg_words;
       
    };


};


#endif
