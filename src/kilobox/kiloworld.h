// Copyright Simon Jones 2015

#ifndef KILOWORLD_H
#define KILOWORLD_H

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "freeglut/freeglut.h"
#endif

#include "Testbed/Framework/Test.h"
#include "Testbed/Framework/Render.h"

//#include "kilolib.h"
#include "worldfile.h"
#include <random>


namespace Kilolib
{
    class Kilobot;
    // Subclass the built in contact class
    class KBContactListener : public b2ContactListener
    {
        void BeginContact(b2Contact *contact);
        void EndContact(b2Contact *contact);
    };
    class Region
    {
    public:
        Region(float _x, float _y, int _rt)
        :   x(_x),
            y(_y),
            rt(_rt)
        {}
        virtual void render() = 0;
        virtual int read_region(float xp, float yp, bool thresh=true) = 0;
        virtual void update(float dt) = 0;
        virtual void set_pheromone(float xp, float yp, float a) {}
        void set_position(float xp, float yp) {x = xp; y = yp;}
    protected:
        float x;
        float y;
        int rt;
    };

    class Circle : public Region
    {
    public:
        Circle(float _x, float _y, float _r, int _rt)
        :   Region(_x, _y, _rt),
            r(_r)
        {}
        void render();
        void update(float dt) {}
        int read_region(float xp, float yp, bool thresh);
    protected:
        float r;
    };

    class Rectangle : public Region
    {
    public:
        Rectangle(float _x, float _y, float _xs, float _ys, int _rt)
        :   Region(_x, _y, _rt),
        xs(_xs),
        ys(_ys)
        {}
        void render();
        void update(float dt) {}
        int read_region(float xp, float yp, bool thresh);
    protected:
        float xs;
        float ys;
    };
    
    class Stigmergy : public Region
    {
    public:
        Stigmergy(float _xs, float _ys, float _decay, float _disp, float _radius, float _rate, Settings *_s)
        :   Region(0, 0, 0),
        xsize       (_xs),
        ysize       (_ys),
        decay       (_decay),
        displacement(_disp),
        radius      (_radius),
        rate        (_rate),
        resolution  (200),
        s           (_s)
        {
            xres    = xsize * resolution;
            yres    = ysize * resolution;
            data.resize(xres * yres, 0);
            printf("Creating stigmergy region xs:%f ys:%f decay:%f disp:%f rad:%f rate:%f\n",
                   xsize, ysize, decay, displacement, radius, rate);
        }
        void render();
        void update(float dt);
        int read_region(float xp, float yp, bool thresh);
        
        
    protected:
        float xsize, ysize, decay, displacement, radius, rate, resolution;
        Settings *s;
    private:
        int                 xres, yres;
        float               internal_dt = 0;
        std::vector<float>  data;
        float               get_data(int x, int y) {return data.data()[y * xres + x];}
        void                set_pheromone(float xp, float yp, float a);
    };
    

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
            gridmargin(0.2),
            simtime(0.0),
            steps(0)
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
        void Finish(Settings* settings);
        
        void build_world();
        void parse_worldfile(float xoffset, float yoffset);
        void make_static_box(float xs, float ys, float xp, float yp);
        void make_static_polygon(float radius, int sides, float xpos, float ypos);
        void make_static_fence(float x1, float y1, float x2, float y2);
        void render_arena();
        void make_kilobot(float xp, float yp, float th);
        
        // Function called by the testbench to create the world
        static Test* Create(Settings *settings)
        {
            return new Kiloworld(settings);
        }

        int get_environment(float x, float y, bool thresh=true);
        void set_pheromone(float xp, float yp, float a);


    private:
        // These are the global settings
        Settings *settings;

        float   xsize;
        float   ysize;
        int     xgrid;
        int     ygrid;
        float   gridmargin;
        float   simtime;
        int     steps;

        std::vector<Kilobot*>   bots;

        // Hold list of regions
        std::vector<Region*>    regions;

        KBContactListener   contact_listener;

        float   rand_intrange(float low, float high)
        {
            std::uniform_real_distribution<float>   dist(low, high);
            return dist(gen);
        }
        
        // Set up a random number generator
        std::mt19937              gen;

        // arena
        //b2Body *arena;
        std::vector<b2Fixture *> arena_fixture;

        Worldfile *wf;

        std::vector<std::string> ctrlarg_words;
        
        void update_regions();
       
    };


};


#endif
