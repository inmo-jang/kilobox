// Copyright Simon Jones 2015

#ifndef KILOWORLD_H
#define KILOWORLD_H

#include <random>
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "freeglut/freeglut.h"
#endif



#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

// Forward declarations
class Kilobot;

// Subclass the built in contact class
class KBContactListener : public b2ContactListener
{
    void BeginContact(b2Contact *contact);
    void EndContact(b2Contact *contact);
};


enum entityCategory
{
    KILOBOT         = 0x0001,
    MESSAGE         = 0x0002,
};

class Kiloworld : public Test
{
public:
    Kiloworld()
    :
        xsize(3.0),
        ysize(2.0)
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
    void make_static_box(float xsize, float ysize, float xpos, float ypos);
    void make_kilobot(float xp, float yp, float th);
    
    static Test* Create()
    {
        return new Kiloworld;
    }
private:
    float   xsize;
    float   ysize;

    
    std::vector<Kilobot*>   bots;
    
    KBContactListener   contact_listener;

    float   rand(float low, float high)
    {
        std::uniform_real_distribution<float>   dist(low, high);
        return dist(gen);
    }
    
    // Set up a random number generator
    std::default_random_engine              gen;
   
};

class Kilobot
{
public:
    static int ids;
    Kilobot(b2World *_m_world, float xp, float yp, float th)
    :
        m_world         (_m_world),
        kbdia           (0.031),
        kbdensity       (10.0),
        kblineardamp    (2.0),
        kbangulardamp   (2.0),
        kbfriction      (1.0),
        kbrestitution   (1.0),
        kbsenserad      (0.1)
    {
        // Generate a unique ID, pre-increment so that first ID is 1
        kb_id = ++ids;
        make_kilobot(xp, yp, th);
    }
    b2Body  *m_body;
    b2World *m_world;
    int     kb_id;


    // Functions to maintain the list of other bots we are in range of
    void acquired(Kilobot *r)
    {
        printf("adding   %d\n", r->kb_id);
        inrange_bots.push_back(r);
    }
    void lost(Kilobot *r)
    {
        printf("removing %d\n", r->kb_id);
        inrange_bots.erase(std::find(inrange_bots.begin(), inrange_bots.end(), r));
    }

    void render();
    
private:
    void    make_kilobot(float xp, float yp, float th);
    
    float   kbdia;
    float   kbdensity;
    float   kblineardamp;
    float   kbangulardamp;
    float   kbfriction;
    float   kbrestitution;
    float   kbsenserad;
    
    std::vector<Kilobot*>   inrange_bots;

};



#endif
