// Copyright Simon Jones 2015


#include "../Framework/Test.h"
#include "kiloworld.h"
#include <math.h>

int Kilobot::ids = 0;

void Kiloworld::Step(Settings* settings)
{
    //run the default physics and rendering
    Test::Step(settings);
    
    
    for(int i=0; i<bots.size(); i++)
    {
        bots[i]->m_body->ApplyForceToCenter(b2Vec2(rand(-0.1, 0.1), rand(-0.1, 0.1)), true);
    }
    
    //show some text in the main screen
    m_debugDraw.DrawString(5, m_textLine, "Kilobot");
    m_textLine += 15;
}

void Kiloworld::build_world()
{
    // Make arena of fixed lines
    make_static_box(xsize, ysize, 0.0, 0.0);
    
    // Now create kilobots randomly distributed
    for(int i=0;i<1000;i++)
        //make_kilobot(rand(-xsize/2+kbdia,+xsize/2-kbdia), rand(-ysize/2+kbdia,+ysize/2-kbdia), rand(-M_PI, M_PI));
        bots.push_back(new Kilobot(m_world,
                                   rand(-xsize/2+0.03,+xsize/2-0.03),
                                   rand(-ysize/2+0.03,+ysize/2-0.03),
                                   rand(-M_PI, M_PI)));
}



void Kiloworld::make_static_box(float xs, float ys, float xp, float yp)
{
    // Create the body
    b2BodyDef   arenadef;
    b2Body      *arena = m_world->CreateBody(&arenadef);
    
    // Create the shape of the fixture
    b2Vec2 vs[4];
    vs[0].Set(xp-xs/2, yp-ys/2);
    vs[1].Set(xp+xs/2, yp-ys/2);
    vs[2].Set(xp+xs/2, yp+ys/2);
    vs[3].Set(xp-xs/2, yp+ys/2);
    b2ChainShape perimeter;
    perimeter.CreateLoop(vs, 4);
    
    // Create the fixture attached to the arena body
    // The fixture is created directly from the shape because
    // we are not altering the default properties of the created
    // fixture
    arena->CreateFixture(&perimeter, 0);
}


void Kilobot::make_kilobot(float xp, float yp, float th)
{
    // Create the body
    b2BodyDef   kbdef;
    kbdef.type              = b2_dynamicBody;
    kbdef.position.Set(xp, yp);
    kbdef.angle             = th;
    kbdef.linearDamping     = kblineardamp;
    kbdef.angularDamping    = kbangulardamp;
    m_body                  = m_world->CreateBody(&kbdef);
    
    b2CircleShape c;
    b2FixtureDef kfdef;
    // Create the shape
    c.m_radius = kbdia/2.0;
    // Create the fixture for the body
    kfdef.shape             = &c;
    kfdef.density           = kbdensity;
    kfdef.friction          = kbfriction;
    kfdef.restitution       = kbrestitution;
    kfdef.filter.categoryBits   = KILOBOT;
    kfdef.filter.maskBits       = KILOBOT | MESSAGE;
    m_body->CreateFixture(&kfdef);
    
    // Now do the fixture for the sensor
    c.m_radius              = kbsenserad;
    kfdef.shape             = &c;
    kfdef.isSensor          = true;
    kfdef.density           = 0;
    kfdef.friction          = 0;
    kfdef.restitution       = 0;
    kfdef.filter.categoryBits   = MESSAGE;
    kfdef.filter.maskBits       = KILOBOT;
    m_body->CreateFixture(&kfdef);
}