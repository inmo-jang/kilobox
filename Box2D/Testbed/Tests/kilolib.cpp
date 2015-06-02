// Copyright Simon Jones 2015


#include "kilolib.h"
#include <math.h>

using namespace Kilolib;

int Kilobot::ids = 0;




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

    // Create a reference to here in the physics world
    m_body->SetUserData(this);
    
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

    // Instantiate a controller
    
}

void Kilobot::check_messages()
{
    
}


void Kilobot::update(float delta_t)
{
    dt = delta_t;
    // Update the kilobot tick counter
    kilo_ticks += dt * 1e6 / master_tick_period;

    // Handle message system
    check_messages();
    printf("%s dt:%8d kilo_ticks:%8d\n", __PRETTY_FUNCTION__, dt, kilo_ticks);

    // Leave pheromone trace in environment

    // Do the physics, this always happens every tick, regardless of loop schelduling
    // This consists of working out what forces to apply to get our desired goal velocities
    // and then applying them

}



void Kilobot::render()
{
    b2Vec2 mypos = m_body->GetPosition();
    printf("##id:%5d x:%8.4f y:%8.4f\n", kb_id, mypos.x, mypos.y);
    glColor3f(1,1,1);//white
    //glLineStipple( 1, 0xF0F0 ); //evenly dashed line
    //glEnable(GL_LINE_STIPPLE);
    glBegin(GL_LINES);
    for (int i = 0; i < inrange_bots.size(); i++) {
        b2Vec2 theirpos = inrange_bots[i]->m_body->GetPosition();
        //printf("  id:%5d x:%8.4f y:%8.4f\n", inrange_bots[i]->kb_id, theirpos.x, theirpos.y);
        glVertex2f(mypos.x, mypos.y);
        glVertex2f(theirpos.x, theirpos.y);
    }
    glEnd();
    //glDisable(GL_LINE_STIPPLE);
}

bool get_contact(b2Contact *contact, Kilobot *&sender, Kilobot *&receiver)
{
    b2Fixture *fa = contact->GetFixtureA();
    b2Fixture *fb = contact->GetFixtureB();
    bool sa = fa->IsSensor();
    bool sb = fb->IsSensor();
    if (!(sa ^ sb))
        return false;
    Kilobot *ka = static_cast<Kilobot*>(fa->GetBody()->GetUserData());
    Kilobot *kb = static_cast<Kilobot*>(fb->GetBody()->GetUserData());
    if (!ka || !kb)
        // If the user data is null, the objct is not a kilobot
        return false;
    if (sa)
    {
        sender      = ka;
        receiver    = kb;
    }
    else
    {
        sender      = kb;
        receiver    = ka;
    }
    return true;
}

void KBContactListener::BeginContact(b2Contact *contact)
{
    Kilobot *sender;
    Kilobot *receiver;
    if (get_contact(contact, sender, receiver))
        sender->acquired(receiver);
}

void KBContactListener::EndContact(b2Contact *contact)
{
    Kilobot *sender;
    Kilobot *receiver;
    if (get_contact(contact, sender, receiver))
        sender->lost(receiver);
}

