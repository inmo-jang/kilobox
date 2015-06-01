// Copyright Simon Jones 2015


#include "../Framework/Test.h"
#include "kiloworld.h"
#include <math.h>

int Kilobot::ids = 0;

void Kiloworld::Step(Settings* settings)
{
    //run the default physics and rendering
    Test::Step(settings);
    
    // Jiggle and render
    for(int i=0; i<bots.size(); i++)
    {
        bots[i]->m_body->ApplyForceToCenter(b2Vec2(rand(-0.1, 0.1), rand(-0.1, 0.1)), true);

        bots[i]->render();
    }
    
    // 


    // Show some text in the main screen
    m_debugDraw.DrawString(5, m_textLine, "Kilobot");
    m_textLine += 15;
}

void Kiloworld::build_world()
{
    // Make arena of fixed lines
    make_static_box(xsize, ysize, 0.0, 0.0);
    
    // Now create kilobots randomly distributed
    for(int i=0;i<100;i++)
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
        printf("  id:%5d x:%8.4f y:%8.4f\n", inrange_bots[i]->kb_id, theirpos.x, theirpos.y);
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

