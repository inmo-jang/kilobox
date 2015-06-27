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
    //kbdef.linearDamping     = kblineardamp;
    //kbdef.angularDamping    = kbangulardamp;
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

void Kilobot::check_messages()
{
    // Messages are sent by placing them in a message queue owned by the recipient
    message_t *msg = 0;
    // Now see if its time to try sending a new message
    uint32_t t = pos->GetWorld()->SimTimeNow();
    if (((int)t - (int)last_message) > (message_period * 1e6))
    {
        //if (kilo_uid == 1)
        //    printf("time:%i last:%i period:%f\n", t, last_message, message_period);
        // Time to try and send a message if there is one. First, update the
        // message attempt timestamp with a bit of randomness of 50ms
        // FIXME!! SJ we really don't know how this will affect sim fidelity
        // and perhaps this should be one of the parameters to evolve
        //last_message = t + rand(0, 200000);
        last_message = t;
        // Call the callback function to see if there is anything there
        msg = (*this.*kilo_message_tx)();
        //printf("%s sending at %u\n", pos->Token(), t);
    
    
        // Check the set of inrange kilobots and copy the message into the
        // target message queue
        if (msg)
        {
            // Signal successful message transmission. This is no guarantee of
            // reception, it just means (in the hardware) that no collision was
            // detected in the first ~250us
            (*this.*kilo_message_tx_success)();
            int s = inrange_bots.size();
            int r = 0;//rand(0, s);
            for(int i=0; i<s; i++)
            {
                int j = (i + r) % s;
                Kilobot *bot = inrange_bots[j];
                // Get a pointer to the recipients message queue
                m_queue_t &meq = bot->message_queue;
                // Calculate distance between centres in mm
                Pose me = pos->GetGlobalPose();
                Pose tx = bot->pos->GetGlobalPose();
                int dist = hypot(me.x - tx.x, me.y - tx.y) * 1000;
                // Copy across the message and the distance
                distance_measurement_t  d;
                d.low_gain = dist;
                // Put the message on the queue
                //printf("placing msg from %s in %s dist %d\n",pos->Token(),bot->pos->Token(), dist);
                m_event_t sendmsg;
                sendmsg.m = *msg;
                sendmsg.d = d;
                sendmsg.s = pos->Token();
                meq.push(sendmsg);
            }
        }
    }
    
    // Now check if we have messages on our queue and for each one, call the 
    // callback function
    m_queue_t &meq = message_queue;
    while (!meq.empty())
    {
        (*this.*kilo_message_rx)(&meq.front().m, &meq.front().d);
        //printf("%s got message from distance %s %i\n", pos->Token(), meq.front().s.c_str(), meq.front().d.low_gain);
        meq.pop();
    }
 
}


void Kilobot::update(float delta_t)
{
    dt = delta_t;
    // Update the kilobot tick counter
    // Because user programs can change kilo_ticks, we check to see if the
    // visible integer count has changed dramatically from the real internal
    // time. If so, adjust the internal time. The tick frequency is about
    // 30Hz, so for an update rate of 10Hz it should only ever differ by 3,
    // for an update rate of 60Hz, should only differ by 1
    simtime += dt * 1e6;
    kilo_ticks_real += dt * 1e6 / master_tick_period;
    if (abs(kilo_ticks_real - kilo_ticks) > 3)
        kilo_ticks_real = kilo_ticks;
    else
        kilo_ticks = kilo_ticks_real;

    // Update our fake ModelPosition that the controller can see
    b2Vec2 p    = m_body->GetPosition();
    pos->pose.x = p.x;
    pos->pose.y = p.y;
    float a     = m_body->GetAngle();
    pos->pose.a = a;
    pos->fake_world.simtime = (usec_t) simtime;

    // Handle message system
    check_messages();
    //printf("%s dt:%8f kilo_ticks:%8d\n", __PRETTY_FUNCTION__, dt, kilo_ticks);

    // Run the user code of the controller
    loop();

    // Leave pheromone trace in environment

    // Do the physics, this always happens every tick, regardless of loop schelduling
    // This consists of working out what forces to apply to get our desired goal velocities
    // and then applying them
    //
    // The kilobot physical model assumes that there is a constant velocity dependent drag.
    // This is called damping in the Box2D world. Our goal velocities should thus translate 
    // directly into forces and torques via the damping constant
    //
    // The goal velocities are all in the frame of the kilobot, transform into world frame
    float xd = xdot_goal * cos(a) - ydot_goal * sin(a);
    float yd = xdot_goal * sin(a) + ydot_goal * cos(a);

    float xf        = 0.0;
    float yf        = 0.0;
    float torque    = 0.0;
    b2Vec2 v        = m_body->GetLinearVelocity();
    float omega     = m_body->GetAngularVelocity();
    float m         = m_body->GetMass();
    float i         = m_body->GetInertia();

    // Calculate the forces using damping from current velocity counteracted 
    // by the goal velocity
    //xd = yd = omega_goal = 0.0;
    xf      += (-v.x + xd) * kblineardamp * m;
    yf      += (-v.y + yd) * kblineardamp * m;
    torque  += (-omega + omega_goal) * kbangulardamp * i;
    
    // Apply the force to the kilobot body
    m_body->ApplyForceToCenter(b2Vec2(xf, yf), true);
    m_body->ApplyTorque(torque, true);

    //printf("%3d xd:%10.6f yd:%10.6f ad:%10.6f xf:%10.6f yf:%10.6f tq:%10.6f\n", 
    //kilo_uid, xd, yd, omega_goal, xf, yf, torque);
}

void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color, bool outline, bool solid)
{
	const float32 k_segments = 16.0f;
	const float32 k_increment = 2.0f * b2_pi / k_segments;
	float32 theta = 0.0f;
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (solid)
	    glColor4f(color.r, color.g, color.b, 1.0f);
    else
	    glColor4f(color.r, color.g, color.b, 0.2f);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < k_segments; ++i)
	{
		b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
		glVertex2f(v.x, v.y);
		theta += k_increment;
	}
	glEnd();
	glDisable(GL_BLEND);

    if (outline)
    {
        theta = 0.0f;
        //glColor4f(color.r, color.g, color.b, 1.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        glBegin(GL_LINE_LOOP);
        for (int32 i = 0; i < k_segments; ++i)
        {
            b2Vec2 v = center + radius * b2Vec2(cosf(theta), sinf(theta));
            glVertex2f(v.x, v.y);
            theta += k_increment;
        }
        glEnd();

        b2Vec2 p = center + radius * axis;
        glBegin(GL_LINES);
        glVertex2f(center.x, center.y);
        glVertex2f(p.x, p.y);
        glEnd();
    }
}


void Kilobot::render()
{
    b2Vec2 mypos = m_body->GetPosition();
    float a = m_body->GetAngle();
    const b2Transform &xf = m_body->GetTransform();
    //printf("##id:%5d x:%8.4f y:%8.4f\n", kilo_uid, mypos.x, mypos.y);


    DrawSolidCircle(mypos, kbsenserad, b2Mul(xf, b2Vec2(1.0f, 0.0f)), b2Color(0.1, 0.0, 1.0), false, false);
    DrawSolidCircle(mypos, kbdia/2, b2Mul(xf, b2Vec2(1.0f, 0.0f)), b2Color(0.9, 0.7, 0.7), true, true);


    glColor3f(1,1,1);//white
    glBegin(GL_LINES);
    for (int i = 0; i < inrange_bots.size(); i++) {
        b2Vec2 theirpos = inrange_bots[i]->m_body->GetPosition();
        //printf("  id:%5d x:%8.4f y:%8.4f\n", inrange_bots[i]->kb_id, theirpos.x, theirpos.y);
        glVertex2f(mypos.x, mypos.y);
        glVertex2f(theirpos.x, theirpos.y);
    }
    glEnd();
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

