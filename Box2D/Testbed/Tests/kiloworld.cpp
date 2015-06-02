

#include "kiloworld.h"
#include "evokilo1.h"

using namespace Kilolib;


void Kiloworld::Step(Settings* settings)
{
    float dt = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);
    //run the default physics and rendering
    Test::Step(settings);
    
    // Jiggle and render
    for(int i=0; i<bots.size(); i++)
    {
        bots[i]->m_body->ApplyForceToCenter(b2Vec2(rand(-0.1, 0.1), rand(-0.1, 0.1)), true);

        bots[i]->update(dt);
        bots[i]->render();
    }
    
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
    {
        ModelPosition *mod = new ModelPosition;
        mod->pose.x = rand(-xsize/2+0.03,+xsize/2-0.03);
        mod->pose.y = rand(-ysize/2+0.03,+ysize/2-0.03);
        mod->pose.a = rand(-M_PI, M_PI);
        mod->world = m_world;
        std::vector<std::string> words;
        bots.push_back((Kilobot*)(new Evokilo1(mod, words)));
    }
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




