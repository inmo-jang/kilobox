
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "kiloworld.h"
#include "evokilo1.h"
#include "worldfile.h"

using namespace Kilolib;


void Kiloworld::Step(Settings* settings)
{
    float dt = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

    //run the default physics and rendering
    Test::Step(settings);
    // Jiggle and render
    for(int i=0; i<bots.size(); i++)
        bots[i]->update(dt);
    // Show some text in the main screen
    
    if (settings->time_to_draw)
    {
        render_arena();
        for(int i=0; i<bots.size(); i++)
            bots[i]->render();
        m_debugDraw.DrawString(5, m_textLine, "Kilobot");
        m_textLine += 15;
    }
}

static std::vector<std::string> split(std::string s)
{
    // Quite frankly, this is inpenetrable, found on stackoverflow, but at least it doesn't require boost
    // Apparently an istream iterator of type string regards whitespace as the separator..
    std::vector<std::string> words;
    std::istringstream iss(s);
    std::copy (std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), back_inserter(words));
    return words;
}


void Kiloworld::build_world()
{
    // Make arena of fixed lines
    make_static_box(xsize, ysize, 0.0, 0.0);
    
    // Read in the worldfile

    printf("%s\n", settings->worldfile.c_str());
    // We need to parse the worldfile, which is in Stage format, but the only
    // things we care about are:
    //  quit_time <number>
    //  define <name> kilobot ( ctrl "<control_string>" )
    //  <name> ( pose [ <x> <y> <dont_care> <angle> ] )
    wf = new Worldfile();
    if (!wf->Load(settings->worldfile))
    {
        exit(1);
    }
    int entity = 0;
    //float quit_time = 0;
    settings->quit_time = wf->ReadFloat(entity, "quit_time", settings->quit_time);
    printf("Got quit time %f\n", settings->quit_time);
    for(entity=1; entity < wf->GetEntityCount(); entity++)
    {
        const char *typestr = (char*)wf->GetEntityType(entity);
        int entity_parent = wf->GetEntityParent(entity);
        //printf("%d %d %s\n", entity, wf->GetEntityParent(entity), typestr);
        // We are interested in the entities of type position with parent entity 0
        // These are top level kilobit models within the world, grab the pose
        // data associated with them, and the controller
        if (entity_parent == 0 && !strcmp(typestr, "position"))
        {
            double x, y, z, a;
            if (!wf->PropertyExists(entity, "pose"))
            {
                printf("No pose found for kilobot entity %d, exitting\n", entity);
                exit(1);
            }
            wf->ReadTuple(entity, "pose", 0, 4, "llla", &x, &y, &z, &a);
            CProperty *ctrlp =  wf->GetProperty(entity, "ctrl");
            if (!ctrlp)
            {
                printf("No controller token for kilobot entity %d, exitting\n", entity);
                exit(1);
            }
            const char *controller = wf->GetPropertyValue(ctrlp, 0);
            if (!controller)
            {
                printf("No controller string for kilobot entity %d, exitting\n", entity);
                exit(1);
            }
            //printf("Kilobot pose[%f,%f,%f,%f] ctrl[%s]\n", x, y, z, a, controller);

            // Now create the kilobot 
            ModelPosition *mod = new ModelPosition;
            mod->pose.x = x;
            mod->pose.y = y;
            mod->pose.a = a;
            mod->world = m_world;
            // tokenize the argument string into words
            std::vector<std::string> words = split(controller);
            bots.push_back((Kilobot*)(new Evokilo1(mod, words, "log.txt")));

        }
    }
    

//    // Now create kilobots randomly distributed
//    for(int i=0;i<100;i++)
//    {
//        ModelPosition *mod = new ModelPosition;
//        mod->pose.x = rand(-xsize/2+0.03,+xsize/2-0.03);
//        mod->pose.y = rand(-ysize/2+0.03,+ysize/2-0.03);
//        mod->pose.a = rand(-M_PI, M_PI);
//        mod->world = m_world;
//        std::vector<std::string> words = {"evokilo1",
//            "-0.686275", "0.294118", "-3.000000", "1.196078",
//            "-3.039216", "-2.372549", "2.294118", "-4.529412",
//            "-4.333333", "3.941176", "-2.411765", "-3.745098",
//            "0.215686", "0.098039", "-4.882353", "3.862745",
//            "0.725490", "-3.901961", "3.588235", "0.882353",
//            "-2.372549", "1.980392", "3.274510", "-4.725490"};
//        bots.push_back((Kilobot*)(new Evokilo1(mod, words, "log.txt")));
//    }
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
    arena_fixture = arena->CreateFixture(&perimeter, 0);
}


void Kiloworld::render_arena()
{
    b2ChainShape *chain = (b2ChainShape*)arena_fixture->GetShape();
	int32 count = chain->m_count;
    const b2Vec2* vertices = chain->m_vertices;

    b2Vec2 v1 = vertices[0];

    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    for (int32 i = 1; i < count; ++i)
    {
        b2Vec2 v2 = vertices[i];
        glVertex2f(v1.x, v1.y);
        glVertex2f(v2.x, v2.y);
        //m_debugDraw.DrawSegment(v1, v2, color);
        //m_debugDraw.DrawCircle(v1, 0.05f, color);
        v1 = v2;
    }
    glEnd();
}




