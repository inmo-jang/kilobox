
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

void Kiloworld::parse_worldfile(float xoffset, float yoffset)
{
    for(int entity=1; entity < wf->GetEntityCount(); entity++)
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
            mod->pose.x = x + xoffset;
            mod->pose.y = y + yoffset;
            mod->pose.a = a;
            mod->world = m_world;
            // tokenize the argument string into words
            std::vector<std::string> words = split(controller);
            printf("ctrargs is %s %s\n", ctrlarg_words[0].c_str(), ctrlarg_words[1].c_str());
            std::string logfile = "";
            if (ctrlarg_words.size() == 2 && ctrlarg_words[0] == "log")
                logfile = ctrlarg_words[1];
            bots.push_back((Kilobot*)(new Evokilo1(mod, words, logfile.c_str())));
        }
    }
}

void Kiloworld::build_world()
{

    
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
    // entity 0 is the world, which all settings are attached to
    settings->quit_time = wf->ReadFloat(entity, "quit_time", settings->quit_time);
    printf("Got quit time %f\n", settings->quit_time);

    // Get any command line args for the controllers
    ctrlarg_words = split(settings->ctrlargs);
    
    // Make a grid of arenas
    for(int y=0; y<ygrid; y++)
    {
        for(int x=0; x<xgrid; x++)
        {
            // Make arena of fixed lines
            float xp = (xsize + gridmargin) * x;
            float yp = (ysize + gridmargin) * y;
            make_static_box(xsize, ysize, xp, yp);
            parse_worldfile(xp, yp);
        }
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
    arena_fixture.push_back(arena->CreateFixture(&perimeter, 0));
}


void Kiloworld::render_arena()
{
    for(int i=0; i<arena_fixture.size(); i++)
    {
        b2ChainShape *chain = (b2ChainShape*)arena_fixture[i]->GetShape();
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
}




