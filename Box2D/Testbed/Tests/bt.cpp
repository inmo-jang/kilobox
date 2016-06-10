

#include <cassert>
#include <string>
#include <set>
#include <random>

#include "bt.h"
#include "kilolib.h"


using namespace BT;

std::mt19937  gen;
float rand_realrange(float low, float high)
{
    std::uniform_real_distribution<>   dist(low, high);
    float r = dist(gen);
    //printf("int   %10s %10i\n",pos->Token(), r);
    return r;
}

Status Node::tick(Blackboard *_b)
{
    b = _b;
    // If this is the first time through a node, the map entry will get
    // allocated and assigned using the default constructor for Status, which is 0 == BT_INVALID
    if (b->stat[this] != BT_RUNNING)
        init();
    b->stat[this] = update();
    if (b->stat[this] != BT_RUNNING)
        finish();
    return b->stat[this];
}


Status Node::update()
{
    return children[0]->tick(b);
}

Status Prisel_node::update()
{
    assert(children.size() > 0);
    for(auto& i : children)
    {
        auto state = i->tick(b);
        if (state == BT_RUNNING || state == BT_SUCCESS)
            return state;
    }
    return BT_FAILURE;
}

Status Priselmem_node::update()
{
    assert(children.size() > 0);
    int &run_index = b->count[this];
    for(int i = run_index; i < children.size(); ++i)
    {
        auto state = children[i]->tick(b);
        if (state == BT_RUNNING)
        {
            run_index = i;
            return state;
        }
        if (state == BT_SUCCESS)
        {
            run_index = 0;
            return state;
        }
    }
    run_index = 0;
    return BT_FAILURE;
}

Status Probsel_node::update()
{
    // Randomly choose one child only to tick, based on weights
    assert(children.size() > 0);
    assert(children.size() == probability.size());
    float r = rand_realrange(0,1);
    float p = 0.0f;
    for(int i = 0; i < children.size(); ++i)
    {
        p += probability[i];
        if (p > r)
        {
            return children[i]->tick(b);
        }
    }
    return BT_FAILURE;
}

void Probselmem_node::init()
{
    b->count[this] = -1;
}
Status Probselmem_node::update()
{
    // Randomly choose one child to tick, based on weights. If the child
    // returns RUNNING, remember it and return to it on subsequent ticks
    // until no longer RUNNING
    assert(children.size() > 0);
    assert(children.size() == probability.size());
    float r = rand_realrange(0,1);
    float p = 0.0f;
    Status state = BT_FAILURE;
    int &run_index = b->count[this];
    if (run_index < 0)
    {
        for(int i = 0; i < children.size(); ++i)
        {
            p += probability[i];
            if (p > r)
            {
                //printf("picked %d at %f\n", i, p);
                state = children[i]->tick(b);
                if (state == BT_RUNNING)
                    run_index = i;
                break;
            }
        }
    }
    else
    {
        state = children[run_index]->tick(b);
        if (state != BT_RUNNING)
            run_index = -1;
    }
    //printf("probm r:%f p:%f idx:%i state:%i\n",r,p,run_index, state);
    return state;
}

Status Sequence_node::update()
{
    assert(children.size() > 0);
    for(auto& i : children)
    {
        auto state = i->tick(b);
        if (state == BT_RUNNING || state == BT_FAILURE)
            return state;
    }
    return BT_SUCCESS;
}

Status Sequencemem_node::update()
{
    assert(children.size() > 0);
    int &run_index = b->count[this];
    for(int i = run_index; i < children.size(); ++i)
    {
        auto state = children[i]->tick(b);
        if (state == BT_RUNNING)
        {
            run_index = i;
            return state;
        }
        if (state == BT_FAILURE)
        {
            run_index = 0;
            return state;
        }
    }
    run_index = 0;
    return BT_SUCCESS;
}

void Repeat_node::init()
{
    //printf("rep init %d\n", r);
    b->count[this]  = r;
    b->stat[this]   = BT_RUNNING;
}
Status Repeat_node::update()
{
    int &count      = b->count[this];
    Status &stat    = b->stat[this];
    //printf("rep update %d\n", count);
    assert(children.size() == 1);
    Status cs = children[0]->tick(b);
    if (cs == BT_FAILURE)
    {
        count = 0;
        stat = BT_FAILURE;
    }
    else if (cs == BT_SUCCESS)
    {
        count--;
        if (count == 0)
        {
            stat = BT_SUCCESS;
        }
    }
    return stat;
}
void Repeat_node::finish()
{
    b->count[this]  = 0;
    b->stat[this]   = BT_SUCCESS;
}
/*Status Parallel_node::update()
{
    assert(children.size() > 0);
    int scount = 0;
    int fcount = 0;
    for(auto& i : children)
    {
        auto state = i->tick(b);
        if (state == BT_SUCCESS) ++scount;
        if (state == BT_FAILURE) ++fcount;
    }
    if (scount >= s) return BT_SUCCESS;
    if (fcount >= f) return BT_FAILURE;
    return BT_RUNNING;
}*/



// In automode, the behaviours and conditions are as follows, all conditions except stop have
// obstacle avoidance embedded:
//  exploration     - straight unless obstacle, then turn away from prox sense for random time
//  stop
//  phototaxis      - towards light
//  anti-phototaxis - away from light
//  attraction      - towards other robots in neighbourhood
//  repulsion       - away from other robots in neighbourhood
// Conditions:
//  black floor
//  grey floor
//  white floor
//  neighbour count
//  inverted neighbour count
// fix probability

// With kilobots, necessarily lower level..
//  mf  move forward for x ticks
//  ml  move left       "
//  mr  move right      "
//  sm  set msg to x
//
//  msg count aka neighbours
//  msg average
//  carrying
//  food
//  nest
//
//

// The tree of nodes is traversed. Nodes may be action or composition. Action nodes may return SUCCESS,
// FAILURE, and RUNNING. Composition nodes return a status based on their childrens statuses.
// A RUNNING action node has yet to complete its task.
// Action nodes can have two states, CLOSED, OPEN. A node visited by tick() becomes
// OPEN. The transition from CLOSED to OPEN calls the init() method. The transition
// from OPEN to CLOSED calls the finish() method. All nodes that are OPEN call the tick()
// method.
//
//
//
// To evaluate the tree:
// On every call to tick() at the root, traverse the tree in depth first order,
// this will result in at most one RUNNING node, if the PAR construct is
// not used. In order that a RUNNING node is not orphaned by a change in conditions
// at some later tick(), record the node. If the previously RUNNING node is not reached,
// call its finish() method
//
void Leaf_node::init()
{
    auto &t = j["type"];
    std::string s = t;
    if (action.count(s) && b->running != nullptr && b->running != this)
    {
        // This is an action node (that might become running) and the currently
        // marked running action node is not this node and not null, so it must be
        // cleanly stopped before starting this one
        b->running->finish();
        b->running = nullptr;
    }

    //printf("leaf %s %d\n", s.c_str(), val);
    b->stat[this] = BT_RUNNING;
    
}
Status Leaf_node::update()
{
    // Look at the json fragment to find out what to do
    auto &t = j["type"];
    std::string s = t;
    
    
    Status &stat = b->stat[this];
    if (s == "mf")
    {
        b->vars[0] = 3.0;
        stat = BT_SUCCESS;
    }
    else if (s == "ml")
    {
        b->vars[0] = 1.0;
        stat = BT_SUCCESS;
    }
    else if (s == "mr")
    {
        b->vars[0] = 2.0;
        stat = BT_SUCCESS;
    }
    else if (s == "set")
    {
        auto &jvar = j["var"];
        auto &jval = j["val"];
        int     var = jvar;
        float   val = jval;
        b->vars[var] = val;
        stat = BT_SUCCESS;
    }
    else if (s == "if")
    {
        auto &v1 = j["var1"];
        auto &v2 = j["var2"];
        auto &c1 = j["con1"];
        auto &c2 = j["con2"];
        std::string rel = j["rel"];
        
        float op1, op2;
        if (v1.is_number())     op1 = b->vars[v1];
        if (v2.is_number())     op2 = b->vars[v2];
        if (c1.is_number())     op1 = c1;
        if (c2.is_number())     op2 = c2;
        
        if (rel == ">") stat = op1 > op2 ? BT_SUCCESS : BT_FAILURE;
        if (rel == "<") stat = op1 < op2 ? BT_SUCCESS : BT_FAILURE;
        
        //printf("if %f %s %f\n", op1, rel.c_str(), op2);
    }
    else
    {
        // Undefined, return success
        stat = BT_SUCCESS;
    }

    //std::cout << j << " " << stat << std::endl;

    //printf("Tick %s %d %d\n", s.c_str(), stat, count);
    return stat;
}


void Leaf_node::finish()
{
    if (b->stat[this] == BT_RUNNING)
    {
        // We are a leaf node and still running, so we should be added to the
        // list of running nodes
        b->running = this;
    }
}

//json j = R"(
//[ "seq", {"a",1},
// [
//  ["cond", {"a",1}],
//  ["act", {"a",2}],
//  ["sel",{"a",3},
//   [
//    ["act", {"a",4}],
//    ["act", {"a",5}]
//   ]
//  ]
// ]
//])"_json;

// The tree is represented as a hierarchical list
// Each node consists of a node type string, followed by a list
//
// The type strings, and what the following list represents is given:
//  node            ::= [ <ctrl_node_type1>, [ <node_list> ] ] |
//                      [ <ctrl_node_type2>, [ <node_list> ], [ <prob_list> ] ] |
//                      [ <decorator_type1>, <node> ] |
//                      [ <decorator_type2>, <value>, <node> ] |
//                      [ "leaf", { <arg_map> } ]
//  node_list       ::= <node_list> , <node> | <node>
//  arg_map         ::= <arg_map>, <arg> | <arg> | <empty>
//  arg             ::= key : value
//  ctrl_node_type1 ::= "seq" | "sel" | "par" | "seqm" | "selm"
//  ctrl_node_type2 ::= "prob" | "probm"
//  decorator_type1 ::= "invert" | "succeed" | "fail"
//  decorator_type2 ::= "repeat"
//  leaf_node_type  ::= act | cond
//

Node::Node(json &j)
{
    // Really simple recursive descent parser with no error checking,
    // creating the tree as we go. The top node is the root node
    // and has only one child.
    // Each node of the json description has 2 or 3 elements, depending on
    // the first element which is the node type
    
    // From the BNF, first must be a node type, always a string,
    // and the second is a map
    std::cout << j << std::endl;

    

    
    for(auto& n: j)
    {
        auto &i     = n[0];
        assert(i.is_string());
        std::string s = i;
        printf("element is %s\n", s.c_str());
        if (s == "leaf")
        {
            // Leaf node has 2 elements, second is an arg map
            auto &am = n[1];
            // Leaf types interpreted for now
            std::cout << "constructing leaf with param: " << am << std::endl;
            children.push_back(new Leaf_node(am));
        }
        else if (ctrl_type1.count(s))
        {
            // Type 1 control nodes have 2 elements, second is list of
            // child nodes
            auto &nl = n[1];
            //std::cout << nl << std::endl;
            if (s == "seq")
            {
                std::cout << "constructing seq" << std::endl;
                children.push_back(new Sequence_node(nl));
            }
            else if (s == "seqm")
            {
                std::cout << "constructing seqm" << std::endl;
                children.push_back(new Sequencemem_node(nl));
            }
            else if (s == "sel")
            {
                std::cout << "constructing sel" << std::endl;
                children.push_back(new Prisel_node(nl));
            }
            else if (s == "selm")
            {
                std::cout << "constructing selm" << std::endl;
                children.push_back(new Priselmem_node(nl));
            }
        }
        else if (ctrl_type2.count(s))
        {
            // Type 2 control nodes have 3 elements, second is list of
            // child nodes, third is list of probabilities
            auto &nl = n[2];
            auto &pl = n[1];
            if (s == "prob")
            {
                std::cout << "constructing prob" << std::endl;
                children.push_back(new Probsel_node(nl, pl));
            }
            else if (s == "probm")
            {
                std::cout << "constructing probm" << std::endl;
                children.push_back(new Probselmem_node(nl, pl));
            }
        }
        else if (dec_type2.count(s))
        {
            auto &i = n[1];
            auto &nd = n[2];
            assert(i.is_number());
            int reps = i;
            std::cout << "constructing repeat" << std::endl;
            children.push_back(new Repeat_node(nd, reps));
        }
    }
    printf("leaving node..\n");
}

Node* BT::mf()
{
    //return new Leaf_node(json::parse("[['leaf',{'type':'mf'}]]"));
}
Node* BT::ml()
{
    //return new Leaf_node(json::parse("[['leaf',{'type':'ml'}]]"));
}
Node* BT::seqm2(Node*op1, Node*op2)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    return new Sequencemem_node(children);
}

