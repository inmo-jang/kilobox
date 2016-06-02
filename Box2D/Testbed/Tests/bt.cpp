

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
    if (stat != BT_RUNNING)
        init();
    stat = update();
    if (stat != BT_RUNNING)
        finish();
    return stat;
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
    if (run_index < 0)
    {
        for(int i = 0; i < children.size(); ++i)
        {
            p += probability[i];
            if (p > r)
            {
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
    
    
    auto &x = j["x"];
    
    int val = 0;
    if (x.is_number()) val = x;
    
    count = 0;

    if (s == "mf")
    {
        b->vars[0] = 1.0;
        count = val;
    }
    else if (s == "ml")
    {
        b->vars[0] = 1.0;
        count = val;
    }
    else if (s == "mr")
    {
        b->vars[0] = 0.0;
        count = val;
    }
    else if (s == "set")
    {
        b->vars[1] = val;
    }
    stat = BT_RUNNING;
}
Status Leaf_node::update()
{
    // Look at the json fragment to find out what to do
    auto &t = j["type"];
    std::string s = t;
    
    
    count--;
    if (count <= 0)
    {
        stat = BT_SUCCESS;
        if (motor.count(s))
        {
            // We were running a motor command, so kill the motors
            b->vars[0] = 0.0;
        }
    }
    
    if (s == "if")
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
    
    //printf("Tick %s %d %d\n", s.c_str(), stat, count);
    return stat;
}


void Leaf_node::finish()
{
    if (stat == BT_RUNNING)
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
//                      [ "leaf", { <arg_map> } ]
//  node_list       ::= <node_list> , <node> | <node>
//  arg_map         ::= <arg_map>, <arg> | <arg> | <empty>
//  arg             ::= key : value
//  ctrl_node_type1 :: = "seq" | "sel" | "par" | "seqm" | "selm"
//  ctrl_node_type2 :: = "prob" | "probm"
//  leaf_node_type  :: = act | cond
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
            auto &nl = n[1];
            auto &pl = n[2];
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
    }
    printf("leaving node..\n");
}


