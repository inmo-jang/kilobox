

#include <cassert>
#include <string>
#include <set>

#include "bt.h"
#include "kilolib.h"


using namespace BT;


Status Node::tick(Blackboard *_b)
{
    b = _b;
    if (stat != BT_RUNNING)
        init();
    stat = update();
    if (stat != BT_RUNNING)
        finish(stat);
    return stat;
}


Status Node::update()
{
    return children[0]->tick(b);
}

Status Select_node::update()
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

Status Selectmem_node::update()
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

Status Parallel_node::update()
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
}



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
// Traverse,
//
void Leaf_node::init()
{
    auto &t = j["type"];
    auto &x = j["x"];
    
    int val = 0; if (x.is_number()) val = x;
    
    std::string s = t;
    count = 0;

    if (s == "mf")
    {
        b->outputs[0] = 1.0;
        b->outputs[1] = 1.0;
        count = val;
    }
    else if (s == "ml")
    {
        b->outputs[0] = 1.0;
        b->outputs[1] = 0.0;
        count = val;
    }
    else if (s == "mr")
    {
        b->outputs[0] = 0.0;
        b->outputs[1] = 1.0;
        count = val;
    }
    else if (s == "sm")
    {
        b->outputs[2] = val;
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
            b->outputs[0] = 0.0;
            b->outputs[1] = 0.0;
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
        if (v1.is_number())     op1 = b->inputs[v1];
        if (v2.is_number())     op2 = b->inputs[v2];
        if (c1.is_number())     op1 = c1;
        if (c2.is_number())     op2 = c2;
        
        if (rel == ">") stat = op1 > op2 ? BT_SUCCESS : BT_FAILURE;
        if (rel == "<") stat = op1 < op2 ? BT_SUCCESS : BT_FAILURE;
        
        //printf("if %f %s %f\n", op1, rel.c_str(), op2);
    }
    
    //printf("Tick %s %d %d\n", s.c_str(), stat, count);
    return stat;
}


void Leaf_node::finish(Status)
{
    if (stat == BT_RUNNING)
    {
        // We are a leaf node and still running, so we should be added to the
        // list of running nodes
        b->running.push_back(this);
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
//  node            ::= [ <ctrl_node_type>, { <arg_map> }, [ <node_list> ] ] |
//                      [ <leaf_node_type>, { <arg_map> } ]
//  node_list       ::= <node_list> , <node> | <node>
//  arg_map         ::= <arg_map>, <arg> | <arg> | <empty>
//  arg             ::= key : value
//  ctrl_node_type  :: = seq | sel | par | seqm | selm
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
        auto &am    = n[1];
        assert(i.is_string());
        std::string s = i;
        printf("element is %s\n", s.c_str());
        if (ctrl_nodes.count(s))
        {
            // Third element for these types
            json &nl = n[2];
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
                children.push_back(new Select_node(nl));
            }
            else if (s == "selm")
            {
                std::cout << "constructing selm" << std::endl;
                children.push_back(new Selectmem_node(nl));
            }
            else if (s == "par")
            {
                int s = am["s"];
                int f = am["f"];
                std::cout << "constructing par" << std::endl;
                //children.push_back(new Parallel_node(nl, s, f));
            }
        }
        else if (leaf_nodes.count(s))
        {
            // Leaf types interpreted for now
            std::cout << "constructing leaf with param: " << am << std::endl;
            children.push_back(new Leaf_node(am));
        }
    }
    printf("leaving node..\n");
}


