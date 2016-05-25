

#include <cassert>
#include <string>
#include <set>

#include "bt.h"

using namespace BT;


int Node::tick()
{
    assert(children.size() == 1);
    return children[0]->tick();
}

int Select_node::tick()
{
    assert(children.size() > 0);
    for(auto& i : children)
    {
        auto state = i->tick();
        if (state == BT_RUNNING || state == BT_SUCCESS)
            return state;
    }
    return BT_FAILURE;
}

int Sequence_node::tick()
{
    assert(children.size() > 0);
    for(auto& i : children)
    {
        auto state = i->tick();
        if (state == BT_RUNNING || state == BT_FAILURE)
            return state;
    }
    return BT_SUCCESS;
}

int Parallel_node::tick()
{
    assert(children.size() > 0);
    int scount = 0;
    int fcount = 0;
    for(auto& i : children)
    {
        auto state = i->tick();
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
//  move straight for x ticks
//  move left       "
//  move right      "
//  set msg to x
//
//  msg count aka neighbours
//  msg average
//  carrying
//  food
//  nest
//
//

// Given a condition vector C, composed of booleans, traversng the tree gives a vector of
// action nodes that are visited with tick(). These nodes may return SUCCESS,
// FAILURE, and RUNNING. A RUNNING action node has yet to complete its task.
// Action nodes can have two states, CLOSED, OPEN. A node visited by tick() becomes
// OPEN. The transition from CLOSED to OPEN calls the init() method. The transition
// from OPEN to CLOSED calls the finish() method. All nodes that are OPEN call the tick()
// method.
//
// Actually, the above is wrong, since action nodes also are effectively part of the
// condition vector
//
// To evaluate the tree:
// Traverse
//
int Leaf_node::tick()
{
    // Look at the json fragment to find out what to do
    auto &t = j["type"];
    printf("%s\n", t.c_str());
    return BT_SUCCESS;
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

    const std::set<std::string> ctrl_nodes {"seq", "sel", "par"};
    const std::set<std::string> leaf_nodes {"act", "cond"};
    
    for(auto& n: j)
    {
        auto &i     = n[0];
        auto &am    = n[1];
        assert(i.is_string());
        std::string s = i;
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
            else if (s == "sel")
            {
                std::cout << "constructing sel" << std::endl;
                children.push_back(new Select_node(nl));
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
            // Only two elements for the leaf types
            std::cout << "constructing leaf with param: " << am << std::endl;
            children.push_back(new Leaf_node(am));
        }
    }
    printf("leaving node..\n");
}


