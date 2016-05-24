

#include <cassert>
#include <string>

#include "bt.h"

using namespace BT;


Select_node *Behaviour_tree_node::select_node()
{
    return new Select_node();
}
Sequence_node *Behaviour_tree_node::sequence_node()
{
    return new Sequence_node();
}
Parallel_node *Behaviour_tree_node::parallel_node(int _s, int _f)
{
    return new Parallel_node(_s, _f);
}
Leaf_node *Behaviour_tree_node::leaf_node(int (*_l)())
{
    return new Leaf_node(_l);
}

int Root_node::tick()
{
    assert(children.size() > 0);
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

json j = R"(
[ "seq",
    [
        ["cond", [1]],
        ["act", [2]],
        ["sel",
            [
                ["act", [3]],
                ["act", [4]]
            ]
        ]
    ]
])"_json;

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

Root_node::Root_node(json &j)
{
    for(auto& i : j)
    {
        if (i.is_string())
        {
            std::string s = i;
            printf("control: %s\n", s.c_str());
        }
        else if (i.is_array())
        {
            printf("entering node..\n");
            behaviour_tree_builder(i);
        }
        else if (i.is_number())
        {
            float f = i;
            printf("leaf: %f\n", f);
        }
        else
        {
            printf("don't know what to do\n");
        }
    }
    printf("leaving node..\n");
    //return nullptr;
}


