

#include <cassert>


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
    for(auto i : children)
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
    for(auto i : children)
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
    for(auto i : children)
    {
        auto state = i->tick();
        if (state == BT_SUCCESS) ++scount;
        if (state == BT_FAILURE) ++fcount;
    }
    if (scount >= s) return BT_SUCCESS;
    if (fcount >= f) return BT_FAILURE;
    return BT_RUNNING;
}



Behaviour_tree_node *behaviour_tree_builder(json j)
{
    
}