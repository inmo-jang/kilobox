
#ifndef _BT_H
#define _BT_H

#include <vector>
#include <json.hpp>
using json = nlohmann::json;


namespace BT
{

    

    enum {
        BT_FAILURE = 0,
        BT_SUCCESS,
        BT_RUNNING
    };
    
    class Select_node;
    class Sequence_node;
    class Parallel_node;
    class Leaf_node;
    
    

    class Node
    {
    public:
        Node() {}
        Node(json &j);
        int tick();
    protected:
        std::vector<Node *> children;
    private:
        //json &j;
    };



    class Select_node : public Node
    {
    public:
        Select_node(json &j) : Node(j) {}
    protected:
        // Tick in order from start, return first running or success,
        // failure if no running or success
        int tick();
    };
    
    class Sequence_node : public Node
    {
    public:
        Sequence_node(json &j) : Node(j) {}
    protected:
        // Tick in order from start, return first running or failure,
        // success if all successful
        int tick();

    };

    class Parallel_node : public Node
    {
    public:
        Parallel_node(json &j, int _s, int _f) : Node(j), s(_s), f(_f) {}
    protected:
        // Tick all, return success if >= s successes,
        // failure if >= f failures
        int tick();
    private:
        int s, f;
    };
    
    class Leaf_node : public Node
    {
    public:
        Leaf_node(json &_j) : j(_j) {}
    protected:
        int tick();
    private:
        json j;
    };
    
    //Behaviour_tree_node *behaviour_tree_builder(json &j);

}

#endif
