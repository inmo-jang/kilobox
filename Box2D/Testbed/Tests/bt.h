
#ifndef _BT_H
#define _BT_H

#include <vector>
#include <set>
#include <json.hpp>
using json = nlohmann::json;


namespace BT
{

    

    enum Status {
        BT_INVALID = 0,
        BT_FAILURE,
        BT_SUCCESS,
        BT_RUNNING
    };
    
    const std::set<std::string> ctrl_nodes  {"seq", "seqm", "sel", "selm", "par"};
    const std::set<std::string> leaf_nodes  {"leaf"};
    const std::set<std::string> motor       {"mf", "ml", "mr"};

    
    class Select_node;
    class Sequence_node;
    class Parallel_node;
    class Leaf_node;
    class Node;
    
    class Blackboard
    {
    public:
        Blackboard(int _i, int _o)
        {
            inputs.resize(_i);
            outputs.resize(_o);
        }
        std::vector<float> inputs;
        std::vector<float> outputs;
        std::vector<Node*> running;
    };

    class Node
    {
    public:
        Node() : stat(BT_INVALID) {}
        Node(json &j);
        Status          tick(Blackboard *_b);
        Status          tick();
        virtual void    init() {}
        virtual Status  update();
        virtual void    finish(Status) {}
        Blackboard *b;
    protected:
        std::vector<Node *> children;
    private:
        Status stat;
    };


    
    class Select_node : public Node
    {
    public:
        Select_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
    };
    
    class Selectmem_node : public Node
    {
    public:
        Selectmem_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
    private:
        int run_index = 0;
    };
    
    class Sequence_node : public Node
    {
    public:
        Sequence_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
        
    };
    
    class Sequencemem_node : public Node
    {
    public:
        Sequencemem_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
    private:
        int run_index = 0;
        
    };

    class Parallel_node : public Node
    {
    public:
        Parallel_node(json &j, int _s, int _f) : Node(j), s(_s), f(_f) {}
    protected:
        virtual Status update();
    private:
        int s, f;
    };
    
    class Leaf_node : public Node
    {
    public:
        Leaf_node(json &_j) : j(_j) {}
    protected:
        virtual void    init();
        virtual Status  update();
        virtual void    finish(Status s);
    private:
        json j;
        Status stat;
        int count = 0;
    };
    
    //Behaviour_tree_node *behaviour_tree_builder(json &j);

}

#endif
