
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
    
    const std::set<std::string> ctrl_type1  {"seq", "seqm", "sel", "selm", "par"};
    const std::set<std::string> ctrl_type2  {"prob", "probm"};
    const std::set<std::string> dec_type1   {"invert", "succeed", "fail"};
    const std::set<std::string> dec_type2   {"repeat"};
    const std::set<std::string> action      {"mf", "ml", "mr", "set"};
    const std::set<std::string> motor       {"mf", "ml", "mr"};

    
    class Select_node;
    class Sequence_node;
    class Parallel_node;
    class Leaf_node;
    class Node;
    
    typedef std::map<Node*, Status> Nodestatmap_t;
    typedef std::map<Node*, int>    Nodecountmap_t;
    class Blackboard
    {
    public:
        Blackboard(int _v)
        {
            vars.resize(_v);
        }
        std::vector<float>  vars;
        Node*               running = nullptr;
        Nodestatmap_t       stat;
        Nodecountmap_t      count;
    };

    class Node
    {
    public:
        Node() {}
        Node(json &j);
        Status          tick(Blackboard *_b);
        virtual void    init() {}
        virtual Status  update();
        virtual void    finish() {}
        Blackboard *b;
    protected:
        std::vector<Node *> children;
    };


    
    class Prisel_node : public Node
    {
    public:
        Prisel_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
    };
    
    class Priselmem_node : public Node
    {
    public:
        Priselmem_node(json &j) : Node(j) {}
    protected:
        virtual Status update();
    };
    
    class Probsel_node : public Node
    {
    public:
        Probsel_node(json &j, json &p) : Node(j)
        {
            for(auto &prob : p)
            if (prob.is_number())
            {
                float fp = prob;
                probability.push_back(fp);
            }
        }
    protected:
        virtual Status update();
    private:
        std::vector<float> probability;
    };
    
    class Probselmem_node : public Node
    {
    public:
        Probselmem_node(json &j, json &p) : Node(j)
        {
            for(auto &prob : p)
            if (prob.is_number())
            {
                float fp = prob;
                probability.push_back(fp);
            }
        }
    protected:
        virtual void init();
        virtual Status update();
    private:
        std::vector<float> probability;
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
    };
    
    class Repeat_node : public Node
    {
    public:
        Repeat_node(json &j, int _r) : Node(j), r(_r) {}
    protected:
        virtual void    init();
        virtual Status  update();
        virtual void    finish();
    private:
        int     r;
    };

/*    class Parallel_node : public Node
    {
    public:
        Parallel_node(json &j, int _s, int _f) : Node(j), s(_s), f(_f) {}
    protected:
        virtual Status update();
    private:
        int s, f;
    };
*/
    class Leaf_node : public Node
    {
    public:
        Leaf_node(json &_j) : j(_j) {}
    protected:
        virtual void    init();
        virtual Status  update();
        virtual void    finish();
    private:
        json j;
    };
    
    //Behaviour_tree_node *behaviour_tree_builder(json &j);

}

#endif
