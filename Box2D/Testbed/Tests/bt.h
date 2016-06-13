
#ifndef _BT_H
#define _BT_H

#include <vector>
#include <set>
#include <map>

//#define BTJSON


#ifdef BTJSON

#include <json.hpp>
using json = nlohmann::json;
#endif

namespace BT
{

    

    enum Status {
        BT_INVALID = 0,
        BT_FAILURE,
        BT_SUCCESS,
        BT_RUNNING
    };
    
#ifdef BTJSON
    const std::set<std::string> ctrl_type1  {"seq", "seqm", "sel", "selm", "par"};
    const std::set<std::string> ctrl_type2  {"prob", "probm"};
    const std::set<std::string> dec_type1   {"invert", "succeed", "fail"};
    const std::set<std::string> dec_type2   {"repeat"};
    const std::set<std::string> action      {"mf", "ml", "mr", "set"};
    const std::set<std::string> motor       {"mf", "ml", "mr"};
#endif

    class Node;
    
    typedef std::map<Node*, Status> Nodestatmap_t;
    typedef std::map<Node*, int>    Nodecountmap_t;
    typedef std::vector<Node*>      Children_t;

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
#ifdef BTJSON
        Node(json &j);
#endif
        Node(Children_t *c)
        {
            for(auto &i : *c)
                children.push_back(i);
        }
        Status          tick(Blackboard *_b);
        virtual void    init() {}
        virtual Status  update();
        virtual void    finish() {}
        Blackboard *b;
    protected:
        Children_t      children;
    };



    
    class Priselmem_node : public Node
    {
    public:
#ifdef BTJSON
        Priselmem_node(json &j) : Node(j) {}
#endif
        Priselmem_node(Children_t *c) : Node(c) {}
        virtual Status  update();
    };
    

    
    class Probselmem_node : public Node
    {
    public:
#ifdef BTJSON
        Probselmem_node(json &j, json &p) : Node(j)
        {
            for(auto &prob : p)
            if (prob.is_number())
            {
                float fp = prob;
                probability.push_back(fp);
            }
        }
#endif
        Probselmem_node(float p0, Children_t *c) : Node(c)
        {
            probability.push_back(p0);
            probability.push_back(1.0);
        }
        Probselmem_node(float p0, float p1, Children_t *c) : Node(c)
        {
            probability.push_back(p0);
            probability.push_back(p1);
            probability.push_back(1.0);
        }
        Probselmem_node(float p0, float p1, float p2, Children_t *c) : Node(c)
        {
            probability.push_back(p0);
            probability.push_back(p1);
            probability.push_back(p2);
            probability.push_back(1.0);
        }
        virtual void init();
        virtual Status update();
    private:
        std::vector<float> probability;
    };
    

    
    class Sequencemem_node : public Node
    {
    public:
#ifdef BTJSON
        Sequencemem_node(json &j) : Node(j) {}
#endif
        Sequencemem_node(Children_t *c) : Node(c) {}
        virtual Status update();
    };
    
    class Repeat_node : public Node
    {
    public:
#ifdef BTJSON
        Repeat_node(json &j, int _r) : Node(j), r(_r) {}
#endif
        virtual void    init();
        virtual Status  update();
        virtual void    finish();
    private:
        int     r;
    };



    
    //--------------------------------------------------
    // Fixed leaf nodes
    class Success_node : public Node
    {
    public:
        virtual Status  update();
    };
    class Fail_node : public Node
    {
    public:
        virtual Status  update();
    };
    class Mf_node : public Node
    {
    public:
        virtual Status  update();
    };
    class Ml_node : public Node
    {
    public:
        virtual Status  update();
    };
    class Mr_node : public Node
    {
    public:
        virtual Status  update();
    };
    class Ifltvar_node : public Node
    {
    public:
        Ifltvar_node(int _op1, int _op2) : op1(_op1), op2(_op2) {}
        virtual Status  update();
    private:
        int op1, op2;
    };
    class Ifgtvar_node : public Node
    {
    public:
        Ifgtvar_node(int _op1, int _op2) : op1(_op1), op2(_op2) {}
        virtual Status  update();
    private:
        int op1, op2;
    };
    class Ifltcon_node : public Node
    {
    public:
        Ifltcon_node(int _op1, float _op2) : op1(_op1), op2(_op2) {}
        virtual Status  update();
    private:
        int op1;
        float op2;
    };
    class Ifgtcon_node : public Node
    {
    public:
        Ifgtcon_node(int _op1, float _op2) : op1(_op1), op2(_op2) {}
        virtual Status  update();
    private:
        int op1;
        float op2;
    };
    class Set_node : public Node
    {
    public:
        Set_node(int _op1, float _op2) : op1(_op1), op2(_op2) {}
        virtual Status  update();
    private:
        int     op1;
        float   op2;
    };
    
    
    //Behaviour_tree_node *behaviour_tree_builder(json &j);
    Node* mf();
    Node* ml();
    Node* mr();
    Node* success();
    Node* fail();
    Node* ifltvar(int op1, int op2);
    Node* ifgtvar(int op1, int op2);
    Node* ifltcon(int op1, float op2);
    Node* ifgtcon(int op1, float op2);
    Node* set(int op1, float op2);
    Node* seqm2(Node*op1, Node*op2);
    Node* seqm3(Node*op1, Node*op2, Node*op3);
    Node* seqm4(Node*op1, Node*op2, Node*op3, Node*op4);
    Node* selm2(Node*op1, Node*op2);
    Node* selm3(Node*op1, Node*op2, Node*op3);
    Node* selm4(Node*op1, Node*op2, Node*op3, Node*op4);
    Node* probm2(float p0, Node*op1, Node*op2);
    Node* probm3(float p0, float p1, Node*op1, Node*op2, Node*op3);
    Node* probm4(float p0, float p1, float p2, Node*op1, Node*op2, Node*op3, Node*op4);

    

}

#endif
