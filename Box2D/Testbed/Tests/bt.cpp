

//#include <cassert>
#include <set>

#include "bt.h"



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


using namespace BT;

#ifndef KILOBOT
#include <random>
std::mt19937  gen;
float rand_realrange(float low, float high)
{
    std::uniform_real_distribution<>   dist(low, high);
    float r = dist(gen);
    //printf("int   %10s %10i\n",pos->Token(), r);
    return r;
}
#else
float rand_realrange(float low, float high)
{
    return low;
}
#endif

Status Node::tick(Blackboard *_b)
{
    b = _b;
    // If this is the first time through a node, the map entry will get
    // allocated and assigned using the default constructor for Status,
    // which is 0 == BT_INVALID
    Status &stat = b->stat[this];
    if (stat != BT_RUNNING)
        init();
    stat = update();
    if (stat != BT_RUNNING)
        finish();
    return stat;
}

// Only the root node is not a derived class, and it only has one child
Status Node::update()
{
    return children[0]->tick(b);
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
    int     &count  = b->count[this];
    Status  &stat   = b->stat[this];
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

// Leaf nodes supported
// success()
// fail()
// mf()
// ml()
// mr()
// ifltvar(op1,op2)
// ifgtvar(op1,op2)
// ifltcon(op1,op2)
// ifgtcon(op1,op2)
Status Success_node::update()
{
    return BT_SUCCESS;
}
Status Fail_node::update()
{
    return BT_SUCCESS;
}
// The motors are a contested resource, if a node tries to get access and they
// are in use, return RUNNING and try again. The
Status Mf_node::update()
{
    if (b->vars[0] > 0.0)
        return BT_RUNNING;
    b->vars[0] = 3.0;
    return BT_SUCCESS;
}
Status Ml_node::update()
{
    if (b->vars[0] > 0.0)
        return BT_RUNNING;
    b->vars[0] = 1.0;
    return BT_SUCCESS;
}
Status Mr_node::update()
{
    if (b->vars[0] > 0.0)
        return BT_RUNNING;
    b->vars[0] = 2.0;
    return BT_SUCCESS;
}
Status Ifltvar_node::update()
{
    return b->vars[op1] < b->vars[op2] ? BT_SUCCESS : BT_FAILURE;
}
Status Ifgtvar_node::update()
{
    return b->vars[op1] > b->vars[op2] ? BT_SUCCESS : BT_FAILURE;
}
Status Ifltcon_node::update()
{
    return b->vars[op1] < op2 ? BT_SUCCESS : BT_FAILURE;
}
Status Ifgtcon_node::update()
{
    return b->vars[op1] > op2 ? BT_SUCCESS : BT_FAILURE;
}
Status Set_node::update()
{
    b->vars[op1] = op2;
    return BT_SUCCESS;
}

#ifndef KILOBOT
Node::Node(json &j)
{
    // Really simple recursive descent parser with no error checking,
    // creating the tree as we go from JSON. The top node is the root node
    // and has only one child.
    // Each node of the json description has 2 or 3 elements, depending on
    // the first element which is the node type
    
    // From the BNF, first must be a node type, always a string,
    // and the second is a map
    std::cout << j << std::endl;
    for(auto& n: j)
    {
        //auto &i     = n[0];
        //assert(i.is_string());
        //std::string s = i;
        std::string s = n[0];
        printf("element is %s\n", s.c_str());
        if (s == "leaf")
        {
            // New form constructed leaf nodes
            auto &am    = n[1];
            std::string s = am["type"];
            printf("leaf type is: %s\n", s.c_str());
            if (s == "mf")
            {
                children.push_back(mf());
            }
            else if (s == "ml")
            {
                children.push_back(ml());
            }
            else if (s == "mr")
            {
                children.push_back(mr());
            }
            else if (s == "success")
            {
                children.push_back(success());
            }
            else if (s == "fail")
            {
                children.push_back(fail());
            }
            else if (s == "ifltvar")
            {
                int     op1 = am["var1"];
                int     op2 = am["var2"];
                children.push_back(ifltvar(op1, op2));
            }
            else if (s == "ifgtvar")
            {
                int     op1 = am["var1"];
                int     op2 = am["var2"];
                children.push_back(ifgtvar(op1, op2));
            }
            else if (s == "ifltcon")
            {
                int     op1 = am["var1"];
                float   op2 = am["con2"];
                children.push_back(ifltcon(op1, op2));
            }
            else if (s == "ifgtcon")
            {
                int     op1 = am["var1"];
                float   op2 = am["con2"];
                children.push_back(ifgtcon(op1, op2));
            }
            else if (s == "set")
            {
                int     op1 = am["var"];
                float   op2 = am["val"];
                children.push_back(set(op1,op2));
            }
        }
        else if (ctrl_type1.count(s))
        {
            // Type 1 control nodes have 2 elements, second is list of
            // child nodes
            auto &nl = n[1];
            //std::cout << nl << std::endl;
            if (s == "seqm")
            {
                std::cout << "constructing seqm" << std::endl;
                children.push_back(new Sequencemem_node(nl));
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
            // probabilities, third is list of child nodes
            auto &nl = n[2];
            auto &pl = n[1];
            if (s == "probm")
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
#endif

Node* BT::mf()
{
    return new Mf_node();
}
Node* BT::ml()
{
    return new Ml_node();
}
Node* BT::mr()
{
    return new Mr_node();
}
Node* BT::success()
{
    return new Success_node();
}
Node* BT::fail()
{
    return new Fail_node();
}
Node* BT::ifltvar(int op1, int op2)
{
    return new Ifltvar_node(op1, op2);
}
Node* BT::ifgtvar(int op1, int op2)
{
    return new Ifgtvar_node(op1, op2);
}
Node* BT::ifltcon(int op1, float op2)
{
    return new Ifltcon_node(op1, op2);
}
Node* BT::ifgtcon(int op1, float op2)
{
    return new Ifgtcon_node(op1, op2);
}
Node* BT::set(int op1, float op2)
{
    return new Set_node(op1, op2);
}
Node* BT::seqm2(Node*op1, Node*op2)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    return new Sequencemem_node(children);
}
Node* BT::seqm3(Node*op1, Node*op2, Node*op3)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    return new Sequencemem_node(children);
}
Node* BT::seqm4(Node*op1, Node*op2, Node*op3, Node*op4)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    children->push_back(op4);
    return new Sequencemem_node(children);
}
Node* BT::selm2(Node*op1, Node*op2)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    return new Priselmem_node(children);
}
Node* BT::selm3(Node*op1, Node*op2, Node*op3)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    return new Priselmem_node(children);
}
Node* BT::selm4(Node*op1, Node*op2, Node*op3, Node*op4)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    children->push_back(op4);
    return new Priselmem_node(children);
}
Node* BT::probm2(float p0, Node*op1, Node*op2)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    return new Probselmem_node(p0, children);
}
Node* BT::probm3(float p0, float p1, Node*op1, Node*op2, Node*op3)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    return new Probselmem_node(p0, p1, children);
}
Node* BT::probm4(float p0, float p1, float p2, Node*op1, Node*op2, Node*op3, Node*op4)
{
    Children_t *children = new Children_t;
    children->push_back(op1);
    children->push_back(op2);
    children->push_back(op3);
    children->push_back(op4);
    return new Probselmem_node(p0, p1, p2, children);
}



