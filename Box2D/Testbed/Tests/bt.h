

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
    
    

    class Behaviour_tree_node
    {
    public:
        //Behaviour_tree_node() {}
        virtual int tick() = 0;
        Select_node *select_node();
        Sequence_node *sequence_node();
        Parallel_node *parallel_node(int _s, int _f);
        Leaf_node *leaf_node(int (*_l)());

        

    protected:
        std::vector<Behaviour_tree_node *> children;
    };

    class Root_node : public Behaviour_tree_node
    {
    protected:
        // Single child, just does tick
        int tick();
    };

    class Select_node : public Behaviour_tree_node
    {
    protected:
        // Tick in order from start, return first running or success,
        // failure if no running or success
        int tick();
    };
    
    class Sequence_node : public Behaviour_tree_node
    {
    protected:
        // Tick in order from start, return first running or failure,
        // success if all successful
        int tick();

    };

    class Parallel_node : public Behaviour_tree_node
    {
    public:
        Parallel_node(int _s, int _f) : s(_s), f(_f) {}
    protected:
        // Tick all, return success if >= s successes,
        // failure if >= f failures
        int tick();
    private:
        int s, f;
    };
    
    class Leaf_node : public Behaviour_tree_node
    {
    public:
        Leaf_node(int (*_l)() ) : l(_l) {}
    protected:
        int tick()
        {
            return l();
        }
    private:
        int (*l)() ;
    };
    
    Behaviour_tree_node *behaviour_tree_builder(json j);

};
