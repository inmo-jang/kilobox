
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>


#include "bts.h"


#ifndef KBCOMPILE
float rand_realrange(float low, float high)
{
    float r = ((float)rand()/RAND_MAX) * (high - low) + low;
    return r;
}
#else
float rand_realrange(float low, float high)
{
    float r = ((float)rand()/RAND_MAX) * (high - low) + low;
    return r;
}
#endif


// Private variable pointing at the blackboard, and setter for it
static float *vars;
void set_vars(float *v)
{
    vars = v;
}

struct SM2
{
    int8_t      idx;
    struct Node *op[2];
};

struct SM3
{
    int8_t      idx;
    struct Node *op[3];
};

struct SM4
{
    int8_t      idx;
    struct Node *op[4];
};

struct PM2
{
    int8_t      idx;
    float       p[1];
    struct Node *op[2];
};

struct PM3
{
    int8_t      idx;
    float       p[2];
    struct Node *op[3];
};

struct PM4
{
    int8_t      idx;
    float       p[3];
    struct Node *op[4];
};

struct IFV
{
    int8_t      op1;
    int8_t      op2;
};

struct IFC
{
    int8_t      op1;
    float       op2;
};

struct REP
{
    int8_t      count;
    int8_t      repeat;
    struct Node *op;
};

struct FIX
{
    struct Node *op;
};


union Ndata
{

    struct SM2  sm2;
    struct SM3  sm3;
    struct SM4  sm4;
    struct PM2  pm2;
    struct PM3  pm3;
    struct PM4  pm4;
    struct IFV  ifv;
    struct IFC  ifc;
    struct REP  rep;
    struct FIX  fix;

};

struct Node
{
    Nodetype    type;
    Status      status;
    union Ndata data;
};




#ifdef KBCOMPILE
// The kilobot has extremely limited RAM, only 2k bytes. Each node gets allocated
// space on the heap with calloc and each node is quite small, between 7 and 23 bytes.
// The standard memory allocator in avr-libc has a 2 byte overhead per allocation, giving
// between 9% and 30% bloat in memory usage. The memory allocated is never freed. For this
// reason, we statically grab a chunk of memory and allocate from this with a replacement
// calloc.
#define MLIMIT 1024
char mem[MLIMIT];
void *calloc(size_t count, size_t size)
{
    static int ptr    = 0;
    if (ptr + size * count >= MLIMIT)
        return 0;
    int p = ptr;
    ptr += size * count;
    return (void*)(p + mem);
}
// Little helper for kilobot to see if in danger of stack clash
#include <avr/common.h>
void print_sp()
{
    char a;
    printf("sp:%04x\r\n",(int)&a);
}
#endif


// Return the size in bytes of the node type.
// Normally, this returns the sizeof the Node struct, which is 2 + the
// size of the union Ndata, but the structs composing Ndata are variable
// in size. Because of the limited space on the kilobot, we return the
// actual used space, allowing successive nodes to overlap the unused space
// at the end of previous nodes.
int nsize(Nodetype type)
{
#ifdef KBCOMPILE
    // Space is very tight on the kilobot
    switch (type)
    {
        case SEQM2:
        case SELM2:     return sizeof(struct SM2) + 2;  // 7
        case SEQM3:
        case SELM3:     return sizeof(struct SM3) + 2;  // 9
        case SEQM4:
        case SELM4:     return sizeof(struct SM4) + 2;  // 11
        case PROBM2:    return sizeof(struct PM2) + 2;  // 11
        case PROBM3:    return sizeof(struct PM3) + 2;  // 17
        case PROBM4:    return sizeof(struct PM4) + 2;  // 23
        case MF:
        case ML:
        case MR:
        case SUCCESSL:
        case FAILUREL:  return 2;
        case IFLTVAR:
        case IFGTVAR:   return sizeof(struct IFV) + 2;  // 4
        case IFLTCON:
        case IFGTCON:
        case SET:       return sizeof(struct IFC) + 2;  // 7
        case REPEAT:    return sizeof(struct REP) + 2;  // 6
        case SUCCESSD:
        case FAILURED:  return sizeof(struct FIX) + 2;  // 4
    }
#endif
    return sizeof(struct Node);
}

typedef struct Node* Nodep;

// Return a pointer to a new node of type type, with space allocated
// and fields filled in
struct Node *newnode(Nodetype type, ...)
{
    va_list args;
    va_start(args, type);
    
    // All fields get zeroed, this means status starts at INVALID
    struct Node *n = (struct Node*)calloc(1, nsize(type));
    //printf("%04x %d\r\n",(int)n,type);
    n->type = type;
    
    switch(type)
    {
        case SEQM2:
        case SELM2:
        {
            n->data.sm2.op[0] = va_arg(args, Nodep);
            n->data.sm2.op[1] = va_arg(args, Nodep);
            break;
        }
        case SEQM3:
        case SELM3:
        {
            n->data.sm3.op[0] = va_arg(args, Nodep);
            n->data.sm3.op[1] = va_arg(args, Nodep);
            n->data.sm3.op[2] = va_arg(args, Nodep);
            break;
        }
        case SEQM4:
        case SELM4:
        {
            n->data.sm4.op[0] = va_arg(args, Nodep);
            n->data.sm4.op[1] = va_arg(args, Nodep);
            n->data.sm4.op[2] = va_arg(args, Nodep);
            n->data.sm4.op[3] = va_arg(args, Nodep);
            break;
        }
        case PROBM2:
        {
            n->data.pm2.idx     = -1;
            n->data.pm2.p[0]    = va_arg(args, double);
            n->data.pm2.op[0]   = va_arg(args, Nodep);
            n->data.pm2.op[1]   = va_arg(args, Nodep);
            break;
        }
        case PROBM3:
        {
            n->data.pm3.idx     = -1;
            n->data.pm3.p[0]    = va_arg(args, double);
            n->data.pm3.p[1]    = va_arg(args, double);
            n->data.pm3.op[0]   = va_arg(args, Nodep);
            n->data.pm3.op[1]   = va_arg(args, Nodep);
            n->data.pm3.op[2]   = va_arg(args, Nodep);
            break;
        }
        case PROBM4:
        {
            n->data.pm4.idx     = -1;
            n->data.pm4.p[0]    = va_arg(args, double);
            n->data.pm4.p[1]    = va_arg(args, double);
            n->data.pm4.p[2]    = va_arg(args, double);
            n->data.pm4.op[0]   = va_arg(args, Nodep);
            n->data.pm4.op[1]   = va_arg(args, Nodep);
            n->data.pm4.op[2]   = va_arg(args, Nodep);
            n->data.pm4.op[3]   = va_arg(args, Nodep);
            break;
        }
        case MF:
        case ML:
        case MR:
        case SUCCESSL:
        case FAILUREL:
        {
            break;
        }
        case IFLTVAR:
        case IFGTVAR:
        {
            n->data.ifv.op1     = va_arg(args, int);
            n->data.ifv.op2     = va_arg(args, int);
            break;
        }
        case IFLTCON:
        case IFGTCON:
        case SET:
        {
            n->data.ifc.op1     = va_arg(args, int);
            n->data.ifc.op2     = va_arg(args, double);
            break;
        }
        case REPEAT:
        {
            n->data.rep.repeat  = va_arg(args, int);
            n->data.rep.op      = va_arg(args, Nodep);
            break;
        }
        case SUCCESSD:
        case FAILURED:
        {
            n->data.fix.op      = va_arg(args, Nodep);
            break;
        }
            
    }
    va_end(args);
    return n;
}

// Forward declare
Status tick(struct Node *bt);

Status update_seqm(struct Node *bt, int8_t count)
{
    // We use the sm4 union member because count ensures we don't
    // run off the end of the array for shorter cases
    DBPRINT("enter  %s %d\n", __func__, count);

    for(int8_t i = bt->data.sm4.idx; i < count; ++i)
    {
        Status s = tick(bt->data.sm4.op[i]);
        if (s == BT_RUNNING)
        {
            bt->data.sm4.idx = i;
            DBPRINT("return %s %d %d\n", __func__, count, s);
            return s;
        }
        if (s == BT_FAILURE)
        {
            bt->data.sm4.idx = 0;
            DBPRINT("return %s %d %d\n", __func__, count, s);
            return s;
        }
    }
    bt->data.sm4.idx = 0;
    DBPRINT("return %s %d %d\n", __func__, count, BT_SUCCESS);
    return BT_SUCCESS;
}

Status update_selm(struct Node *bt, int8_t count)
{
    // We use the sm4 union member because count ensures we don't
    // run off the end of the array for shorter cases
    DBPRINT("enter  %s %d\n", __func__, count);

    for(int8_t i = bt->data.sm4.idx; i < count; ++i)
    {
        Status s = tick(bt->data.sm4.op[i]);
        if (s == BT_RUNNING)
        {
            bt->data.sm4.idx = i;
            DBPRINT("return %s %d %d\n", __func__, count, s);
            return s;
        }
        if (s == BT_SUCCESS)
        {
            bt->data.sm4.idx = 0;
            DBPRINT("return %s %d %d\n", __func__, count, s);
            return s;
        }
    }
    bt->data.sm4.idx = 0;
    DBPRINT("return %s %d %d\n", __func__, count, BT_FAILURE);
    return BT_FAILURE;
}

Status update_probm(struct Node *bt, int8_t count)
{
    // probm is slightly more problematic because of variable
    // number of probability values before the variable number
    // of children
    DBPRINT("enter  %s %d %f\n", __func__, count, bt->data.pm4.p[0]);
    Status s = BT_FAILURE;
    float r = rand_realrange(0,1);
    float p = 0.0f;
    struct Node **children =    count == 2 ? bt->data.pm2.op :
                                count == 3 ? bt->data.pm3.op :
                                             bt->data.pm4.op;
    if (bt->data.pm4.idx < 0)
    {
        for(int8_t i = 0; i < count; ++i)
        {
            // There is one less probability value than there are
            // choices, because the last value is assumed to sum up
            // to 1
            p += (i == (count - 1)) ? 1.0 : bt->data.pm4.p[i];
            if (p > r)
            {
                s = tick(children[i]);
                if (s == BT_RUNNING)
                    bt->data.pm4.idx = i;
                break;
            }
        }
    }
    else
    {
        s = tick(children[bt->data.pm4.idx]);
        if (s != BT_RUNNING)
            bt->data.pm4.idx = -1;
    }
    DBPRINT("return %s %d %f %f %d\n", __func__, count, bt->data.pm4.p[0], r, s);
    return s;
}

Status update_mf()
{
    DBPRINT("%s\n", __func__);

    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 3.0f;
    return BT_SUCCESS;
}
Status update_ml()
{
    DBPRINT("%s\n", __func__);
    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 1.0f;
    return BT_SUCCESS;
}
Status update_mr()
{
    DBPRINT("%s\n", __func__);
    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 2.0f;
    return BT_SUCCESS;
}
Status update_ifltvar(struct Node *bt)
{
    DBPRINT("%s %d %d\n", __func__, bt->data.ifv.op1 , bt->data.ifv.op2);
    return vars[bt->data.ifv.op1] < vars[bt->data.ifv.op2]  ? BT_SUCCESS : BT_FAILURE;
}
Status update_ifgtvar(struct Node *bt)
{
    DBPRINT("%s %d %d\n", __func__, bt->data.ifv.op1 , bt->data.ifv.op2);
    return vars[bt->data.ifv.op1] >= vars[bt->data.ifv.op2] ? BT_SUCCESS : BT_FAILURE;
}
Status update_ifltcon(struct Node *bt)
{
    DBPRINT("%s %d %f\n", __func__, bt->data.ifc.op1 , bt->data.ifc.op2);
    return vars[bt->data.ifc.op1] < bt->data.ifc.op2        ? BT_SUCCESS : BT_FAILURE;
}
Status update_ifgtcon(struct Node *bt)
{
    DBPRINT("%s %d %f\n", __func__, bt->data.ifc.op1 , bt->data.ifc.op2);
    return vars[bt->data.ifc.op1] >= bt->data.ifc.op2       ? BT_SUCCESS : BT_FAILURE;
}
Status update_set(struct Node *bt)
{
    DBPRINT("%s\n %d %f", __func__, bt->data.ifc.op1, bt->data.ifc.op2);
    vars[bt->data.ifc.op1] = bt->data.ifc.op2;
    return BT_SUCCESS;
}
Status update_repeat(struct Node *bt)
{
    DBPRINT("enter  %s %d %d\n", __func__, bt->data.rep.repeat, bt->data.rep.count);
    Status s = tick(bt->data.rep.op);
    if (s == BT_FAILURE)
    {
        bt->data.rep.count  = 0;
        DBPRINT("return %s %d\n", __func__, BT_FAILURE);
        return BT_FAILURE;
    }
    else if (s == BT_SUCCESS)
    {
        bt->data.rep.count --;
        if (bt->data.rep.count == 0)
        {
            DBPRINT("return %s %d\n", __func__, BT_SUCCESS);
            return BT_SUCCESS;
        }
    }
    DBPRINT("return %s %d\n", __func__, BT_RUNNING);
    return BT_RUNNING;
}
Status update_successd(struct Node *bt)
{
    DBPRINT("%s\n", __func__);
    Status s = tick(bt->data.fix.op);
    if (s == BT_RUNNING)
        return BT_RUNNING;
    return BT_SUCCESS;
}
Status update_failured(struct Node *bt)
{
    DBPRINT("%s\n", __func__);
    Status s = tick(bt->data.fix.op);
    if (s == BT_RUNNING)
        return BT_RUNNING;
    return BT_FAILURE;
}

// All nodes execute in three phases, although most have empty init and finish phases.
// When we introduce parallel and non mem composition nodes, these will be necessary to
// deal with the cases when a running node is orphaned and needs to be stopped in an
// orderly way, and to track multiple running nodes.
// With only ..M memory type composition nodes and no par, there is only ever zero or
// one running leaf node (and the composition nodes leading to that leaf).
void init(struct Node *bt)
{
    switch(bt->type)
    {
        case REPEAT:
        {
            bt->data.rep.count  = bt->data.rep.repeat;
            bt->status          = BT_RUNNING;
            return;
        }
        default:
            return;
    }
}
Status update(struct Node *bt)
{
#ifdef KBCOMPILE
    print_sp();
#endif
    switch (bt->type)
    {
        case SEQM2:     return update_seqm(bt, 2);
        case SEQM3:     return update_seqm(bt, 3);
        case SEQM4:     return update_seqm(bt, 4);
        case SELM2:     return update_selm(bt, 2);
        case SELM3:     return update_selm(bt, 3);
        case SELM4:     return update_selm(bt, 4);
        case PROBM2:    return update_probm(bt, 2);
        case PROBM3:    return update_probm(bt, 3);
        case PROBM4:    return update_probm(bt, 4);
        case MF:        return update_mf();
        case ML:        return update_ml();
        case MR:        return update_mr();
        case IFLTVAR:   return update_ifltvar(bt);
        case IFGTVAR:   return update_ifgtvar(bt);
        case IFLTCON:   return update_ifltcon(bt);
        case IFGTCON:   return update_ifgtcon(bt);
        case SET:       return update_set(bt);
        case REPEAT:    return update_repeat(bt);
        case SUCCESSD:  return update_successd(bt);
        case FAILURED:  return update_failured(bt);
        case SUCCESSL:  return BT_SUCCESS;
        case FAILUREL:  return BT_FAILURE;
    }
    return BT_SUCCESS;
}
void finish(struct Node *bt)
{
}

Status tick(struct Node *bt)
{
    if (bt->status != BT_RUNNING)
        init(bt);
    bt->status = update(bt);
    if (bt->status != BT_RUNNING)
        finish(bt);
    DBPRINT("%s, %d\n", __func__, bt->status);
    return bt->status;
}

void print_tree(struct Node *bt, int level)
{
    switch(bt->type)
    {
        case SEQM2:
        {
            printf("%3d seqm2\n",level);
            print_tree(bt->data.sm2.op[0], level+1);
            print_tree(bt->data.sm2.op[1], level+1);
            break;
        }
        case SEQM3:
        {
            printf("%3d seqm3\n",level);
            print_tree(bt->data.sm3.op[0], level+1);
            print_tree(bt->data.sm3.op[1], level+1);
            print_tree(bt->data.sm3.op[2], level+1);
            break;
        }
        case SEQM4:
        {
            printf("%3d seqm4\n",level);
            print_tree(bt->data.sm4.op[0], level+1);
            print_tree(bt->data.sm4.op[1], level+1);
            print_tree(bt->data.sm4.op[2], level+1);
            print_tree(bt->data.sm4.op[3], level+1);
            break;
        }
        case SELM2:
        {
            printf("%3d selm2\n",level);
            print_tree(bt->data.sm2.op[0], level+1);
            print_tree(bt->data.sm2.op[1], level+1);
            break;
        }
        case SELM3:
        {
            printf("%3d selm3\n",level);
            print_tree(bt->data.sm3.op[0], level+1);
            print_tree(bt->data.sm3.op[1], level+1);
            print_tree(bt->data.sm3.op[2], level+1);
            break;
        }
        case SELM4:
        {
            printf("%3d selm4\n",level);
            print_tree(bt->data.sm4.op[0], level+1);
            print_tree(bt->data.sm4.op[1], level+1);
            print_tree(bt->data.sm4.op[2], level+1);
            print_tree(bt->data.sm4.op[3], level+1);
            break;
        }
        case MF:
        {
            printf("%3d mf\n",level);
            break;
        }
        case ML:
        {
            printf("%3d ml\n",level);
            break;
        }
        case MR:
        {
            printf("%3d mr\n",level);
            break;
        }
    }
}




