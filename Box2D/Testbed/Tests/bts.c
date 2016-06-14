
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>


#include "bts.h"



float rand_realrange(float low, float high)
{
    return low;
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

union Ndata
{

    struct SM2  sm2;
    struct SM3  sm3;
    struct SM4  sm4;
    struct PM2  pm2;
    struct PM3  pm3;
    struct PM4  pm4;
};

struct Node
{
    Nodetype    type;
    Status      status;
    union Ndata data;
};



// Return the size in bytes of the node type.
// Currently just the size that sizeof() gives, which
// is the largest size, but will be possible to tune this
int nsize(Nodetype type)
{
    return sizeof(struct Node);
}

// Return a pointer to a new node of type type, with space allocated
// and fields filled in
struct Node *newnode(Nodetype type,...)
{
    va_list args;
    va_start(args, type);
    
    // All fields get zeroed, this means status starts at INVALID
    struct Node *n = (struct Node*)calloc(1, nsize(type));
    n->type = type;
    
    switch(type)
    {
        case SEQM2:
        case SELM2:
        {
            n->data.sm2.op[0] = va_arg(args, struct Node*);
            n->data.sm2.op[1] = va_arg(args, struct Node*);
            break;
        }
        case SEQM3:
        case SELM3:
        {
            n->data.sm3.op[0] = va_arg(args, struct Node*);
            n->data.sm3.op[1] = va_arg(args, struct Node*);
            n->data.sm3.op[2] = va_arg(args, struct Node*);
            break;
        }
        case SEQM4:
        case SELM4:
        {
            n->data.sm4.op[0] = va_arg(args, struct Node*);
            n->data.sm4.op[1] = va_arg(args, struct Node*);
            n->data.sm4.op[2] = va_arg(args, struct Node*);
            n->data.sm4.op[3] = va_arg(args, struct Node*);
            break;
        }
        case PROBM2:
        {
            n->data.pm2.idx     = -1;
            n->data.pm2.p[0]    = va_arg(args, double);
            n->data.pm2.op[0]   = va_arg(args, struct Node*);
            n->data.pm2.op[1]   = va_arg(args, struct Node*);
            break;
        }
        case PROBM3:
        {
            n->data.pm3.idx     = -1;
            n->data.pm3.p[0]    = va_arg(args, double);
            n->data.pm3.p[1]    = va_arg(args, double);
            n->data.pm3.op[0]   = va_arg(args, struct Node*);
            n->data.pm3.op[1]   = va_arg(args, struct Node*);
            n->data.pm3.op[2]   = va_arg(args, struct Node*);
            break;
        }
        case PROBM4:
        {
            n->data.pm4 .idx     = -1;
            n->data.pm4.p[0]    = va_arg(args, double);
            n->data.pm4.p[1]    = va_arg(args, double);
            n->data.pm4.p[2]    = va_arg(args, double);
            n->data.pm4.op[0]   = va_arg(args, struct Node*);
            n->data.pm4.op[1]   = va_arg(args, struct Node*);
            n->data.pm4.op[2]   = va_arg(args, struct Node*);
            n->data.pm4.op[3]   = va_arg(args, struct Node*);
            break;
        }
        case MF:
        case ML:
        case MR:
        {
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
    for(int8_t i = bt->data.sm4.idx; i < count; ++i)
    {
        Status s = tick(bt->data.sm4.op[i]);
        if (s == BT_RUNNING)
        {
            bt->data.sm4.idx = i;
            return s;
        }
        if (s == BT_FAILURE)
        {
            bt->data.sm4.idx = 0;
            return s;
        }
    }
    bt->data.sm4.idx = 0;
    return BT_SUCCESS;
}

Status update_selm(struct Node *bt, int8_t count)
{
    // We use the sm4 union member because count ensures we don't
    // run off the end of the array for shorter cases
    for(int8_t i = bt->data.sm4.idx; i < count; ++i)
    {
        Status s = tick(bt->data.sm4.op[i]);
        if (s == BT_RUNNING)
        {
            bt->data.sm4.idx = i;
            return s;
        }
        if (s == BT_SUCCESS)
        {
            bt->data.sm4.idx = 0;
            return s;
        }
    }
    bt->data.sm4.idx = 0;
    return BT_FAILURE;
}

Status update_probm(struct Node *bt, int8_t count)
{
    // probm is slightly more problematic because of variable
    // number of probability values before the variable number
    // of children
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
            p += bt->data.pm4.p[i];
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
    return s;
}

Status update_mf()
{
    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 3.0f;
    return BT_SUCCESS;
}
Status update_ml()
{
    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 1.0f;
    return BT_SUCCESS;
}
Status update_mr()
{
    if (vars[0] > 0.0f)
        return BT_RUNNING;
    vars[0] = 2.0f;
    return BT_SUCCESS;
}


void init(struct Node *bt)
{
}
Status update(struct Node *bt)
{
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
    return bt->status;
}

struct Node *test()
{
    return
    newnode(SEQM2,
        newnode(PROBM2, 0.3,
            newnode(MF),
            newnode(ML)
        ),
        newnode(ML)
    );
}





