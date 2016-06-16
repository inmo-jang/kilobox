
#ifndef BTS_H
#define BTS_H


typedef enum
{
    SEQM2,
    SEQM3,
    SEQM4,
    SELM2,
    SELM3,
    SELM4,
    PROBM2,
    PROBM3,
    PROBM4,
    MF,
    ML,
    MR,
    IFLTVAR,
    IFGTVAR,
    IFLTCON,
    IFGTCON,
    SET,
    REPEAT,
    SUCCESSD,
    FAILURED,
    SUCCESSL,
    FAILUREL
} Nodetype;

typedef enum
{
    BT_INVALID = 0,
    BT_FAILURE,
    BT_SUCCESS,
    BT_RUNNING
} Status;

struct Node;
void            set_vars(float *v);
struct Node     *newnode(Nodetype type,...);
Status          tick(struct Node *bt);


#endif