
#ifndef BTS_H
#define BTS_H


typedef enum
{
    SEQM2,      // 0
    SEQM3,      // 1
    SEQM4,      // 2
    SELM2,      // 3
    SELM3,      // 4
    SELM4,      // 5
    PROBM2,     // 6
    PROBM3,     // 7
    PROBM4,     // 8
    MF,         // 9
    ML,         // 10
    MR,         // 11
    IFLTVAR,    // 12
    IFGTVAR,    // 13
    IFLTCON,    // 14
    IFGTCON,    // 15
    SET,        // 16
    REPEAT,     // 17
    SUCCESSD,   // 18
    FAILURED,   // 19
    SUCCESSL,   // 20
    FAILUREL    // 21
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
void print_tree(struct Node *bt, int level);

#define DEBUG

#ifdef DEBUG
#define DBPRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DBPRINT(fmt, ...)
#endif

#endif