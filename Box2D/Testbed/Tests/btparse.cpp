
#include "btparse.h"

extern "C" {
#include "bts.h"
}

#include <iostream>
#include <string>
#include <regex>
#include <stdlib.h>



// Regex to match lower case alphabetic words with optional single digit ending,
// and numbers possibly prefixed
// with + or - and possibly having a decimal point with one or more digits after it
std::regex e("[a-z]+[234]?|[-+]?[0-9]+\\.?[0-9]*");



std::string words[]
{
    "seqm2",
    "seqm3",
    "seqm4",
    "selm2",
    "selm3",
    "selm4",
    "probm2",
    "probm3",
    "probm4",
    "mf",
    "ml",
    "mr",
    "ifltvar",
    "ifgtvar",
    "ifltcon",
    "ifgtcon",
    "set",
    "repeat",
    "successd",
    "failured",
    "successl",
    "failurel"
};

std::regex_token_iterator<std::string::iterator> rend;
std::regex_token_iterator<std::string::iterator> a;

std::string check(std::string msg)
{
    if (a == rend)
    {
        printf("%s\n", msg.c_str());
        exit(1);
    }
    std::string s = *a;
    printf("<%s>", s.c_str());

    return s;
}

struct Node *pt()
{
    // This gives us a stream of tokens which are either words or
    // signed decimal numbers (no exponents).

    auto s = check("Unexpected end of tokens parsing node\n");
    a++;
    if (s == words[SEQM2])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        return newnode(SEQM2, op1, op2);
    }
    else if (s == words[SEQM3])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        return newnode(SEQM3, op1, op2, op3);
    }
    else if (s == words[SEQM4])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        struct Node *op4 = pt();
        return newnode(SEQM4, op1, op2, op3, op4);
    }
    else if (s == words[SELM2])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        return newnode(SELM2, op1, op2);
    }
    else if (s == words[SELM3])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        return newnode(SELM3, op1, op2, op3);
    }
    else if (s == words[SELM4])
    {
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        struct Node *op4 = pt();
        return newnode(SELM4, op1, op2, op3, op4);
    }
    else if (s == words[PROBM2])
    {
        s = check("Unexpected end of tokens parsing probm2 p1\n");
        a++;
        double p1 = atof(s.c_str());
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        return newnode(PROBM2, p1, op1, op2);
    }
    else if (s == words[PROBM3])
    {
        s = check("Unexpected end of tokens parsing probm3 p1\n");
        a++;
        double p1 = atof(s.c_str());
        s = check("Unexpected end of tokens parsing probm3 p2\n");
        a++;
        double p2 = atof(s.c_str());
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        return newnode(PROBM3, p1, p2, op1, op2, op3);
    }
    else if (s == words[PROBM4])
    {
        s = check("Unexpected end of tokens parsing probm4 p1\n");
        a++;
        double p1 = atof(s.c_str());
        s = check("Unexpected end of tokens parsing probm4 p2\n");
        a++;
        double p2 = atof(s.c_str());
        s = check("Unexpected end of tokens parsing probm4 p3\n");
        a++;
        double p3 = atof(s.c_str());
        struct Node *op1 = pt();
        struct Node *op2 = pt();
        struct Node *op3 = pt();
        struct Node *op4 = pt();
        return newnode(PROBM4, p1, p2, p3, op1, op2, op3, op4);
    }
    else if (s == words[MF])
    {
        return newnode(MF);
    }
    else if (s == words[ML])
    {
        return newnode(ML);
    }
    else if (s == words[MR])
    {
        return newnode(MR);
    }
    else if (s == words[IFLTVAR])
    {
        s = check("Unexpected end of tokens parsing ifltvar op1\n");
        a++;
        int op1 = atol(s.c_str());
        s = check("Unexpected end of tokens parsing ifltvar op2\n");
        a++;
        int op2 = atol(s.c_str());
        return newnode(IFLTVAR, op1, op2);
    }
    else if (s == words[IFGTVAR])
    {
        s = check("Unexpected end of tokens parsing ifgtvar op1\n");
        a++;
        int op1 = atol(s.c_str());
        s = check("Unexpected end of tokens parsing ifgtvar op2\n");
        a++;
        int op2 = atol(s.c_str());
        return newnode(IFGTVAR, op1, op2);
    }
    else if (s == words[IFLTCON])
    {
        s = check("Unexpected end of tokens parsing ifltcon op1\n");
        a++;
        int op1 = atol(s.c_str());
        s = check("Unexpected end of tokens parsing ifltcon op2\n");
        a++;
        double op2 = atof(s.c_str());
        return newnode(IFLTCON, op1, op2);
    }
    else if (s == words[IFGTCON])
    {
        s = check("Unexpected end of tokens parsing ifgtcon op1\n");
        a++;
        int op1 = atol(s.c_str());
        s = check("Unexpected end of tokens parsing ifgtcon op2\n");
        a++;
        double op2 = atof(s.c_str());
        return newnode(IFGTCON, op1, op2);
    }
    else if (s == words[SET])
    {
        s = check("Unexpected end of tokens parsing set op1\n");
        a++;
        int op1 = atol(s.c_str());
        s = check("Unexpected end of tokens parsing set op2\n");
        a++;
        double op2 = atof(s.c_str());
        return newnode(SET, op1, op2);
    }
    else if (s == words[REPEAT])
    {
        s = check("Unexpected end of tokens parsing repeat\n");
        a++;
        int reps = atol(s.c_str());
        return newnode(REPEAT, reps, pt());
    }
    else if (s == words[SUCCESSD])
    {
        return newnode(SUCCESSD, pt());
    }
    else if (s == words[FAILURED])
    {
        return newnode(FAILURED, pt());
    }
    else if (s == words[SUCCESSL])
    {
        return newnode(SUCCESSL);
    }
    else if (s == words[FAILUREL])
    {
        return newnode(FAILUREL);
    }
    return nullptr;
}

struct Node *parse_tree(std::string s)
{
    a = std::regex_token_iterator<std::string::iterator>(s.begin(), s.end(), e);

    struct Node *n = pt();
    return n;
}

