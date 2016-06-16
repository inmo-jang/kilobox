#ifndef BTPARSE_H
#define BTPARSE_H

#include <string>

extern "C" {
#include "bts.h"
}

struct Node *parse_tree(std::string s);

#endif