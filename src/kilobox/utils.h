#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector> // vector
#include <algorithm> // find

std::vector<unsigned char> slice(std::vector<unsigned char> const &v, int m, int n);
std::vector<unsigned char> Int2Vec(unsigned short int input);
unsigned short int Vec2Int(std::vector<unsigned char> input);
bool IsIncludedInVec(std::vector<unsigned char> vec, unsigned char item);
int GetIndexFromVec(std::vector<unsigned char> vec, unsigned char item);
std::vector<unsigned char> IniVec(std::vector<unsigned char> vec);
std::vector<unsigned char> AddVecToAnother(std::vector<unsigned char> vec_to_add, std::vector<unsigned char> vec_existing);
int GetMinIndex(std::vector<unsigned int> vec);
int GetMinValue(std::vector<unsigned int> vec);
unsigned int UpperClamp(unsigned int input, unsigned int upper_bound);
void PrintVec(std::vector<unsigned char> vec);
void PrintVecInt(std::vector<unsigned short int> vec);
int RandIntRange(int lower, int upper);

#endif
