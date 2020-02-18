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
int GetMinIndex(std::vector<signed long int> vec);
signed long int GetMinValue(std::vector<signed long int> vec);
unsigned char GetMinValueChar(std::vector<unsigned char> vec);
signed long int UpperClamp(signed long int input, signed long int upper_bound);
void PrintVec(std::vector<unsigned char> vec);
void PrintVecInt(std::vector<unsigned short int> vec);
void PrintVecSignedInt(std::vector<signed long int> vec);
int RandIntRange(int lower, int upper);
signed long int SumVec(std::vector<unsigned short int> vec);
#endif
