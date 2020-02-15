#include "utils.h"

std::vector<unsigned char> slice(std::vector<unsigned char> const &v, int m, int n) // Get the part of the vector "v" from index m to index n
{
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n + 1;

    std::vector<unsigned char> vec(first, last);
    return vec;
}

// Transform 2-byte int to 2-element char vector
std::vector<unsigned char> Int2Vec(unsigned short int input){
	std::vector<unsigned char> output(2);	
    output[0] = input & 0xFF; // Lower part
    output[1] = (input>>8) & 0xFF; // Higher part
    
    return output;
    
    
}
// Transform 2-element char vector to 2-byte int
unsigned short int Vec2Int(std::vector<unsigned char> input){
    unsigned short int output = 0;
    output = input[0];
    output += (input[1] << 8);    
    return output;
}

// True if item is included in vec
bool IsIncludedInVec(std::vector<unsigned char> vec, unsigned char item){
    std::vector<unsigned char>::iterator it;
    it = std::find(vec.begin(), vec.end(), item);
    bool result = it != vec.end();
    return result;
}

// Get the index of "item" if it is included in "vec", otherwise the result is the last index + 1
int GetIndexFromVec(std::vector<unsigned char> vec, unsigned char item){
    std::vector<unsigned char>::iterator it;
    it = std::find(vec.begin(), vec.end(), item);
    int index = std::distance(vec.begin(), it);
    
    return index;
}

// Initialise a given vector with 0 values, keeping the same size. 
std::vector<unsigned char> IniVec(std::vector<unsigned char> vec){
    std::vector<unsigned char> new_vec(vec.size(),0);
    return new_vec;
}

// Add a vector to the end of another vector
std::vector<unsigned char> AddVecToAnother(std::vector<unsigned char> vec_to_add, std::vector<unsigned char> vec_existing){
    vec_existing.insert(vec_existing.end(), vec_to_add.begin(), vec_to_add.end());
    return vec_existing;
}

int GetMinIndex(std::vector<unsigned int> vec){
    int min_index = std::min_element(vec.begin(), vec.end()) - vec.begin();    
    return min_index;
}

int GetMinValue(std::vector<unsigned int> vec){
    int min_value = *std::min_element(vec.begin(), vec.end());    
    return min_value;
}



unsigned int UpperClamp(unsigned int input, unsigned int upper_bound){
    unsigned int output;
    if (input > upper_bound){
        output = upper_bound;
    }
    else{
        output = input;
    }
    return output;
}

void PrintVec(std::vector<unsigned char> vec){
    printf("Vector Element = [");
    for(int i=0; i<vec.size(); i++){
        if (i % 10 == 0){
            printf(":", vec[i]);
        }
        printf("%d,", vec[i]);
        
    }printf("]\n");
}

void PrintVecInt(std::vector<unsigned short int> vec){
    printf("Vector Element = [");
    for(int i=0; i<vec.size(); i++){
        if (i % 10 == 0){
            printf(":", vec[i]);
        }
        printf("%d,", vec[i]);
        
    }printf("]\n");
}

int RandIntRange(int lower, int upper){ // Generage a random integer from lower to upper
    int num = (std::rand() % (upper - lower + 1)) + lower;  
    return num;
}