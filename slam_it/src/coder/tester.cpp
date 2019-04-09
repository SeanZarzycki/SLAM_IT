#include "base64.h"
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{
    if(argc != 2)
    {
        cout << "Wrong number of inputs given, should be 1\n";
        return 0;
    }
    char* file = argv[1];
    ifstream myfile;
    myfile.open(file, ios::binary);
    if(!myfile.is_open())
    {
        cout << "Could not read file\n";
        return 0;
    }

    streampos size;
    char * memblock;
    string s = "";
    

   // std::string encoded = base64_encode(reinterpret_cast<const unsigned char*>(s.c_str()), s.length());
    //std::string decoded = base64_decode(encoded);

    std::string encoded = base64_encode(memblock, size.);

    std::cout << "encoded: " << std::endl << encoded << std::endl << std::endl;
    std::cout << "decoded: " << std::endl << decoded << std::endl;

    delete[] memblock;

    return 0;
}