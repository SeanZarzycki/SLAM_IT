

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

using namespace std;

int parseHighArgument(char* arg);
int run(vector<char*> argv, Eigen::Matrix<float, 4, 4> &s2d);

int mode;

int main(int argc, char** argv)
{
    // default to normal calulation
    mode = 0;

    vector<char*> pars;
    pars.push_back(argv[0]);
    for(int i = 1;i < argc;i++)
    {
        if(parseHighArgument(argv[i]) == -1)
        {
            pars.push_back(argv[i]);
        }
    }

    int pass = 0;
    if(mode == 0)
    {
        Eigen::Matrix<float, 4, 4> trans;
        pass = run(pars, trans);
        cout << trans << endl;
    }
    else
    {
        
    }
    


    return pass;
}





int parseHighArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];
	
	if(1==sscanf(arg,"mode=%d", &option))
    {
        mode = option;
        return 0;
    }
    /*
    if(1==sscanf(arg,"-%s", buf))
    {
        if(strcmp(buf, "c") == 0 || strcmp(buf, "-colors") == 0)
        {

            return 0;
        }
        if(strcmp(buf, "k") == 0 || strcmp(buf, "-keys") == 0)
        {
            
            return 0;
        }
    }
    */

    return -1;
}