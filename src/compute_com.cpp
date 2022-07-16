#include "ros/ros.h"

#include <iostream>

// class link {
//     public:
//         std::string name;
//         int sister;
//         int child;
// };

typedef struct {
    std::string name; // name of the link
    int sister; // id of the sister link
    int child; // id of the child node
    int mother; // id of the parent node
    int m; // mass of link

} ulink;

// Prints all link names
void print_link_names(ulink *links, int j) {
    if (j != -1) {
        std::cout << links[j].name + '\n';
        print_link_names(links, links[j].child);
        print_link_names(links, links[j].sister);
    }
}

// Returns the total mass of the robot
int total_mass(ulink *links, int j) {
    if (j == -1) return 0;
    else return links[j].m + 
        total_mass(links, links[j].sister) + 
        total_mass(links, links[j].child);
}

int main(int argc, char **argv) {

    ulink links[13]; 

    // base_link
    links[0].name = "BODY";
    links[0].sister = -1;
    links[0].child = 1;
    links[0].mother = -1; 
    links[0].m = 1;
    
    // r_hip_yaw
    links[1].name = "RLEG_J0";
    links[1].sister = 7;
    links[1].child = 2;
    links[1].mother = 0; 
    links[1].m = 1;

    // r_hip_roll
    links[2].name = "RLEG_J1";
    links[2].sister = -1;
    links[2].child = 3;
    links[2].mother = 1; 
    links[2].m = 1;
  
    // r_hip_pitch
    links[3].name = "RLEG_J2";
    links[3].sister = -1;
    links[3].child = 4;
    links[3].mother = 2; 
    links[3].m = 1;

    // r_knee_pitch
    links[4].name = "RLEG_J3";
    links[4].sister = -1;
    links[4].child = 5;
    links[4].mother = 3; 
    links[4].m = 1;

    // r_ankle_pitch
    links[5].name = "RLEG_J4";
    links[5].sister = -1;
    links[5].child = 6;
    links[5].mother = 4; 
    links[5].m = 1;

    // r_ankle_roll
    links[6].name = "RLEG_J5";
    links[6].sister = -1;
    links[6].child = -1;
    links[6].mother = 5; 
    links[6].m = 1;

    // l_hip_yaw
    links[7].name = "LLEG_J0";
    links[7].sister = -1;
    links[7].child = 8;
    links[7].mother = 0;    
    links[7].m = 1;

    // l_hip_roll
    links[8].name = "LLEG_J1";
    links[8].sister = -1;
    links[8].child = 9;
    links[8].mother = 7;    
    links[8].m = 1;

    // l_hip_pitch
    links[9].name = "LLEG_J2";
    links[9].sister = -1;
    links[9].child = 10;
    links[9].mother = 8;  
    links[9].m = 1;

    // l_knee_pitch
    links[10].name = "LLEG_J3";
    links[10].sister = -1;
    links[10].child = 11;
    links[10].mother = 9;  
    links[10].m = 1;

    // l_ankle_pitch
    links[11].name = "LLEG_J4";
    links[11].sister = -1;
    links[11].child = 12;
    links[11].mother = 10;  
    links[11].m = 1;

    // l_ankle_roll
    links[12].name = "LLEG_J5";
    links[12].sister = -1;
    links[12].child = -1;
    links[12].mother = 11;  
    links[12].m = 1;
    
    print_link_names(links, 0);

    std::cout << total_mass(links, 0) + "\n";

    return 0;
}
