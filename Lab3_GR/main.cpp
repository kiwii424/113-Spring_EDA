#include "function.h"

#include <ctime>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char *argv[]){

    if (argc != 5){
        cerr << "Usage: " << argv[0] << " <gmp_path> <gcl_path> <cst_path> <lg_path>\n";
        return 1;
    }

    clock_t start, end;
    start = clock();

    cout << "start" << endl;

    ifstream gmp_input(argv[1]), gcl_input(argv[2]), cst_input(argv[3]);
    ofstream lg_output(argv[4]);

    // ====== read input ======
    GridMap grid_map;
    vector<vector<GCell>> gcell_vec;
    AStar a_star;

    a_star.read_gmp(gmp_input, grid_map);
    a_star.read_gcl(gcl_input, grid_map, gcell_vec);
    a_star.read_cst(cst_input, grid_map);


    // ====== routing ======
    vector<vector<Point>> routing_paths;
    vector<Net> net_vec;

    //reset gcell edge occupied
    for(int i=0; i<gcell_vec.size(); i++){
        for(int j=0; j<gcell_vec[i].size(); j++){
            gcell_vec[i][j].le_occupied = 0;
            gcell_vec[i][j].be_occupied = 0;
        }
    }

    for(int i=0; i<grid_map.bump_vec1.size(); i++){
        Net tmp_net;
        tmp_net.index = i+1;
        vector<Point> routing_path;
        a_star.a_star_route(grid_map.bump_vec1[i], grid_map.bump_vec2[i], gcell_vec, grid_map, routing_path);
        tmp_net.routing_path = routing_path;
        net_vec.push_back(tmp_net);
        routing_paths.push_back(routing_path);
    }

    a_star.output_result(lg_output, routing_paths, grid_map);

    gmp_input.close();
    gcl_input.close();
    cst_input.close();
    lg_output.close();

    end = clock();
    double duration = double(end - start) / CLOCKS_PER_SEC;
    cout << "Time taken: " << duration << " seconds" << endl;
    cout << "end" << endl;

    return 0;
}
