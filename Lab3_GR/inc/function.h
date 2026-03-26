#ifndef _FUNCTION_H
#define _FUNCTION_H

#define INITIAL_COST -1


#include <string>
#include <algorithm>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;


#define M1 true
#define M2 false


class Point{
public:
    int x = 0, y = 0;
    double g_cost = 0, h_cost = 0, of_cost = 0;
};

class GridMap{
public:
    int routing_area_w, routing_area_h;
    int grid_w, grid_h;
    double alpha, beta, gamma, delta;
    Point routing_area_origin;
    Point chip1_origin, chip2_origin;//1 is source, 2 is target
    int chip1_w, chip1_h, chip2_w, chip2_h;
    double via_cost, max_cell_cost;
    vector<Point> bump_vec1, bump_vec2;
    vector<vector<double>> cost_map1, cost_map2;

    Point get_index(Point cell_origin);
};

class GCell{
public:
    double g_cost = 0, h_cost = 0, of_cost = 0, wire_length = 0;
    int le_capacity, be_capacity;
    int le_occupied, be_occupied;
    bool closed = false;
    bool queued = false;
    Point origin;
    GCell *father = nullptr;
    bool layer;

    double get_cost_for_astar();
    double get_cost_for_net(double alpha, double beta, double gamma);
};

class Net{
public:
    int index;
    vector<Point> routing_path;
    double g_cost = 0;
    double of_cost = 0;
    double wire_length = 0;

    double get_cost(GridMap &grid_map, vector<vector<GCell>> &gcell_vec);
};

class AStar{
public:
    vector<vector<GCell>> gcell_vec;
    GridMap grid_map;
    vector<Point> routing_path;

    void read_gmp(ifstream &gmp_input, GridMap &grid_map);
    void read_gcl(ifstream &gcl_input, GridMap &grid_map, vector<vector<GCell>> &gcell_vec);
    void read_cst(ifstream &cst_input, GridMap &grid_map);
    void init_gcell(vector<vector<GCell>> &gcell_vec, GCell *target_ptr);
    void a_star_route(Point &source, Point &target, vector<vector<GCell>> &gcell_vec, GridMap &grid_map, vector<Point> &routing_path);
    void output_result(ofstream &lg_output, vector<vector<Point>> &routing_paths, GridMap &grid_map);
    void insert_cell(vector<GCell*> &open_list, GCell *cell_now, double &alpha, double &beta, double &gamma, double &delta);
    void erase_cell(vector<GCell*> &open_list, GCell *cell_now);

};


#endif
