#include "function.h"

using namespace std;

// Grid Map
Point GridMap::get_index(Point cell_origin) {
    int x_idx, y_idx;
    Point result;
    x_idx = (cell_origin.x - routing_area_origin.x) / grid_w;
    y_idx = (cell_origin.y - routing_area_origin.y) / grid_h;

    result.x = x_idx, result.y = y_idx;
    return result;
}

// GCell
double GCell::get_cost_for_astar() {
    return 1.1 * g_cost + 1.1 * h_cost + 1.5 * of_cost;
}

double GCell::get_cost_for_net(double alpha, double beta, double gamma) {
    return gamma * g_cost + beta * of_cost + alpha * wire_length;
}

// Net
double Net::get_cost(GridMap &grid_map, vector<vector<GCell>> &gcell_vec) {
    g_cost = 0;
    wire_length = 0;
    of_cost = 0;
    for (int i = 0; i < routing_path.size(); i++) {
        if (i != routing_path.size() - 1) {
            if (routing_path[i + 1].x != routing_path[i].x) {
                wire_length += grid_map.grid_w;
            } else {
                wire_length += grid_map.grid_h;
            }
        }
        of_cost += routing_path[i].of_cost;
    }
    if (routing_path[routing_path.size() - 1].x != routing_path.back().x) {
        wire_length += grid_map.grid_w;
    } else {
        wire_length += grid_map.grid_h;
    }
    g_cost = routing_path[0].g_cost;

    return grid_map.gamma * g_cost + grid_map.alpha * wire_length;
}

void AStar::read_gmp(ifstream &gmp_input, GridMap &grid_map) {
    string dump;
    int idx;

    // chip1
    gmp_input >> dump; //.ra
    gmp_input >> grid_map.routing_area_origin.x >> grid_map.routing_area_origin.y >> grid_map.routing_area_w >> grid_map.routing_area_h;

    gmp_input >> dump; //.g
    gmp_input >> grid_map.grid_w >> grid_map.grid_h;

    gmp_input >> dump; //.c
    gmp_input >> grid_map.chip1_origin.x >> grid_map.chip1_origin.y >> grid_map.chip1_w >> grid_map.chip1_h;
    grid_map.chip1_origin.x += grid_map.routing_area_origin.x;
    grid_map.chip1_origin.y += grid_map.routing_area_origin.y;

    gmp_input >> dump; //.b
    while (gmp_input >> dump, dump != ".c") {
        Point bump_point;
        gmp_input >> bump_point.x >> bump_point.y;
        bump_point.x += grid_map.chip1_origin.x;
        bump_point.y += grid_map.chip1_origin.y;
        grid_map.bump_vec1.push_back(bump_point);
    }

    // chip2
    // .c
    gmp_input >> grid_map.chip2_origin.x >> grid_map.chip2_origin.y >> grid_map.chip2_w >> grid_map.chip2_h;
    grid_map.chip2_origin.x += grid_map.routing_area_origin.x;
    grid_map.chip2_origin.y += grid_map.routing_area_origin.y;

    gmp_input >> dump; //.b
    while (gmp_input >> idx) {
        Point bump_point;
        gmp_input >> bump_point.x >> bump_point.y;
        bump_point.x += grid_map.chip2_origin.x;
        bump_point.y += grid_map.chip2_origin.y;
        grid_map.bump_vec2.push_back(bump_point);
    }
}

void AStar::read_gcl(ifstream &gcl_input, GridMap &grid_map, vector<vector<GCell>> &gcell_vec) {
    string dump;
    int cell_num_horizontal = grid_map.routing_area_w / grid_map.grid_w;
    int cell_num_vertical = grid_map.routing_area_h / grid_map.grid_h;

    gcl_input >> dump; //.ec

    for (int i = 0; i < cell_num_vertical; i++) {
        vector<GCell> row_vec;
        for (int j = 0; j < cell_num_horizontal; j++) {
            GCell gcell;
            gcl_input >> gcell.le_capacity >> gcell.be_capacity;
            gcell.origin.x = j * grid_map.grid_w + grid_map.routing_area_origin.x;
            gcell.origin.y = i * grid_map.grid_h + grid_map.routing_area_origin.y;
            row_vec.push_back(gcell);
        }
        gcell_vec.push_back(row_vec);
    }
}

void AStar::read_cst(ifstream &cst_input, GridMap &grid_map) {
    string dump;
    int cell_num_horizontal = grid_map.routing_area_w / grid_map.grid_w;
    int cell_num_vertical = grid_map.routing_area_h / grid_map.grid_h;

    double max_cost = 0;
    cst_input >> dump >> grid_map.alpha;
    cst_input >> dump >> grid_map.beta;
    cst_input >> dump >> grid_map.gamma;
    cst_input >> dump >> grid_map.delta;
    cst_input >> dump; //.v
    cst_input >> grid_map.via_cost;
    cst_input >> dump; //.l
    for (int i = 0; i < cell_num_vertical; i++) {
        vector<double> row_cost;
        for (int j = 0; j < cell_num_horizontal; j++) {
            double cost;
            cst_input >> cost;
            row_cost.push_back(cost);

            if (cost > max_cost)
                max_cost = cost;
        }
        grid_map.cost_map1.push_back(row_cost);
    }
    cst_input >> dump; //.l
    for (int i = 0; i < cell_num_vertical; i++) {
        vector<double> row_cost;
        for (int j = 0; j < cell_num_horizontal; j++) {
            double cost;
            cst_input >> cost;
            row_cost.push_back(cost);

            if (cost > max_cost)
                max_cost = cost;
        }
        grid_map.cost_map2.push_back(row_cost);
    }
    grid_map.max_cell_cost = max_cost;
}

void AStar::init_gcell(vector<vector<GCell>> &gcell_vec, GCell *target_ptr) {
    for (int i = 0; i < gcell_vec.size(); i++) {
        for (int j = 0; j < gcell_vec[i].size(); j++) {
            gcell_vec[i][j].h_cost = (abs(gcell_vec[i][j].origin.x - target_ptr->origin.x) + abs(gcell_vec[i][j].origin.y - target_ptr->origin.y));
            gcell_vec[i][j].g_cost = INITIAL_COST; // infinity
            gcell_vec[i][j].of_cost = 0;
            gcell_vec[i][j].wire_length = 0;
            gcell_vec[i][j].queued = false;
            gcell_vec[i][j].father = nullptr;
            gcell_vec[i][j].closed = false;
        }
    }
}

void AStar::a_star_route(Point &source, Point &target, vector<vector<GCell>> &gcell_vec, GridMap &grid_map, vector<Point> &routing_path) {
    vector<GCell *> open_list;
    Point source_idx, curr_cell_idx, target_idx;
    GCell *curr_cell_ptr, *source_ptr, *target_ptr;
    double alpha, beta, gamma, delta;
    alpha = grid_map.alpha;
    beta = grid_map.beta;
    gamma = grid_map.gamma;
    delta = grid_map.delta;

    source_idx = grid_map.get_index(source);
    target_idx = grid_map.get_index(target);
    source_ptr = &gcell_vec[source_idx.y][source_idx.x];
    target_ptr = &gcell_vec[target_idx.y][target_idx.x];
    source_ptr->layer = M1;
    target_ptr->layer = M1;

    init_gcell(gcell_vec, target_ptr);

    source_ptr->g_cost = grid_map.cost_map1[curr_cell_idx.y][curr_cell_idx.x];
    open_list.push_back(source_ptr);
    source_ptr->queued = true;

    static constexpr int dx[4] = {1, -1, 0, 0};
    static constexpr int dy[4] = {0, 0, 1, -1};
    static constexpr bool newLayer[4] = {M2, M2, M1, M1};

    while (!open_list.empty()) {
        curr_cell_ptr = open_list.back();
        open_list.pop_back();
        curr_cell_idx = grid_map.get_index(curr_cell_ptr->origin);

        if (curr_cell_ptr == target_ptr) {
            GCell *father_ptr = curr_cell_ptr;
            father_ptr->origin.g_cost = father_ptr->g_cost;
            father_ptr->origin.h_cost = father_ptr->h_cost;
            routing_path.push_back(father_ptr->origin);
            while (father_ptr->father != nullptr) {

                if (father_ptr->father->origin.y != father_ptr->origin.y)
                    father_ptr->be_occupied++;
                else
                    father_ptr->le_occupied++;

                father_ptr = father_ptr->father;
                father_ptr->origin.g_cost = father_ptr->g_cost;
                father_ptr->origin.h_cost = father_ptr->h_cost;
                father_ptr->origin.of_cost = father_ptr->of_cost;
                routing_path.push_back(father_ptr->origin);
            }
            return;
        }
        for (int k = 0; k < 4; ++k) {
            int next_x = curr_cell_idx.x + dx[k];
            int next_y = curr_cell_idx.y + dy[k];
            if (next_x < 0 || next_y < 0 || next_x >= grid_map.routing_area_w / grid_map.grid_w || next_y >= grid_map.routing_area_h / grid_map.grid_h)
                continue;

            GCell *neighbor_ptr = &gcell_vec[next_y][next_x];

            // via weight cost
            double via_weight_cost = grid_map.delta * grid_map.via_cost +
                        grid_map.gamma * 0.5 * (grid_map.cost_map1[curr_cell_idx.y][curr_cell_idx.x] + grid_map.cost_map2[curr_cell_idx.y][curr_cell_idx.x]);

            // tentative g-score
            double tentative_gscore = curr_cell_ptr->g_cost + ((dx[k] != 0)
                                                              ? grid_map.cost_map2[next_y][next_x]
                                                              : grid_map.cost_map1[next_y][next_x]);
            // via cost
            if ((dx[k] != 0 && curr_cell_ptr->layer == M1) || (dx[k] == 0 && curr_cell_ptr->layer != M1))
                tentative_gscore += via_weight_cost;

            // overflow cost
            double overflow_cost = 0;
            if (dx[k] != 0) {
                int occupied = neighbor_ptr->le_occupied + 1,
                    capacity = neighbor_ptr->le_capacity;
                if (occupied > capacity)
                    overflow_cost = (occupied - capacity) * 0.5 * grid_map.max_cell_cost;
            } else {
                int occupied = neighbor_ptr->be_occupied + 1,
                    capacity = neighbor_ptr->be_capacity;
                if (occupied > capacity)
                    overflow_cost = (occupied - capacity) * 0.5 * grid_map.max_cell_cost;
            }

            if (!neighbor_ptr->closed && !neighbor_ptr->queued) {
                double fscore = gamma * tentative_gscore + alpha * neighbor_ptr->wire_length + beta * overflow_cost;
                if (fscore < neighbor_ptr->get_cost_for_net(alpha, beta, gamma) || neighbor_ptr->g_cost == INITIAL_COST) {
                    neighbor_ptr->of_cost = overflow_cost;
                    neighbor_ptr->g_cost = tentative_gscore;
                    neighbor_ptr->wire_length = (dx[k] != 0 ? grid_map.grid_w : grid_map.grid_h) + curr_cell_ptr->wire_length;
                    neighbor_ptr->layer = newLayer[k];
                    neighbor_ptr->father = curr_cell_ptr;
                    erase_cell(open_list, neighbor_ptr);
                    neighbor_ptr->queued = false;
                }
                if (!neighbor_ptr->queued) {
                    insert_cell(open_list, neighbor_ptr, alpha, beta, gamma, delta);
                    neighbor_ptr->queued = true;
                }
            }
        }
        // remove cell from queue
        curr_cell_ptr->closed = true;
        curr_cell_ptr->queued = false;
    }
}

void AStar::output_result(ofstream &lg_output, vector<vector<Point>> &routing_paths, GridMap &grid_map) {
    for (int i = 0; i < routing_paths.size(); i++) {
        lg_output << "n" << i + 1 << "\n";
        Point source = routing_paths[i].back(), target = routing_paths[i].front(), next_point, curr_point;
        Point wire_start, wire_end;
        string layer;
        next_point = routing_paths[i][routing_paths[i].size() - 2];

        if (next_point.x == source.x + grid_map.grid_w || next_point.x == source.x - grid_map.grid_w) { // right or left
            lg_output << "via\n";
            layer = "M2";
        } else {
            layer = "M1";
        }
        wire_start.x = source.x, wire_start.y = source.y;
        lg_output << layer << " " << wire_start.x << " " << wire_start.y;

        for (int j = routing_paths[i].size() - 2; j > 0; j--) {
            curr_point = routing_paths[i][j];
            next_point = routing_paths[i][j - 1];

            bool right_or_left = (next_point.x == curr_point.x + grid_map.grid_w || next_point.x == curr_point.x - grid_map.grid_w);
            bool up_or_bottom = (next_point.y == curr_point.y + grid_map.grid_h || next_point.y == curr_point.y - grid_map.grid_h);

            if(layer == "M1" && right_or_left || layer == "M2" && up_or_bottom) {
                wire_end.x = curr_point.x;
                wire_end.y = curr_point.y;
                lg_output << " " << wire_end.x << " " << wire_end.y << "\n";
                lg_output << "via\n";
                layer = (layer == "M1") ? "M2" : "M1"; // switch layer
                wire_start.x = curr_point.x;
                wire_start.y = curr_point.y;
                lg_output << layer << " " << wire_start.x << " " << wire_start.y;
            }
        }
        lg_output << " " << target.x << " " << target.y << "\n";
        if (layer == "M2") {
            lg_output << "via\n";
        }
        lg_output << ".end\n";
    }
}

void AStar::insert_cell(vector<GCell *> &open_list, GCell *curr_cell_ptr, double &alpha, double &beta, double &gamma, double &delta) {
    double param_h = 0;
    param_h = max(max(max(max(alpha, beta), gamma), delta), 0.0) / 2;

    double curr_cost = beta * curr_cell_ptr->of_cost + gamma * curr_cell_ptr->g_cost + alpha * (curr_cell_ptr->wire_length) + param_h * curr_cell_ptr->h_cost;

    if (!curr_cell_ptr)
        return;

    for (int i = 0; i < open_list.size(); i++) {
        double comp_cost;
        GCell *cell_ptr = open_list[i];
        comp_cost = beta * cell_ptr->of_cost + gamma * cell_ptr->g_cost + alpha * (cell_ptr->wire_length) + param_h * cell_ptr->h_cost;
        if (curr_cost > comp_cost) {
            open_list.insert(open_list.begin() + i, curr_cell_ptr);
            return;
        } else if (curr_cost == comp_cost) {
            if (curr_cell_ptr->h_cost < cell_ptr->h_cost) {
                if (i < open_list.size()) {
                    open_list.insert(open_list.begin() + i + 1, curr_cell_ptr);
                } else
                    open_list.push_back(curr_cell_ptr);
            } else {
                open_list.insert(open_list.begin() + i, curr_cell_ptr);
            }
            return;
        }
    }
    open_list.push_back(curr_cell_ptr);
}

void AStar::erase_cell(vector<GCell *> &open_list, GCell *curr_cell_ptr) {
    if (open_list.empty())
        return;
    for (int i = 0; i < open_list.size(); i++) {
        if (open_list[i]->origin.x == curr_cell_ptr->origin.x && open_list[i]->origin.y == curr_cell_ptr->origin.y) {
            open_list.erase(open_list.begin() + i);
            return;
        }
    }
}
