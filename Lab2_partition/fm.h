#ifndef FM_H
#define FM_H

#include "partition.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <random>
using namespace std;

class FM {
public:
    // constructor and destructor
    FM(int num_cells, int num_nets, vector<Net*>& net_list, vector<Cell*>& cell_list, int max_num_pins)
        : num_cells_(num_cells), num_nets_(num_nets), net_list_(net_list), cell_list_(cell_list), max_num_pins(max_num_pins){
        part_size_[0] = 0;
        part_size_[1] = 0;
        lower_bound_ = static_cast<int>(floor(num_cells * 0.45));
        upper_bound_ = static_cast<int>(ceil (num_cells * 0.55));
        buckets_[0] = new BucketList(max_num_pins);
        buckets_[1] = new BucketList(max_num_pins);
    }
    ~FM() {
        for (Cell* c : cell_list_)  delete c;
        for (Net* n : net_list_) delete n;
        delete buckets_[0];
        delete buckets_[1];
    }

    int GetPartSize(const int part) const { return part_size_[part]; }
    void Result(ofstream &fout);

    // solver
    void Solve();
    void InitPartition();
    void InitNetPartSize();
    void InitCell();
    Cell* FindMaxGainCell();
    bool IsMoveLegal(Cell* move_cell);
    void MoveCell(Cell* move_cell);
    void UpdateCellGain(Cell* move_cell);
    void UndoMoves(int max_index);


    // others
    int CalculateCutSize();

private:
    // basic
    int num_cells_;
    int num_nets_;
    int max_num_pins;
    int part_size_[2];
    double lower_bound_;
    double upper_bound_;
    vector<Cell*> cell_list_;
    vector<Net*> net_list_;

    // fm algorithm
    vector<int> gain_stack_;
    vector<Cell*> move_stack_;
    BucketList* buckets_[2];
};

#endif
