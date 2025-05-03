#include "fm.h"

void FM::Result(ofstream &fout) {
    for (int i = 0; i < num_cells_; ++i){
        fout << cell_list_[i] -> GetPart() << endl;
    }
    cout << "Cutsize: " << this->CalculateCutSize() << endl << endl;
}

/* ---------- solver ---------- */
void FM::Solve() {

    this->InitPartition();
    this->InitNetPartSize();

    int index_constrain = -1, max_index = 0;
    while (max_index >= 0) {
        this->InitCell();

        gain_stack_.clear();
        move_stack_.clear();
        vector<Cell *> temp;
        int move_cnt = 0, total_gain = 0, max_gain = INT_MIN;
        max_index = -1;
        bool bug_flag = false;
        while (index_constrain == -1 || move_cnt < index_constrain) {

            Cell *move_cell = this->FindMaxGainCell();
            if (move_cell == nullptr) break;

            bool from = move_cell->GetPart();
            buckets_[from]->DeleteMaxGainNode();

            if (move_cell->IsLock()) {
                bug_flag = true;
                break;
            }
            else if (this->IsMoveLegal(move_cell)) {
                move_cnt++;
                gain_stack_.push_back(move_cell->GetGain());
                move_stack_.push_back(move_cell);

                move_cell->Lock();
                this->UpdateCellGain(move_cell);
                this->MoveCell(move_cell);

                total_gain += move_cell->GetGain();
                if (total_gain > max_gain && part_size_[0] >= lower_bound_ && part_size_[0] <= upper_bound_) {
                    max_gain = total_gain;
                    max_index = move_cnt;
                }

                for (Cell* c : temp)
                    buckets_[c->GetPart()]->InsertNode(c->GetNode(), c->GetGain());
                temp.clear();
            }
            else {
                temp.push_back(move_cell);
            }
        }

        if (index_constrain == -1 && bug_flag == false)
            index_constrain = max_index;

        this->UndoMoves(max_index);

        if (max_gain <= 0)
            break;
    }
}

void FM::InitPartition() {
    part_size_[0] = 0;
    part_size_[1] = 0;

    // put all cells in partition 0
    for (Cell* cell : cell_list_) {
        cell->SetPart(0);
        part_size_[0]++;
    }

    // for (int i = 0; i < num_cells_; ++i) {
    //     bool side = (i % 2 == 0);
    //     cell_list_[i]->SetPart(side);
    //     part_size_[side]++;
    // }
}

void FM::InitNetPartSize() {
    // Reset part counts on each net
    for (Net* net : net_list_) {
        net->SetPartSize(0, 0);
        net->SetPartSize(1, 0);
    }

    // Accumulate counts by iterating over each cell's pins
    for (Cell* cell : cell_list_) {
        for (Net* net : cell->GetNetList()) {
            net->IncPartSize(cell->GetPart());
        }
    }
}

void FM::InitCell() {
    for (Cell* cell : cell_list_) {
        cell->Unlock();
        cell->SetGain(0);
    }

    buckets_[0]->Reset();
    buckets_[1]->Reset();

    for (Cell* cell : cell_list_) {
        bool from = cell->GetPart(), to = !from;
        int gain = 0;

        for (auto* net : cell->GetNetList()) {
            if (net->GetPartSize(from) == 1) ++gain;
            if (net->GetPartSize(to) ==   0) --gain;
        }
        cell->SetGain(gain);

        buckets_[from]->InsertNode(cell->GetNode(), gain);
    }
}

Cell* FM::FindMaxGainCell() {
    int selectIdx;
    if (part_size_[0] == lower_bound_)      selectIdx = 1;
    else if (part_size_[1] == lower_bound_) selectIdx = 0;
    else
        selectIdx = (buckets_[0]->GetMaxGain() > buckets_[1]->GetMaxGain()) ? 0 : 1;

    Node* maxNode = buckets_[selectIdx]->GetMaxGainNode();
    if (!maxNode)
        return nullptr;

    return maxNode->GetCell();
}

bool FM::IsMoveLegal(Cell *move_cell) {
    bool from = move_cell->GetPart(), to = !from;
    return ((part_size_[from] - 1) >= lower_bound_ && (part_size_[to] + 1) <= upper_bound_);
}

void FM::MoveCell(Cell *move_cell) {
    bool from = move_cell->GetPart(), to = !from;

    // cell
    move_cell->Move();

    // net
    for (auto* net : move_cell->GetNetList()) {
        net->SetPartSize(from, net->GetPartSize(from) - 1);
        net->SetPartSize(to,   net->GetPartSize(to)   + 1);
    }

    // fm
    part_size_[from]--;
    part_size_[to]++;
}

void FM::UpdateCellGain(Cell* move_cell) {
    bool from = move_cell->GetPart(), to = !from;

    auto adjust = [&](Cell* c, int delta, int bucketIdx) {
        if (c->IsLock()) return;
        int newGain = c->GetGain() + delta;
        c->SetGain(newGain);
        buckets_[bucketIdx]->DeleteNode(c->GetNode());
        buckets_[bucketIdx]->InsertNode(c->GetNode(), newGain);
    };

    for (Net* net : move_cell->GetNetList()) {

        int F = net->GetPartSize(from), T = net->GetPartSize(to);

        if (T == 0) {
            for (Cell* c : net->GetCellList())
                adjust(c, +1, from);
        }
        else if (T == 1) {
            for (Cell* c : net->GetCellList()) {
                if (!c->IsLock() && c->GetPart() == to) {
                    adjust(c, -1, to);
                    break;
                }
            }
        }

        --F;
        ++T;

        if (F == 0) {
            for (Cell* c : net->GetCellList())
                adjust(c, -1, to);
        }
        else if (F == 1) {
            for (Cell* c : net->GetCellList()) {
                if (!c->IsLock() && c->GetPart() == from) {
                    adjust(c, +1, from);
                    break;
                }
            }
        }
    }
}

void FM::UndoMoves(int max_index) {
    int size = move_stack_.size();
    for (int i = max_index; i < size; ++i) {
        this->MoveCell(move_stack_[i]);
    }
}

int FM::CalculateCutSize() {
    this->InitNetPartSize();
    int cut_size_ = 0;

    for (Net* net : net_list_) {
        if (net->GetPartSize(0) && net->GetPartSize(1))
            cut_size_++;
    }
    return cut_size_;
}
