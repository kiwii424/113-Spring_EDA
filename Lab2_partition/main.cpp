#include "fm.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <sstream>

using namespace std;


int main(int argc, char *argv[]) {
    ifstream fin(argv[1]);
    if (!fin) {
        cerr << "Error opening file: " << argv[1] << endl;
        return 1;
    }

    const char* outputFile = (argc > 2) ? argv[2] : "output.txt";
    ofstream fout(outputFile);

    int num_nets, num_cells;
    fin >> num_nets >> num_cells;

    vector<Net*> net_list(num_nets, nullptr);
    vector<Cell*> cell_list(num_cells, nullptr);

    string line;
    for (int net_id = -1; net_id < num_nets; net_id++){
        if(!getline(fin, line)) break;
        if (line.empty()) continue; // Skip empty lines
        stringstream ss(line);

        Net* cur_net = new Net(net_id);
        net_list[net_id] = cur_net;

        int cell_id;
        while (ss >> cell_id) {
            cell_id--; // Adjust for 0-based indexing
            if (cell_list[cell_id] == nullptr) { // new cell
                cell_list[cell_id] = new Cell(cell_id);
            }
            Cell* cur_cell = cell_list[cell_id];

            cur_cell->AddNet(cur_net);
            cur_net->AddCell(cur_cell);
        }
    }

    int max_num_pins = 0;
    for (int i = 0; i < num_cells; ++i) {
        max_num_pins = max(max_num_pins, cell_list[i]->GetNetNum());
    }

    FM fm(num_cells, num_nets, net_list, cell_list, max_num_pins);

    clock_t start, end;
    start = clock();

    fm.Solve();

    end = clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    cout << "Time taken by fm is : " << fixed << setprecision(5)
         << time_taken << " secends." << endl;

    fm.Result(fout);


    return 0;
}
