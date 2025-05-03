# FM Partitioning Program

This project implements the Fiduccia–Mattheyses (FM) heuristic for hypergraph partitioning under balance constraints, with multi-restart and multiple initialization strategies for improved solution quality.

## Prerequisites

- A C++11-capable compiler (e.g., `g++`, `clang++`)
- `make` utility for building and running

## Usage

### Automatic (via Makefile):

Simply run:

```bash
make
```

This single command will:

1.	Compile source files (`main.cpp`, `fm.cpp`) into the executable `Lab2`.
2.	Execute `Lab2` on three predefined input files:

    ```bash
    ./Lab2 input/input.hgr   output/output1.txt
    ./Lab2 input/input2.txt.hgr   output/output2.txt
    ./Lab2 input/input3.txt.hgr   output/output3.txt
    ```

3.	Place all generated output files in the `output/` directory.
4.	Print the best cut size and runtime for each run in the terminal.

### Manual testing:

You can also run the executable directly:

```bash
# Single argument: output written to root as output.txt
./Lab2 <input_file>

# Specify both input and output paths:
./Lab2 <input_file> <output_file>
```
No further commands are needed.



## Makefile Targets
 * make (default): Build and run all test cases, outputs in output/.
 * make all: Builds the executable (Lab2) only.
 * make run: Alias for default (identical to make).
 * make clean: Remove temporary object files and the executable.


## Algorithm Overview

1. Initialization: Generate balanced partitions using greedy or random methods.

2. Gain Computation: Compute and bucket each cell's gain.

3. Iterative Moves: Move the highest-gain cell legally, update gains, and track the best move prefix.

4. Undo Excess Moves: Revert moves beyond the optimal prefix to maximize cut reduction.

5. Multi-Restart: Repeat with various initial partitions, choosing the one with the lowest cut size.

## Troubleshooting

* Segmentation Faults: Rebuild with debug flags (`CFLAGS += -g -O0`) and use `gdb` or `lldb`.

* Unbalanced Partitions: Verify `lower_bound_` and `upper_bound_` in `FM` constructor.
