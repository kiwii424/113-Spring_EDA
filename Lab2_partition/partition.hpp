// partition.hpp
#ifndef PARTITION_HPP
#define PARTITION_HPP

#include <vector>
#include <string>
#include <map>
#include <cassert>
using namespace std;

class Cell;
class Net;
class Node;
class BucketList;

// ---------------------------------------------------
// Node
class Node {
    public:
        // constructor and destructor
        Node(Cell* cell) {
            cell_ = cell;
            prev_ = nullptr;
            next_ = nullptr;
        }
        ~Node(){}

        // get and set
        Cell* GetCell() const { return cell_; }
        Node* GetPrev() const { return prev_; }
        Node* GetNext() const { return next_; }
        void SetPrev(Node* prev) { prev_ = prev; }
        void SetNext(Node* next) { next_ = next; }

    private:
        Cell* cell_;
        Node* prev_;
        Node* next_;
    };

// ---------------------------------------------------
// Net
class Net {
    public:
        // constructor and destructor
        Net(int name) {
            part_size_[0] = 0;
            part_size_[1] = 0;
            name_ = name;
        }
        ~Net(){}

        // get and set
        int GetPartSize(const int part) const { return part_size_[part]; }
        Cell* GetCell(const int i)      const { return cell_list_[i]; }
        void SetPartSize(const int part, const int gain) { part_size_[part] = gain; }
        vector<Cell*> GetCellList() { return cell_list_; }

        // modify
        void IncPartSize(const int part)   { ++part_size_[part]; }
        void DecPartSize(const int part)   { --part_size_[part]; }
        void AddCell(Cell* cell) { cell_list_.push_back(cell); }

    private:
        int part_size_[2];
        int name_;
        vector<Cell*> cell_list_;
    };

// ---------------------------------------------------
// Cell
class Cell
{
public:
    // constructor and destructor
    Cell(const int cell_id){
        gain_ = 0;
        lock_ = 0;
        part_ = 0;
        node_ = new Node(this);
        name_ = cell_id;
    }
    ~Cell()
    {
        delete node_;
    }

    // get and set
    int GetGain() const { return gain_; }
    bool IsLock() const { return lock_; }
    bool GetPart() const { return part_; }
    Node *GetNode() const { return node_; }
    int GetName() const { return name_ + 1; } // 1-based index
    int GetNetNum() const { return net_list_.size(); }
    vector<Net *> GetNetList() const { return net_list_; }
    Net *GetNet(const int i) const { return net_list_[i]; }
    void SetGain(const int gain) { gain_ = gain; }
    void SetPart(const bool part) { part_ = part; }

    // modify
    void Lock() { lock_ = true; }
    void Unlock() { lock_ = false; }
    void Move() { part_ = 1 - part_; }
    void AddNet(Net *net) { net_list_.push_back(net); }

private:
    int gain_;
    bool lock_;
    bool part_;
    Node *node_;
    int name_;
    vector<Net *> net_list_;
};

class BucketList{
    public:
        // constructor and destructor
        BucketList(int max_num_pins) {
            max_gain_ = -max_num_pins;
            max_num_pins_ = max_num_pins;
            for (int i = -max_num_pins_; i <= max_num_pins_; ++i) {
                Node* head_node = new Node(nullptr);
                head_nodes_.insert( pair<int, Node*>(i, head_node));
            }
        }
        ~BucketList() {
            for (int i = -max_num_pins_; i <= max_num_pins_; ++i)
                delete head_nodes_[i];
        }

        // get and set
        int GetMaxGain() const { return max_gain_; }
        Node* GetMaxGainNode() {
            assert(head_nodes_.find(max_gain_) != head_nodes_.end());
            return head_nodes_[max_gain_]->GetNext();
        }
        void Reset() {
            for (int i = -max_num_pins_; i <= max_num_pins_; ++i) {
                head_nodes_[i]->SetNext(nullptr);
            }
        }

        // modify
        void DeleteMaxGainNode() { // (head)-(node)-(next) >> head-next
            Node* head = head_nodes_[max_gain_];
            Node* node = head->GetNext();
            assert(node != nullptr);
            Node* next = node->GetNext();

            head->SetNext(next);
            node->SetPrev(nullptr);
            node->SetNext(nullptr);
            if (next != nullptr)
                next->SetPrev(head);
            // find next max gain
            while( max_gain_ > -max_num_pins_ && head_nodes_[max_gain_]->GetNext() == nullptr) {
                max_gain_--;
            }
        }
        void DeleteNode(Node* node) { // (prev)-(node)-(next) >> (prev)-(next)
            Node* prev = node->GetPrev();
            Node* next = node->GetNext();

            if (prev != nullptr)
                prev->SetNext(next);
            node->SetPrev(nullptr);
            node->SetNext(nullptr);
            if (next != nullptr)
                next->SetPrev(prev);
        }
        void InsertNode(Node* node, int gain) { // head-next >> head-node-next
            assert(head_nodes_.find(gain) != head_nodes_.end());
            Node* next = head_nodes_[gain]->GetNext();

            head_nodes_[gain]->SetNext(node);
            node->SetPrev(head_nodes_[gain]);
            node->SetNext(next);
            if (next != nullptr)
                next->SetPrev(node);

            if (gain > max_gain_)
                max_gain_ = gain;
        }

    private:
        int max_gain_;
        int max_num_pins_;
        map<int, Node*> head_nodes_;
    };

#endif // PARTITION_HPP
