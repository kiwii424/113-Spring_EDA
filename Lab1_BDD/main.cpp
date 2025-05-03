#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <memory>
#include <tuple>
#include <queue>
#include <fstream>

using namespace std;

// -------------------- Expression AST --------------------

struct Expr {
    virtual bool eval(const unordered_map<char, bool>& env) const = 0;
    virtual ~Expr() = default;
};

struct Var : Expr {
    char name;
    bool negated;
    Var(char c, bool n) : name(c), negated(n) {}
    bool eval(const unordered_map<char, bool>& env) const override {
        bool val = env.at(name);
        return negated ? !val : val;
    }
};

struct And : Expr {
    vector<shared_ptr<Expr>> terms;
    bool eval(const unordered_map<char, bool>& env) const override {
        for (auto& t : terms) {
            if (!t->eval(env)) return false;
        }
        return true;
    }
};

struct Or : Expr {
    vector<shared_ptr<Expr>> terms;
    bool eval(const unordered_map<char, bool>& env) const override {
        for (auto& t : terms) {
            if (t->eval(env)) return true;
        }
        return false;
    }
};

// Parse a SOP string like "ab+CD" => OR( AND(a, b), AND(C', D) )
shared_ptr<Expr> parse_expr(const string& sop) {
    auto or_node = make_shared<Or>();
    size_t i = 0;
    while (i < sop.size()) {
        auto and_node = make_shared<And>();
        while (i < sop.size() && sop[i] != '+') {
            bool neg = isupper(sop[i]);
            and_node->terms.push_back(make_shared<Var>(tolower(sop[i]), neg));
            ++i;
        }
        or_node->terms.push_back(and_node);
        if (i < sop.size() && sop[i] == '+') {
            ++i; // skip plus
        }
    }
    return or_node;
}

// -------------------- BDD Node structure --------------------

struct BDDNode {
    string var;          // variable name if non-terminal
    BDDNode* low;        // 0-branch
    BDDNode* high;       // 1-branch
    bool is_terminal;    // true if terminal
    bool terminal_val;   // which terminal (true/false)

    BDDNode(string v, BDDNode* l, BDDNode* h)
      : var(v), low(l), high(h), is_terminal(false), terminal_val(false) {}

    BDDNode(bool val)
      : var(""), low(nullptr), high(nullptr),
        is_terminal(true), terminal_val(val) {}
};

// Shared terminal nodes
static BDDNode* TRUE_NODE = new BDDNode(true);
static BDDNode* FALSE_NODE = new BDDNode(false);

// We'll store subgraphs by key = (var, lowPtr, highPtr)
struct BDDKeyHash {
    size_t operator()(const tuple<string, BDDNode*, BDDNode*>& key) const {
        const string& var = std::get<0>(key);
        BDDNode* low = std::get<1>(key);
        BDDNode* high = std::get<2>(key);

        size_t h1 = hash<string>()(var);
        size_t h2 = reinterpret_cast<uintptr_t>(low);
        size_t h3 = reinterpret_cast<uintptr_t>(high);
        return (h1 ^ (h2 << 1)) ^ (h3 << 2);
    }
};

struct BDDKeyEqual {
    bool operator()(const tuple<string, BDDNode*, BDDNode*>& a,
                    const tuple<string, BDDNode*, BDDNode*>& b) const {
        return get<0>(a) == get<0>(b)
            && get<1>(a) == get<1>(b)
            && get<2>(a) == get<2>(b);
    }
};

static unordered_map<tuple<string, BDDNode*, BDDNode*>, BDDNode*, BDDKeyHash, BDDKeyEqual> unique_table;

// Shannon expansion to build BDD
BDDNode* build_ro_bdd(const shared_ptr<Expr>& expr,
                      const vector<char>& vars,
                      unordered_map<char, bool> env,
                      size_t index)
{
    // if we've assigned all variables => evaluate
    if (index == vars.size()) {
        bool val = expr->eval(env);
        return val ? TRUE_NODE : FALSE_NODE;
    }

    // expand on var
    char var = vars[index];

    env[var] = false;
    BDDNode* low = build_ro_bdd(expr, vars, env, index + 1);

    env[var] = true;
    BDDNode* high = build_ro_bdd(expr, vars, env, index + 1);

    // reduce if same branch
    if (low == high) {
        return low; // share subgraph
    }

    // check unique table
    auto key = make_tuple(string(1, var), low, high);
    auto it = unique_table.find(key);
    if (it != unique_table.end()) {
        return it->second;
    }

    // create new node
    BDDNode* node = new BDDNode(string(1, var), low, high);
    unique_table[key] = node;
    return node;
}

int final_count(BDDNode* root) {
    unordered_set<BDDNode*> visited;
    queue<BDDNode*> q;

    q.push(root);
    visited.insert(root);

    bool visited_true = false;
    bool visited_false = false;

    while (!q.empty()) {
        BDDNode* cur = q.front();
        q.pop();

        if (cur == TRUE_NODE) visited_true = true;
        if (cur == FALSE_NODE) visited_false = true;

        if (!cur->is_terminal) {
            if (cur->low && !visited.count(cur->low)) {
                visited.insert(cur->low);
                q.push(cur->low);
            }
            if (cur->high && !visited.count(cur->high)) {
                visited.insert(cur->high);
                q.push(cur->high);
            }
        }
    }

    int nonterm = 0;
    for (auto* node : visited) {
        if (!node->is_terminal) {
            nonterm++;
        }
    }

    int t_count = 0;
    if (visited_true)  t_count++;
    if (visited_false) t_count++;

    return nonterm + t_count;
}

int main(int argc, char *argv[]) {

    ifstream fin(argv[1]);
    if (!fin) {
        cerr << "Error opening file: " << argv[1] << endl;
        return 1;
    }

    ofstream fout(argv[2]);
    if (!fout) {
        cerr << "Error opening output file." << endl;
        return 1;
    }

    string expr_str;
    getline(fin, expr_str);
    if (!expr_str.empty() && expr_str.back() == '.') {
        expr_str.pop_back();
    }
    else if (expr_str.back() == 13) {
        expr_str.pop_back();
        expr_str.pop_back();
    }

    vector<string> var_orders;
    while (true) {
        string line;
        if (!getline(fin, line)) break;
        if (!line.empty()) {
            if (line.back() == '.') {
                line.pop_back();
            }
            else if (line.back() == 13) {
                line.pop_back();
                line.pop_back();
            }
            var_orders.push_back(line);
        }
    }

    shared_ptr<Expr> expr = parse_expr(expr_str);
    int answer = 1e9;

    for (auto& order : var_orders) {

        unique_table.clear();
        unordered_map<char, bool> env;
        vector<char> vars(order.begin(), order.end());

        BDDNode* root = build_ro_bdd(expr, vars, env, 0);
        int c = final_count(root);
        answer = min(answer, c);
    }

    fout << answer << "\n";
    return 0;
}
