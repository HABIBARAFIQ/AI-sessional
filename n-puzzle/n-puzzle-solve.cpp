#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cmath>
using namespace std;

// Utility structure to store the puzzle state
struct Node {
    vector<vector<int>> board;
    int g, h, f;
    pair<int, int> blankPos;  // Position of the blank space
    Node* parent;             // To track the path to the goal

    // Constructor
    Node(vector<vector<int>> b, int g_moves, int h_heuristic, pair<int, int> blank, Node* parent_node) 
        : board(b), g(g_moves), h(h_heuristic), blankPos(blank), parent(parent_node) {
        f = g + h;  // Priority function
    }

    // Comparison operator to order nodes in the priority queue
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// Global counters
int total_expanded_nodes = 0;
int total_explored_nodes = 0;

// Heuristic Functions
int hammingDistance(const vector<vector<int>>& board, const vector<vector<int>>& goal) {
    int distance = 0;
    int k = board.size();
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            if (board[i][j] != 0 && board[i][j] != goal[i][j]) {
                distance++;
            }
        }
    }
    return distance;
}

int manhattanDistance(const vector<vector<int>>& board, const vector<vector<int>>& goal) {
    int distance = 0;
    int k = board.size();
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            if (board[i][j] != 0) {
                int value = board[i][j] - 1;
                int goal_i = value / k;
                int goal_j = value % k;
                distance += abs(i - goal_i) + abs(j - goal_j);
            }
        }
    }
    return distance;
}

// Check if puzzle is solvable
bool isSolvable(const vector<vector<int>>& board) {
    int n = board.size();
    
    // Flatten the 2D board into a 1D array to count inversions
    vector<int> flat_board;
    int blank_row = 0;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (board[i][j] != 0) {
                flat_board.push_back(board[i][j]);
            } else {
                // Record the row of the blank space (0)
                blank_row = i;
            }
        }
    }

    // Count inversions
    int inversions = 0;
    for (int i = 0; i < flat_board.size(); ++i) {
        for (int j = i + 1; j < flat_board.size(); ++j) {
            if (flat_board[i] > flat_board[j]) {
                ++inversions;
            }
        }
    }

    // For odd grid size, puzzle is solvable if inversion count is even
    if (n % 2 != 0) {
        return inversions % 2 == 0;
    }

    // For even grid size, puzzle is solvable if:
    // (1) The blank is on an even row (from bottom) and number of inversions is odd.
    // (2) The blank is on an odd row (from bottom) and number of inversions is even.
    int blank_row_from_bottom = n - blank_row;
    if (blank_row_from_bottom % 2 == 0) {
        return inversions % 2 != 0;
    } else {
        return inversions % 2 == 0;
    }
}

// Utility function to find the position of the blank (0) space
pair<int, int> findBlank(const vector<vector<int>>& board) {
    for (int i = 0; i < board.size(); i++) {
        for (int j = 0; j < board[i].size(); j++) {
            if (board[i][j] == 0) {
                return {i, j};
            }
        }
    }
    return {-1, -1};
}

// Function to generate neighboring states
vector<Node*> generateNeighbors(Node* node, const vector<vector<int>>& goal, bool useHamming) {
    vector<Node*> neighbors;
    vector<vector<int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // Up, Down, Left, Right
    int n = node->board.size();
    int blank_x = node->blankPos.first;
    int blank_y = node->blankPos.second;

    for (const auto& dir : directions) {
        int new_x = blank_x + dir[0];
        int new_y = blank_y + dir[1];
        if (new_x >= 0 && new_x < n && new_y >= 0 && new_y < n) {
            vector<vector<int>> new_board = node->board;
            swap(new_board[blank_x][blank_y], new_board[new_x][new_y]);
            int new_h = useHamming ? hammingDistance(new_board, goal) : manhattanDistance(new_board, goal);
            neighbors.push_back(new Node(new_board, node->g + 1, new_h, {new_x, new_y}, node));
        }
    }
    return neighbors;
}

// A* Search Algorithm
pair<int, vector<Node*>> aStarSearch(const vector<vector<int>>& initial_board, const vector<vector<int>>& goal, bool useHamming) {
    auto cmp = [](Node* left, Node* right) { return *left > *right; };
    priority_queue<Node*, vector<Node*>, decltype(cmp)> open_list(cmp);
    unordered_set<string> closed_set;

    // Create initial node
    pair<int, int> blank = findBlank(initial_board);
    int h = useHamming ? hammingDistance(initial_board, goal) : manhattanDistance(initial_board, goal);
    Node* initial_node = new Node(initial_board, 0, h, blank, nullptr);
    open_list.push(initial_node);

    while (!open_list.empty()) {
        total_expanded_nodes++; // Increment expanded nodes count
        Node* current = open_list.top();
        open_list.pop();

        // Check if we reached the goal
        if (current->board == goal) {
            vector<Node*> path;
            Node* temp = current;
            while (temp) {
                path.push_back(temp);
                temp = temp->parent;
            }
            return {current->g, path};  // Return cost and path
        }

        // Serialize current board for set comparison
        string serialized;
        for (const auto& row : current->board) {
            for (const auto& num : row) {
                serialized += to_string(num) + ",";
            }
        }

        if (closed_set.find(serialized) != closed_set.end()) {
            continue;
        }
        closed_set.insert(serialized);
        total_explored_nodes++; // Increment explored nodes count

        // Generate and push neighbors
        vector<Node*> neighbors = generateNeighbors(current, goal, useHamming);
        for (auto neighbor : neighbors) {
            open_list.push(neighbor);
        }
    }
    return {-1, {}};  // If no solution
}

// Function to print the board
void printBoard(const vector<vector<int>>& board) {
    for (const auto& row : board) {
        for (const auto& num : row) {
            if (num == 0)
                cout << "* ";
            else
                cout << num << " ";
        }
        cout << endl;
    }
}

// Puzzle Solver
void solvePuzzle(int k, const vector<vector<int>>& initial_board) {
    vector<vector<int>> goal(k, vector<int>(k));
    int val = 1;
    for (int i = 0; i < k; i++) {
        for (int j = 0; j < k; j++) {
            goal[i][j] = val++;
        }
    }
    goal[k-1][k-1] = 0;  // Blank space

    cout << "Initial Board:" << endl;
    printBoard(initial_board);

    if (!isSolvable(initial_board)) {
        cout << "The puzzle is not solvable." << endl;
        return;
    }

    cout << "The puzzle is solvable." << endl;

    // Solve using Hamming Distance
    cout << "\nSolving using Hamming Distance heuristic..." << endl;
    total_expanded_nodes = 0;  // Reset counters
    total_explored_nodes = 0;
    pair<int, vector<Node*>> hammingResult = aStarSearch(initial_board, goal, true);
    int cost_hamming = hammingResult.first;
    vector<Node*> path_hamming = hammingResult.second;

    if (cost_hamming != -1) {
        cout << "Optimal cost: " << cost_hamming << endl;
        cout << "Steps:" << endl;
        for (int i = path_hamming.size() - 1; i >= 0; i--) {
            printBoard(path_hamming[i]->board);
            cout << endl;
        }
    }
    cout << "Total expanded nodes: " << total_expanded_nodes << endl;
    cout << "Total explored nodes: " << total_explored_nodes << endl;

    // Solve using Manhattan Distance
    cout << "\nSolving using Manhattan Distance heuristic..." << endl;
    total_expanded_nodes = 0;  // Reset counters
    total_explored_nodes = 0;
    pair<int, vector<Node*>> manhattanResult = aStarSearch(initial_board, goal, false);
    int cost_manhattan = manhattanResult.first;
    vector<Node*> path_manhattan = manhattanResult.second;

    if (cost_manhattan != -1) {
        cout << "Optimal cost: " << cost_manhattan << endl;
        cout << "Steps:" << endl;
        for (int i = path_manhattan.size() - 1; i >= 0; i--) {
            printBoard(path_manhattan[i]->board);
            cout << endl;
        }
    }
    cout << "Total expanded nodes: " << total_expanded_nodes << endl;
    cout << "Total explored nodes: " << total_explored_nodes << endl;
}

int main() {
    int k = 3;
    vector<vector<int>> initial_board = {
        {1 ,2 ,3},
        {0 ,4 ,5},
        {6 ,7 ,8}
    };

    solvePuzzle(k, initial_board);
    return 0;
}
