/*
 * Project: Erebus: The Self-Optimizing Pathfinding Engine
 * Authors: Saif ur Rehman, Huzaifa Imran, Ayesha Tasneem
 * Date: December 31, 2025
 * 
 * Description: 
 * A robust, real-time pathfinding visualizer using SFML.
 * Features Dijkstra, A*, BFS, DFS, and Swarm algorithms with dynamic
 * obstacle handling and Google Maps-inspired aesthetics.
 */

#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <stack>
#include <deque>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <functional>
#include <set>
#include <map>
#include <optional>
#include <ctime>

using namespace std;
using namespace sf;

// --- Constants & Colors ---
int ROWS = 25;
int COLS = 35; 
const int UI_WIDTH = 280;
float CELL_SIZE = 30.0f;

int WINDOW_WIDTH = 800; // Default for Setup
int WINDOW_HEIGHT = 600;

const Color COLOR_BG(232, 234, 237);         // Google Maps Land
const Color COLOR_ROAD(255, 255, 255);       // Road
const Color COLOR_WALL(60, 64, 67);          // Building/Wall
const Color COLOR_START(66, 133, 244);       // Start (Blue)
const Color COLOR_END(234, 67, 53);          // End (Red)
const Color COLOR_PATH_LINE(0, 0, 153);    // Google Route Blue
const Color COLOR_VISITED(200, 220, 255);    // Expanded nodes
const Color COLOR_OPEN(100, 255, 100);       // Frontier nodes

// --- Drawing Helpers ---
void drawPin(RenderWindow& window, float x, float y) {
    float size = CELL_SIZE / 2.5f;
    float cx = x + CELL_SIZE/2.0f;
    float cy = y + CELL_SIZE/2.0f;

    // Pin Point (Triangle inverted)
    ConvexShape point;
    point.setPointCount(3);
    point.setPoint(0, Vector2f(cx - size*0.8f, cy - size/2.0f));
    point.setPoint(1, Vector2f(cx + size*0.8f, cy - size/2.0f));
    point.setPoint(2, Vector2f(cx, cy + size));
    point.setFillColor(COLOR_END);
    window.draw(point);

    // Pin Head (Circle)
    CircleShape head(size);
    head.setOrigin({size, size});
    head.setPosition({cx, cy - size/2.0f});
    head.setFillColor(COLOR_END);
    window.draw(head);
    
    // Inner White Dot
    CircleShape dot(size * 0.4f);
    dot.setOrigin({size * 0.4f, size * 0.4f});
    dot.setPosition({cx, cy - size/2.0f});
    dot.setFillColor(Color::White);
    window.draw(dot);
}

void drawStart(RenderWindow& window, float x, float y) {
    float size = CELL_SIZE / 4.0f;
    float cx = x + CELL_SIZE/2.0f;
    float cy = y + CELL_SIZE/2.0f;
    
    // Aura
    CircleShape aura(size * 3.0f);
    aura.setOrigin({size * 3.0f, size * 3.0f});
    aura.setPosition({cx, cy});
    aura.setFillColor(Color(66, 133, 244, 100)); // Transparent Blue
    window.draw(aura);

    // Core
    CircleShape core(size);
    core.setOrigin({size, size});
    core.setPosition({cx, cy});
    core.setFillColor(COLOR_START);
    core.setOutlineThickness(2);
    core.setOutlineColor(Color::White);
    window.draw(core);
}

// --- Enums ---
enum Algorithm {
    DIJKSTRA,
    ASTAR,
    GREEDY,
    SWARM,
    CONVERGENT_SWARM,
    BIDIRECTIONAL,
    BFS,
    DFS
};

enum AppState {
    SETUP,
    IDLE,
    RUNNING,
    FINISHED
};

enum InteractionState {
    NONE,
    DRAG_START,
    DRAG_END,
    DRAW_WALL
};

// --- Structures ---
struct Node {
    int row, col;
    bool blocked = false;
    bool visited = false;
    bool inOpenSet = false;
    float g = 1e9;
    float h = 0;
    float f = 1e9;
    Node* parent = nullptr;

    void resetPathData() {
        visited = false;
        inOpenSet = false;
        g = 1e9;
        h = 0;
        f = 1e9;
        parent = nullptr;
    }
};

struct Button {
    RectangleShape shape;
    Text label;
    function<void()> onClick;
    bool isHovered = false;

    Button(float x, float y, float w, float h, string text, Font& font, function<void()> clickHandler) 
        : onClick(clickHandler), label(font, text, 14) {
        shape.setSize({w, h});
        shape.setPosition({x, y});
        shape.setFillColor(Color::White);
        shape.setOutlineThickness(-2);
        shape.setOutlineColor(Color(200, 200, 200));

        label.setFillColor(Color::Black);
        setText(text);
    }

    void setText(string t) {
        label.setString(t);
        FloatRect bounds = label.getLocalBounds();
        label.setOrigin({bounds.position.x + bounds.size.x/2.0f, bounds.position.y + bounds.size.y/2.0f});
        label.setPosition({shape.getPosition().x + shape.getSize().x/2.0f, shape.getPosition().y + shape.getSize().y/2.0f});
    }

    bool isClicked(Vector2i mousePos) {
        return shape.getGlobalBounds().contains({(float)mousePos.x, (float)mousePos.y});
    }

    void updateHover(Vector2i mousePos) {
        isHovered = isClicked(mousePos);
        if (isHovered) {
            shape.setFillColor(Color(240, 240, 240));
            shape.setOutlineColor(Color(66, 133, 244));
        } else {
            shape.setFillColor(Color::White);
            shape.setOutlineColor(Color(200, 200, 200));
        }
    }

    void draw(RenderWindow& window) {
        window.draw(shape);
        window.draw(label);
    }
    
    void setSelected(bool selected) {
        if(selected) shape.setFillColor(Color(200, 230, 255));
        else if (!isHovered) shape.setFillColor(Color::White); // Only reset if not hovered
    }
};

// --- Global State ---
vector<vector<Node>> grid;
Node* startNode = nullptr;
Node* endNode = nullptr;
AppState appState = SETUP;
InteractionState currentInteraction = NONE;
bool isAddingWall = true;

// Visualization Speed Control
float stepDelay = 0.012f; // Seconds between steps (lower = faster)
Clock stepClock; 

// Path Memory (Knowledge Base)
map<pair<int, int>, int> knowledgeBase;

void loadKnowledge() {
    ifstream file("knowledge.txt");
    int r, c, count;
    while (file >> r >> c >> count) {
        knowledgeBase[{r, c}] = count;
    }
}

void saveKnowledge(const vector<Node*>& currentPath) {
    for (Node* n : currentPath) {
        knowledgeBase[{n->row, n->col}]++;
    }
    ofstream file("knowledge.txt");
    for (auto const& [pos, count] : knowledgeBase) {
        file << pos.first << " " << pos.second << " " << count << "\n";
    }
}

Algorithm currentAlgorithm = DIJKSTRA;
vector<Node*> path;
vector<Node*> visitedOrder; // For visuals

// --- Search State Containers (Iterative) ---
// For Priority Queue based (A*, Dijkstra, etc)
vector<Node*> openList; 
// For BFS / Bidirectional
deque<Node*> bfsQ;
deque<Node*> biIdx1, biIdx2; // using deque for queue behavior
map<Node*, Node*> biParents1, biParents2;
// For DFS
vector<Node*> dfsStack;


void resetGridPathData() {
    for(auto& row : grid)
        for(auto& n : row)
            n.resetPathData();
    path.clear();
    visitedOrder.clear();
    
    // Clear Containers
    openList.clear();
    bfsQ.clear();
    biIdx1.clear(); biIdx2.clear();
    biParents1.clear(); biParents2.clear();
    dfsStack.clear();
}

void clearWalls() {
    for(auto& row : grid)
        for(auto& n : row)
            n.blocked = false;
    resetGridPathData();
    appState = IDLE;
}

void generateMaze() {
    clearWalls();
    for(auto& row : grid)
        for(auto& n : row)
            n.blocked = true;

    stack<Node*> stk;
    Node* start = startNode;
    start->blocked = false;
    stk.push(start);

    while(!stk.empty()) {
        Node* curr = stk.top();
        vector<pair<int, int>> neighbors;

        int dr2[] = {-2, 2, 0, 0};
        int dc2[] = {0, 0, -2, 2};

        for(int i = 0; i < 4; i++) {
            int nr = curr->row + dr2[i];
            int nc = curr->col + dc2[i];
            if(nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && grid[nr][nc].blocked) {
                neighbors.push_back({nr, nc});
            }
        }

        if(!neighbors.empty()) {
            int idx = rand() % neighbors.size();
            int nr = neighbors[idx].first;
            int nc = neighbors[idx].second;

            // Unblock neighbor and the cell between
            grid[nr][nc].blocked = false;
            grid[(curr->row + nr) / 2][(curr->col + nc) / 2].blocked = false;

            stk.push(&grid[nr][nc]);
        } else {
            stk.pop();
        }
    }

    startNode->blocked = false;
    endNode->blocked = false;
    resetGridPathData();
    appState = IDLE;
}

// --- Algorithms ---

float heuristic(Node* a, Node* b, Algorithm algo) {
    float h = 0;
    if (algo == DIJKSTRA || algo == BFS || algo == DFS) h = 0;
    else if (algo == SWARM) h = (float)(abs(a->row - b->row) + abs(a->col - b->col)) * 4.0f; // High weight
    else if (algo == CONVERGENT_SWARM) h = (float)(abs(a->row - b->row) + abs(a->col - b->col)) * 100.0f; // Very high weight
    else if (algo == ASTAR || algo == GREEDY) h = (float)abs(a->row - b->row) + abs(a->col - b->col);
    // else h = sqrt(pow(a->row - b->row, 2) + pow(a->col - b->col, 2)); // Euclidean for other A* variants if needed

    // Knowledge Bonus: Favor paths we've taken before
    if (knowledgeBase.count({a->row, a->col})) {
        float bonus = knowledgeBase[{a->row, a->col}] * 0.2f; 
        h -= min(h * 0.5f, bonus); // Don't let bonus invert the distance
    }
    return h;
}

void reconstructPath(Node* current) {
    path.clear();
    while (current) {
        path.push_back(current);
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    saveKnowledge(path); // Learn from this path
    appState = FINISHED;
}

void reconstructBidirectional(Node* meet, map<Node*, Node*>& p1Map, map<Node*, Node*>& p2Map) {
    path.clear();
    // Start side
    vector<Node*> p1;
    Node* curr = meet;
    while(curr) { p1.push_back(curr); curr = p1Map[curr]; }
    reverse(p1.begin(), p1.end());
    
    // End side
    curr = p2Map[meet];
    while(curr) { path.push_back(curr); curr = p2Map[curr]; }
    
    // Combine
    p1.insert(p1.end(), path.begin(), path.end());
    path = p1;
    saveKnowledge(path); // Learn from this path
    appState = FINISHED;
}

// Helper for neighbors
int dr[] = {-1, 1, 0, 0};
int dc[] = {0, 0, -1, 1};
vector<Node*> getNeighbors(Node* n) {
    vector<Node*> neighbors;
    for(int i=0; i<4; i++){
        int r = n->row + dr[i];
        int c = n->col + dc[i];
        if(r >= 0 && r < ROWS && c >= 0 && c < COLS && !grid[r][c].blocked)
            neighbors.push_back(&grid[r][c]);
    }
    return neighbors;
}

// Comparison for Heap
auto heapCmp = [](Node* a, Node* b) { return a->f > b->f; };

// Reactivate Node: Link a newly unblocked node and propagate cost improvements
void reactivateNode(Node* n) {
    if (!n || n->blocked) return;

    bool improved = false;
    Node* bestParent = n->parent;
    float minG = n->g;

    // Check neighbors to see if any can provide a better path to 'n'
    for (int i = 0; i < 4; i++) {
        int r = n->row + dr[i];
        int c = n->col + dc[i];
        if (r >= 0 && r < ROWS && c >= 0 && c < COLS) {
            Node* neighbor = &grid[r][c];
            if ((neighbor->visited || neighbor->inOpenSet) && neighbor->g + 1 < minG) {
                minG = neighbor->g + 1;
                bestParent = neighbor;
                improved = true;
            }
        }
    }

    if (improved || (n->g >= 1e8 && bestParent)) {
        n->parent = bestParent;
        n->g = minG;
        n->h = heuristic(n, endNode, currentAlgorithm);
        n->f = n->g + n->h;
        n->visited = false; // Force re-evaluation
        n->inOpenSet = true;

        // Add back to active frontier based on current algorithm
        if (currentAlgorithm == BFS) {
            bfsQ.push_back(n);
        } else if (currentAlgorithm == DFS) {
            dfsStack.push_back(n);
        } else if (currentAlgorithm == BIDIRECTIONAL) {
             // For simplicity in dynamic unblocking, treat as a frontier expansion
            if (bestParent && bestParent->visited) { 
                biParents1[n] = bestParent;
                biIdx1.push_back(n);
            } else {
                biParents2[n] = bestParent;
                biIdx2.push_back(n);
            }
        } else {
            // A*, Dijkstra, Swarm, etc.
            openList.push_back(n);
            push_heap(openList.begin(), openList.end(), heapCmp);
        }

        // PROPAGATION: Check if unblocking/improving 'n' helps its neighbors
        for (Node* neighbor : getNeighbors(n)) {
            if (n->g + 1 < neighbor->g) {
                // n offers a strictly better path to neighbor
                reactivateNode(neighbor);
            }
        }

        appState = RUNNING;
    }
}

// Prune Branch: Recursively reset nodes that depend on 'root'
void pruneBranch(Node* root) {
    if(!root || root == startNode) return;

    Node* anchor = root->parent; // Save the parent to restart search from

    // 1. Identify all descendants using BFS on parent pointers
    set<Node*> toReset;
    queue<Node*> q;
    
    q.push(root);
    toReset.insert(root);

    while(!q.empty()){
        Node* curr = q.front(); q.pop();

        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        for(int i=0; i<4; i++){
             int r = curr->row + dr[i];
             int c = curr->col + dc[i];
             if(r>=0 && r<ROWS && c>=0 && c<COLS){
                 Node* child = &grid[r][c];
                 if(child->parent == curr && toReset.find(child) == toReset.end()) {
                     q.push(child);
                     toReset.insert(child);
                 }
             }
        }
    }

    // 2. Reset Data
    for(Node* n : toReset) {
        n->resetPathData();
        n->visited = false; 
        // Important for Bidirectional: Clear maps
        biParents1.erase(n);
        biParents2.erase(n);
    }

    // 3. Remove from visitedOrder for visuals
    visitedOrder.erase(
        remove_if(visitedOrder.begin(), visitedOrder.end(), 
            [&](Node* n){ return toReset.count(n); }),
        visitedOrder.end());
    
    // 4. Re-activate Anchor to prevent stalling
    if (anchor && !anchor->blocked) {
        anchor->visited = false;
        anchor->inOpenSet = true;
        
        if (currentAlgorithm == BFS) {
             bfsQ.push_front(anchor);
        } else if (currentAlgorithm == DFS) {
             dfsStack.push_back(anchor);
        } else if (currentAlgorithm == BIDIRECTIONAL) {
             // Re-add to the appropriate side
             if (biParents1.count(anchor)) biIdx1.push_front(anchor);
             else if (biParents2.count(anchor)) biIdx2.push_front(anchor);
        } else {
             // A*, Dijkstra, Greedy, Swarm
             openList.push_back(anchor);
             push_heap(openList.begin(), openList.end(), heapCmp);
        }
    }

    path.clear();
    appState = RUNNING;
}


void startAlgorithm() {
    resetGridPathData();
    appState = RUNNING;

    if (currentAlgorithm == BFS) {
        bfsQ.push_back(startNode);
        startNode->visited = true;
        startNode->g = 0;
    }
    else if (currentAlgorithm == DFS) {
        dfsStack.push_back(startNode);
        startNode->g = 0;
        // DFS marks visited on pop usually, or push. 
        // Standard iterative DFS: Push start.
    }
    else if (currentAlgorithm == BIDIRECTIONAL) {
        biIdx1.push_back(startNode);
        biIdx2.push_back(endNode);
        startNode->visited = true; // Utilizing visited/inOpenSet flexibly
        endNode->inOpenSet = true; // Use inOpenSet as visited-by-end
        biParents1[startNode] = nullptr;
        biParents2[endNode] = nullptr;
        startNode->g = 0;
        endNode->g = 0;
    }
    else {
        // A*, Dijkstra, Greedy, Swarm
        startNode->g = 0;
        startNode->f = heuristic(startNode, endNode, currentAlgorithm);
        openList.push_back(startNode);
        push_heap(openList.begin(), openList.end(), heapCmp);
        startNode->inOpenSet = true;
    }
}

void stepAlgorithm() {
    if (appState != RUNNING) return;

    // --- BFS ---
    if (currentAlgorithm == BFS) {
        if (bfsQ.empty()) { appState = FINISHED; return; }
        
        Node* current = bfsQ.front(); bfsQ.pop_front();
        
        // Check if pruned/orphaned
        if ((current->parent == nullptr && current != startNode) || current->g >= 1e8) return;
        if (current->blocked) return; 

        visitedOrder.push_back(current);
        if (current == endNode) { reconstructPath(endNode); return; }

        for(Node* n : getNeighbors(current)) {
            if (!n->visited && !n->blocked) {
                n->visited = true;
                n->parent = current;
                n->g = current->g + 1; // Track depth for visuals
                bfsQ.push_back(n);
            }
        }
    }
    // --- DFS ---
    else if (currentAlgorithm == DFS) {
        if (dfsStack.empty()) { appState = FINISHED; return; }

        Node* current = dfsStack.back(); dfsStack.pop_back();

        if (current->blocked) return;
        
        if (!current->visited) {
            current->visited = true;
            visitedOrder.push_back(current);
            if (current == endNode) { reconstructPath(endNode); return; }

            for (Node* n : getNeighbors(current)) {
                if (!n->visited && !n->blocked) {
                    n->parent = current; // DFS Set parent on push? or pop? 
                    n->g = current->g + 1; // Visuals
                    // To trace back correctly, usually parent is set when discovered.
                    // But for DFS stack, we might revisit. 
                    // Let's set parent here but only if not visited.
                     dfsStack.push_back(n);
                }
            }
        }
    }
    // --- BIDIRECTIONAL ---
    else if (currentAlgorithm == BIDIRECTIONAL) {
        if (biIdx1.empty() && biIdx2.empty()) { appState = FINISHED; return; }

        // Step Start Side
        if (!biIdx1.empty()) {
            Node* curr = biIdx1.front(); biIdx1.pop_front();
            
            // Check if pruned/orphaned
            if ((curr->parent != nullptr || curr == startNode) && curr->g < 1e8) {
                if(!curr->blocked) {
                    visitedOrder.push_back(curr);
                    // Check collision with End Set
                    if (biParents2.count(curr)) { reconstructBidirectional(curr, biParents1, biParents2); return; }

                    for(Node* n : getNeighbors(curr)) {
                        if(!biParents1.count(n) && !n->blocked) {
                            biParents1[n] = curr;
                            n->parent = curr; // Support pruning
                            n->visited = true; // Mark as part of search area
                            n->g = curr->g + 1; // Visuals
                            biIdx1.push_back(n);
                        }
                    }
                }
            }
        }
        
        // Step End Side
        if (appState != FINISHED && !biIdx2.empty()) {
            Node* curr = biIdx2.front(); biIdx2.pop_front();

            // Check if pruned/orphaned
            if ((curr->parent != nullptr || curr == endNode) && curr->g < 1e8) {
                if(!curr->blocked) {
                    visitedOrder.push_back(curr);
                    if (biParents1.count(curr)) { reconstructBidirectional(curr, biParents1, biParents2); return; }

                    for(Node* n : getNeighbors(curr)) {
                        if(!biParents2.count(n) && !n->blocked) {
                            biParents2[n] = curr;
                            n->parent = curr; // Support pruning
                            n->inOpenSet = true; // Mark as part of search area (Side 2)
                            n->g = curr->g + 1; // Visuals
                            biIdx2.push_back(n);
                        }
                    }
                }
            }
        }
    }
    // --- HEURISTIC (A*, Dijkstra, etc) ---
    else {
        if (openList.empty()) { appState = FINISHED; return; }

        pop_heap(openList.begin(), openList.end(), heapCmp);
        Node* current = openList.back(); openList.pop_back();

        // Check if pruned/orphaned (pointer in heap but data reset)
        if ((current->parent == nullptr && current != startNode) || current->g >= 1e8) return;
        if (current->blocked) return; // Skip if user walled it off
        
        if (current->visited) return; // Already processed
        current->visited = true;
        visitedOrder.push_back(current);

        if (current == endNode) { reconstructPath(endNode); return; }

        for (Node* neighbor : getNeighbors(current)) {
             if (neighbor->visited || neighbor->blocked) continue;

             float tentG = current->g + 1;
             if (tentG < neighbor->g) {
                 neighbor->parent = current;
                 neighbor->g = tentG;
                 if (currentAlgorithm == GREEDY) neighbor->g = 0;
                 neighbor->h = heuristic(neighbor, endNode, currentAlgorithm);
                 neighbor->f = neighbor->g + neighbor->h;

                 if (!neighbor->inOpenSet) {
                     openList.push_back(neighbor);
                     push_heap(openList.begin(), openList.end(), heapCmp);
                     neighbor->inOpenSet = true;
                 } else {
                     // Update Heap (Optimization: push duplicate instead of re-heapify)
                     // The 'visited' check at the top of the loop gracefully handles these duplicates later.
                     openList.push_back(neighbor);
                     push_heap(openList.begin(), openList.end(), heapCmp);
                 }
             }
        }
    }
}


// --- Main ---

int main() {
    // Random Seed
    srand(time(NULL));

    // Globals for Setup
    string strRows = "25";
    string strCols = "35";
    int activeField = 0; // 0=None, 1=Rows, 2=Cols

    // Font
    Font font;
    if(!font.openFromFile("assets/arial.ttf")) {
        cerr << "Error: Could not load font from assets/arial.ttf" << endl;
        return -1;
    }

    ContextSettings settings;
    settings.antiAliasingLevel = 8;
    // SFML 3.0: VideoMode takes Vector2u, and strings for Titles are implicit? Or String.
    RenderWindow window(VideoMode({600, 400}), "Erebus: Setup", Style::Default, State::Windowed, settings);
    window.setFramerateLimit(60);
    Clock clock;

    // View Management
    View gridView;
    View uiView;
    bool isFullscreen = false;

    // Helper to Initialize/Reset Simulation
    auto initSimulation = [&]() {
        // Calculate dynamic CELL_SIZE to fit screen
        VideoMode desktop = VideoMode::getDesktopMode();
        // SFML 3.0: desktop.size.x
        float availableW = (float)desktop.size.x * 0.9f - UI_WIDTH;
        float availableH = (float)desktop.size.y * 0.9f;
        
        float sizeW = availableW / COLS;
        float sizeH = availableH / ROWS;
        CELL_SIZE = min(sizeW, sizeH);
        if(CELL_SIZE < 5.0f) CELL_SIZE = 5.0f; // Minimum size cap

        WINDOW_WIDTH = (int)(COLS * CELL_SIZE + UI_WIDTH);
        WINDOW_HEIGHT = (int)(ROWS * CELL_SIZE);
        
        // Recreate Window
        // SFML 3.0: VideoMode({w, h})
        window.create(VideoMode({(unsigned int)WINDOW_WIDTH, (unsigned int)WINDOW_HEIGHT}), "Erebus: The Self-Optimizing Pathfinding Engine", Style::Default, State::Windowed, settings);
        window.setFramerateLimit(60);

        // Update Views
        float gridWidth = (float)COLS * CELL_SIZE;
        float totalWidth = gridWidth + UI_WIDTH;
        float gridRatio = gridWidth / totalWidth;
        float uiRatio = (float)UI_WIDTH / totalWidth;

        // SFML 3.0: Rect({pos}, {size})
        gridView = View(FloatRect({0, 0}, {gridWidth, (float)ROWS * CELL_SIZE}));
        uiView = View(FloatRect({0, 0}, {(float)UI_WIDTH, (float)WINDOW_HEIGHT}));
        
        gridView.setViewport(FloatRect({0, 0}, {gridRatio, 1.0f}));
        uiView.setViewport(FloatRect({gridRatio, 0}, {uiRatio, 1.0f}));

        // Resize Grid
        grid.clear();
        grid.resize(ROWS, vector<Node>(COLS));
        for(int i=0; i<ROWS; i++)
            for(int j=0; j<COLS; j++){
                grid[i][j].row = i;
                grid[i][j].col = j;
            }
        
        // Dynamic Start/End Position
        int midRow = ROWS / 2;
        int startCol = COLS / 5;
        int endCol = COLS - (COLS / 5) - 1;
        if (endCol <= startCol) endCol = COLS - 1;

        startNode = &grid[midRow][startCol];
        endNode = &grid[midRow][endCol];

        loadKnowledge();
        appState = IDLE;
    };
    
    // UI Buttons
    vector<Button> buttons;
    int btnY = 10;
    auto addBtn = [&](string text, Algorithm algo) {
        buttons.emplace_back(10, btnY, UI_WIDTH - 20, 30, text, font, [=]() {
            currentAlgorithm = algo;
            resetGridPathData();
        });
        btnY += 40;
    };
    
    addBtn("Dijkstra's Algorithm", DIJKSTRA);
    addBtn("A* Search", ASTAR);
    addBtn("Greedy Best-first Search", GREEDY);
    addBtn("Swarm Algorithm", SWARM);
    addBtn("Convergent Swarm Algo", CONVERGENT_SWARM);
    addBtn("Bidirectional Swarm", BIDIRECTIONAL);
    addBtn("Breadth-first Search", BFS);
    addBtn("Depth-first Search", DFS);

    btnY += 20;
    buttons.emplace_back(10, btnY, UI_WIDTH - 20, 40, "Visualize!", font, [&](){
        if(appState != RUNNING) startAlgorithm();
    });
    
    btnY += 45;
    buttons.emplace_back(10, btnY, UI_WIDTH - 20, 30, "Generate Maze", font, [&](){
        generateMaze();
    });

    btnY += 45;
    buttons.emplace_back(10, btnY, (UI_WIDTH - 30)/2, 30, "Faster <<", font, [&](){
        stepDelay *= 0.5f;
        if (stepDelay < 0.001f) stepDelay = 0.001f;
    });
    buttons.emplace_back(10 + (UI_WIDTH-30)/2 + 10, btnY, (UI_WIDTH - 30)/2, 30, "Slower >>", font, [&](){
        stepDelay *= 2.0f;
        if (stepDelay > 1.0f) stepDelay = 1.0f;
    });

    btnY += 40;
    buttons.emplace_back(10, btnY, (UI_WIDTH - 30)/2, 30, "Clear Walls", font, [&](){
        clearWalls();
    });
    buttons.emplace_back(10 + (UI_WIDTH-30)/2 + 10, btnY, (UI_WIDTH - 30)/2, 30, "Clear Path", font, [&](){
        resetGridPathData();
    });

    while(window.isOpen()) {
        while(const optional event = window.pollEvent()) {
            if(event->is<Event::Closed>()) window.close();

            if (const auto* resize = event->getIf<Event::Resized>()) {
                if (appState != SETUP) {
                    float newW = (float)resize->size.x;
                    float newH = (float)resize->size.y;
                    float gridWidth = (float)COLS * CELL_SIZE;
                    float totalWidth = gridWidth + UI_WIDTH;
                    float gridRatio = gridWidth / totalWidth;
                    float uiRatio = (float)UI_WIDTH / totalWidth;
                    
                    gridView.setViewport(FloatRect({0, 0}, {gridRatio, 1.0f}));
                    uiView.setViewport(FloatRect({gridRatio, 0}, {uiRatio, 1.0f}));
                } else {
                     window.setView(window.getDefaultView());
                }
            }

            if (const auto* keyPress = event->getIf<Event::KeyPressed>()) {
                if (keyPress->code == Keyboard::Key::F11 && appState != SETUP) {
                    isFullscreen = !isFullscreen;
                    window.create(VideoMode({(unsigned int)WINDOW_WIDTH, (unsigned int)WINDOW_HEIGHT}), 
                                 "Erebus: Self-Optimizing Engine", 
                                 Style::Default, 
                                 isFullscreen ? State::Fullscreen : State::Windowed, 
                                 settings);
                    window.setFramerateLimit(60);
                }
            }
            
            // --- SETUP INPUT ---
            if (appState == SETUP) {
                if (const auto* text = event->getIf<Event::TextEntered>()) {
                    if (text->unicode >= '0' && text->unicode <= '9') {
                        if (activeField == 1) strRows += (char)text->unicode;
                        if (activeField == 2) strCols += (char)text->unicode;
                    } else if (text->unicode == 8) { // Backspace
                        if (activeField == 1 && !strRows.empty()) strRows.pop_back();
                        if (activeField == 2 && !strCols.empty()) strCols.pop_back();
                    }
                }
                
                if (const auto* mouse = event->getIf<Event::MouseButtonPressed>()) {
                   Vector2i mPos = Mouse::getPosition(window);
                   // SFML 3.0 Rect({pos}, {size})
                   if (IntRect({200, 100}, {200, 40}).contains(mPos)) activeField = 1;
                   else if (IntRect({200, 180}, {200, 40}).contains(mPos)) activeField = 2;
                   else if (IntRect({200, 260}, {200, 50}).contains(mPos)) {
                       // Parsing
                       try {
                           ROWS = stoi(strRows);
                           COLS = stoi(strCols);
                       } catch(...) { ROWS=25; COLS=35; }
                       
                       if (ROWS < 5) ROWS = 5;
                       if (COLS < 5) COLS = 5;
                       
                       initSimulation();
                   }
                   else activeField = 0;
                }
            }

            else { // Not SETUP -> Simulation Interaction
                if(const auto* mousePress = event->getIf<Event::MouseButtonPressed>()) {
                    Vector2i pixelPos = Mouse::getPosition(window);
                    
                    // Sidebar Click
                    Vector2f uiPos = window.mapPixelToCoords(pixelPos, uiView);
                    if(uiPos.x >= 0 && uiPos.x <= UI_WIDTH && uiPos.y >= 0 && uiPos.y <= WINDOW_HEIGHT) {
                        for(auto& btn : buttons) {
                            if(btn.shape.getGlobalBounds().contains(uiPos)) btn.onClick();
                        }
                    }
                    // Grid Click
                    else {
                        Vector2f gridPos = window.mapPixelToCoords(pixelPos, gridView);
                        int r = gridPos.y / CELL_SIZE;
                        int c = gridPos.x / CELL_SIZE;
                        if(r >= 0 && r < ROWS && c >= 0 && c < COLS) {
                            Node* hovered = &grid[r][c];
                            if(hovered == startNode) currentInteraction = DRAG_START;
                            else if(hovered == endNode) currentInteraction = DRAG_END;
                            else {
                                currentInteraction = DRAW_WALL;
                                isAddingWall = !hovered->blocked; 
                                
                                if (isAddingWall && (hovered->visited || hovered->inOpenSet) && hovered != startNode && hovered != endNode) {
                                    pruneBranch(hovered);
                                }
                                hovered->blocked = isAddingWall;
                                if (!isAddingWall) reactivateNode(hovered);
                                if (appState == FINISHED && !isAddingWall) appState = RUNNING;
                            }
                        }
                    }
                }

                if(event->is<Event::MouseButtonReleased>()) {
                    currentInteraction = NONE;
                }

                if(event->is<Event::MouseMoved>()) {
                    Vector2i pixelPos = Mouse::getPosition(window);
                    
                    // Update button hover states
                    Vector2f uiPos = window.mapPixelToCoords(pixelPos, uiView);
                    for(auto& btn : buttons) {
                        btn.isHovered = btn.shape.getGlobalBounds().contains(uiPos);
                        if (btn.isHovered) {
                            btn.shape.setFillColor(Color(240, 240, 240));
                            btn.shape.setOutlineColor(Color(66, 133, 244));
                        } else {
                            btn.shape.setFillColor(Color::White);
                            btn.shape.setOutlineColor(Color(200, 200, 200));
                        }
                    }

                    Vector2f gridPos = window.mapPixelToCoords(pixelPos, gridView);
                    int r = gridPos.y / CELL_SIZE;
                    int c = gridPos.x / CELL_SIZE;
                    
                    if(r >= 0 && r < ROWS && c >= 0 && c < COLS) {
                        Node* hovered = &grid[r][c];

                        if(currentInteraction == DRAG_START && hovered != endNode && !hovered->blocked && hovered != startNode) {
                            startNode = hovered;
                        }
                        else if(currentInteraction == DRAG_END && hovered != startNode && !hovered->blocked && hovered != endNode) {
                            endNode = hovered;
                        }
                        else if(currentInteraction == DRAW_WALL && hovered != startNode && hovered != endNode) {
                              if (hovered->blocked != isAddingWall) {
                                  if (isAddingWall && (hovered->visited || hovered->inOpenSet)) {
                                       pruneBranch(hovered);
                                  }
                                  hovered->blocked = isAddingWall;
                                  if (!isAddingWall) reactivateNode(hovered);
                                  if (appState == FINISHED && !isAddingWall) appState = RUNNING;
                              }
                        }
                    }
                }
            }
        }

        // --- Logic Update (Timer-based Iterative Step) ---
        if(appState == RUNNING) {
            if (stepClock.getElapsedTime().asSeconds() >= stepDelay) {
                stepAlgorithm();
                stepClock.restart();
            }
        }

        // --- Button Animation ---
        if (buttons.size() > 8) {
            Button& visBtn = buttons[8];
            if(appState == RUNNING) {
                visBtn.setText("Visualizing...");
                float time = clock.getElapsedTime().asSeconds();
                float pulse = (sin(time * 10) + 1) / 2; // 0 to 1
                visBtn.shape.setFillColor(Color(100 + pulse*50, 255, 100 + pulse*50)); 
            } else {
                 if(visBtn.label.getString() != "Visualize!") {
                     visBtn.setText("Visualize!");
                     visBtn.shape.setFillColor(Color::White);
                 }
            }
        }

        // --- Render ---
        window.clear(COLOR_BG);

        if (appState == SETUP) {
             window.setView(window.getDefaultView());
             
             // SFML 3.0: Text(font, string, size)
             Text title(font, "Erebus Configuration", 30); 
             title.setPosition({150, 30});
             title.setFillColor(Color::Black);
             window.draw(title);

             auto drawBox = [&](int x, int y, string label, string val, bool active) {
                 Text lbl(font, label, 20);
                 lbl.setPosition({(float)x, (float)y - 25});
                 lbl.setFillColor(Color::Black);
                 window.draw(lbl);

                 RectangleShape box(Vector2f(200, 40));
                 box.setPosition({(float)x, (float)y});
                 box.setFillColor(Color::White);
                 box.setOutlineThickness(active ? 2 : 1);
                 box.setOutlineColor(active ? Color(66, 133, 244) : Color::Black);
                 window.draw(box);

                 Text tVal(font, val + (active && (int)clock.getElapsedTime().asMilliseconds() % 1000 < 500 ? "|" : ""), 20);
                 tVal.setPosition({(float)x + 10, (float)y + 8});
                 tVal.setFillColor(Color::Black);
                 window.draw(tVal);
             };

             drawBox(200, 100, "Rows:", strRows, activeField == 1);
             drawBox(200, 180, "Columns:", strCols, activeField == 2);

             // Start Button
             RectangleShape startBtn(Vector2f(200, 50));
             startBtn.setPosition({200, 260});
             bool hoverStart = IntRect({200, 260}, {200, 50}).contains(Mouse::getPosition(window));
             startBtn.setFillColor(hoverStart ? Color(50, 100, 200) : Color(66, 133, 244));
             window.draw(startBtn);

             Text btnTxt(font, "Start Simulation", 20);
             btnTxt.setPosition({225, 272});
             btnTxt.setFillColor(Color::White);
             window.draw(btnTxt);
             
        } else {
            // Draw Grid
            window.setView(gridView);
            for(int i=0; i<ROWS; i++){
                for(int j=0; j<COLS; j++){
                    RectangleShape cell(Vector2f(CELL_SIZE, CELL_SIZE));
                    cell.setPosition({(float)j*CELL_SIZE, (float)i*CELL_SIZE});
                    if(grid[i][j].blocked) {
                        cell.setSize(Vector2f(CELL_SIZE-2.0f, CELL_SIZE-2.0f));
                        cell.setPosition({(float)j*CELL_SIZE+1.0f, (float)i*CELL_SIZE+1.0f});
                        cell.setFillColor(COLOR_WALL);
                        window.draw(cell);
                    } else {
                        cell.setFillColor(COLOR_ROAD);
                        cell.setOutlineThickness(-1);
                        cell.setOutlineColor(Color(220, 220, 220));
                        window.draw(cell);
                    }
                }
            }
            
            for(Node* n : visitedOrder) {
                 if(n == startNode || n == endNode) continue;
                 RectangleShape v(Vector2f(CELL_SIZE, CELL_SIZE));
                 v.setPosition({(float)n->col*CELL_SIZE, (float)n->row*CELL_SIZE});
                 int dist = (int)n->g;
                 int r_val = max(0, min(255, 0 + dist * 2)); 
                 int g_val = max(0, min(255, 200 - dist * 2));
                 v.setFillColor(Color(r_val, g_val, 255));
                 v.setOutlineThickness(-1.f);
                 v.setOutlineColor(Color(255, 255, 255, 60)); 
                 window.draw(v);
            }

            if(appState == FINISHED && !path.empty()) {
                float thickness = CELL_SIZE / 4.0f; // Scale path thickness
                VertexArray ribbon(PrimitiveType::TriangleStrip);
                for(size_t i = 0; i < path.size(); ++i) {
                    Vector2f currentP(path[i]->col * CELL_SIZE + CELL_SIZE/2.0f, path[i]->row * CELL_SIZE + CELL_SIZE/2.0f);
                    Vector2f nextP = currentP, prevP = currentP;
                    if (i < path.size() - 1) nextP = Vector2f(path[i+1]->col * CELL_SIZE + CELL_SIZE/2.0f, path[i+1]->row * CELL_SIZE + CELL_SIZE/2.0f);
                    if (i > 0) prevP = Vector2f(path[i-1]->col * CELL_SIZE + CELL_SIZE/2.0f, path[i-1]->row * CELL_SIZE + CELL_SIZE/2.0f);
                    Vector2f dir = nextP - prevP;
                    if (dir.x == 0 && dir.y == 0) {
                        if (i < path.size() - 1) dir = nextP - currentP;
                        else if (i > 0) dir = currentP - prevP;
                    }
                    float len = sqrt(dir.x*dir.x + dir.y*dir.y);
                    if (len != 0) dir /= len;
                    Vector2f perp(-dir.y, dir.x);
                    ribbon.append({currentP + perp * (thickness/2.0f), COLOR_PATH_LINE});
                    ribbon.append({currentP - perp * (thickness/2.0f), COLOR_PATH_LINE});
                }
                window.draw(ribbon);
                CircleShape cap(thickness/2.0f);
                cap.setOrigin({thickness/2.0f, thickness/2.0f});
                cap.setFillColor(COLOR_PATH_LINE);
                if (!path.empty()) {
                    cap.setPosition({(float)path.front()->col*CELL_SIZE + CELL_SIZE/2, (float)path.front()->row*CELL_SIZE + CELL_SIZE/2});
                    window.draw(cap);
                    cap.setPosition({(float)path.back()->col*CELL_SIZE + CELL_SIZE/2, (float)path.back()->row*CELL_SIZE + CELL_SIZE/2});
                    window.draw(cap);
                }
            }

            drawStart(window, startNode->col * CELL_SIZE, startNode->row * CELL_SIZE);
            drawPin(window, endNode->col * CELL_SIZE, endNode->row * CELL_SIZE);

            window.setView(uiView);
            RectangleShape sidebar(Vector2f(UI_WIDTH, WINDOW_HEIGHT));
            sidebar.setPosition({0, 0});
            sidebar.setFillColor(Color(245, 245, 245));
            sidebar.setOutlineThickness(-1);
            sidebar.setOutlineColor(Color::Black);
            window.draw(sidebar);

            for(int i=0; i<buttons.size(); i++) {
                 if (i < 8) buttons[i].setSelected(i == (int)currentAlgorithm); 
                 buttons[i].draw(window);
            }
        }
        window.display();
    }
    return 0;
}
