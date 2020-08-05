#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {}

double Search::computeHeuristic(int i1, int j1, int i2, int j2, const EnvironmentOptions &options) {
    double H = 0.0;

    int dx = abs(i1 - i2);
    int dy = abs(j1 - j1);

    if (options.metrictype == CN_SP_MT_DIAG) {
        H = std::min(dx, dy) * (sqrt(2) - 1) + std::max(dx, dy);
    }

    if (options.metrictype == CN_SP_MT_MANH) {
        H = dx + dy;
    }

    if (options.metrictype == CN_SP_MT_EUCL) {
        H = sqrt(dx * dx + dy * dy);
    }

    if (options.metrictype == CN_SP_MT_CHEB) {
        H = std::max(dx, dy);
    }

    return H;
}

std::vector<Node> Search::get_neighbours(Node currentNode, const Map &map, const EnvironmentOptions &options) {

    std::vector<Node> neighbors;
    Node neighbor;
    for (int i = -1 ; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            if (i != 0 || j != 0) {
                if (map.CellOnGrid(currentNode.i + i, currentNode.j + j) &&
                    map.CellIsTraversable(currentNode.i + i, currentNode.j + j)) {
                    if ((CLOSE.find((currentNode.i + i) + (map.getMapHeight() * (currentNode.j + j))) == CLOSE.end())) {
                        neighbor.i = currentNode.i + i;
                        neighbor.j = currentNode.j + j;
                        if ((i != 0) && (j != 0)) {
                            neighbor.g = currentNode.g + sqrt(2);
                        } else {
                            neighbor.g = currentNode.g + 1.0;
                        }
                        neighbor.H = computeHeuristic(neighbor.i, neighbor.j, map.getGoali(), map.getGoalj(), options);
                        neighbor.F = neighbor.g + neighbor.H;
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }
    }
    return neighbors;
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start, finish;
    start = std::chrono::system_clock::now();

    Node currentNode;
    currentNode.parent = nullptr;
    currentNode.i = map.getStarti();
    currentNode.j = map.getStartj();
    currentNode.g = 0.0;
    currentNode.H = computeHeuristic(currentNode.i, currentNode.j, map.getGoali(), map.getGoalj(), options);
    currentNode.F = currentNode.g + currentNode.H;
    OPEN.insert(currentNode);

    bool pathFound = false;

    while (!OPEN.empty()) {
        Node currentNode = *OPEN.begin();
        CLOSE.insert({currentNode.i + (currentNode.j * map.getMapHeight()), currentNode});
        OPEN.erase(OPEN.begin());

        if ((currentNode.i == map.getGoali()) && (currentNode.j == map.getGoalj())) {
            pathFound = true;
            sresult.pathlength = currentNode.g;
            break;
        }

        std::vector<Node> neighbors = get_neighbours(currentNode, map, options);
        for (auto& node : neighbors) {
            std::set<Node>::iterator it;
            it = find(OPEN.begin(), OPEN.end(), node);
            if(it != OPEN.end()) {
                if(node.F <= it->F) {
                    OPEN.erase(it);
                    node.parent = &currentNode;
                    OPEN.insert(node);
                }
            } else {
                node.parent = &currentNode;
                OPEN.insert(node);
            }
        }
    }

    finish = std::chrono::system_clock::now();
    sresult.time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count()) / 1000000000;

    sresult.pathfound = pathFound;
    sresult.nodescreated = CLOSE.size() + OPEN.size();
    sresult.numberofsteps = CLOSE.size();

    sresult.hppath = &hppath;
    sresult.lppath = &lppath;

    return sresult;
    //need to implement

    /*sresult.pathfound = ;
    sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;*/

    //return sresult;
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
