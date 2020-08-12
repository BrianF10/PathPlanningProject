#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {}

double Search::computeHeuristic(int i1, int j1, int i2, int j2, const EnvironmentOptions &options) {
    double H = 0.0;

    if (options.searchtype == CN_SP_ST_DIJK) {
        return H;
    }

    double dx = abs(i1 - i2);
    double dy = abs(j1 - j2);

    if (options.metrictype == CN_SP_MT_EUCL) {
        H = sqrt(dx * dx + dy * dy);
        return H;
    }

    if (options.metrictype == CN_SP_MT_MANH) {
        H = dx + dy;
        return H;
    }

    if (options.metrictype == CN_SP_MT_DIAG) {
        H = std::min(dx, dy) * (sqrt(2) - 1) + std::max(dx, dy);
        return H;
    }

    if (options.metrictype == CN_SP_MT_CHEB) {
        H = std::max(dx, dy);
        return H;
    }
}

std::vector<Node> Search::get_neighbours(Node currentNode, const Map &map, const EnvironmentOptions &options) {

    std::vector<Node> neighbors;
    Node neighbor;
    for (int i = -1 ; i < 2; ++i) {
        for (int j = -1; j < 2; ++j) {
            if (i != 0 || j != 0) {
                if (map.CellOnGrid(currentNode.i + i, currentNode.j + j) &&
                    map.CellIsTraversable(currentNode.i + i, currentNode.j + j)) {
                    bool allow = true;
                    if (options.allowdiagonal) {
                        if (i != 0 && j != 0) {
                            if (!(options.cutcorners)) {
                                /*if (!(map.CellIsTraversable(currentNode.i + i, currentNode.j)) ||
                                    !(map.CellIsTraversable(currentNode.i, currentNode.j + j))) {
                                        allow = false;
                                }*/
                                if (!(map.CellIsTraversable(currentNode.i + i, currentNode.j) &&
                                    map.CellIsTraversable(currentNode.i, currentNode.j + j))) {
                                        allow = false;
                                }
                                /*if (!(options.allowsqueeze)) {
                                    if (!(map.CellIsTraversable(currentNode.i + i, currentNode.j)) &&
                                    !(map.CellIsTraversable(currentNode.i, currentNode.j + j))) {
                                        allow = false;
                                    }
                                }*/
                                if (!(options.allowsqueeze) && allow) {
                                    if (!(map.CellIsTraversable(currentNode.i + i, currentNode.j) ||
                                    map.CellIsTraversable(currentNode.i, currentNode.j + j))) {
                                        allow = false;
                                    }
                                }
                            }
                        }
                    } else {
                        if (i * j != 0) {
                            allow = false; 
                        }

                    }
                
                    if ((CLOSE.find((currentNode.i + i) + (map.getMapHeight() * (currentNode.j + j))) == CLOSE.end()) && allow) {
                        neighbor.i = currentNode.i + i;
                        neighbor.j = currentNode.j + j;
                        if ((i != 0) && (j != 0)) {
                            neighbor.g = currentNode.g + sqrt(2);
                        } else {
                            neighbor.g = currentNode.g + 1.0;
                        }
                        neighbor.H = computeHeuristic(neighbor.i, neighbor.j, map.getGoali(), map.getGoalj(), options);
                        neighbor.F = neighbor.g + options.hweigth * neighbor.H;

                        neighbors.push_back(neighbor);
                    }
                }
            }
        }
    }
    return neighbors;
}

//Node Search::FindMin(const EnvironmentOptions &options, std::set<Node, NodePosComparator>::iterator iter)
Node Search::FindMin(const EnvironmentOptions &options) 
{
    //auto it = OPEN.begin();
    //Node minNode = *it;
    Node minNode = *OPEN.begin();
    if (options.searchtype == CN_SP_ST_DIJK) {
        return minNode;
    }
    double it_f = OPEN.begin()->F;
    /*std::cout << "Go: ";
    std::cout << " i: " << minNode.i;
    std::cout << " j: " << minNode.j << std::endl;*/

    for (auto iter = OPEN.begin(); iter != OPEN.end(); ++iter) {
        if (it_f == iter->F) {
        //if (abs(it_f - iter->F) <= 0.001) {
            if (options.breakingties) {
                if (iter->g >= minNode.g) {    
                    minNode = *iter;
                    /*std::cout << "optim F: " << minNode.F;
                    std::cout << " g: " << minNode.g;
                    std::cout << " H: " << minNode.H;
                    std::cout << " i: " << minNode.i;
                    std::cout << " j: " << minNode.j << std::endl;*/
                } 
                /*else {
                    std::cout << "not F: " << iter->F;
                    std::cout << " g: " << iter->g;
                    std::cout << " H: " << iter->H;
                    std::cout << " i: " << iter->i;
                    std::cout << " j: " << iter->j << std::endl;
                }*/
            } else {
                if (iter->g <= minNode.g) {
                    minNode = *iter;
                }
            }       
        } else {
            break;
        }
    }

    /*std::cout << "*min F: " << minNode.F;
    std::cout << "*min g: " << minNode.g;
    std::cout << "*min H: " << minNode.H;
    std::cout << "*min i: " << minNode.i;
    std::cout << "*min j: " << minNode.j << std::endl;*/

    return minNode;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    std::chrono::time_point<std::chrono::system_clock> start, finish;
    start = std::chrono::system_clock::now();
    
    /*std::cout << "options.breakingties: ";
    std::cout << options.breakingties << std::endl;*/

    Node currentNode;
    currentNode.parent = nullptr;
    currentNode.i = map.getStarti();
    currentNode.j = map.getStartj();
    currentNode.g = 0.0;
    currentNode.H = computeHeuristic(currentNode.i, currentNode.j, map.getGoali(), map.getGoalj(), options);
    currentNode.F = currentNode.g + options.hweigth * currentNode.H;
    OPEN.insert(currentNode);

    bool pathFound = false;
    int i = 0;
    while (!OPEN.empty()) {
        //Node currentNode = *OPEN.begin();
        currentNode = FindMin(options);
        
        CLOSE.insert({currentNode.i + (currentNode.j * map.getMapHeight()), currentNode});
        //OPEN.erase(OPEN.begin());
        auto curr_it = find(OPEN.begin(), OPEN.end(), currentNode);
        OPEN.erase(curr_it);
        
        if ((currentNode.i == map.getGoali()) && (currentNode.j == map.getGoalj())) {
            pathFound = true;
            sresult.pathlength = currentNode.g;
            makePrimaryPath(currentNode);
            break;
            
        }

        std::vector<Node> neighbors = get_neighbours(currentNode, map, options);
        for (auto& node : neighbors) {
            std::set<Node>::iterator it;
            it = find(OPEN.begin(), OPEN.end(), node);
            if(it != OPEN.end()) {
                if(node.F < it->F) {
                    OPEN.erase(it);
                    node.parent = &(CLOSE.find(currentNode.i + (map.getMapHeight() * currentNode.j))->second);
                    OPEN.insert(node);
                } else if (node.F == it->F && options.breakingties && node.g >= it->g) {
                    OPEN.erase(it);
                    node.parent = &(CLOSE.find(currentNode.i + (map.getMapHeight() * currentNode.j))->second);
                    OPEN.insert(node);
                } else if (node.F == it->F && !(options.breakingties) && node.g <= it->g) {
                    OPEN.erase(it);
                    node.parent = &(CLOSE.find(currentNode.i + (map.getMapHeight() * currentNode.j))->second);
                    OPEN.insert(node);
                }
            } else {
                node.parent = &(CLOSE.find(currentNode.i + (map.getMapHeight() * currentNode.j))->second);
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
    
    /*std::cout << "Len of list: " << lppath.size();
    std::cout << "breakingties: " << options.breakingties;*/

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

void Search::makePrimaryPath(Node currentNode)
{
    Node thisNode = currentNode;

    while (thisNode.parent != nullptr) { 
        lppath.push_front(thisNode);
        thisNode = *(thisNode.parent);
    }
    lppath.push_front(thisNode);
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
