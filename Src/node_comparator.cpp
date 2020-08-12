#include "node_comparator.h"


/*bool NodePosComparator::operator()(const Node &first, const Node &second) const {
    return first.F <= second.F;    
}*/

bool NodePosComparator::operator()(const Node &first, const Node &second) const {
    return first.F <= second.F; 
    /*if (first.F <= second.F) {
        return true;
    }
    return false;*/    
}
