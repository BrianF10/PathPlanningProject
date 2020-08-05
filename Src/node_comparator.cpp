#include "node_comparator.h"

bool NodePosComparator::operator()(const Node &first, const Node &second) const {
    return first.F <= second.F;
}
