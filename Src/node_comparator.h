#ifndef NODE_COMPARATOR_H
#define NODE_COMPARATOR_H

#include "node.h"
#include "environmentoptions.h"
#include <cmath>

struct NodePosComparator {
    bool operator()(const Node &first, const Node &second) const;
};

#endif // NODE_COMPARATOR_H
