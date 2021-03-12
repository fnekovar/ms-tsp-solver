#ifndef __TARGET_H__
#define __TARGET_H__

class TargetSet {
public:
    int index;
    int inCoordIndex;

    TargetSet(const int n, const int i) : index(n), inCoordIndex(i) {}

    void reverse() {
        inCoordIndex = (inCoordIndex + 1) % 2;
    }

    int const cost_index() {
        return index + inCoordIndex;
    }

    bool operator==(const TargetSet& rhs)
    {
        return (this->index + this->inCoordIndex) == (rhs.index + rhs.inCoordIndex);
    }

};

//    bool operator==(const TargetSet& rhs)
//    {
//        return (index + inCoordIndex) == (rhs.index + rhs.inCoordIndex);
//    }
inline bool operator==(const TargetSet& lhs, const TargetSet& rhs)
{
    return (lhs.index + lhs.inCoordIndex) == (rhs.index + rhs.inCoordIndex);
}

typedef std::vector<TargetSet> TargetSetVector;
typedef std::vector<std::vector<TargetSet>> TargetSetVectorVector;

#endif
