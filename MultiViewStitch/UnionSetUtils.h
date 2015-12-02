#ifndef UNION_SET_UTILS_H
#define UNION_SET_UTILS_H

#include <vector>

class UnionSet{
public:
	UnionSet(int N = 0);
	int Find(int x);
	void Merge(int x, int y);
	int ProminentRepresent();
	int ConnectRegion();
private:
	std::vector<int> par;
	std::vector<int> rank;
};

#endif