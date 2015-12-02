#include "UnionSetUtils.h"
#include <set>

UnionSet::UnionSet(int N){
	par.resize(N);
	rank.resize(N, 1);
	for (int i = 0; i < N; ++i){ par[i] = i; }
}

int UnionSet::Find(int x){
	int i, j, r;
	r = x;
	while (par[r] != r){ r = par[r]; }
	i = x;
	while (i != r){
		j = par[i];
		par[i] = r;
		i = j;
	}
	return r;
}
void UnionSet::Merge(int x, int y){
	int a = Find(x);
	int b = Find(y);
	if (a == b)	return;
	if (rank[a] > rank[b]){
		par[b] = a;
		rank[a] += rank[b];
	}
	else{
		par[a] = b;
		rank[b] += rank[a];
	}
}
int UnionSet::ProminentRepresent(){
	int maxRank = rank[0];
	int r = 0;
	for (int i = 1; i < (int)rank.size(); ++i){
		if (maxRank < rank[i]){
			maxRank = rank[i];
			r = i;
		}
	}
	return r;
}

int UnionSet::ConnectRegion(){
	std::set<int> component;
	for (int i = 0; i < par.size(); ++i){
		component.insert(Find(i));
	}
	return component.size();
}