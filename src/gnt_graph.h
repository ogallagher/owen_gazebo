#ifndef GNT_GRAPH_H
#define GNT_GRAPH_H

/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include <iostream>
#include <vector>

using namespace std;

struct TreeEdge {
	double x1,y1,x2,y2;
};

class TreeNode {
	public:
		enum NodeType type;
		double x,y,radius,reach;
		bool visible;
		bool explored;
		
	    vector<TreeNode> children;
	    vector<TreeEdge> branches;
		
		TreeNode();
		TreeNode(NodeType t, double x, double y, double rad, double rch);
		void addChild(TreeNode* child);
		void moveTo(double x, double y);
		void move(double x, double y);
};

class GNTGraph {
	public:
	    TreeNode* root;
		double branchLength;
		double branchWidth;
		double nodeRadius;
		
		GNTGraph();
		GNTGraph(double x, double y);
};

#endif
