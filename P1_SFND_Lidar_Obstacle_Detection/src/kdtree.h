/* \author Aaron Brown */
// Quiz on implementing kd tree
#pragma once
#include <vector>
#include <cmath>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(Node **root, int depth, std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(*root == nullptr) {
			*root = new Node(point, id);
		} else {
			int cd = depth % 2;
			int v = (*root)->point[cd];	
			if(point[cd] < v) {
				insert(&((*root)->left), depth + 1, point, id);
			} else {
				insert(&((*root)->right), depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert(&root, 0, point, id);
	}

	float distance(std::vector<float> &x, std::vector<float> &y) {
		float a = x[0] - y[0];
		float b = x[1] - y[1];
		float c = x[2] - y[2];
		return sqrt(a * a + b * b + c * c);
	}

	void search(Node *root, int depth, std::vector<float> target, float distanceTol, std::vector<int> &ans) 
	{
		if(!root) return;
		float x0 = root->point[0], y0 = root->point[1], z0 = root->point[2];
		float xt = target[0], yt = target[1], zt = target[2];

		if(fabs(x0 - xt) <= distanceTol && fabs(y0 - yt) <= distanceTol  && fabs(z0 - zt) <= distanceTol) 
		{
			if(distance(root->point, target) <= distanceTol) {
				ans.push_back(root->id);
			}
		}


		if(target[depth % 3] - distanceTol < root->point[depth % 3]) {
			search(root->left, depth + 1, target, distanceTol, ans);
		} 
		if(target[depth % 3] + distanceTol > root->point[depth % 3]) {
			search(root->right, depth + 1, target, distanceTol, ans);
		} 
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(root, 0, target, distanceTol, ids);
		return ids;
	}
	

};




