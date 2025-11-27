#include <bits/stdc++.h>
using namespace std;

struct Node {
    int val;
    Node* left;
    Node* right;
    Node(int v): val(v), left(nullptr), right(nullptr) {}
};

int findIndex(vector<int>& inorder, int start, int end, int value) {
    for (int i = start; i <= end; i++)
        if (inorder[i] == value) return i;
    return -1;
}

Node* buildPreIn(vector<int>& preorder, vector<int>& inorder, int& preIndex, int inStart, int inEnd) {
    if (inStart > inEnd) return nullptr;
    Node* root = new Node(preorder[preIndex++]);
    int pos = findIndex(inorder, inStart, inEnd, root->val);
    root->left = buildPreIn(preorder, inorder, preIndex, inStart, pos - 1);
    root->right = buildPreIn(preorder, inorder, preIndex, pos + 1, inEnd);
    return root;
}

int main() {
    vector<int> preorder = {3,9,20,15,7};
    vector<int> inorder  = {9,3,15,20,7};
    int preIndex = 0;
    Node* root = buildPreIn(preorder, inorder, preIndex, 0, inorder.size()-1);
    return 0;
}
