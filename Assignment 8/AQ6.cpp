#include <bits/stdc++.h>
using namespace std;

struct Node {
    int val;
    Node* left;
    Node* right;
    Node(int v): val(v), left(nullptr), right(nullptr) {}
};

int findIndexIn(vector<int>& inorder, int start, int end, int value) {
    for (int i = start; i <= end; i++)
        if (inorder[i] == value) return i;
    return -1;
}

Node* buildInPost(vector<int>& inorder, vector<int>& postorder, int& postIndex, int inStart, int inEnd) {
    if (inStart > inEnd) return nullptr;
    Node* root = new Node(postorder[postIndex--]);
    int pos = findIndexIn(inorder, inStart, inEnd, root->val);
    root->right = buildInPost(inorder, postorder, postIndex, pos + 1, inEnd);
    root->left = buildInPost(inorder, postorder, postIndex, inStart, pos - 1);
    return root;
}

int main() {
    vector<int> inorder   = {9,3,15,20,7};
    vector<int> postorder = {9,15,7,20,3};
    int postIndex = postorder.size() - 1;
    Node* root = buildInPost(inorder, postorder, postIndex, 0, inorder.size()-1);
    return 0;
}
