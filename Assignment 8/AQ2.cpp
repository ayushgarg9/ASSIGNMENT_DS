#include <iostream>
#include <vector>
using namespace std;

class Node {
public:
    int data;
    Node* lchild;
    Node* rchild;
    Node(int v) : data(v), lchild(nullptr), rchild(nullptr) {}
};

vector<Node*> generateBSTs(int start, int end) {
    vector<Node*> result;

    if (start > end) {
        result.push_back(nullptr);
        return result;
    }

    for (int i = start; i <= end; ++i) {
        vector<Node*> leftList = generateBSTs(start, i - 1);
        vector<Node*> rightList = generateBSTs(i + 1, end);

        for (Node* left : leftList) {
            for (Node* right : rightList) {
                Node* root = new Node(i);
                root->lchild = left;
                root->rchild = right;
                result.push_back(root);
            }
        }
    }
    return result;
}

vector<Node*> generateTrees(int n) {
    if (n == 0) return {};
    return generateBSTs(1, n);
}

void printTree(Node* root) {
    if (!root) {
        cout << "null ";
        return;
    }
    cout << root->data << " ";
    printTree(root->lchild);
    printTree(root->rchild);
}

int main() {
    int n = 3;
    vector<Node*> trees = generateTrees(n);

    cout << "Total trees = " << trees.size() << "\n";
    for (size_t i = 0; i < trees.size(); ++i) {
        cout << "Tree " << (i+1) << ": ";
        printTree(trees[i]);
        cout << "\n";
    }

    return 0;
}
