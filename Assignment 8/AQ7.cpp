#include <bits/stdc++.h>
using namespace std;

struct Node {
    int val;
    Node* left;
    Node* right;
    Node(int v): val(v), left(nullptr), right(nullptr) {}
};

void inorder(Node* root, vector<int>& out) {
    if (!root) return;
    inorder(root->left, out);
    out.push_back(root->val);
    inorder(root->right, out);
}

vector<int> mergeSorted(const vector<int>& a, const vector<int>& b) {
    vector<int> res;
    int i = 0, j = 0;
    while (i < (int)a.size() && j < (int)b.size()) {
        if (a[i] <= b[j]) res.push_back(a[i++]);
        else res.push_back(b[j++]);
    }
    while (i < (int)a.size()) res.push_back(a[i++]);
    while (j < (int)b.size()) res.push_back(b[j++]);
    return res;
}

Node* buildDLL(const vector<int>& vals) {
    if (vals.empty()) return nullptr;
    Node* head = new Node(vals[0]);
    Node* prev = head;
    for (int k = 1; k < (int)vals.size(); ++k) {
        Node* cur = new Node(vals[k]);
        prev->right = cur;
        cur->left = prev;
        prev = cur;
    }
    return head;
}

void printDLL(Node* head) {
    Node* cur = head;
    while (cur) {
        cout << cur->val;
        if (cur->right) cout << " <--> ";
        else cout << " <--> null";
        cur = cur->right;
    }
    cout << "\n";
}

int main() {
    Node* T1 = new Node(20);
    T1->left = new Node(10);
    T1->right = new Node(30);
    T1->right->left = new Node(25);
    T1->right->right = new Node(100);

    Node* T2 = new Node(50);
    T2->left = new Node(5);
    T2->right = new Node(70);

    vector<int> a, b;
    inorder(T1, a);
    inorder(T2, b);

    vector<int> merged = mergeSorted(a, b);

    Node* head = buildDLL(merged);
    printDLL(head);

    return 0;
}
