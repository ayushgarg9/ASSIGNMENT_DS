#include <iostream>
#include <algorithm> // for std::max, std::min
using namespace std;

class Node {
public:
    int data;
    Node *lchild;
    Node *rchild;

    Node(int data) {
        this->data = data;
        lchild = nullptr;
        rchild = nullptr;
    }
};

class BST_implementation {
public:
    Node *root;

    BST_implementation() {
        root = nullptr;
    }

    void insert_unique_element(int value) {
        Node *newNode = new Node(value);

        if (root == nullptr) {
            root = newNode;
            return;
        }

        Node *current = root;
        Node *parent = nullptr;

        while (current != nullptr) {
            parent = current;

            if (value < current->data)
                current = current->lchild;
            else if (value > current->data)
                current = current->rchild;
            else {
                cout << "Same values cannot be inserted in the BST!" << endl;
                delete newNode;
                return;
            }
        }

        if (value > parent->data)
            parent->rchild = newNode;
        else
            parent->lchild = newNode;
    }

    Node* min_node(Node* node) {
        Node* current = node;
        while (current && current->lchild != nullptr)
            current = current->lchild;
        return current;
    }

    Node* deleteElement(Node* root, int value) {
        if (root == nullptr)
            return root;

        if (value < root->data)
            root->lchild = deleteElement(root->lchild, value);
        else if (value > root->data)
            root->rchild = deleteElement(root->rchild, value);
        else {
            if (root->lchild == nullptr) {
                Node* temp = root->rchild;
                delete root;
                return temp;
            } else if (root->rchild == nullptr) {
                Node* temp = root->lchild;
                delete root;
                return temp;
            }

            Node* temp = min_node(root->rchild);
            root->data = temp->data;
            root->rchild = deleteElement(root->rchild, temp->data);
        }
        return root;
    }

    int maxDepth(Node *root){
        if (root == nullptr) {               // <<-- fixed: use == not =
            return 0;
        }
        int L = maxDepth(root->lchild);
        int R = maxDepth(root->rchild);
        return 1 + std::max(L, R);
    }

    int minDepth(Node* root){
        if (root == nullptr) {               // <<-- fixed: use == not =
            return 0;
        }
        if (root->lchild == nullptr && root->rchild == nullptr){
            return 1;
        }
        if (root->lchild == nullptr){
            return 1 + minDepth(root->rchild);
        }
        if (root->rchild == nullptr){
            return 1 + minDepth(root->lchild);
        }

        return 1 + std::min(minDepth(root->lchild), minDepth(root->rchild));
    }

    void inorder_traversal(Node* root){
        if(root != nullptr){
            inorder_traversal(root->lchild);
            cout << root->data << " ";
            inorder_traversal(root->rchild);
        }
    }

    void preorder_traversal(Node* root){
        if(root != nullptr){
            cout << root->data << " ";
            preorder_traversal(root->lchild);
            preorder_traversal(root->rchild);
        }
    }

    void postorder_traversal(Node* root){
        if(root != nullptr){
            postorder_traversal(root->lchild);
            postorder_traversal(root->rchild);
            cout << root->data << " ";
        }
    }

};

int main() {
    BST_implementation B1;

    B1.insert_unique_element(4);
    B1.insert_unique_element(2);
    B1.insert_unique_element(6);
    B1.insert_unique_element(1);
    B1.insert_unique_element(3);
    B1.insert_unique_element(5);
    B1.insert_unique_element(7);

    B1.inorder_traversal(B1.root);
    cout << endl;
    B1.preorder_traversal(B1.root);
    cout << endl;
    B1.postorder_traversal(B1.root);


    return 0;
}
