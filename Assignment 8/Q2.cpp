#include <iostream>
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
        if (root == nullptr) {               
            return 0;
        }
        int L = maxDepth(root->lchild);
        int R = maxDepth(root->rchild);
        return 1 + std::max(L, R);
    }

    int minDepth(Node* root){
        if (root == nullptr) {              
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

        return 1 + min(minDepth(root->lchild), minDepth(root->rchild));
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

    void Recursive_search(Node* root, int value){
        if(root == nullptr){
            cout << "Value not found! ";
            return;
        }
        if(root->data == value) {
            cout << "Element found!";
            return;
        }
        if(value < root->data){
            Recursive_search(root->lchild, value);
        }
        else if(value > root->data){
            Recursive_search(root->rchild, value);
        }
    }

    void Iterative_search(Node* root, int value){
        Node* current = root;

        while(current != nullptr && current->data != value){
            if(value < current->data){
                current = current->lchild;
            }
            else if(value > current->data){
                current =current->rchild;
            }
        }
        if(current == nullptr){
            cout << "Value not found!";
        }
        else{
            cout << "Value found!";
        }
    }

    int max_element(Node* root){
        if (root == nullptr){
            cout << "Tree is empty!";
            return -1;
        }

        Node *current = root;
        while(current->rchild != nullptr){
            current = current->rchild;
        }
        return current->data;
    }

    int min_element(Node* root){
        if (root == nullptr){
            cout << "Tree is empty!";
            return -1;
        }

        Node *current = root;
        while(current->lchild != nullptr){
            current = current->lchild;
        }
        return current->data;
    }

    int inorder_successor(Node* root, int value) {
        if (root == nullptr) {
            cout << "Tree is empty! Cannot find the inorder successor!\n";
            return -1;
        }

        Node* current = root;
        Node* target = nullptr;
        Node* successor = nullptr; 

        while (current != nullptr) {
            if (value < current->data) {
                successor = current;
                current = current->lchild;
            } else if (value > current->data) {
                current = current->rchild;
            } else {
                target = current;
                break;
            }
        }

        if (target == nullptr) {
            cout << "Value " << value << " not found in the tree.\n";
            return -1;
        }

        if (target->rchild != nullptr) {
            Node* temp = min_node(target->rchild);
            if (temp) {
                cout << "The inorder successor of " << value << " is " << temp->data << '\n';
                return temp->data;
            }
        }

        if (successor != nullptr) {
            cout << "The inorder successor of " << value << " is " << successor->data << '\n';
            return successor->data;
        }

        cout << "The element " << value << " has no inorder successor (it's the maximum).\n";
        return -1;
    }

    int inorder_predecessor(Node* root, int value) {
        if (root == nullptr) {
            cout << "Tree is empty! Cannot find the inorder predecessor!\n";
            return -1;
        }

        Node* current = root;
        Node* target = nullptr;
        Node* predecessor = nullptr;

        while (current != nullptr) {
            if (value > current->data) {
                predecessor = current;
                current = current->rchild;
            }
            else if (value < current->data) {
                current = current->lchild;
            }
            else {
                target = current;
                break;
            }
        }

        if (target == nullptr) {
            cout << "Value " << value << " not found in the BST.\n";
            return -1;
        }

        if (target->lchild != nullptr) {
            Node* temp = target->lchild;
            while (temp->rchild != nullptr)
                temp = temp->rchild; 
            cout << "The inorder predecessor of " << value << " is " << temp->data << '\n';
            return temp->data;
        }

        if (predecessor != nullptr) {
            cout << "The inorder predecessor of " << value << " is " << predecessor->data << '\n';
            return predecessor->data;
        }

        cout << "The element " << value << " has no inorder predecessor (it's the minimum).\n";
        return -1;
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
    B1.deleteElement(B1.root, 7);

    B1.inorder_traversal(B1.root);
    cout << endl;
    // B1.preorder_traversal(B1.root);
    // cout << endl;
    // B1.postorder_traversal(B1.root);
    B1.Iterative_search(B1.root, 2);
    cout << endl;
    cout << B1.max_element(B1.root) << endl;
    cout << B1.min_element(B1.root) << endl;
    B1.inorder_successor(B1.root, 3);

    return 0;
}
