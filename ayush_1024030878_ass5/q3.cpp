#include <iostream>
using namespace std;

class Node {
public:
    int data;
    Node* next;

    Node(int val){
        data=val;
        next=nullptr;
    }
};

class LinkedList {
private:
    Node* head;

public:
    LinkedList() : head(nullptr) {}

    void insertAtEnd(int value) {
        Node* newNode = new Node(value);
        if (!head) {
            head = newNode;
            return;
        }
        Node* temp = head;
        while (temp->next) temp = temp->next;
        temp->next = newNode;
    }

    void display() {
        if (!head) {
            cout << "List is empty.\n";
            return;
        }
        Node* temp = head;
        while (temp) {
            cout << temp->data << " -> ";
            temp = temp->next;
        }
        cout << "NULL\n";
    }

    void findMiddle() {
        if (!head) {
            cout << "List is empty.\n";
            return;
        }
        Node* slow = head;
        Node* fast = head;

        while (fast != nullptr && fast->next != nullptr) {
            slow = slow->next;
            fast = fast->next->next;
        }
        cout << "Middle element is: " << slow->data << "\n";
    }
};

int main() {
    LinkedList list;
    int n, val;

    cout << "Enter number of nodes: ";
    cin >> n;

    cout << "Enter node values:\n";
    for (int i = 0; i < n; i++) {
        cin >> val;
        list.insertAtEnd(val);
    }

    cout << "Current list: ";
    list.display();

    list.findMiddle();

    return 0;
}