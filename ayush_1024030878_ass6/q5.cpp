#include <iostream>
using namespace std;

class Node {
public:
    int data;
    Node* next;

    Node(int d) {
        data = d;
        next = nullptr;
    }
};

class LinkedList {
private:
    Node* head;

public:
    LinkedList() {
        head = nullptr;
    }

    
    void insertLast(int data) {
        Node* newNode = new Node(data);
        if (!head) {
            head = newNode;
            return;
        }
        Node* temp = head;
        while (temp->next != nullptr)
            temp = temp->next;
        temp->next = newNode;
    }

   
    void makeCircular() {
        if (!head) return;
        Node* temp = head;
        while (temp->next != nullptr)
            temp = temp->next;
        temp->next = head; 
    }

 
    bool isCircular() {
        if (!head) return false;  

        Node* temp = head->next;
        while (temp != nullptr && temp != head)
            temp = temp->next;

        return (temp == head);
    }

    void display() {
        if (!head) {
            cout << "List is empty.\n";
            return;
        }
        Node* temp = head;
        cout << "Linked List: ";
        int count = 0; 
        while (temp && count < 20) {
            cout << temp->data << " ";
            temp = temp->next;
            count++;
        }
        cout << endl;
    }
};


int main() {
    LinkedList list;

    
    list.insertLast(10);
    list.insertLast(20);
    list.insertLast(30);
    list.insertLast(40);

    list.display();
    cout << "Is circular? " << (list.isCircular() ? "Yes" : "No") << endl;

    
    list.makeCircular();

    cout << "\nAfter making it circular...\n";
    cout << "Is circular? " << (list.isCircular() ? "Yes" : "No") << endl;

    return 0;
}