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

class CircularLinkedList {
private:
    Node* head;

public:
    CircularLinkedList() {
        head = nullptr;
    }

    void insertLast(int data) {
        Node* newNode = new Node(data);
        if (!head) {
            head = newNode;
            newNode->next = head;  
            return;
        }
        Node* temp = head;
        while (temp->next != head)
            temp = temp->next;
        temp->next = newNode;
        newNode->next = head;  
    }

    void display() {
        if (!head) {
            cout << "List is empty.\n";
            return;
        }
        Node* temp = head;
        cout << "Circular Linked List: ";
        do {
            cout << temp->data << " ";
            temp = temp->next;
        } while (temp != head);
        cout << head->data << endl; 
    }
};


int main() {
    CircularLinkedList cll;

    
    cll.insertLast(20);
    cll.insertLast(100);
    cll.insertLast(40);
    cll.insertLast(80);
    cll.insertLast(60);

    
    cll.display();

    return 0;
}