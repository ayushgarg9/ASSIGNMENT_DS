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

    int getSize() {
        if (!head) return 0;
        int count = 0;
        Node* temp = head;
        do {
            count++;
            temp = temp->next;
        } while (temp != head);
        return count;
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
        cout << endl;
    }
};

int main() {
    CircularLinkedList cll;
    cll.insertLast(5);
    cll.insertLast(15);
    cll.insertLast(25);
    cll.insertLast(35);

    cll.display();
    cout << "Size of Circular Linked List = " << cll.getSize() << endl;

    return 0;
}