#include <iostream>
using namespace std;


class DoublyLinkedList {
    struct Node {
        int data;
        Node* prev;
        Node* next;
        Node(int d) : data(d), prev(nullptr), next(nullptr) {}
    };
    Node* head;

public:
    DoublyLinkedList() : head(nullptr) {}

    void insertFirst(int data) {
        Node* newNode = new Node(data);
        if (head) {
            newNode->next = head;
            head->prev = newNode;
        }
        head = newNode;
    }

    void insertLast(int data) {
        Node* newNode = new Node(data);
        if (!head) {
            head = newNode;
            return;
        }
        Node* temp = head;
        while (temp->next)
            temp = temp->next;
        temp->next = newNode;
        newNode->prev = temp;
    }

    void insertAfter(int after, int data) {
        Node* temp = head;
        while (temp && temp->data != after)
            temp = temp->next;
        if (!temp) {
            cout << "Node not found!\n";
            return;
        }
        Node* newNode = new Node(data);
        newNode->next = temp->next;
        newNode->prev = temp;
        if (temp->next)
            temp->next->prev = newNode;
        temp->next = newNode;
    }

    void deleteNode(int key) {
        Node* temp = head;
        while (temp && temp->data != key)
            temp = temp->next;
        if (!temp) {
            cout << "Node not found!\n";
            return;
        }
        if (temp == head)
            head = temp->next;
        if (temp->next)
            temp->next->prev = temp->prev;
        if (temp->prev)
            temp->prev->next = temp->next;
        delete temp;
        cout << "Node deleted.\n";
    }

    void search(int key) {
        Node* temp = head;
        int pos = 1;
        while (temp) {
            if (temp->data == key) {
                cout << "Found at position " << pos << endl;
                return;
            }
            temp = temp->next;
            pos++;
        }
        cout << "Not found!\n";
    }

    void display() {
        if (!head) {
            cout << "Empty list.\n";
            return;
        }
        Node* temp = head;
        cout << "Doubly Linked List: ";
        while (temp) {
            cout << temp->data << " ";
            temp = temp->next;
        }
        cout << endl;
    }
};


class CircularLinkedList {
    struct Node {
        int data;
        Node* next;
        Node(int d) : data(d), next(nullptr) {}
    };
    Node* head;

public:
    CircularLinkedList() : head(nullptr) {}

    void insertFirst(int data) {
        Node* newNode = new Node(data);
        if (!head) {
            head = newNode;
            newNode->next = head;
            return;
        }
        Node* temp = head;
        while (temp->next != head)
            temp = temp->next;
        newNode->next = head;
        temp->next = newNode;
        head = newNode;
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

    void deleteNode(int key) {
        if (!head) {
            cout << "Empty list.\n";
            return;
        }
        Node *curr = head, *prev = nullptr;
        do {
            if (curr->data == key) {
                if (curr == head && curr->next == head) {
                    delete curr;
                    head = nullptr;
                    return;
                }
                if (curr == head) {
                    prev = head;
                    while (prev->next != head)
                        prev = prev->next;
                    head = head->next;
                    prev->next = head;
                } else {
                    prev->next = curr->next;
                }
                delete curr;
                cout << "Node deleted.\n";
                return;
            }
            prev = curr;
            curr = curr->next;
        } while (curr != head);
        cout << "Node not found!\n";
    }

    void search(int key) {
        if (!head) {
            cout << "Empty list.\n";
            return;
        }
        Node* temp = head;
        int pos = 1;
        do {
            if (temp->data == key) {
                cout << "Found at position " << pos << endl;
                return;
            }
            temp = temp->next;
            pos++;
        } while (temp != head);
        cout << "Not found!\n";
    }

    void display() {
        if (!head) {
            cout << "Empty list.\n";
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
    DoublyLinkedList dll;
    CircularLinkedList cll;
    int type, choice, data, ref;

    while (true) {
        cout << "\n==== MAIN MENU ====\n";
        cout << "1. Operate on Doubly Linked List\n";
        cout << "2. Operate on Circular Linked List\n";
        cout << "0. Exit\nChoice: ";
        cin >> type;
        if (type == 0)
            break;

        cout << "1. Insert First\n2. Insert Last\n3. Insert After (Doubly only)\n"
                "4. Delete Node\n5. Search Node\n6. Display\n0. Back\n";
        cout << "Enter your choice: ";
        cin >> choice;

        if (type == 1) { 
            switch (choice) {
                case 1: cout << "Enter data: "; cin >> data; dll.insertFirst(data); break;
                case 2: cout << "Enter data: "; cin >> data; dll.insertLast(data); break;
                case 3: cout << "Insert after which value? "; cin >> ref; cout << "Enter data: "; cin >> data; dll.insertAfter(ref, data); break;
                case 4: cout << "Enter data to delete: "; cin >> data; dll.deleteNode(data); break;
                case 5: cout << "Enter data to search: "; cin >> data; dll.search(data); break;
                case 6: dll.display(); break;
            }
        } else if (type == 2) { 
            switch (choice) {
                case 1: cout << "Enter data: "; cin >> data; cll.insertFirst(data); break;
                case 2: cout << "Enter data: "; cin >> data; cll.insertLast(data); break;
                case 4: cout << "Enter data to delete: "; cin >> data; cll.deleteNode(data); break;
                case 5: cout << "Enter data to search: "; cin >> data; cll.search(data); break;
                case 6: cll.display(); break;
            }
        }
    }
    return 0;
}