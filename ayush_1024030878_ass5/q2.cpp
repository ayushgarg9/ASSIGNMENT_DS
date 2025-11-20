#include <iostream>
using namespace std;

class Node {
public:
    int data;
    Node* next;

    Node(int val) {
        data = val;
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

    
    int countOccurrences(int key) {
        int count = 0;
        Node* temp = head;
        while (temp) {
            if (temp->data == key) count++;
            temp = temp->next;
        }
        return count;
    }

    
    void deleteAllOccurrences(int key) {
      
        while (head && head->data == key) {
            Node* temp = head;
            head = head->next;
            delete temp;
        }

        Node* curr = head;
        Node* prev = nullptr;

        while (curr) {
            if (curr->data == key) {
                Node* temp = curr;
                prev->next = curr->next;
                curr = curr->next;
                delete temp;
            } else {
                prev = curr;
                curr = curr->next;
            }
        }
    }
};

int main() {
    LinkedList list;
    int n, val, key;

    cout << "Enter number of nodes: ";
    cin >> n;

    cout << "Enter node values:\n";
    for (int i = 0; i < n; i++) {
        cin >> val;
        list.insertAtEnd(val);
    }

    cout << "Current list: ";
    list.display();

    cout << "Enter key to count and delete: ";
    cin >> key;

    int count = list.countOccurrences(key);
    cout << "Number of occurrences of " << key << ": " << count << "\n";

    if (count > 0) {
        list.deleteAllOccurrences(key);
        cout << "List after deleting all occurrences of " << key << ": ";
        list.display();
    } else {
        cout << "Key not found in list.\n";
    }

    return 0;
}