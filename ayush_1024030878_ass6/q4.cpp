#include <iostream>
using namespace std;

class Node {
public:
    char data;
    Node* prev;
    Node* next;

    Node(char d) {
        data = d;
        prev = nullptr;
        next = nullptr;
    }
};

class DoublyLinkedList {
private:
    Node* head;

public:
    DoublyLinkedList() {
        head = nullptr;
    }

    
    void insertLast(char data) {
        Node* newNode = new Node(data);
        if (!head) {
            head = newNode;
            return;
        }
        Node* temp = head;
        while (temp->next != nullptr)
            temp = temp->next;
        temp->next = newNode;
        newNode->prev = temp;
    }

    
    bool isPalindrome() {
        if (!head) return true; 

        
        Node* tail = head;
        while (tail->next)
            tail = tail->next;

        
        Node* start = head;
        while (start != tail && tail->next != start) {
            if (start->data != tail->data)
                return false;
            start = start->next;
            tail = tail->prev;
        }
        return true;
    }

    void display() {
        Node* temp = head;
        cout << "Doubly Linked List: ";
        while (temp) {
            cout << temp->data << " ";
            temp = temp->next;
        }
        cout << endl;
    }
};


int main() {
    DoublyLinkedList dll;
    string str;

    cout << "Enter a string: ";
    cin >> str;

    
    for (char c : str)
        dll.insertLast(c);

    dll.display();

    if (dll.isPalindrome())
        cout << "The list is a PALINDROME." << endl;
    else
        cout << "The list is NOT a palindrome." << endl;

    return 0;
}