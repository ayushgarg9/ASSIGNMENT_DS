#include <iostream>
using namespace std;

struct Node {
    int data;
    Node* next;
};


Node* head = nullptr;


void insertAtBeginning(int value) {
    Node* newNode = new Node{value, head};
    head = newNode;
    cout << "Inserted at beginning.\n";
}


void insertAtEnd(int value) {
    Node* newNode = new Node{value, nullptr};
    if (head == nullptr) {
        head = newNode;
    } else {
        Node* temp = head;
        while (temp->next != nullptr) {
            temp = temp->next;
        }
        temp->next = newNode;
    }
    cout << "Inserted at end.\n";
}


void insertBefore(int target, int value) {
    if (head == nullptr) {
        cout << "List is empty.\n";
        return;
    }

    if (head->data == target) {
        insertAtBeginning(value);
        return;
    }

    Node* prev = nullptr;
    Node* curr = head;

    while (curr != nullptr && curr->data != target) {
        prev = curr;
        curr = curr->next;
    }

    if (curr == nullptr) {
        cout << "Target value not found.\n";
        return;
    }

    Node* newNode = new Node{value, curr};
    prev->next = newNode;
    cout << "Inserted " << value << " before " << target << ".\n";
}


void insertAfter(int target, int value) {
    Node* curr = head;

    while (curr != nullptr && curr->data != target) {
        curr = curr->next;
    }

    if (curr == nullptr) {
        cout << "Target value not found.\n";
        return;
    }

    Node* newNode = new Node{value, curr->next};
    curr->next = newNode;
    cout << "Inserted " << value << " after " << target << ".\n";
}


void deleteFromBeginning() {
    if (head == nullptr) {
        cout << "List is empty.\n";
        return;
    }
    Node* temp = head;
    head = head->next;
    delete temp;
    cout << "Deleted from beginning.\n";
}


void deleteFromEnd() {
    if (head == nullptr) {
        cout << "List is empty.\n";
        return;
    }
    if (head->next == nullptr) {
        delete head;
        head = nullptr;
        cout << "Deleted last node.\n";
        return;
    }
    Node* temp = head;
    while (temp->next->next != nullptr) {
        temp = temp->next;
    }
    delete temp->next;
    temp->next = nullptr;
    cout << "Deleted from end.\n";
}


void deleteNode(int value) {
    if (head == nullptr) {
        cout << "List is empty.\n";
        return;
    }

    if (head->data == value) {
        Node* temp = head;
        head = head->next;
        delete temp;
        cout << "Node " << value << " deleted.\n";
        return;
    }

    Node* curr = head;
    Node* prev = nullptr;

    while (curr != nullptr && curr->data != value) {
        prev = curr;
        curr = curr->next;
    }

    if (curr == nullptr) {
        cout << "Value not found.\n";
        return;
    }

    prev->next = curr->next;
    delete curr;
    cout << "Node " << value << " deleted.\n";
}


void searchNode(int value) {
    Node* temp = head;
    int position = 1;

    while (temp != nullptr) {
        if (temp->data == value) {
            cout << "Value found at position " << position << ".\n";
            return;
        }
        temp = temp->next;
        position++;
    }

    cout << "Value not found.\n";
}


void displayList() {
    if (head == nullptr) {
        cout << "List is empty.\n";
        return;
    }
    Node* temp = head;
    cout << "Linked List: ";
    while (temp != nullptr) {
        cout << temp->data << " -> ";
        temp = temp->next;
    }
    cout << "NULL\n";
}

int main() {
    int choice, value, target;

    while (true) {
        cout << "\n--- Singly Linked List Menu ---\n";
        cout << "1. Insert at Beginning\n";
        cout << "2. Insert at End\n";
        cout << "3. Insert Before a Node\n";
        cout << "4. Insert After a Node\n";
        cout << "5. Delete from Beginning\n";
        cout << "6. Delete from End\n";
        cout << "7. Delete Specific Node\n";
        cout << "8. Search for a Node\n";
        cout << "9. Display List\n";
        cout << "10. Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1:
                cout << "Enter value: ";
                cin >> value;
                insertAtBeginning(value);
                break;
            case 2:
                cout << "Enter value: ";
                cin >> value;
                insertAtEnd(value);
                break;
            case 3:
                cout << "Enter value to insert: ";
                cin >> value;
                cout << "Enter target value to insert before: ";
                cin >> target;
                insertBefore(target, value);
                break;
            case 4:
                cout << "Enter value to insert: ";
                cin >> value;
                cout << "Enter target value to insert after: ";
                cin >> target;
                insertAfter(target, value);
                break;
            case 5:
                deleteFromBeginning();
                break;
            case 6:
                deleteFromEnd();
                break;
            case 7:
                cout << "Enter value to delete: ";
                cin >> value;
                deleteNode(value);
                break;
            case 8:
                cout << "Enter value to search: ";
                cin >> value;
                searchNode(value);
                break;
            case 9:
                displayList();
                break;
            case 10:
                cout << "Exiting...\n";
                return 0;
            default:
                cout << "Invalid choice. Try again.\n";
        }
    }
}