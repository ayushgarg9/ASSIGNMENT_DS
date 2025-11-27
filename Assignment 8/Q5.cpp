#include <bits/stdc++.h>
using namespace std;

void siftDownMax(vector<int>& a, int start, int end) {
    int root = start;
    while (true) {
        int left = 2*root + 1;
        if (left > end) break;
        int right = left + 1;
        int idx = left;
        if (right <= end && a[right] > a[left]) idx = right;
        if (a[idx] > a[root]) {
            swap(a[root], a[idx]);
            root = idx;
        } else break;
    }
}

void buildMaxHeap(vector<int>& a) {
    for (int i = (a.size()-2)/2; i >= 0; i--)
        siftDownMax(a, i, a.size()-1);
}

void heapsortAsc(vector<int>& a) {
    buildMaxHeap(a);
    int end = a.size()-1;
    while (end > 0) {
        swap(a[0], a[end]);
        end--;
        siftDownMax(a, 0, end);
    }
}

void siftDownMin(vector<int>& a, int start, int end) {
    int root = start;
    while (true) {
        int left = 2*root + 1;
        if (left > end) break;
        int right = left + 1;
        int idx = left;
        if (right <= end && a[right] < a[left]) idx = right;
        if (a[idx] < a[root]) {
            swap(a[root], a[idx]);
            root = idx;
        } else break;
    }
}

void buildMinHeap(vector<int>& a) {
    for (int i = (a.size()-2)/2; i >= 0; i--)
        siftDownMin(a, i, a.size()-1);
}

void heapsortDesc(vector<int>& a) {
    buildMinHeap(a);
    int end = a.size()-1;
    while (end > 0) {
        swap(a[0], a[end]);
        end--;
        siftDownMin(a, 0, end);
    }
}

int main() {
    vector<int> a = {5,3,8,1,9,2};
    heapsortAsc(a);
    for (int x : a) cout << x << " ";
    cout << "\n";
    vector<int> b = {5,3,8,1,9,2};
    heapsortDesc(b);
    for (int x : b) cout << x << " ";
    return 0;
}
