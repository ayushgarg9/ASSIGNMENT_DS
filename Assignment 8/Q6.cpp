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

struct MaxPQ {
    vector<int> h;

    void push(int x) {
        h.push_back(x);
        int i = h.size()-1;
        while (i > 0) {
            int p = (i-1)/2;
            if (h[p] >= h[i]) break;
            swap(h[p], h[i]);
            i = p;
        }
    }

    int top() {
        if (h.empty()) return INT_MIN;
        return h[0];
    }

    void pop() {
        if (h.empty()) return;
        h[0] = h.back();
        h.pop_back();
        if (!h.empty()) siftDownMax(h, 0, h.size()-1);
    }

    bool empty() {
        return h.empty();
    }
};

int main() {
    MaxPQ pq;
    pq.push(10);
    pq.push(5);
    pq.push(20);
    pq.push(1);
    cout << pq.top() << "\n";
    pq.pop();
    cout << pq.top() << "\n";
    return 0;
}
