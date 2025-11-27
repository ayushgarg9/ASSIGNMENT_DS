#include <iostream>
#include <vector>
using namespace std;

void improvedSelectionSort(vector<int>& a) {
    int left = 0, right = a.size() - 1;

    while(left < right) {
        int minIndex = left, maxIndex = left;

        for(int i = left; i <= right; i++) {
            if(a[i] < a[minIndex]) minIndex = i;
            if(a[i] > a[maxIndex]) maxIndex = i;
        }

    
        swap(a[left], a[minIndex]);

        
        if(maxIndex == left) 
            maxIndex = minIndex;

    
        swap(a[right], a[maxIndex]);

        left++;
        right--;
    }
}

