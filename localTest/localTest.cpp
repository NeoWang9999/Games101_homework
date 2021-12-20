#include<iostream>
#include<vector>
#include<numeric>
using namespace std;
int main() {
    vector<int> v = { 2,7,6,10 };
    cout << "Sum of all the elements are:" << endl;
    cout << accumulate(v.begin(), v.end(), 2);
}