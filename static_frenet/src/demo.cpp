// #include <iostream>
// #include<cmath>

// int main()
// {

//     double yaw = std::atan2(-0.00138855,-0.921158);
//     std::cout << "yaw"<<yaw << std::endl;
// }

// #include <iostream>
// #include <forward_list>

// using std::forward_list; 
// using std::cout;

// auto remove_odds(forward_list<int>& flist)
// {
//     auto is_odd = [] (int i) { return i & 0x1; };//////??????????????????????????????
//     flist.remove_if(is_odd);
// }

// int main()
// {
//     forward_list<int> data = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
//     remove_odds(data);
//     for (auto i : data) 
//         cout << i << " ";

//     return 0;
// }
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include <cmath>

long getTime(const std::vector<int>& piles, int speed) {
    long time = 0;
    for (int pile : piles) {
        int curTime = (pile + speed - 1) / speed;
        time += curTime;
    }
    return time;
}

int main() {
    std::vector<int> piles={100,100};
    int h = 3;
    // int k = 0, sum = 0, low=1, high=0;
    // for (int pile : piles) {
    //     high = std::max(high, pile);
    // }
    // if(piles.size() == h)return high;
    // while(low<high)
    // {
    //     int speed =(low+high)/2;
    //     std::cout <<speed << std::endl;
    //     for(int i=0;i<piles.size();++i)
    //     {
    //         sum+=std::ceil(1.0*piles[i]/speed);
    //     }
    //     if(sum>h){
    //         low=speed+1;
    //         sum=0;
    //     }else{
    //         k=speed;
    //         high=speed;
    //         sum=0;
    //     }
    // }
    // std::cout<<k<<std::endl;
    // return k;
    int low = 1;
    int high = 0;
    for (int pile : piles) {
        high = std::max(high, pile);
    }
    int k = high;
    while (low < high) {
        int speed = (high - low) / 2 + low;
        std::cout<<speed<<std::endl;
        long time = getTime(piles, speed);
        if (time <= h) {
            k = speed;
            high = speed;
        } else {
            low = speed + 1;
        }
    }
    return k;
}

