// #include <algorithm>
// #include <stdio.h>

// int main()
// {
//     float corner11x, corner12x, corner21x, corner22x;
//     float corner11y, corner12y, corner21y, corner22y;
    
//     corner11x = 0.0;
//     corner12x = 10.0;
//     corner21x = 2.0;
//     corner22x = 8.0;

//     corner11y = 9.0;
//     corner12y = 8.0;
//     corner21y = 2.0;
//     corner22y = 0.0;

//     float pose_x_array[] = {corner11x, corner12x, corner21x, corner22x};
//     float pose_y_array[] = {corner11y, corner12y, corner21y, corner22y};

//     std::sort(pose_x_array[0], pose_x_array[3]);

//     printf("%f\n", pose_x_array[0]);
//     printf("%f\n", pose_x_array[1]);
//     printf("%f\n", pose_x_array[2]);
//     printf("%f\n", pose_x_array[3]);
// }

// C++ program to sort an array
#include <algorithm>
#include <iostream>
#include <stdio.h>

using namespace std;
 
void show(float a[], int array_size)
{
    for (int i = 0; i < array_size; ++i)
        cout << a[i] << " ";
}
 
// Driver code
int main()
{
    float a[] = { 11.0, 10.0, 8.0, 9.0};
   
    // print the array
    show(a, 4);
    
    printf("\n");
      // sort the array
    sort(a, a + 4);
 
   
    // print the array after sorting
    show(a, 4);
 
    return 0;
}