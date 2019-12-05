#include <utility>

#define SWAP std::swap

template <typename T, typename V>
void c_qsort(T t[],
             V addition[],
             int left, int right,
             int (*cmp)(T, T))
{
    int size = right - left;
    if (size <= 0)
    {
        return;
    }

    
};
