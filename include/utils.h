#include <utility>

#define SWAP std::swap

template <typename T, typename V>
void c_bsort(T t[],
             V addition[],
             int size,
             int (*cmp)(const T, const T))
{
    if (size <= 0)
    {
        return;
    }

    for (size_t i = 0; i < size - 1; i++)
    {
        for (size_t j = 0; j < size; j++)
        {
            if (cmp(t[i], t[j]) < 0)
            {
                SWAP(t[i], t[j]);
                SWAP(addition[i], addition[j]);
            }
        }
    }
};


template <typename T, typename V>
void qsort(T t[],
           V addition[],
           size_t low,
           size_t high)
{
    if (low < high)
    {
        T pivot = t[high];
        size_t left = low, right = high - 1;
        while(1){
            while (left <= right && t[left] < pivot)
            {
                left++;
            }
            while (right >= left && t[right] > pivot)
            {
                right--;
            } 
            if (left >= right)
            {
                break;
            }
            SWAP(t[left], t[right]);
            SWAP(addition[left], addition[right]);
            left++;
            right--;
        }
        SWAP(t[left], t[high]);
        SWAP(addition[left], addition[high]);
        
        // size_t pi = partition(t, addition, low, high);
        // size_t pi = left;

        qsort(t, addition, low, left - 1);
        qsort(t, addition, left + 1, high);
    }
}

template <typename T, typename V>
void c_qsort(T t[],
             V addition[],
			 int size)
{
    if (size <= 0)
    {
        return;
    }

    size_t first=0, last=size-1;
    qsort(t, addition, first, last);
}
