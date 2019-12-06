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
