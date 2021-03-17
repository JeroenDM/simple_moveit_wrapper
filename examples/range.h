#pragma once

/** \brief Create a simple 1D range, similar to NumPy's and Matlab's linspace. **/
template <typename T>
std::vector<T> range(T lower_bound, T upper_bound, int num_samples)
{
    std::vector<T> r;
    if (num_samples == 1)
    {
        r = { lower_bound };
    }
    else
    {
        r.resize(num_samples);
        T increment = (upper_bound - lower_bound) / (num_samples - 1);
        for (int i = 0; i < num_samples; ++i)
        {
            r[i] = lower_bound + i * increment;
        }
    }
    return r;
}
