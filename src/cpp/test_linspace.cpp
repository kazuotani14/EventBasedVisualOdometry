// C++ implementations of matlab functions (helpers)

#include <vector>
#include <iostream>
#include <algorithm>

// From: https://gist.github.com/jmbr/2375233
// look at link for possible improvements

template <typename T>
std::vector<T> linspace(T start, T end, int N)
{
	std::vector<T> vec(N);

	T h = (end - start) / static_cast<T>(N-1);

	vec[0] = start;
	for(int i=1; i<N; i++)
		vec[i] = vec[i-1] + h;

	return vec;
}

int main()
{
	double min_depth = 0.1;
	double max_depth = 1.5;
	int N_planes = 10;

	std::vector<double> planes_depths = linspace(1/min_depth, 1/max_depth, N_planes);
	std::for_each(planes_depths.begin(), planes_depths.end(), [](double& d){ d = 1/d;});
	for(auto const& value : planes_depths)
		std::cout << value << ' ';
}
