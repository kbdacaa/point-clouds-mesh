#pragma once

#include <vector>

using namespace std;

template <class T>
struct Array3D
{
	Array3D()
	{
		clear();
	}

	std::vector<T> data;
	int size[3];

	void resize(int x, int y, int z)
	{
		data.resize(x*y*z);
		size[0] = x;
		size[1] = y;
		size[2] = z;
	}

	T &operator()(int x, int y, int z)
	{
		assert(x >= 0 && y >= 0 && z >= 0);
		assert(x < size[0] && y < size[1] && z < size[2]);

		return data[(z*size[1] + y)*size[0] + x];
	}

	int index(int x, int y, int z)
	{
		return (z*size[1] + y)*size[0] + x;
	}

	void clear()
	{
		size[2] = size[1] = size[0] = 0;
		data.clear();
	}
};
