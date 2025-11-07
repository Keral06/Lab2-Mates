#pragma once
#include <cstddef>

struct OpsCounter 
{
	std::size_t add = 0, mul = 0;

	void Reset() 
	{ 
		add = mul = 0;
	}

	void IncAdd(std::size_t k = 1) 
	{ 
		add += k; 
	}

	void IncMul(std::size_t k = 1) 
	{ 
		mul += k; 
	}
};