#pragma once

#include <array>
#include <memory>
#include <vector>

#ifdef MMP_EXPORTS
#define MMP_DECL __declspec(dllexport)
#else
#define MMP_DECL __declspec(dllimport)
#endif

class MMP_DECL MMP final {
public:
	MMP(std::vector<std::array<double, 3>> pointHeap, std::vector<std::size_t> indices);
	~MMP();

private:
	class Impl;

	std::unique_ptr<Impl> impl;
};