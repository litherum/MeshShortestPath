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
	// FIXME: This is too restrictive.
	MMP() = delete;
	MMP(const MMP&) = delete;
	MMP(MMP&&) = delete;
	void operator=(const MMP&) = delete;
	void operator=(MMP&&) = delete;

	typedef std::vector<std::array<double, 3>> PointHeap;
	typedef std::vector<std::array<PointHeap::size_type, 3>> TriangleIndices;
	MMP(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v);
	~MMP();

	void run();

	struct HalfedgeInterval {
		std::array<double, 3> unfoldedRoot;
		double beginpointFraction; // FIXME: Remove this when we are guaranteed to have a covering.
		double endpointFraction;
		double depth;
	};
	// Parallel to "triangles" argument passed to constructor.
	std::vector<std::array<std::vector<HalfedgeInterval>, 3>> intervals() const;

private:
	class Impl;

	std::unique_ptr<Impl> impl;
};