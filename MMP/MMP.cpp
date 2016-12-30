#include "pch.h"
#include "MMP.h"

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

typedef CGAL::Simple_cartesian<double> Kernel;

template <class Refs>
class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef HalfedgeWithIntervalVector<Refs> Halfedge;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;

class MMP::Impl {
public:
	// FIXME: This is too restrictive.
	Impl() = delete;
	Impl(const Impl&) = delete;
	Impl(Impl&&) = delete;
	void operator=(const Impl&) = delete;
	void operator=(Impl&&) = delete;

	Impl(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v);

	void run();
	std::vector<std::array<std::vector<HalfedgeInterval>, 3>> intervals() const;

private:
	Polyhedron polyhedron;
	Polyhedron::Facet_handle startingFacet;
	double startingU;
	double startingV;
};

class Generator : public CGAL::Modifier_base<Polyhedron::HalfedgeDS> {
public:
	Generator(const MMP::PointHeap& pointHeap, const MMP::TriangleIndices& triangles, MMP::TriangleIndices::size_type startingPointOwningTriangle) : pointHeap(pointHeap), triangles(triangles), startingPointOwningTriangle(startingPointOwningTriangle) {
	}

	Polyhedron::Facet_handle getStartingFacet() {
		return startingFacet;
	}

private:
	void operator()(Polyhedron::HalfedgeDS& halfedgeDS) {
		CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> builder(halfedgeDS, true);
		builder.begin_surface(0, 0, 0);
		for (const auto& point : pointHeap) {
			builder.add_vertex(Polyhedron::Point(std::get<0>(point), std::get<1>(point), std::get<2>(point)));
		}
		for (MMP::TriangleIndices::size_type i = 0; i < triangles.size(); ++i) {
			const auto& triangle = triangles[i];
			auto facet = builder.begin_facet();
			builder.add_vertex_to_facet(std::get<0>(triangle));
			builder.add_vertex_to_facet(std::get<1>(triangle));
			builder.add_vertex_to_facet(std::get<2>(triangle));
			builder.end_facet();
			if (i == startingPointOwningTriangle) {
				startingFacet = facet;
				// FIXME: Perhaps fixup u and v here.
			}
		}
		builder.end_surface();
	}

	const MMP::PointHeap& pointHeap;
	const MMP::TriangleIndices& triangles;
	MMP::TriangleIndices::size_type startingPointOwningTriangle;
	Polyhedron::Facet_handle startingFacet;
};

class PlaneEquation {
public:
	Polyhedron::Plane_3 operator()(Polyhedron::Facet& facet) {
		auto halfedge = facet.halfedge();
		auto p0 = halfedge->vertex()->point();
		auto p1 = halfedge->next()->vertex()->point();
		auto p2 = halfedge->next()->next()->vertex()->point();
		return Polyhedron::Plane_3(p0, p1, p2);
	}
};

MMP::Impl::Impl(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) : startingU(u), startingV(v) {
	// FIXME: Make sure that everything is in bounds.
	Generator generator(pointHeap, triangles, startingPointOwningTriangle);
	polyhedron.delegate(generator);
	// FIXME: Make sure we are a manifold.
	startingFacet = generator.getStartingFacet();
	std::transform(polyhedron.facets_begin(), polyhedron.facets_end(), polyhedron.planes_begin(), PlaneEquation());
}

void MMP::Impl::run() {
	// FIXME: Implement this.
}

auto MMP::Impl::intervals() const -> std::vector<std::array<std::vector<HalfedgeInterval>, 3>> {
	// FIXME: Implement this.
	return {};
}

MMP::MMP(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) : impl(std::make_unique<Impl>(pointHeap, triangles, startingPointOwningTriangle, u, v)) {
}

MMP::~MMP() {
}

void MMP::run() {
	impl->run();
}

auto MMP::intervals() const -> std::vector<std::array<std::vector<HalfedgeInterval>, 3>> {
	return impl->intervals();
}
