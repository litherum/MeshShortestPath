#include "pch.h"
#include "MMP.h"

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

#include <cassert>

typedef CGAL::Simple_cartesian<double> Kernel;

template <class Refs>
class MMPHalfedge : public CGAL::HalfedgeDS_halfedge_base<Refs> {
public:
	uint8_t index; // 0 means "pointing from vertex 0 to vertex 1"
};

template <class Refs, class Traits>
class MMPFace : public CGAL::HalfedgeDS_face_base<Refs, CGAL::Tag_true, typename Traits::Plane_3> {
public:
	MMP::TriangleIndices::size_type index;
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef MMPHalfedge<Refs> Halfedge;
	};
	template <class Refs, class Traits>
	struct Face_wrapper {
		typedef typename Traits::Plane_3 Plane;
		typedef MMPFace<Refs, Traits> Face;
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
	Polyhedron::Point startingPoint;
};

static Polyhedron::Point convertPoint(const MMP::PointHeap::value_type& point) {
	return Polyhedron::Point(std::get<0>(point), std::get<1>(point), std::get<2>(point));
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
		builder.begin_surface(pointHeap.size(), triangles.size());
		std::vector<Polyhedron::Vertex_handle> vertices;
		for (MMP::PointHeap::size_type i = 0; i < pointHeap.size(); ++i) {
			const auto& point = pointHeap[i];
			auto vertex = builder.add_vertex(convertPoint(point));
			vertices.push_back(vertex);
		}
		for (MMP::TriangleIndices::size_type i = 0; i < triangles.size(); ++i) {
			const auto& triangle = triangles[i];
			auto facet = builder.begin_facet();
			builder.add_vertex_to_facet(std::get<0>(triangle));
			builder.add_vertex_to_facet(std::get<1>(triangle));
			builder.add_vertex_to_facet(std::get<2>(triangle));
			auto halfedge = builder.end_facet();

			assert(halfedge->vertex() == vertices[std::get<0>(triangle)]);
			assert(halfedge->next()->vertex() == vertices[std::get<1>(triangle)]);
			assert(halfedge->next()->next()->vertex() == vertices[std::get<2>(triangle)]);
			assert(halfedge->next()->next()->next() == halfedge);

			facet->index = i;
			auto p0 = halfedge->vertex()->point();
			auto p1 = halfedge->next()->vertex()->point();
			auto p2 = halfedge->next()->next()->vertex()->point();
			facet->plane() = Polyhedron::Plane_3(p0, p1, p2);
			halfedge->index = 2;
			halfedge->next()->index = 0;
			halfedge->next()->next()->index = 1;
			if (i == startingPointOwningTriangle) {
				startingFacet = facet;
			}
		}
		builder.end_surface();
	}

	const MMP::PointHeap& pointHeap;
	const MMP::TriangleIndices& triangles;
	MMP::TriangleIndices::size_type startingPointOwningTriangle;
	Polyhedron::Facet_handle startingFacet;
};

MMP::Impl::Impl(PointHeap pointHeap, TriangleIndices triangles, TriangleIndices::size_type startingPointOwningTriangle, double u, double v) {
	// FIXME: Make sure that everything is in bounds.
	Generator generator(pointHeap, triangles, startingPointOwningTriangle);
	polyhedron.delegate(generator);
	// FIXME: Make sure we are a manifold.
	startingFacet = generator.getStartingFacet();

	auto startingTriangle = triangles[startingPointOwningTriangle];
	auto p0 = convertPoint(pointHeap[std::get<0>(startingTriangle)]);
	auto p1 = convertPoint(pointHeap[std::get<1>(startingTriangle)]);
	auto p2 = convertPoint(pointHeap[std::get<2>(startingTriangle)]);
	Kernel::Vector_3 v0 = p1 - p0;
	Kernel::Vector_3 v1 = p2 - p0;
	startingPoint = p0 + u * v0 + v * v1;
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
