// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#define CGAL_CHECK_EXPENSIVE
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#undef CGAL_CHECK_EXPENSIVE

typedef CGAL::Simple_cartesian<double> Kernel;

class CandidateInterval {
public:
	CandidateInterval(Kernel::FT length, Kernel::Point_3 unfoldedRoot) : m_length(length), m_unfoldedRoot(unfoldedRoot) {
	}

private:
	Kernel::FT m_length;
	Kernel::Point_3 m_unfoldedRoot;
};

template <class Refs>
class HalfedgeWithIntervalVector : public CGAL::HalfedgeDS_halfedge_base<Refs> {
public:
	void insertInterval(CandidateInterval&& interval) {
		intervals.emplace_back(std::move(interval));
	}

private:
	std::vector<CandidateInterval> intervals;
};

struct MMP_Items : public CGAL::Polyhedron_items_3 {
	template <class Refs, class Traits>
	struct Halfedge_wrapper {
		typedef HalfedgeWithIntervalVector<Refs> Halfedge;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MMP_Items> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

class Generator : public CGAL::Modifier_base<HalfedgeDS> {
public:
	Polyhedron::Facet_handle halfOfFrontFace() {
		assert(m_halfOfFrontFace != Polyhedron::Facet_handle());
		return m_halfOfFrontFace;
	}

private:
	Polyhedron::Facet_handle addTriangle(CGAL::Polyhedron_incremental_builder_3<HalfedgeDS>& builder, std::size_t index0, std::size_t index1, std::size_t index2) {
		auto facet = builder.begin_facet();
		builder.add_vertex_to_facet(index0);
		builder.add_vertex_to_facet(index1);
		builder.add_vertex_to_facet(index2);
		builder.end_facet();
		return facet;
	}

	void operator()(HalfedgeDS& halfedgeDS) {
		CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder(halfedgeDS, true);
		builder.begin_surface(0, 0, 0);
		builder.add_vertex(Polyhedron::Point(0, 0, 0));
		builder.add_vertex(Polyhedron::Point(0, 1, 0));
		builder.add_vertex(Polyhedron::Point(1, 1, 0));
		builder.add_vertex(Polyhedron::Point(1, 0, 0));
		builder.add_vertex(Polyhedron::Point(0, 0, 1));
		builder.add_vertex(Polyhedron::Point(0, 1, 1));
		builder.add_vertex(Polyhedron::Point(1, 1, 1));
		builder.add_vertex(Polyhedron::Point(1, 0, 1));
		// Front
		m_halfOfFrontFace = addTriangle(builder, 0, 1, 2);
		addTriangle(builder, 2, 3, 0);
		// Left
		addTriangle(builder, 4, 5, 1);
		addTriangle(builder, 1, 0, 4);
		// Right
		addTriangle(builder, 3, 2, 6);
		addTriangle(builder, 6, 7, 3);
		// Back
		addTriangle(builder, 7, 6, 5);
		addTriangle(builder, 5, 4, 7);
		// Top
		addTriangle(builder, 1, 5, 6);
		addTriangle(builder, 6, 2, 1);
		// Bottom
		addTriangle(builder, 4, 0, 3);
		addTriangle(builder, 3, 7, 4);
		builder.end_surface();
	}

	Polyhedron::Facet_handle m_halfOfFrontFace;
};

Kernel::FT halfedgeLength(Polyhedron::Halfedge_handle halfedge) {
	auto point0 = halfedge->vertex()->point();
	auto point1 = halfedge->opposite()->vertex()->point();
	Kernel::Vector_3 displacement(point1.x() - point0.x(), point1.y() - point0.y(), point1.z() - point0.z());
	return std::sqrt(displacement.squared_length());
}

template <typename T>
void iterateHalfedges(Polyhedron::Facet_handle facet, T callback) {
	auto halfedge = facet->halfedge();
	auto halfedge2 = halfedge;
	do {
		halfedge2 = halfedge2->next();
		callback(halfedge2);
	} while (halfedge2 != halfedge);
}

static void mmp(const Polyhedron& polyhedron, Polyhedron::Facet_handle startFacet, Polyhedron::Point pointOnStartFace) {
	// Event queue is priority queue
	// ILIST is a heap. It might be unnecessary.
	iterateHalfedges(startFacet, [&](Polyhedron::Halfedge_handle halfedge) {
		halfedge->insertInterval(CandidateInterval(halfedgeLength(halfedge), pointOnStartFace));
	});
}

static Polyhedron::Point middlePoint(const Polyhedron::Facet_handle facet) {
	std::vector<Polyhedron::Point> points;
	iterateHalfedges(facet, [&](Polyhedron::Halfedge_handle halfedge) {
		points.emplace_back(halfedge->vertex()->point());
	});
	Polyhedron::Point result(0, 0, 0);
	for (auto point : points) {
		result = Polyhedron::Point(result.x() + point.x(), result.y() + point.y(), result.z() + point.z());
	}
	return Polyhedron::Point(result.x() / points.size(), result.y() / points.size(), result.z() / points.size());
}

int main()
{
	Polyhedron polyhedron;
	Generator generator;
	polyhedron.delegate(generator);
	auto halfOfFrontFace = generator.halfOfFrontFace();
	mmp(polyhedron, halfOfFrontFace, middlePoint(halfOfFrontFace));
    return 0;
}

