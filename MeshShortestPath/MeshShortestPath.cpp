// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <list>
#include <algorithm>
#include <vector>
#include <cassert>

#include "Types.h"
#include "GenerateGeometry.h"

namespace MeshShortestPath {

	template <typename T>
	static void iterateHalfedges(Polyhedron::Facet_handle facet, T callback) {
		auto halfedge = facet->halfedge();
		auto halfedge2 = halfedge;
		do {
			halfedge2 = halfedge2->next();
			callback(halfedge2);
		} while (halfedge2 != halfedge);
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

	class MMP {
	public:
		MMP(const Polyhedron& polyhedron, Polyhedron::Facet_handle initialFacet, Polyhedron::Point pointOnInitialFacet) : polyhedron(polyhedron) {
			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				CandidateInterval interval(halfedge, pointOnInitialFacet, pointOnInitialFacet, pointOnInitialFacet, 0, 0, 1);
				candidateIntervals.emplace_back(std::move(interval));
				auto iterator = candidateIntervals.end();
				--iterator;
				assert(&*iterator == &candidateIntervals.back());
				halfedge->insertInterval(iterator);
				eventQueue.place(Event(Event::Type::FrontierPoint, iterator->getFrontierPoint(), iterator));
				eventQueue.place(Event(Event::Type::EndPoint, iterator->getLeftExtent(), iterator));
				eventQueue.place(Event(Event::Type::EndPoint, iterator->getRightExtent(), iterator));
			});
		}

		void populate() {
			while (!eventQueue.empty()) {
				eventQueue.remove(); // FIXME: Implement this
			}
		}

	private:
		static Kernel::FT halfedgeLength(Polyhedron::Halfedge_handle halfedge) {
			auto point0 = halfedge->vertex()->point();
			auto point1 = halfedge->opposite()->vertex()->point();
			Kernel::Vector_3 displacement(point1.x() - point0.x(), point1.y() - point0.y(), point1.z() - point0.z());
			return std::sqrt(displacement.squared_length());
		}

		const Polyhedron polyhedron;
		std::list<CandidateInterval> candidateIntervals;
		EventQueue eventQueue;
	};

}

int main()
{
	MeshShortestPath::Polyhedron polyhedron = MeshShortestPath::generateCube();
	auto facet = polyhedron.facets_begin();
	MeshShortestPath::MMP mmp(polyhedron, facet, middlePoint(facet));
	mmp.populate();
    return 0;
}

