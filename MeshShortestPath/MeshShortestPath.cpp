// MeshShortestPath.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <list>
#include <algorithm>
#include <vector>
#include <cassert>
#include <memory>

#include "Types.h"
#include "GenerateGeometry.h"

namespace MeshShortestPath {

	static void printPoint(std::ostream& stream, Polyhedron::Point point) {
		stream << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
	}

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
		// FIXME: The input can either be a point on the interior of a triangle, a point on the interior of an edge, or a vertex.
		MMP(Polyhedron& polyhedron, Polyhedron::Facet_handle initialFacet, Polyhedron::Point pointOnInitialFacet) : polyhedron(polyhedron) {
			std::vector<Polyhedron::Point> initialFacetPoints;
			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				initialFacetPoints.push_back(halfedge->vertex()->point());
			});
			std::cout << "Starting with point ";
			printPoint(std::cout, pointOnInitialFacet);
			std::cout << " on facet ";
			for (auto& point : initialFacetPoints) {
				if (&point != &initialFacetPoints.front())
					std::cout << ", ";
				printPoint(std::cout, point);
			}

			iterateHalfedges(initialFacet, [&](Polyhedron::Halfedge_handle halfedge) {
				CandidateInterval interval(halfedge, pointOnInitialFacet, pointOnInitialFacet, 0, 0, 1);
				candidateIntervals.emplace_back(std::move(interval));
				std::list<CandidateInterval>::iterator iterator = candidateIntervals.end();
				--iterator;
				assert(&*iterator == &candidateIntervals.back());
				halfedge->insertInterval(iterator);
				eventQueue.place(std::make_unique<FrontierPointEvent>(iterator->getFrontierPoint(), iterator));
				eventQueue.place(std::make_unique<EndPointEvent>(halfedge->vertex()->point(), *iterator));
			});
		}

		void populate() {
			while (!eventQueue.empty()) {
				std::unique_ptr<Event> e = eventQueue.remove();
				if (FrontierPointEvent* event = dynamic_cast<FrontierPointEvent*>(e.get())) {
					// FIXME: Possibly permanently label the event
					// FIXME: Do propagate(event->getCandidateInterval())
				}
				else if (EndPointEvent* event = dynamic_cast<EndPointEvent*>(e.get())) {
					// FIXME: Possibly permanently label the event
				}
			}
		}

		~MMP() {
			// FIXME: Find some way to export the data
			for (auto i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end(); ++i) {
				i->clear();
			}
		}

	private:
		Polyhedron& polyhedron;
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

